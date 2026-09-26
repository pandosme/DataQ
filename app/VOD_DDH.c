#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include <datahub/client.h>
#include <datahub/subscriber.h>
#include <glib.h>

#include "VOD.h"

#define DDH_TOPIC "com.axis.scene.object_track.v1"
#define DDH_FRAME_TOPIC "com.axis.scene.frame.v1"
#define DDH_MAX_CACHED_OBJECTS 256
#define DDH_REPLAY_SILENCE_MS 500
#define DDH_STALE_TIMEOUT_MS 30000

#define LOG(fmt, args...) { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_WARN(fmt, args...) { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args); }

typedef struct {
    char *json;
    char *topic;
    gint generation;
} ddh_sample_t;

typedef struct {
    vod_object_t object;
    vod_attribute_t color_attribute;
    char color[64];
    gint64 last_real_ms;
} ddh_cached_object_t;

static DHClient *ddh_client = NULL;
static DHSubscriber *ddh_subscriber = NULL;
static vod_callback_t object_callback = NULL;
static void *object_callback_data = NULL;
static GHashTable *object_cache = NULL;
static GHashTable *observed_labels = NULL;
static GHashTable *advertised_labels = NULL;
static GHashTable *advertised_properties = NULL;
static cJSON *topic_definitions = NULL;
static guint replay_timer_id = 0;
static guint reconnect_timer_id = 0;
static int ddh_channel_id = 1;
static gint ddh_connected = 0;
static gint ddh_shutting_down = 0;
static gint reconnect_in_progress = 0;
static gint reset_pending = 0;
static gint stream_generation = 0;
static gint active_object_count = 0;
static gint malformed_log_count = 0;
static gint unclassified_log_count = 0;
static GMutex metrics_mutex;
static guint64 real_sample_count = 0;
static guint64 frame_sample_count = 0;
static guint64 track_summary_count = 0;
static guint64 ended_object_count = 0;
static guint64 stale_object_count = 0;
static guint64 malformed_sample_count = 0;
static guint64 ignored_sample_count = 0;
static guint64 synthetic_batch_count = 0;
static guint64 cache_eviction_count = 0;
static gint64 last_real_sample_ms = 0;

static gboolean reconnect_client(gpointer user_data);

static void increment_metric(guint64 *metric) {
    g_mutex_lock(&metrics_mutex);
    (*metric)++;
    g_mutex_unlock(&metrics_mutex);
}

static void record_real_sample(void) {
    g_mutex_lock(&metrics_mutex);
    real_sample_count++;
    last_real_sample_ms = g_get_real_time() / 1000;
    g_mutex_unlock(&metrics_mutex);
}

static void log_malformed_sample(const char *reason, cJSON *root) {
    if (g_atomic_int_add(&malformed_log_count, 1) >= 5) return;

    char *json = cJSON_PrintUnformatted(root);
    LOG_WARN("%s: %s: %.1024s\n", __func__, reason, json ? json : "<unprintable>");
    free(json);
}

static void log_unclassified_sample(cJSON *root) {
    if (g_atomic_int_add(&unclassified_log_count, 1) >= 5) return;

    char *json = cJSON_PrintUnformatted(root);
    LOG_WARN("%s: DDH track has no classes: %.1024s\n",
             __func__, json ? json : "<unprintable>");
    free(json);
}

static void log_ddh_error(DHError **error, const char *context) {
    if (!error || !*error) return;
    LOG_WARN("%s: %s\n", context, dh_error_to_string(*error));
    dh_error_destroy(*error);
    *error = NULL;
}

static void collect_definition_metadata(cJSON *node, const char *property_name) {
    if (cJSON_IsObject(node)) {
        cJSON *properties = cJSON_GetObjectItemCaseSensitive(node, "properties");
        if (cJSON_IsObject(properties)) {
            cJSON *property = NULL;
            cJSON_ArrayForEach(property, properties) {
                if (advertised_properties && property->string)
                    g_hash_table_add(advertised_properties, g_strdup(property->string));
                collect_definition_metadata(property, property->string);
            }
        }

        cJSON *enum_values = cJSON_GetObjectItemCaseSensitive(node, "enum");
        if (property_name && strcmp(property_name, "type") == 0 && cJSON_IsArray(enum_values)) {
            cJSON *value = NULL;
            cJSON_ArrayForEach(value, enum_values) {
                if (cJSON_IsString(value) && value->valuestring && advertised_labels)
                    g_hash_table_add(advertised_labels, g_strdup(value->valuestring));
            }
        }

        cJSON *child = NULL;
        cJSON_ArrayForEach(child, node) {
            if (child == properties || child == enum_values) continue;
            collect_definition_metadata(child, child->string);
        }
    } else if (cJSON_IsArray(node)) {
        cJSON *item = NULL;
        cJSON_ArrayForEach(item, node)
            collect_definition_metadata(item, property_name);
    }
}

static void refresh_definition_metadata(void) {
    if (advertised_labels) g_hash_table_remove_all(advertised_labels);
    if (advertised_properties) g_hash_table_remove_all(advertised_properties);
    if (!topic_definitions) return;

    cJSON *definition = NULL;
    cJSON_ArrayForEach(definition, topic_definitions)
        collect_definition_metadata(definition, NULL);
}

static void cache_topic_definition(const char *topic_name, const char *definition_json) {
    if (!topic_definitions || !topic_name || !definition_json) return;
    cJSON *definition = cJSON_Parse(definition_json);
    if (!definition) {
        LOG_WARN("%s: Invalid definition for %s\n", __func__, topic_name);
        return;
    }
    if (cJSON_GetObjectItemCaseSensitive(topic_definitions, topic_name))
        cJSON_ReplaceItemInObjectCaseSensitive(topic_definitions, topic_name, definition);
    else
        cJSON_AddItemToObject(topic_definitions, topic_name, definition);
    refresh_definition_metadata();
}

static cJSON *string_set_to_json(GHashTable *set) {
    cJSON *list = cJSON_CreateArray();
    if (!list || !set) return list;
    GHashTableIter iterator;
    gpointer value = NULL;
    g_hash_table_iter_init(&iterator, set);
    while (g_hash_table_iter_next(&iterator, &value, NULL))
        cJSON_AddItemToArray(list, cJSON_CreateString(value));
    return list;
}

static void log_topic_inventory(void) {
    DHError *error = NULL;
    DHTopicList *topics = dh_client_get_topic_list(ddh_client, &error);
    if (!topics) {
        log_ddh_error(&error, "DDH topic inventory failed");
        return;
    }

    uint32_t topic_count = dh_topic_list_get_count(topics);
    LOG("%s: Device Data Hub advertises %u topics\n", __func__, topic_count);
    for (uint32_t topic_index = 0; topic_index < topic_count; ++topic_index) {
        const char *topic_name = dh_topic_list_get_name(topics, topic_index);
        if (!topic_name) continue;
        LOG("%s: DDH topic[%u]=%s\n", __func__, topic_index, topic_name);
        if (!strstr(topic_name, ".scene.")) continue;

        DHTopic *topic = dh_client_get_topic(ddh_client, topic_name, &error);
        if (topic) {
            const char *definition = dh_topic_get_json_definition(topic, &error);
            if (definition) {
                LOG("%s: DDH scene topic definition: %.4096s\n", __func__, definition);
                if (strcmp(topic_name, DDH_FRAME_TOPIC) == 0 || strcmp(topic_name, DDH_TOPIC) == 0)
                    cache_topic_definition(topic_name, definition);
            } else {
                log_ddh_error(&error, "DDH scene topic definition failed");
            }
            dh_topic_destroy(topic);
        } else {
            log_ddh_error(&error, "DDH scene topic lookup failed");
        }

        DHTopicInstanceList *instances = dh_client_get_topic_instances(ddh_client, topic_name, &error);
        if (!instances) {
            log_ddh_error(&error, "DDH scene instance inventory failed");
            continue;
        }
        uint32_t instance_count = dh_topic_instance_list_get_count(instances);
        LOG("%s: DDH scene topic %s advertises %u instances\n",
            __func__, topic_name, instance_count);
        for (uint32_t instance_index = 0; instance_index < instance_count; ++instance_index) {
            const DHTopicInstance *instance = dh_topic_instance_list_get(instances, instance_index);
            if (!instance) continue;
            const DHInstanceKeys *keys = dh_topic_instance_get_keys(instance);
            int64_t channel_id = 0;
            bool has_channel = keys && dh_instance_keys_get_integer(keys, "channel_id", &channel_id, &error);
            if (error) log_ddh_error(&error, "DDH scene instance channel lookup failed");
            LOG("%s: DDH scene instance[%u] channel_id=%s%lld info=%s\n",
                __func__, instance_index, has_channel ? "" : "unknown/",
                (long long)channel_id,
                dh_topic_instance_get_info(instance) ? dh_topic_instance_get_info(instance) : "<none>");
        }
        dh_topic_instance_list_destroy(instances);
    }
    dh_topic_list_destroy(topics);
}

static double clamp_unit(double value) {
    if (value < 0.0) return 0.0;
    if (value > 1.0) return 1.0;
    return value;
}

static cJSON *highest_score(cJSON *items) {
    if (!cJSON_IsArray(items)) return NULL;

    cJSON *best = NULL;
    double best_score = -1.0;
    cJSON *item = NULL;
    cJSON_ArrayForEach(item, items) {
        cJSON *score = cJSON_GetObjectItemCaseSensitive(item, "score");
        if (cJSON_IsNumber(score) && score->valuedouble > best_score) {
            best = item;
            best_score = score->valuedouble;
        }
    }
    return best;
}

static gboolean reset_objects_on_main(gpointer user_data) {
    (void)user_data;
    g_atomic_int_set(&reset_pending, 0);
    if (g_atomic_int_get(&ddh_shutting_down)) return G_SOURCE_REMOVE;

    if (object_cache) g_hash_table_remove_all(object_cache);
    g_atomic_int_set(&active_object_count, 0);
    if (object_callback)
        object_callback(NULL, 0, VOD_BATCH_RESET, object_callback_data);
    return G_SOURCE_REMOVE;
}

static void request_object_reset(void) {
    if (g_atomic_int_get(&ddh_shutting_down)) return;
    if (g_atomic_int_compare_and_exchange(&reset_pending, 0, 1)) {
        g_atomic_int_inc(&stream_generation);
        g_main_context_invoke(NULL, reset_objects_on_main, NULL);
    }
}

static gboolean schedule_reconnect_on_main(gpointer user_data) {
    (void)user_data;
    if (!g_atomic_int_get(&ddh_shutting_down) && !reconnect_timer_id)
        reconnect_timer_id = g_timeout_add_seconds(5, reconnect_client, NULL);
    return G_SOURCE_REMOVE;
}

static gboolean replay_active_objects(gpointer user_data) {
    (void)user_data;
    if (!object_callback || !object_cache || !g_atomic_int_get(&ddh_connected))
        return G_SOURCE_CONTINUE;

    guint count = g_hash_table_size(object_cache);
    if (count == 0) return G_SOURCE_CONTINUE;

    vod_object_t *objects = calloc(count, sizeof(vod_object_t));
    if (!objects) return G_SOURCE_CONTINUE;

    GHashTableIter iterator;
    gpointer key = NULL;
    gpointer value = NULL;
    guint index = 0;
    gint64 now_ms = g_get_monotonic_time() / 1000;
    g_hash_table_iter_init(&iterator, object_cache);
    while (g_hash_table_iter_next(&iterator, &key, &value)) {
        ddh_cached_object_t *cached = value;
        gint64 silence_ms = now_ms - cached->last_real_ms;
        if (silence_ms >= DDH_STALE_TIMEOUT_MS) {
            cached->object.active = false;
            object_callback(&cached->object, 1, VOD_BATCH_REAL, object_callback_data);
            g_hash_table_iter_remove(&iterator);
            increment_metric(&stale_object_count);
        } else if (silence_ms >= DDH_REPLAY_SILENCE_MS && index < count) {
            objects[index++] = cached->object;
        }
    }

    if (index > 0) {
        object_callback(objects, index, VOD_BATCH_SYNTHETIC, object_callback_data);
        increment_metric(&synthetic_batch_count);
    }
    g_atomic_int_set(&active_object_count, g_hash_table_size(object_cache));
    free(objects);
    return G_SOURCE_CONTINUE;
}

static int update_bbox(vod_object_t *object, cJSON *bbox) {
    cJSON *left = bbox ? cJSON_GetObjectItemCaseSensitive(bbox, "left") : NULL;
    cJSON *top = bbox ? cJSON_GetObjectItemCaseSensitive(bbox, "top") : NULL;
    cJSON *right = bbox ? cJSON_GetObjectItemCaseSensitive(bbox, "right") : NULL;
    cJSON *bottom = bbox ? cJSON_GetObjectItemCaseSensitive(bbox, "bottom") : NULL;
    if (!object || !cJSON_IsNumber(left) || !cJSON_IsNumber(top) ||
        !cJSON_IsNumber(right) || !cJSON_IsNumber(bottom)) return 0;

    double x1 = clamp_unit(left->valuedouble);
    double y1 = clamp_unit(top->valuedouble);
    double x2 = clamp_unit(right->valuedouble);
    double y2 = clamp_unit(bottom->valuedouble);
    if (x2 <= x1 || y2 <= y1) return 0;

    object->x = (int)(x1 * 1000.0 + 0.5);
    object->y = (int)(y1 * 1000.0 + 0.5);
    object->w = (int)((x2 - x1) * 1000.0 + 0.5);
    object->h = (int)((y2 - y1) * 1000.0 + 0.5);
    return 1;
}

static ddh_cached_object_t *cache_object(const char *id) {
    ddh_cached_object_t *cached = g_hash_table_lookup(object_cache, id);
    if (cached) return cached;

    if (g_hash_table_size(object_cache) >= DDH_MAX_CACHED_OBJECTS) {
        GHashTableIter iterator;
        gpointer oldest_key = NULL;
        gpointer key = NULL;
        gpointer value = NULL;
        gint64 oldest_time = G_MAXINT64;
        g_hash_table_iter_init(&iterator, object_cache);
        while (g_hash_table_iter_next(&iterator, &key, &value)) {
            ddh_cached_object_t *candidate = value;
            if (candidate->last_real_ms < oldest_time) {
                oldest_time = candidate->last_real_ms;
                oldest_key = key;
            }
        }
        if (oldest_key) {
            ddh_cached_object_t *evicted = g_hash_table_lookup(object_cache, oldest_key);
            if (evicted && object_callback) {
                evicted->object.active = false;
                object_callback(&evicted->object, 1, VOD_BATCH_REAL, object_callback_data);
            }
            g_hash_table_remove(object_cache, oldest_key);
            increment_metric(&cache_eviction_count);
        }
    }

    cached = calloc(1, sizeof(ddh_cached_object_t));
    char *cache_key = strdup(id);
    if (!cached || !cache_key) {
        free(cached);
        free(cache_key);
        return NULL;
    }
    snprintf(cached->object.id, sizeof(cached->object.id), "%s", id);
    cached->object.type = -1;
    g_hash_table_insert(object_cache, cache_key, cached);
    return cached;
}

static int update_classification(ddh_cached_object_t *cached, cJSON *classification) {
    cJSON *type = classification ? cJSON_GetObjectItemCaseSensitive(classification, "type") : NULL;
    cJSON *score = classification ? cJSON_GetObjectItemCaseSensitive(classification, "score") : NULL;
    if (!cached || !cJSON_IsString(type) || !type->valuestring || !cJSON_IsNumber(score)) return 0;

    snprintf(cached->object.class_name, sizeof(cached->object.class_name), "%s", type->valuestring);
    if (observed_labels)
        g_hash_table_add(observed_labels, g_strdup(type->valuestring));
    cached->object.confidence = (float)(clamp_unit(score->valuedouble) * 100.0);
    cached->object.attributes = NULL;
    cached->object.num_attributes = 0;
    cached->color[0] = '\0';

    cJSON *colors = cJSON_GetObjectItemCaseSensitive(classification, "colors");
    cJSON *best_color = highest_score(colors);
    cJSON *color_name = best_color ? cJSON_GetObjectItemCaseSensitive(best_color, "name") : NULL;
    if (cJSON_IsString(color_name) && color_name->valuestring) {
        snprintf(cached->color, sizeof(cached->color), "%s", color_name->valuestring);
        cached->color_attribute.name = "vehicle_color";
        cached->color_attribute.value = cached->color;
        cached->object.attributes = &cached->color_attribute;
        cached->object.num_attributes = 1;
    }
    return 1;
}

static int end_cached_object(const char *id, cJSON *classification, cJSON *bbox) {
    if (!id) return 0;
    ddh_cached_object_t *cached = g_hash_table_lookup(object_cache, id);
    if (!cached) return 0;
    if (classification) update_classification(cached, classification);
    if (bbox) update_bbox(&cached->object, bbox);
    cached->object.active = false;
    if (object_callback)
        object_callback(&cached->object, 1, VOD_BATCH_REAL, object_callback_data);
    g_hash_table_remove(object_cache, id);
    increment_metric(&ended_object_count);
    record_real_sample();
    return 1;
}

static void process_frame(cJSON *root) {
    increment_metric(&frame_sample_count);
    cJSON *detections = cJSON_GetObjectItemCaseSensitive(root, "detections");
    if (!cJSON_IsArray(detections)) {
        increment_metric(&malformed_sample_count);
        return;
    }
    int detection_count = cJSON_IsArray(detections) ? cJSON_GetArraySize(detections) : 0;
    vod_object_t *updates = detection_count > 0 ? calloc(detection_count, sizeof(vod_object_t)) : NULL;
    GHashTable *seen_ids = g_hash_table_new(g_str_hash, g_str_equal);
    size_t update_count = 0;

    cJSON *detection = NULL;
    cJSON_ArrayForEach(detection, detections) {
        cJSON *id = cJSON_GetObjectItemCaseSensitive(detection, "object_track_id");
        cJSON *bbox = cJSON_GetObjectItemCaseSensitive(detection, "bounding_box");
        cJSON *classification = cJSON_GetObjectItemCaseSensitive(detection, "class");
        if (!cJSON_IsString(id) || !id->valuestring || !bbox) {
            increment_metric(&ignored_sample_count);
            continue;
        }

        g_hash_table_add(seen_ids, id->valuestring);

        vod_object_t geometry = {0};
        if (!update_bbox(&geometry, bbox)) {
            increment_metric(&malformed_sample_count);
            continue;
        }

        ddh_cached_object_t *cached = g_hash_table_lookup(object_cache, id->valuestring);
        if (!cached) {
            cJSON *type = classification ? cJSON_GetObjectItemCaseSensitive(classification, "type") : NULL;
            cJSON *score = classification ? cJSON_GetObjectItemCaseSensitive(classification, "score") : NULL;
            if (!cJSON_IsString(type) || !cJSON_IsNumber(score)) {
                increment_metric(&ignored_sample_count);
                continue;
            }
            cached = cache_object(id->valuestring);
        }
        if (!cached) continue;
        cached->object.x = geometry.x;
        cached->object.y = geometry.y;
        cached->object.w = geometry.w;
        cached->object.h = geometry.h;
        if (classification) update_classification(cached, classification);
        if (!cached->object.class_name[0]) {
            increment_metric(&ignored_sample_count);
            continue;
        }
        cached->object.active = true;
        cached->last_real_ms = g_get_monotonic_time() / 1000;
        if (updates && update_count < (size_t)detection_count)
            updates[update_count++] = cached->object;
    }

    if (update_count > 0 && object_callback) {
        object_callback(updates, update_count, VOD_BATCH_REAL, object_callback_data);
        record_real_sample();
    }
    free(updates);

    /* A frame is a complete active-object snapshot. Remove tracks omitted by it. */
    GHashTableIter cache_iterator;
    gpointer cache_key = NULL;
    gpointer cache_value = NULL;
    g_hash_table_iter_init(&cache_iterator, object_cache);
    while (g_hash_table_iter_next(&cache_iterator, &cache_key, &cache_value)) {
        if (g_hash_table_contains(seen_ids, cache_key)) continue;
        ddh_cached_object_t *cached = cache_value;
        cached->object.active = false;
        if (object_callback)
            object_callback(&cached->object, 1, VOD_BATCH_REAL, object_callback_data);
        g_hash_table_iter_remove(&cache_iterator);
        increment_metric(&ended_object_count);
    }
    g_hash_table_destroy(seen_ids);

    cJSON *events = cJSON_GetObjectItemCaseSensitive(root, "track_events");
    cJSON *event = NULL;
    cJSON_ArrayForEach(event, events) {
        cJSON *type = cJSON_GetObjectItemCaseSensitive(event, "type");
        if (!cJSON_IsString(type) || !type->valuestring) continue;
        if (strcmp(type->valuestring, "TrackEnded") == 0) {
            cJSON *id = cJSON_GetObjectItemCaseSensitive(event, "object_track_id");
            if (cJSON_IsString(id) && id->valuestring)
                end_cached_object(id->valuestring, NULL, NULL);
        } else if (strcmp(type->valuestring, "Rename") == 0) {
            cJSON *from_id = cJSON_GetObjectItemCaseSensitive(event, "from_id");
            if (cJSON_IsString(from_id) && from_id->valuestring)
                end_cached_object(from_id->valuestring, NULL, NULL);
        }
    }
    g_atomic_int_set(&active_object_count, g_hash_table_size(object_cache));
}

static int end_part_references(cJSON *parts, cJSON *classification, cJSON *bbox,
                               const char **first_id) {
    int ended = 0;
    cJSON *part = NULL;
    cJSON_ArrayForEach(part, parts) {
        cJSON *id = cJSON_GetObjectItemCaseSensitive(part, "object_track_id");
        if (cJSON_IsString(id) && id->valuestring) {
            if (!*first_id) *first_id = id->valuestring;
            ended += end_cached_object(id->valuestring, classification, bbox);
        }
        cJSON *nested = cJSON_GetObjectItemCaseSensitive(part, "parts");
        if (cJSON_IsArray(nested))
            ended += end_part_references(nested, classification, bbox, first_id);
    }
    return ended;
}

static void process_track_summary(cJSON *root) {
    increment_metric(&track_summary_count);
    cJSON *classes = cJSON_GetObjectItemCaseSensitive(root, "classes");
    cJSON *path = cJSON_GetObjectItemCaseSensitive(root, "path");
    cJSON *classification = highest_score(classes);
    cJSON *position = cJSON_IsArray(path) && cJSON_GetArraySize(path) > 0
        ? cJSON_GetArrayItem(path, cJSON_GetArraySize(path) - 1) : NULL;
    cJSON *bbox = position ? cJSON_GetObjectItemCaseSensitive(position, "bounding_box") : NULL;
    cJSON *parts = cJSON_GetObjectItemCaseSensitive(root, "parts");
    const char *first_id = NULL;
    int ended = cJSON_IsArray(parts)
        ? end_part_references(parts, classification, bbox, &first_id) : 0;

    if (ended == 0 && classification && bbox) {
        cJSON *summary_id = cJSON_GetObjectItemCaseSensitive(root, "id");
        const char *id = first_id ? first_id
            : (cJSON_IsString(summary_id) ? summary_id->valuestring : NULL);
        ddh_cached_object_t terminal = {0};
        if (id) {
            snprintf(terminal.object.id, sizeof(terminal.object.id), "%s", id);
            terminal.object.type = -1;
            terminal.object.active = false;
            if (update_classification(&terminal, classification) &&
                update_bbox(&terminal.object, bbox) && object_callback) {
                object_callback(&terminal.object, 1, VOD_BATCH_REAL, object_callback_data);
                record_real_sample();
            }
        }
    } else if (!classification) {
        increment_metric(&ignored_sample_count);
        log_unclassified_sample(root);
    }
    g_atomic_int_set(&active_object_count, g_hash_table_size(object_cache));
}

static gboolean process_sample_on_main(gpointer user_data) {
    ddh_sample_t *sample = user_data;
    if (!sample) return G_SOURCE_REMOVE;

    if (g_atomic_int_get(&ddh_shutting_down) ||
        sample->generation != g_atomic_int_get(&stream_generation)) {
        free(sample->json);
        free(sample->topic);
        free(sample);
        return G_SOURCE_REMOVE;
    }

    cJSON *root = cJSON_Parse(sample->json);
    free(sample->json);
    if (!root) {
        free(sample->topic);
        free(sample);
        increment_metric(&malformed_sample_count);
        LOG_WARN("%s: Invalid JSON payload\n", __func__);
        return G_SOURCE_REMOVE;
    }

    cJSON *channel = cJSON_GetObjectItemCaseSensitive(root, "channel_id");
    if (!cJSON_IsNumber(channel) || channel->valueint != ddh_channel_id) {
        increment_metric(cJSON_IsNumber(channel) ? &ignored_sample_count : &malformed_sample_count);
    } else if (strcmp(sample->topic, DDH_FRAME_TOPIC) == 0) {
        process_frame(root);
    } else {
        process_track_summary(root);
    }

    cJSON_Delete(root);
    free(sample->topic);
    free(sample);
    return G_SOURCE_REMOVE;
}

static void on_data_received(const DHTopicSample *topic_sample, void *user_data) {
    (void)user_data;
    if (!topic_sample || g_atomic_int_get(&ddh_shutting_down)) return;
    gint generation = g_atomic_int_get(&stream_generation);

    const char *topic = dh_topic_sample_get_topic_name(topic_sample);
    if (!topic || (strcmp(topic, DDH_FRAME_TOPIC) != 0 && strcmp(topic, DDH_TOPIC) != 0) ||
        dh_topic_sample_is_historical(topic_sample)) return;

    const DHTopicData *topic_data = dh_topic_sample_get_data(topic_sample);
    const char *json = topic_data ? dh_topic_data_get_json_data(topic_data) : NULL;
    if (!json) return;

    ddh_sample_t *sample = calloc(1, sizeof(ddh_sample_t));
    if (!sample) return;
    sample->json = strdup(json);
    sample->topic = strdup(topic);
    if (!sample->json || !sample->topic) {
        free(sample->json);
        free(sample->topic);
        free(sample);
        return;
    }
    sample->generation = generation;
    g_main_context_invoke(NULL, process_sample_on_main, sample);
}

static void on_connection_update(DHConnectionState state, void *user_data) {
    (void)user_data;
    g_atomic_int_set(&ddh_connected, state == DH_CONN_CONNECTED);
    LOG("%s: Device Data Hub %s\n", __func__,
        state == DH_CONN_CONNECTED ? "connected" : "disconnected");
    if (state == DH_CONN_DISCONNECTED && !g_atomic_int_get(&reconnect_in_progress)) {
        request_object_reset();
        g_main_context_invoke(NULL, schedule_reconnect_on_main, NULL);
    }
}

static void on_instance_update(const DHTopicInstance *topic_instance,
                               DHTopicInstanceUpdateType update_type, void *user_data) {
    (void)user_data;
    if (!topic_instance || update_type != DH_TOPIC_INSTANCE_DELETED) return;
    const char *topic = dh_topic_instance_get_topic_name(topic_instance);
    const DHInstanceKeys *keys = dh_topic_instance_get_keys(topic_instance);
    int64_t channel_id = 0;
    DHError *error = NULL;
    bool matching_channel = keys &&
        dh_instance_keys_get_integer(keys, "channel_id", &channel_id, &error) &&
        channel_id == ddh_channel_id;
    if (error) dh_error_destroy(error);
    if (topic && strcmp(topic, DDH_TOPIC) == 0 && matching_channel) request_object_reset();
}

static int setup_subscription(void) {
    DHError *error = NULL;
    ddh_subscriber = dh_client_create_subscriber(ddh_client, "DataQ scene metadata", &error);
    if (!ddh_subscriber) {
        log_ddh_error(&error, "DDH subscriber creation failed");
        return 0;
    }

    if (!dh_subscriber_set_data_callback(ddh_subscriber, on_data_received, NULL, &error)) {
        log_ddh_error(&error, "DDH data callback registration failed");
        return 0;
    }
    if (!dh_subscriber_set_instance_update_callback(ddh_subscriber, on_instance_update, NULL, &error)) {
        log_ddh_error(&error, "DDH instance callback registration failed");
        return 0;
    }

    DHFilter *filter = dh_filter_create();
    DHSubscribeOptions *options = dh_subscribe_options_create();
    if (!filter || !options) {
        LOG_WARN("%s: Failed to allocate DDH subscription configuration\n", __func__);
        dh_filter_destroy(filter);
        dh_subscribe_options_destroy(options);
        return 0;
    }

    bool configured = dh_filter_add_topic_name(filter, DDH_TOPIC, &error);
    if (!configured) log_ddh_error(&error, "DDH topic filter failed");
    if (configured) configured = dh_filter_add_topic_name(filter, DDH_FRAME_TOPIC, &error);
    if (!configured) log_ddh_error(&error, "DDH frame topic filter failed");
    if (configured) configured = dh_subscribe_options_add_filter(options, filter, &error);
    if (!configured) log_ddh_error(&error, "DDH subscription filter failed");

    dh_filter_destroy(filter);

    if (configured) {
        dh_subscribe_options_set_enable_data_updates(options, true);
        dh_subscribe_options_set_enable_instance_updates(options, true);
        dh_subscribe_options_set_start_from(options, DH_START_FROM_NOW);
        configured = dh_subscriber_subscribe(ddh_subscriber, options, &error);
        if (!configured) log_ddh_error(&error, "DDH subscription failed");
    }
    dh_subscribe_options_destroy(options);
    return configured ? 1 : 0;
}

static void destroy_connection(void) {
    if (ddh_subscriber) {
        LOG("%s: Destroying Device Data Hub subscriber\n", __func__);
        dh_subscriber_destroy(ddh_subscriber);
        ddh_subscriber = NULL;
        LOG("%s: Device Data Hub subscriber destroyed\n", __func__);
    }
    if (ddh_client) {
        if (dh_client_get_connection_state(ddh_client) == DH_CONN_CONNECTED) {
            DHError *error = NULL;
            LOG("%s: Disconnecting Device Data Hub client\n", __func__);
            if (!dh_client_disconnect(ddh_client, &error))
                log_ddh_error(&error, "DDH disconnect failed");
            LOG("%s: Device Data Hub client disconnected\n", __func__);
        }
        LOG("%s: Destroying Device Data Hub client\n", __func__);
        dh_client_destroy(ddh_client);
        ddh_client = NULL;
        LOG("%s: Device Data Hub client destroyed\n", __func__);
    }
    g_atomic_int_set(&ddh_connected, 0);
}

static int initialize_connection(void) {
    DHError *error = NULL;
    ddh_client = dh_client_create("DataQ DDH client", &error);
    if (!ddh_client) {
        log_ddh_error(&error, "DDH client creation failed");
        return 0;
    }

    if (!dh_client_set_connection_update_callback(ddh_client, on_connection_update, NULL, &error))
        log_ddh_error(&error, "DDH connection callback registration failed");
    if (!dh_client_set_logging(ddh_client, DH_LOG_WARNING, DH_LOG_TARGET_SYSLOG, &error))
        log_ddh_error(&error, "DDH logging setup failed");
    if (!dh_client_connect(ddh_client, &error)) {
        log_ddh_error(&error, "DDH connection failed");
        destroy_connection();
        return 0;
    }
    g_atomic_int_set(&ddh_connected, 1);
    log_topic_inventory();

    if (!setup_subscription()) {
        destroy_connection();
        return 0;
    }
    return 1;
}

static gboolean reconnect_client(gpointer user_data) {
    (void)user_data;
    if (g_atomic_int_get(&ddh_shutting_down)) {
        reconnect_timer_id = 0;
        return G_SOURCE_REMOVE;
    }
    if (g_atomic_int_get(&ddh_connected)) {
        reconnect_timer_id = 0;
        return G_SOURCE_REMOVE;
    }

    LOG("%s: Reconnecting to Device Data Hub\n", __func__);
    g_atomic_int_set(&reconnect_in_progress, 1);
    destroy_connection();
    int connected = initialize_connection();
    g_atomic_int_set(&reconnect_in_progress, 0);
    if (connected) {
        LOG("%s: Device Data Hub subscription restored\n", __func__);
        reconnect_timer_id = 0;
        return G_SOURCE_REMOVE;
    }
    return G_SOURCE_CONTINUE;
}

int VOD_Init(int channel, vod_callback_t callback, void *user_data, int predictions) {
    (void)predictions;
    if (ddh_client) return 0;

    object_callback = callback;
    object_callback_data = user_data;
    ddh_channel_id = channel + 1;
    g_atomic_int_set(&ddh_shutting_down, 0);
    object_cache = g_hash_table_new_full(g_str_hash, g_str_equal, free, free);
    observed_labels = g_hash_table_new_full(g_str_hash, g_str_equal, free, NULL);
    advertised_labels = g_hash_table_new_full(g_str_hash, g_str_equal, free, NULL);
    advertised_properties = g_hash_table_new_full(g_str_hash, g_str_equal, free, NULL);
    topic_definitions = cJSON_CreateObject();
    if (!object_cache || !observed_labels || !advertised_labels ||
        !advertised_properties || !topic_definitions) {
        VOD_Shutdown();
        return -1;
    }

    if (!initialize_connection()) {
        VOD_Shutdown();
        return -1;
    }

    replay_timer_id = g_timeout_add_seconds(1, replay_active_objects, NULL);
    LOG("%s: Subscribed to %s and %s for channel_id %d\n",
        __func__, DDH_FRAME_TOPIC, DDH_TOPIC, ddh_channel_id);
    return 0;
}

cJSON *VOD_Detector_Information(void) {
    cJSON *information = cJSON_CreateObject();
    if (!information) return NULL;
    cJSON_AddStringToObject(information, "transport", "device-data-hub");
    cJSON_AddStringToObject(information, "topic", DDH_FRAME_TOPIC);
    cJSON_AddStringToObject(information, "summary_topic", DDH_TOPIC);
    cJSON_AddNumberToObject(information, "channel_id", ddh_channel_id);
    cJSON_AddBoolToObject(information, "connected", g_atomic_int_get(&ddh_connected));
    cJSON_AddNumberToObject(information, "active_objects", g_atomic_int_get(&active_object_count));
    cJSON_AddItemToObject(information, "advertised_labels", string_set_to_json(advertised_labels));
    cJSON_AddItemToObject(information, "advertised_properties", string_set_to_json(advertised_properties));
    if (topic_definitions)
        cJSON_AddItemToObject(information, "topic_definitions", cJSON_Duplicate(topic_definitions, 1));
    g_mutex_lock(&metrics_mutex);
    const char *classification_status = real_sample_count > 0
        ? "receiving-classified"
        : ignored_sample_count > 0 ? "unclassified-only" : "waiting";
    cJSON_AddStringToObject(information, "classification_status", classification_status);
    cJSON_AddNumberToObject(information, "real_samples", (double)real_sample_count);
    cJSON_AddNumberToObject(information, "frame_samples", (double)frame_sample_count);
    cJSON_AddNumberToObject(information, "track_summaries", (double)track_summary_count);
    cJSON_AddNumberToObject(information, "ended_objects", (double)ended_object_count);
    cJSON_AddNumberToObject(information, "stale_objects", (double)stale_object_count);
    cJSON_AddNumberToObject(information, "malformed_samples", (double)malformed_sample_count);
    cJSON_AddNumberToObject(information, "ignored_samples", (double)ignored_sample_count);
    cJSON_AddNumberToObject(information, "synthetic_batches", (double)synthetic_batch_count);
    cJSON_AddNumberToObject(information, "cache_evictions", (double)cache_eviction_count);
    cJSON_AddNumberToObject(information, "last_real_sample", (double)last_real_sample_ms);
    g_mutex_unlock(&metrics_mutex);
    return information;
}

cJSON *VOD_Label_List(void) {
    cJSON *list = cJSON_CreateArray();
    if (!list) return NULL;
    GHashTable *label_sets[] = { advertised_labels, observed_labels };
    GHashTable *added = g_hash_table_new_full(g_str_hash, g_str_equal, free, NULL);
    for (size_t index = 0; index < sizeof(label_sets) / sizeof(label_sets[0]); ++index) {
        GHashTable *set = label_sets[index];
        if (!set) continue;
        GHashTableIter iterator;
        gpointer label_name = NULL;
        g_hash_table_iter_init(&iterator, set);
        while (g_hash_table_iter_next(&iterator, &label_name, NULL)) {
            if (g_hash_table_contains(added, label_name)) continue;
            cJSON *label = cJSON_CreateObject();
            if (!label) continue;
            cJSON_AddStringToObject(label, "id", label_name);
            cJSON_AddItemToArray(list, label);
            g_hash_table_add(added, g_strdup(label_name));
        }
    }
    g_hash_table_destroy(added);
    return list;
}

void VOD_Reset(void) {
    g_atomic_int_inc(&stream_generation);
    if (object_cache) g_hash_table_remove_all(object_cache);
    g_atomic_int_set(&active_object_count, 0);
    if (object_callback)
        object_callback(NULL, 0, VOD_BATCH_RESET, object_callback_data);
}

void VOD_Shutdown(void) {
    LOG("%s: Shutting down Device Data Hub adapter\n", __func__);
    g_atomic_int_set(&ddh_shutting_down, 1);
    g_atomic_int_set(&ddh_connected, 0);
    if (replay_timer_id) {
        g_source_remove(replay_timer_id);
        replay_timer_id = 0;
    }
    if (reconnect_timer_id) {
        g_source_remove(reconnect_timer_id);
        reconnect_timer_id = 0;
    }
    destroy_connection();
    if (object_cache) {
        g_hash_table_destroy(object_cache);
        object_cache = NULL;
    }
    if (observed_labels) {
        g_hash_table_destroy(observed_labels);
        observed_labels = NULL;
    }
    if (advertised_labels) {
        g_hash_table_destroy(advertised_labels);
        advertised_labels = NULL;
    }
    if (advertised_properties) {
        g_hash_table_destroy(advertised_properties);
        advertised_properties = NULL;
    }
    if (topic_definitions) {
        cJSON_Delete(topic_definitions);
        topic_definitions = NULL;
    }
    g_atomic_int_set(&active_object_count, 0);
    object_callback = NULL;
    object_callback_data = NULL;
    LOG("%s: Device Data Hub adapter shut down\n", __func__);
}