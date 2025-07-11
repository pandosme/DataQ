#include "VOD.h"
#include "video_object_detection_subscriber.h"
#include "video_object_detection.pb-c.h"
#include <syslog.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <stdbool.h>
#include <glib.h>

#define LOG(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args);}
#define LOG_WARN(fmt, args...)    { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args);}
#define LOG_TRACE(fmt, args...)    {}

typedef struct {
    uint32_t type_id;
    char type_name[64];
    uint32_t class_id;
    char class_name[64];
    uint32_t score;
} vod_tracked_attribute_t;

typedef struct {
    float confidence;
    int type;
    char class_name[64];
    int x, y, w, h;
    bool active;
    vod_tracked_attribute_t *attributes;
    size_t num_attributes;
} tracked_object_t;

typedef struct object_map_entry {
    uint32_t id;
    tracked_object_t obj;
    struct object_map_entry *next;
} object_map_entry_t;

typedef struct {
    vod_callback_t cb;
    void *user_data;
    VOD__DetectorInformation *det_info;
    video_object_detection_subscriber_class_t **det_classes;
    int num_classes;
    object_map_entry_t *object_map;
} vod_internal_ctx_t;

cJSON* VOD_labels = 0;

static vod_internal_ctx_t g_ctx = {0};
static video_object_detection_subscriber_t *subscriber = NULL;

// --- Helper to free a tracked object ---
static void free_tracked_object(tracked_object_t *obj) {
    if (obj->attributes) free(obj->attributes);
}

// --- Helper to clear the entire object map ---
static void clear_object_map(void) {
    object_map_entry_t *entry = g_ctx.object_map;
    while (entry) {
        object_map_entry_t *next = entry->next;
        free_tracked_object(&entry->obj);
        free(entry);
        entry = next;
    }
    g_ctx.object_map = NULL;
}

// --- Get or create a tracked object by id ---
static tracked_object_t *get_or_create_tracked(uint32_t id) {
    object_map_entry_t *entry = g_ctx.object_map;
    while (entry) {
        if (entry->id == id) return &entry->obj;
        entry = entry->next;
    }
    entry = calloc(1, sizeof(*entry));
    if (!entry) {
        LOG_WARN("Memory allocation failed for tracked object id=%u", id);
        return NULL;
    }
    entry->id = id;
    entry->obj.active = true;
    entry->next = g_ctx.object_map;
    g_ctx.object_map = entry;
    return &entry->obj;
}

// --- Remove all inactive objects from the cache ---
static void remove_inactive_objects_from_cache(void) {
    object_map_entry_t **pentry = &g_ctx.object_map;
    while (*pentry) {
        if (!(*pentry)->obj.active) {
            object_map_entry_t *to_remove = *pentry;
            *pentry = to_remove->next;
            free_tracked_object(&to_remove->obj);
            free(to_remove);
        } else {
            pentry = &(*pentry)->next;
        }
    }
}

// --- Attribute/class utilities ---
static const char *get_class_name(int class_id) {
    for (int i = 0; i < g_ctx.num_classes; ++i) {
        if (video_object_detection_subscriber_det_class_id(g_ctx.det_classes[i]) == class_id)
            return video_object_detection_subscriber_det_class_name(g_ctx.det_classes[i]);
    }
    return "Unknown";
}

static VOD__AttributeType *find_attr_type(uint32_t type_id) {
    for (size_t i = 0; i < g_ctx.det_info->n_attribute_types; ++i) {
        if (g_ctx.det_info->attribute_types[i]->id == type_id)
            return g_ctx.det_info->attribute_types[i];
    }
    return NULL;
}

static const char *get_attr_type_name(uint32_t type_id) {
    VOD__AttributeType *type = find_attr_type(type_id);
    return (type && type->name) ? type->name : "UnknownType";
}

static const char *get_attr_class_name(uint32_t type_id, uint32_t class_id) {
    VOD__AttributeType *type = find_attr_type(type_id);
    if (!type) return "UnknownClass";
    for (size_t i = 0; i < type->n_classes; ++i) {
        if (type->classes[i]->id == class_id)
            return type->classes[i]->name ? type->classes[i]->name : "UnknownClass";
    }
    return "UnknownClass";
}

// --- Bounding box transformation ---
static void transform_bbox(float left, float top, float right, float bottom, int *x, int *y, int *w, int *h) {
    int x1 = (int)(left * 4096) + 4096;
    int y1 = 4096 - (int)(top * 4096);
    int x2 = (int)(right * 4096) + 4096;
    int y2 = 4096 - (int)(bottom * 4096);
    *x = (x1 * 1000) / 8192;
    *y = (y1 * 1000) / 8192;
    *w = ((x2 - x1) * 1000) / 8192;
    *h = ((y2 - y1) * 1000) / 8192;
}

// --- Main detection callback ---
static void detection_callback(const uint8_t *data, size_t size, void *user_data) {
    VOD__Scene *scene = vod__scene__unpack(NULL, size, data);
    LOG_TRACE("<");

    if (!scene) {
        LOG_WARN("%s: Failed to unpack detection protobuf data",__func__);
        LOG_TRACE(">\n");
        return;
    }

    // 1. Handle DeleteOperation events: just mark as inactive
    for (size_t e = 0; e < scene->n_events; ++e) {
        VOD__Event *ev = scene->events[e];
        if (ev->action == VOD__EVENT_ACTION__EVENT_DELETE) {
            tracked_object_t *track = get_or_create_tracked((uint32_t)ev->object_id);
            if (track) {
                track->active = false;
            } else {
                LOG_WARN("Delete event for unknown object id=%d", ev->object_id);
            }
        }
    }

    // 2. Process detections (update/create objects, mark as active)
    for (size_t i = 0; i < scene->n_detections; ++i) {
        VOD__Detection *det = scene->detections[i];
        tracked_object_t *track = get_or_create_tracked(det->id);
        if (!track) continue;

        // Update max confidence and type
        if (det->score > track->confidence) {
            track->confidence = det->score;
            track->type = det->det_class;
            strncpy(track->class_name, get_class_name(det->det_class), sizeof(track->class_name)-1);
            track->class_name[sizeof(track->class_name) - 1] = '\0';
            if (strlen(track->class_name) < 1)
                LOG_WARN("%s: Invalid track->class_name",__func__);
        }

        // Transform bbox
        transform_bbox(det->left, det->top, det->right, det->bottom, &track->x, &track->y, &track->w, &track->h);

        // Track attributes: for each type, keep max score
        for (size_t a = 0; a < det->n_attributes; ++a) {
            VOD__Attribute *attr = det->attributes[a];
            if (attr->has_score_case != VOD__ATTRIBUTE__HAS_SCORE_SCORE) continue;

            // Find if attribute type is already tracked
            size_t idx = 0;
            while (idx < track->num_attributes && track->attributes[idx].type_id != attr->type) idx++;

            const char *type_name = get_attr_type_name(attr->type);
            const char *class_name = get_attr_class_name(attr->type, attr->attr_class);

            if (idx == track->num_attributes) {
                // New attribute type
                track->attributes = realloc(track->attributes, (track->num_attributes + 1) * sizeof(vod_tracked_attribute_t));
                track->attributes[idx].type_id = attr->type;
                strncpy(track->attributes[idx].type_name, type_name, sizeof(track->attributes[idx].type_name)-1);
                track->attributes[idx].type_name[sizeof(track->attributes[idx].type_name) - 1] = '\0';
                if (strlen(track->attributes[idx].type_name) < 1)
                    LOG_WARN("%s: Invalid track->attributes[idx].type_name",__func__);

                track->attributes[idx].class_id = attr->attr_class;
                strncpy(track->attributes[idx].class_name, class_name, sizeof(track->attributes[idx].class_name)-1);
                track->attributes[idx].class_name[sizeof(track->attributes[idx].class_name) - 1] = '\0';
                if (strlen(track->attributes[idx].class_name) < 1)
                    LOG_WARN("%s: track->attributes[idx].class_name",__func__);
                track->attributes[idx].score = attr->score;
                track->num_attributes++;
            } else if (attr->score > track->attributes[idx].score) {
                // Update only if score increases
                track->attributes[idx].class_id = attr->attr_class;
                strncpy(track->attributes[idx].class_name, class_name, sizeof(track->attributes[idx].class_name)-1);
                track->attributes[idx].class_name[sizeof(track->attributes[idx].class_name) - 1] = '\0';
                if (strlen(track->attributes[idx].class_name) < 1)
                    LOG_WARN("%s: track->attributes[idx].class_name",__func__);
                track->attributes[idx].score = attr->score;
            }
        }
        // Mark as active on detection
        track->active = true;
    }

    // 3. Prepare ALL objects in cache for callback
    //    Count objects
    size_t count = 0;
    for (object_map_entry_t *entry = g_ctx.object_map; entry; entry = entry->next)
        ++count;

    vod_object_t *out_objs = calloc(count, sizeof(vod_object_t));
    size_t out_count = 0;

    for (object_map_entry_t *entry = g_ctx.object_map; entry; entry = entry->next) {
        tracked_object_t *track = &entry->obj;
        vod_object_t *obj = &out_objs[out_count++];
        snprintf(obj->id, sizeof(obj->id), "%u", entry->id);
        obj->confidence = track->confidence;
        obj->type = track->type;
        strncpy(obj->class_name, track->class_name, sizeof(obj->class_name)-1);
        obj->class_name[sizeof(obj->class_name) - 1] = '\0';
        if (strlen(obj->class_name) < 1) {
			sprintf(obj->class_name,"Invlaid");
            LOG_WARN("%s: obj->class_name",__func__);
		}

        obj->x = track->x; obj->y = track->y; obj->w = track->w; obj->h = track->h;
        obj->active = track->active;
        obj->num_attributes = track->num_attributes;
        if (obj->num_attributes > 0) {
            obj->attributes = calloc(obj->num_attributes, sizeof(vod_attribute_t));
            for (size_t j = 0; j < obj->num_attributes; ++j) {
                obj->attributes[j].name = strdup(track->attributes[j].type_name);
                obj->attributes[j].value = strdup(track->attributes[j].class_name);
            }
        }
    }

    // 4. Send all objects to callback
    if (out_count > 0)
        g_ctx.cb(out_objs, out_count, g_ctx.user_data);

    // 5. Free output allocation
    for (size_t i = 0; i < out_count; ++i) {
        for (size_t j = 0; j < out_objs[i].num_attributes; ++j) {
            free(out_objs[i].attributes[j].name);
            free(out_objs[i].attributes[j].value);
        }
        free(out_objs[i].attributes);
    }
    free(out_objs);

    // 6. Remove all inactive objects from cache
    remove_inactive_objects_from_cache();

    vod__scene__free_unpacked(scene, NULL);
    LOG_TRACE(">\n");
}

// --- Optional: Periodic debug function ---
gboolean
VOD_Debug_timer(gpointer user_data) {
    int count = 0;
    for (object_map_entry_t *entry = g_ctx.object_map; entry; entry = entry->next)
        ++count;
    LOG("VOD Cache: %d\n", count);
    return G_SOURCE_CONTINUE; // Return TRUE to keep the timer running
}

cJSON* VOD_Labels_List() {
	return VOD_labels;
}

// --- Initialization and shutdown ---
int VOD_Init(int channel, vod_callback_t cb, void *user_data) {
    LOG_TRACE("%s: Entry\n",__func__);
    if (!cb) {
        LOG_WARN("%s: Callback function is NULL", __func__);
        return -1;
    }
    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.cb = cb;
    g_ctx.user_data = user_data;

    g_ctx.num_classes = video_object_detection_subscriber_det_classes_get(&g_ctx.det_classes);
    if (g_ctx.num_classes <= 0) {
        LOG_WARN("%s: No detection classes found", __func__);
        return -2;
    }
	int i = 0;
	VOD_labels = cJSON_CreateArray();
	for( i = 0; i < g_ctx.num_classes; i++)
		cJSON_AddItemToArray(VOD_labels, cJSON_CreateString(video_object_detection_subscriber_det_class_name(g_ctx.det_classes[i])));

    uint8_t *buffer = NULL;
    size_t size = 0;
    if (video_object_detection_subscriber_get_detector_information(&buffer, &size) != 0) {
        LOG_WARN("%s: Failed to get detector information", __func__);
        return -3;
    }
    g_ctx.det_info = vod__detector_information__unpack(NULL, size, buffer);
    free(buffer);
    if (!g_ctx.det_info) {
        LOG_WARN("%s: Failed to unpack detector information", __func__);
        return -4;
    }

    if (video_object_detection_subscriber_create(&subscriber, NULL, channel) != 0) {
        LOG_WARN("%s: Failed to create subscriber", __func__);
        return -5;
    }
    if (video_object_detection_subscriber_set_get_detection_callback(subscriber, detection_callback) != 0) {
        LOG_WARN("%s: Failed to set detection callback", __func__);
        return -6;
    }
    if (video_object_detection_subscriber_subscribe(subscriber) != 0) {
        LOG_WARN("%s: Failed to subscribe to detection service", __func__);
        return -7;
    }
    LOG_TRACE("%s: Successful on channel %d", __func__, channel);
//    g_timeout_add_seconds(60, VOD_Debug_timer, NULL);

    return 0;
}

void VOD_Shutdown(void) {
    if (subscriber) {
        video_object_detection_subscriber_unsubscribe(subscriber);
        video_object_detection_subscriber_delete(&subscriber);
        subscriber = NULL;
    }
    if (g_ctx.det_classes)
        video_object_detection_subscriber_det_classes_free(g_ctx.det_classes, g_ctx.num_classes);
    if (g_ctx.det_info)
        vod__detector_information__free_unpacked(g_ctx.det_info, NULL);
    clear_object_map();
    closelog();
    LOG("VOD_Shutdown completed\n");
}
