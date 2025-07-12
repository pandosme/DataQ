#include "VOD.h"
#include "video_object_detection_subscriber.h"
#include "video_object_detection.pb-c.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <glib.h>
#include <pthread.h>
#include <syslog.h>
#include "cJSON.h"

#define LOG(fmt, args...) { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_WARN(fmt, args...) { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args); }
//#define LOG_TRACE(fmt, args...) { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_TRACE(fmt, args...) {}
//#define LOG_DEEP(fmt, args...) { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_DEEP(fmt, args...) {}


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
    int missing_frames;
    bool detected_this_frame;
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
static pthread_mutex_t vod_mutex = PTHREAD_MUTEX_INITIALIZER;

// --- Helper to free a tracked object ---
static void free_tracked_object(tracked_object_t *obj) {
    if (obj->attributes) {
        free(obj->attributes);
        obj->attributes = NULL;
        obj->num_attributes = 0;
    }
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
    strncpy(entry->obj.class_name, "Unknown", sizeof(entry->obj.class_name) - 1);
    entry->obj.class_name[sizeof(entry->obj.class_name) - 1] = '\0';
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
        if (video_object_detection_subscriber_det_class_id(g_ctx.det_classes[i]) == class_id) {
            const char *name = video_object_detection_subscriber_det_class_name(g_ctx.det_classes[i]);
            if (name && strlen(name) > 0) return name;
        }
    }
	LOG_WARN("%s: Invalid class id %d\n", __func__,class_id);
    return "Unknown";
}

static VOD__AttributeType *find_attr_type(uint32_t type_id) {
    if (!g_ctx.det_info) return NULL;
    for (size_t i = 0; i < g_ctx.det_info->n_attribute_types; ++i) {
        if (g_ctx.det_info->attribute_types[i] && g_ctx.det_info->attribute_types[i]->id == type_id)
            return g_ctx.det_info->attribute_types[i];
    }
    return NULL;
}

static const char *get_attr_type_name(uint32_t type_id) {
    VOD__AttributeType *type = find_attr_type(type_id);
    if (type && type->name && strlen(type->name) > 0)
        return type->name;
    return "UnknownType";
}

static const char *get_attr_class_name(uint32_t type_id, uint32_t class_id) {
    VOD__AttributeType *type = find_attr_type(type_id);
    if (!type) return "UnknownClass";
    for (size_t i = 0; i < type->n_classes; ++i) {
        if (type->classes[i] && type->classes[i]->id == class_id) {
            if (type->classes[i]->name && strlen(type->classes[i]->name) > 0)
                return type->classes[i]->name;
            break;
        }
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
    pthread_mutex_lock(&vod_mutex);
	LOG_DEEP("<VOD");
    VOD__Scene *scene = vod__scene__unpack(NULL, size, data);
    if (!scene) {
        LOG_WARN("%s: Failed to unpack detection protobuf data", __func__);
        pthread_mutex_unlock(&vod_mutex);
	LOG_DEEP("ERROR VOD>");
        return;
    }

    // 1. Handle DeleteOperation events
    for (size_t e = 0; e < scene->n_events; ++e) {
        VOD__Event *ev = scene->events[e];
        if (!ev) continue;
        if (ev->action == VOD__EVENT_ACTION__EVENT_DELETE) {
            tracked_object_t *track = get_or_create_tracked((uint32_t)ev->object_id);
            if (track) {
                track->active = false;
                track->missing_frames = 0;
            } else {
                LOG_WARN("Delete event for unknown object id=%d", ev->object_id);
            }
        }
    }

    // 2. Mark all objects as not detected this frame
    for (object_map_entry_t *entry = g_ctx.object_map; entry; entry = entry->next) {
        entry->obj.detected_this_frame = false;
    }

    // 3. Process detections with validation
    for (size_t i = 0; i < scene->n_detections; ++i) {
        VOD__Detection *det = scene->detections[i];
        if (!det) continue;
        if (det->detection_status != VOD__DETECTION__DETECTION_STATUS__TRACKED_CONFIDENT) continue;

        bool valid_class = (det->det_class >= 0) && (det->det_class < g_ctx.num_classes);
        bool valid_score = (det->score > 0);
        if (!valid_class || !valid_score) {
            LOG_WARN("Skipping detection id=%u: invalid class_id=%d or score=%u", det->id, det->det_class, det->score);
            continue;
        }

        tracked_object_t *track = get_or_create_tracked(det->id);
        if (!track) continue;

        track->confidence = det->score;
        track->type = det->det_class;
        const char *cls_name = get_class_name(det->det_class);
        if (cls_name && strlen(cls_name) > 0) {
            strncpy(track->class_name, cls_name, sizeof(track->class_name) - 1);
            track->class_name[sizeof(track->class_name) - 1] = '\0';
        } else {
            strncpy(track->class_name, "Unknown", sizeof(track->class_name) - 1);
            track->class_name[sizeof(track->class_name) - 1] = '\0';
            LOG_WARN("%s: Invalid track->class_name", __func__);
        }

        transform_bbox(det->left, det->top, det->right, det->bottom, &track->x, &track->y, &track->w, &track->h);

        // Attributes: Only process those with valid score
        for (size_t a = 0; a < det->n_attributes; ++a) {
            VOD__Attribute *attr = det->attributes[a];
            if (!attr) continue;
            if (attr->has_score_case != VOD__ATTRIBUTE__HAS_SCORE_SCORE) continue;

            size_t idx = 0;
            while (idx < track->num_attributes && track->attributes[idx].type_id != attr->type) idx++;

            const char *type_name = get_attr_type_name(attr->type);
            const char *class_name = get_attr_class_name(attr->type, attr->attr_class);

            if (idx == track->num_attributes) {
                vod_tracked_attribute_t *new_attrs = realloc(track->attributes, (track->num_attributes + 1) * sizeof(vod_tracked_attribute_t));
                if (!new_attrs) {
                    LOG_WARN("%s: Memory allocation failed for attributes", __func__);
                    continue;
                }
                track->attributes = new_attrs;
                track->attributes[idx].type_id = attr->type;
                strncpy(track->attributes[idx].type_name, type_name, sizeof(track->attributes[idx].type_name) - 1);
                track->attributes[idx].type_name[sizeof(track->attributes[idx].type_name) - 1] = '\0';
                track->attributes[idx].class_id = attr->attr_class;
                strncpy(track->attributes[idx].class_name, class_name, sizeof(track->attributes[idx].class_name) - 1);
                track->attributes[idx].class_name[sizeof(track->attributes[idx].class_name) - 1] = '\0';
                track->attributes[idx].score = attr->score;
                track->num_attributes++;
            } else if (attr->score > track->attributes[idx].score) {
                track->attributes[idx].class_id = attr->attr_class;
                strncpy(track->attributes[idx].class_name, class_name, sizeof(track->attributes[idx].class_name) - 1);
                track->attributes[idx].class_name[sizeof(track->attributes[idx].class_name) - 1] = '\0';
                track->attributes[idx].score = attr->score;
            }
        }

        track->active = true;
        track->missing_frames = 0;
        track->detected_this_frame = true;
    }

    // 4. Increment missing_frames for objects not detected this frame, and mark inactive if needed
    for (object_map_entry_t *entry = g_ctx.object_map; entry; entry = entry->next) {
        tracked_object_t *track = &entry->obj;
        if (!track->detected_this_frame) {
//            track->missing_frames++;
            if (track->active && track->missing_frames >= 5) {
                track->active = false;
            }
        }
    }

    // 5. Prepare ALL objects in cache for callback (active or inactive)
    size_t count = 0;
    for (object_map_entry_t *entry = g_ctx.object_map; entry; entry = entry->next)
        count++;
    if (count > 0 && g_ctx.cb) {
        vod_object_t *out_objs = calloc(count, sizeof(vod_object_t));
        if (!out_objs) {
            LOG_WARN("%s: Memory allocation failed for output objects", __func__);
            vod__scene__free_unpacked(scene, NULL);
            pthread_mutex_unlock(&vod_mutex);
			LOG_DEEP("Error VOD>");
            return;
        }
        size_t out_count = 0;
        for (object_map_entry_t *entry = g_ctx.object_map; entry; entry = entry->next) {
            tracked_object_t *track = &entry->obj;
            vod_object_t *obj = &out_objs[out_count++];
            snprintf(obj->id, sizeof(obj->id), "%u", entry->id);
            obj->confidence = track->confidence;
            obj->type = track->type;
            if (track->class_name && strlen(track->class_name) > 0) {
                strncpy(obj->class_name, track->class_name, sizeof(obj->class_name) - 1);
                obj->class_name[sizeof(obj->class_name) - 1] = '\0';
            } else {
                snprintf(obj->class_name, sizeof(obj->class_name), "Invalid");
                LOG_WARN("%s: obj->class_name invalid for id %u", __func__, entry->id);
            }
            obj->x = track->x;
            obj->y = track->y;
            obj->w = track->w;
            obj->h = track->h;
            obj->active = track->active;
            obj->num_attributes = track->num_attributes;
            if (obj->num_attributes > 0) {
                obj->attributes = calloc(obj->num_attributes, sizeof(vod_attribute_t));
                if (!obj->attributes) {
                    LOG_WARN("%s: Memory allocation failed for attributes in output object id %u", __func__, entry->id);
                    obj->num_attributes = 0;
                } else {
                    for (size_t j = 0; j < obj->num_attributes; ++j) {
                        obj->attributes[j].name = strdup(track->attributes[j].type_name);
                        obj->attributes[j].value = strdup(track->attributes[j].class_name);
                        if (!obj->attributes[j].name || !obj->attributes[j].value) {
                            LOG_WARN("%s: strdup failed for attribute in output object id %u", __func__, entry->id);
                            free(obj->attributes[j].name);
                            free(obj->attributes[j].value);
                            obj->attributes[j].name = NULL;
                            obj->attributes[j].value = NULL;
                        }
                    }
                }
            } else {
                obj->attributes = NULL;
            }
        }
        g_ctx.cb(out_objs, out_count, g_ctx.user_data);
        for (size_t i = 0; i < out_count; ++i) {
            if (out_objs[i].attributes) {
                for (size_t j = 0; j < out_objs[i].num_attributes; ++j) {
                    free(out_objs[i].attributes[j].name);
                    free(out_objs[i].attributes[j].value);
                }
                free(out_objs[i].attributes);
            }
        }
        free(out_objs);
    }

    // 6. Remove all inactive objects from cache
    remove_inactive_objects_from_cache();

    vod__scene__free_unpacked(scene, NULL);
	LOG_DEEP("VOD>");
	
    pthread_mutex_unlock(&vod_mutex);
}

// --- Optional: Periodic debug function ---
gboolean VOD_Debug_timer(gpointer user_data) {
    pthread_mutex_lock(&vod_mutex);
    int count = 0;
    for (object_map_entry_t *entry = g_ctx.object_map; entry; entry = entry->next)
        count++;
    LOG_DEEP("VOD Cache: %d\n", count);
    pthread_mutex_unlock(&vod_mutex);
    return G_SOURCE_CONTINUE;
}

cJSON* VOD_Labels_List() {
    pthread_mutex_lock(&vod_mutex);
    cJSON* labels = VOD_labels;
    pthread_mutex_unlock(&vod_mutex);
    return labels;
}

// --- Initialization and shutdown ---
int VOD_Init(int channel, vod_callback_t cb, void *user_data) {
    pthread_mutex_lock(&vod_mutex);
    LOG_TRACE("%s: Entry\n", __func__);
    if (!cb) {
        LOG_WARN("%s: Callback function is NULL", __func__);
        pthread_mutex_unlock(&vod_mutex);
        return -1;
    }
    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.cb = cb;
    g_ctx.user_data = user_data;
	
	int major,minor;
	if( video_object_detection_subscriber_get_version(&major,&minor) != VIDEO_OBJECT_DETECTION_SUBSCRIBER_SUCCESS ) {
        LOG_WARN("%s: No version detected", __func__);
        pthread_mutex_unlock(&vod_mutex);
	}
	LOG("%s: Version = %d.%d\n",__func__,major,minor);
	
    g_ctx.num_classes = video_object_detection_subscriber_det_classes_get(&g_ctx.det_classes);
    if (g_ctx.num_classes <= 0) {
        LOG_WARN("%s: No detection classes found", __func__);
        pthread_mutex_unlock(&vod_mutex);
        return -2;
    }

    if (VOD_labels) {
        cJSON_Delete(VOD_labels);
        VOD_labels = NULL;
    }
    VOD_labels = cJSON_CreateArray();
	LOG("Detection classes:\n");
    for (int i = 0; i < g_ctx.num_classes; i++) {
        const char *name = video_object_detection_subscriber_det_class_name(g_ctx.det_classes[i]);
        if (name) {
			cJSON_AddItemToArray(VOD_labels, cJSON_CreateString(name));
			LOG("%d: %s\n",i,name);
		} else {
			LOG("%d: Label not set\n",i);
		}
    }

    uint8_t *buffer = NULL;
    size_t size = 0;
    if (video_object_detection_subscriber_get_detector_information(&buffer, &size) != 0) {
        LOG_WARN("%s: Failed to get detector information", __func__);
        pthread_mutex_unlock(&vod_mutex);
        return -3;
    }
    g_ctx.det_info = vod__detector_information__unpack(NULL, size, buffer);
    free(buffer);
    if (!g_ctx.det_info) {
        LOG_WARN("%s: Failed to unpack detector information", __func__);
        pthread_mutex_unlock(&vod_mutex);
        return -4;
    }


    if (video_object_detection_subscriber_create(&subscriber, NULL, channel) != 0) {
        LOG_WARN("%s: Failed to create subscriber", __func__);
        pthread_mutex_unlock(&vod_mutex);
        return -5;
    }

    if (video_object_detection_subscriber_set_get_detection_callback(subscriber, detection_callback) != 0) {
        LOG_WARN("%s: Failed to set detection callback", __func__);
        pthread_mutex_unlock(&vod_mutex);
        return -6;
    }

    if (video_object_detection_subscriber_set_receive_empty_hits(subscriber, detection_callback) != 0) {
        LOG_WARN("%s: Failed to set empty hits", __func__);
        pthread_mutex_unlock(&vod_mutex);
        return -7;
	}

	
    if (video_object_detection_subscriber_subscribe(subscriber) != 0) {
        LOG_WARN("%s: Failed to subscribe to detection service", __func__);
        pthread_mutex_unlock(&vod_mutex);
        return -8;
    }
    LOG("Object detection successsful on channel %d", channel);
    pthread_mutex_unlock(&vod_mutex);
    return 0;
}

void VOD_Shutdown(void) {
    pthread_mutex_lock(&vod_mutex);
    if (subscriber) {
        video_object_detection_subscriber_unsubscribe(subscriber);
        video_object_detection_subscriber_delete(&subscriber);
        subscriber = NULL;
    }
    if (g_ctx.det_classes) {
        video_object_detection_subscriber_det_classes_free(g_ctx.det_classes, g_ctx.num_classes);
        g_ctx.det_classes = NULL;
        g_ctx.num_classes = 0;
    }
    if (g_ctx.det_info) {
        vod__detector_information__free_unpacked(g_ctx.det_info, NULL);
        g_ctx.det_info = NULL;
    }
    clear_object_map();
    if (VOD_labels) {
        cJSON_Delete(VOD_labels);
        VOD_labels = NULL;
    }
    closelog();
    LOG("VOD_Shutdown completed\n");
    pthread_mutex_unlock(&vod_mutex);
}

void VOD_Reset(void) {
    pthread_mutex_lock(&vod_mutex);
    LOG("VOD cache reset\n");
    clear_object_map();
    if (g_ctx.det_classes) {
        video_object_detection_subscriber_det_classes_free(g_ctx.det_classes, g_ctx.num_classes);
        g_ctx.det_classes = NULL;
        g_ctx.num_classes = 0;
    }
    if (g_ctx.det_info) {
        vod__detector_information__free_unpacked(g_ctx.det_info, NULL);
        g_ctx.det_info = NULL;
    }
    if (VOD_labels) {
        cJSON_Delete(VOD_labels);
        VOD_labels = NULL;
    }
    LOG("VOD_Reset: All caches cleared\n");
    pthread_mutex_unlock(&vod_mutex);
}
