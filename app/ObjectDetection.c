/**
 * ObjectDetection.c
 * Updated for detection state management, occupancy, visualization, perspective, and rotation
 * Fred Juhlin 2025
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <syslog.h>
#include <assert.h>
#include <glib.h>
#include <time.h>
#include "ObjectDetection.h"
#include "cJSON.h"
#include "ACAP.h"
#include "VOD.h"

#define LOG(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args);}
#define LOG_WARN(fmt, args...)    { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args);}
#define LOG_TRACE(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
//#define LOG_TRACE(fmt, args...)    {}

#define IDLE_THRESHOLD_PCT 50

typedef struct {
    char id[32];
    char class_name[64];
    float confidence;
    int x, y, w, h;
    int bx, by; // Birth position (center)
    int birthTime; // ms epoch
    int lastTime;  // ms epoch
    int prev_cx, prev_cy;
    float age;     // seconds
    bool valid;
    bool idle;
    float idle_duration; // seconds
    int idle_start_time; // ms epoch
    GHashTable *attributes; // key: name, value: strdup(val)
    bool active;
} detection_cache_entry_t;

static GHashTable *detectionCache = NULL;

static int config_tracker_confidence = 1;
static int config_min_confidence = 50;
static int config_cog = 0;
static int config_rotation = 0;
static int config_max_idle = 0;
static int config_min_height = 10;
static int config_max_height = 800;
static int config_min_width = 10;
static int config_max_width = 800;
static int config_x1 = 0;
static int config_x2 = 1000;
static int config_y1 = 0;
static int config_y2 = 1000;
static int config_hanging_objects = 0;
static cJSON* config_blacklist = 0;

static ObjectDetection_Callback detectionsCallback = 0;
static ObjectDetection_Callback trackerCallback = 0;

static int get_epoch_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (int)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static float calc_distance(int x1, int y1, int x2, int y2) {
    int dx = x2 - x1;
    int dy = y2 - y1;
    return sqrtf((float)(dx * dx + dy * dy));
}

static int ObjectDetection_Blacklisted(const char* label) {
    if (!config_blacklist || !label)
        return 0;
    cJSON* item = config_blacklist->child;
    while (item) {
        if (strcmp(label, item->valuestring) == 0)
            return 1;
        item = item->next;
    }
    return 0;
}

static void rotate_bbox(int x, int y, int w, int h, int *rx, int *ry, int *rw, int *rh, int rotation) {
    // All coordinates in [0,1000]
    switch (rotation) {
        case 0:
            *rx = x; *ry = y; *rw = w; *rh = h;
            break;
        case 90:
            *rx = y;
            *ry = 1000 - (x + w);
            *rw = h;
            *rh = w;
            break;
        case 180:
            *rx = 1000 - (x + w);
            *ry = 1000 - (y + h);
            *rw = w;
            *rh = h;
            break;
        case 270:
            *rx = 1000 - (y + h);
            *ry = x;
            *rw = h;
            *rh = w;
            break;
        default:
            *rx = x; *ry = y; *rw = w; *rh = h;
            break;
    }
}

void ObjectDetection_Config(cJSON* data) {
    LOG_TRACE("%s: Entry\n", __func__);
    if (!data) {
        LOG_WARN("%s: Invalid input\n", __func__);
        return;
    }
    config_min_confidence = cJSON_GetObjectItem(data, "confidence") ? cJSON_GetObjectItem(data, "confidence")->valueint : 40;
    config_cog = cJSON_GetObjectItem(data, "cog") ? cJSON_GetObjectItem(data, "cog")->valueint : 0;
    config_rotation = cJSON_GetObjectItem(data, "rotation") ? cJSON_GetObjectItem(data, "rotation")->valueint : 0;
    config_tracker_confidence = cJSON_GetObjectItem(data, "tracker_confidence") ? cJSON_GetObjectItem(data, "tracker_confidence")->valueint : 1;
    config_max_idle = cJSON_GetObjectItem(data, "maxIdle") ? cJSON_GetObjectItem(data, "maxIdle")->valueint : 0;
    config_blacklist = cJSON_GetObjectItem(data, "ignoreClass") ? cJSON_GetObjectItem(data, "ignoreClass") : cJSON_CreateArray();
    config_min_height = cJSON_GetObjectItem(data, "minHeight") ? cJSON_GetObjectItem(data, "minHeight")->valueint : 10;
    config_max_height = cJSON_GetObjectItem(data, "maxHeight") ? cJSON_GetObjectItem(data, "maxHeight")->valueint : 800;
    config_min_width = cJSON_GetObjectItem(data, "minWidth") ? cJSON_GetObjectItem(data, "minWidth")->valueint : 10;
    config_max_width = cJSON_GetObjectItem(data, "maxWidth") ? cJSON_GetObjectItem(data, "maxWidth")->valueint : 800;
    config_hanging_objects = 0;
    cJSON *aoi = cJSON_GetObjectItem(data, "aoi");
    if (aoi) {
        config_x1 = cJSON_GetObjectItem(aoi, "x1") ? cJSON_GetObjectItem(aoi, "x1")->valueint : 0;
        config_x2 = cJSON_GetObjectItem(aoi, "x2") ? cJSON_GetObjectItem(aoi, "x2")->valueint : 1000;
        config_y1 = cJSON_GetObjectItem(aoi, "y1") ? cJSON_GetObjectItem(aoi, "y1")->valueint : 0;
        config_y2 = cJSON_GetObjectItem(aoi, "y2") ? cJSON_GetObjectItem(aoi, "y2")->valueint : 1000;
    }
    LOG_TRACE("%s: Exit\n", __func__);
}

void ObjectDetection_Reset() {
    if (detectionCache) {
        g_hash_table_destroy(detectionCache);
        detectionCache = NULL;
    }
    detectionCache = g_hash_table_new_full(g_str_hash, g_str_equal, free, free);
}

static void free_attributes(GHashTable *attrs) {
    if (!attrs) return;
    GHashTableIter iter;
    gpointer key, value;
    g_hash_table_iter_init(&iter, attrs);
    while (g_hash_table_iter_next(&iter, &key, &value)) {
        free(key);
        free(value);
    }
    g_hash_table_destroy(attrs);
}

static void free_detection_cache_entry(gpointer data) {
    detection_cache_entry_t *entry = (detection_cache_entry_t*)data;
    if (entry->attributes) free_attributes(entry->attributes);
    free(entry);
}

// Defensive, robust implementation of publish_detections
static void publish_detections(GHashTable *cache, ObjectDetection_Callback cb) {
    if (!cb || !cache) return;

    cJSON *arr = cJSON_CreateArray();
    if (!arr) return;

    GHashTableIter iter;
    gpointer key, value;
    g_hash_table_iter_init(&iter, cache);

    while (g_hash_table_iter_next(&iter, &key, &value)) {
        detection_cache_entry_t *entry = (detection_cache_entry_t*)value;
        if (!entry) continue;

        // Defensive: check critical fields
        if (!entry->class_name || !entry->id) continue;

        cJSON *obj = cJSON_CreateObject();
        if (!obj) continue;

//LOG("%s %s %f %d",entry->class_name,entry->id,entry->age, entry->active);
/*
        cJSON_AddStringToObject(obj, "class", entry->class_name);
        cJSON_AddNumberToObject(obj, "confidence", entry->confidence);
        cJSON_AddNumberToObject(obj, "age", entry->age);
        cJSON_AddNumberToObject(obj, "x", entry->x);
        cJSON_AddNumberToObject(obj, "y", entry->y);
        cJSON_AddNumberToObject(obj, "w", entry->w);
        cJSON_AddNumberToObject(obj, "h", entry->h);
        cJSON_AddNumberToObject(obj, "cx", entry->prev_cx);
        cJSON_AddNumberToObject(obj, "cy", entry->prev_cy);
        cJSON_AddNumberToObject(obj, "bx", entry->bx);
        cJSON_AddNumberToObject(obj, "by", entry->by);
        cJSON_AddNumberToObject(obj, "timestamp", entry->lastTime);
        cJSON_AddNumberToObject(obj, "birthTime", entry->birthTime);
        cJSON_AddStringToObject(obj, "id", entry->id);
        cJSON_AddBoolToObject(obj, "active", entry->active);
*/
        // Add attributes, with null checks
        if (entry->attributes) {
            GHashTableIter aiter;
            gpointer ak, av;
            g_hash_table_iter_init(&aiter, entry->attributes);
            while (g_hash_table_iter_next(&aiter, &ak, &av)) {
//LOG("%s %s",(char*)ak,(char*)av);
//                if (ak && av) 
//                    cJSON_AddStringToObject(obj, (char*)ak, (char*)av);
            }
        }
//        cJSON_AddItemToArray(arr, obj);
    }

    cb(arr);
    cJSON_Delete(arr);
}

static void VOD_Data(const vod_object_t *objects, size_t num_objects, void *user_data) {
    if (!detectionCache) {
        detectionCache = g_hash_table_new_full(g_str_hash, g_str_equal, free, free_detection_cache_entry);
        if (!detectionCache) {
            LOG_WARN("Failed to allocate detectionCache");
            return;
        }
    }
    int now = get_epoch_ms();
    GHashTableIter iter;
    gpointer key, value;

    // Process current detections
    for (size_t i = 0; i < num_objects; ++i) {
        const vod_object_t *obj = &objects[i];
        if (!obj || !obj->class_name) continue;

        // Apply rotation to bbox
        int rx, ry, rw, rh;
        rotate_bbox(obj->x, obj->y, obj->w, obj->h, &rx, &ry, &rw, &rh, config_rotation);

        // Calculate cx, cy based on config_cog
        int cx, cy;
        if (config_cog == 0) {
            cx = rx + rw / 2;
            cy = ry + rh / 2;
        } else {
            cx = rx + rw / 2;
            cy = ry + rh;
        }

        // Validity criteria
        bool valid = true;
        if (obj->confidence < config_min_confidence) valid = false;
        if (cx < config_x1 || cx > config_x2) valid = false;
        if (cy < config_y1 || cy > config_y2) valid = false;
        if (ObjectDetection_Blacklisted(obj->class_name)) valid = false;
        if (rw < config_min_width || rh < config_min_height) valid = false;
        detection_cache_entry_t *entry = (detection_cache_entry_t*)g_hash_table_lookup(detectionCache, obj->id);
        if (!entry) {
            entry = calloc(1, sizeof(detection_cache_entry_t));
            if (!entry) {
                LOG_WARN("Failed to allocate detection_cache_entry_t");
                continue;
            }
            if (!obj->id) {
                free(entry);
                continue;
            }
            strncpy(entry->id, obj->id, sizeof(entry->id)-1);
            entry->id[sizeof(entry->id) - 1] = '\0';
            if (strlen(entry->id) < 1) LOG_WARN("%s: entry->id", __func__);
            strncpy(entry->class_name, obj->class_name, sizeof(entry->class_name)-1);
            entry->class_name[sizeof(entry->class_name) - 1] = '\0';
            if (strlen(entry->class_name) < 1) LOG_WARN("%s: entry->class_name", __func__);
            entry->confidence = obj->confidence;
            entry->x = rx;
            entry->y = ry;
            entry->w = rw;
            entry->h = rh;
            entry->bx = cx;
            entry->by = cy;
            entry->birthTime = now;
            entry->lastTime = now;
            entry->prev_cx = cx;
            entry->prev_cy = cy;
            entry->age = 0.0f;
            entry->valid = valid;
            entry->idle = false;
            entry->idle_duration = 0.0f;
            entry->idle_start_time = now;
            entry->attributes = g_hash_table_new_full(g_str_hash, g_str_equal, free, free);
            entry->active = obj->active;
            // Defensive attribute copy
            if (obj->num_attributes > 0 && obj->attributes) {
                for (size_t a = 0; a < obj->num_attributes; ++a) {
                    if (!obj->attributes[a].name || !obj->attributes[a].value) continue;
                    char *aname = strdup(obj->attributes[a].name);
                    char *aval = strdup(obj->attributes[a].value);
                    if (aname && aval) {
                        g_hash_table_insert(entry->attributes, aname, aval);
                    } else {
                        free(aname);
                        free(aval);
                    }
                }
            }
            char *keycopy = strdup(entry->id);
            if (!keycopy) {
                free_detection_cache_entry(entry);
                continue;
            }
            g_hash_table_insert(detectionCache, keycopy, entry);
        } else {
            // Update existing entry
            float dist = calc_distance(entry->prev_cx, entry->prev_cy, cx, cy);
            if (dist < IDLE_THRESHOLD_PCT) {
                if (!entry->idle) {
                    entry->idle = true;
                    entry->idle_start_time = now;
                }
                entry->idle_duration = (now - entry->idle_start_time) / 1000.0f;
                if (config_max_idle > 0 && entry->idle_duration > config_max_idle) {
                    entry->idle = true;
                }
            } else {
                entry->idle = false;
                entry->idle_duration = 0.0f;
                entry->idle_start_time = now;
                entry->prev_cx = cx;
                entry->prev_cy = cy;
            }
			entry->confidence = obj->confidence;
            entry->x = rx;
            entry->y = ry;
            entry->w = rw;
            entry->h = rh;
            entry->age = (now - entry->birthTime) / 1000.0f;
            entry->lastTime = now;
            entry->valid = valid;
            // Replace Attributes
            if (entry->attributes) g_hash_table_destroy(entry->attributes);
            entry->attributes = g_hash_table_new_full(g_str_hash, g_str_equal, free, free);
            if (obj->num_attributes > 0 && obj->attributes) {
                for (size_t a = 0; a < obj->num_attributes; ++a) {
                    if (!obj->attributes[a].name || !obj->attributes[a].value) continue;
                    char *aname = strdup(obj->attributes[a].name);
                    char *aval = strdup(obj->attributes[a].value);
                    if (aname && aval) {
                        g_hash_table_insert(entry->attributes, aname, aval);
                    } else {
                        free(aname);
                        free(aval);
                    }
                }
            }
            entry->active = obj->active;
        }
    }

    publish_detections(detectionCache, detectionsCallback);
    // Remove all objects with active == false after publishing
    g_hash_table_iter_init(&iter, detectionCache);
    GList *remove_list = NULL;
    while (g_hash_table_iter_next(&iter, &key, &value)) {
        detection_cache_entry_t *entry = (detection_cache_entry_t*)value;
        if (!entry->active) {
            remove_list = g_list_prepend(remove_list, key);
        }
    }
    for (GList *l = remove_list; l != NULL; l = l->next) {
if( remove_list ) LOG("<");
        g_hash_table_remove(detectionCache, l->data);
if( remove_list ) LOG(">");
    }
if( remove_list ) LOG("[");
    g_list_free(remove_list);
if( remove_list ) LOG("]");
}

int ObjectDetection_Init(ObjectDetection_Callback detections, ObjectDetection_Callback tracker) {
	LOG_TRACE("%s: Entry\n",__func__);
    detectionsCallback = detections;
    trackerCallback = tracker;
    if (!detectionCache)
        detectionCache = g_hash_table_new_full(g_str_hash, g_str_equal, free, free_detection_cache_entry);
    if (VOD_Init(0, VOD_Data, NULL) != 0) {
        LOG_WARN("%s: Object detection service failed\n", __func__);
		LOG_TRACE("%s: Exit\n",__func__);
        return 0;
    }
	LOG_TRACE("%s: Exit\n",__func__);
    return 1;
}
