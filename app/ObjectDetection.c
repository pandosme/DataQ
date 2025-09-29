/**
 * ObjectDetection.c
 * Thread-safe version for detection state management, occupancy, visualization, perspective, and rotation
 * Fred Juhlin 2025, updated for thread safety
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

#define LOG(fmt, args...) { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_WARN(fmt, args...) { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args); }
//#define LOG_TRACE(fmt, args...) { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_TRACE(fmt, args...) {}
//#define LOG_DEEP(fmt, args...) { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_DEEP(fmt, args...) {}

#define IDLE_THRESHOLD_PCT 50
#define DIRECTION_CHANGE_THRESHOLD_RAD (M_PI / 4) // 45 degrees 

typedef struct {
    char name[64];
    char value[64];
} od_attribute_t;

typedef struct {
    const char *key;
    const char *value;
} NameMapEntry;

static const NameMapEntry name_map[] = {
    { "car", "Car" },
    { "truck", "Truck" },
    { "motorcycle_bicycle", "Bike" },
    { "bus", "Bus" },
    { "truck","Truck" },
    { "vehicle_other","Other" },
    { "vehicle","Vehicle" },	
    { "license_plate", "LicensePlate" },
    { "human", "Human" },
    { "human_face", "Head" },
    { "human_head", "Head" },
    { "head", "Head" },
    { "face", "Head" },	
    { "hat", "Hat" },
    { "animal", "Animal" },	
    { "no_hat", "" },
    { "hat_other", "Hat" },
    { "hard_hat", "Helmet" },
    { "bag", "Bag" },
    { "bag_other", "Bag" },
    { "backpack", "Bag" },
    { "bag", "suitcase" },
    { "red", "Red" },
    { "blue", "Blue" },
    { "black", "Black" },
    { "white", "White" },
    { "green", "Green" },
    { "beige", "Beige" },
    { "yellow", "Yellow" },
    { "gray", "Gray" },
    { "unknown", "" },
};

#define NAME_MAP_SIZE (sizeof(name_map) / sizeof(name_map[0]))

typedef struct {
    char id[32];
    char class_name[64];
    int confidence;
    int x, y, w, h;
    int cx, cy;
    int bx, by;
    int dx, dy;
    double timestamp; // ms epoch
    double birthTime; // ms epoch
    int prev_cx, prev_cy;
	double prev_angle;
	int directions;
    int distance;
    double age;     // seconds
    bool valid;
    bool idle;
    bool sleep;
    bool trackerSleep;	
	double speed;
	double maxSpeed;	
    double idle_duration; // seconds
    double max_idle_duration; // seconds
    double idle_start_time; // ms epoch
    double last_published_tracker;  //The timestamp the tracker was published
    od_attribute_t *attributes; // Dynamically allocated array
    size_t num_attributes;      // Number of attributes
    bool active;
} detection_cache_entry_t;

// ---- THREAD SAFETY ----
static GMutex detection_mutex;

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
static TrackerDetection_Callback trackerCallback = 0;

static double get_epoch_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (double)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
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
    if (entry->attributes) free(entry->attributes);
    free(entry);
}

const char* NiceName(const char* input) {
    if (!input) return NULL;
    for (size_t i = 0; i < NAME_MAP_SIZE; ++i) {
        if (strcmp(input, name_map[i].key) == 0) {
            return name_map[i].value;
        }
    }
    return input;
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
    g_mutex_lock(&detection_mutex);
    LOG_TRACE("%s: Entry\n", __func__);
    if (!data) {
        LOG_WARN("%s: Invalid input\n", __func__);
        g_mutex_unlock(&detection_mutex);
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
/*	
    cJSON *significantMovement = cJSON_GetObjectItem(data, "significantMovement");
    if (significantMovement) {
		config_significan_upperArea = cJSON_GetObjectItem(significantMovement, "upperArea") ? cJSON_GetObjectItem(significantMovement, "upperArea")->valueint : 30;
		config_significan_lowerArea = cJSON_GetObjectItem(significantMovement, "lowerArea") ? cJSON_GetObjectItem(significantMovement, "lowerArea")->valueint : 70;
		config_significan_line = cJSON_GetObjectItem(significantMovement, "line") ? cJSON_GetObjectItem(significantMovement, "line")->valueint : 400;
	}
*/	
    LOG_TRACE("%s: Exit\n", __func__);
    g_mutex_unlock(&detection_mutex);
	ObjectDetection_Reset();
}


static void publish_tracker(detection_cache_entry_t *entry, int timer ) {
    cJSON *obj = cJSON_CreateObject();
    if (!obj || !entry || !entry->id) return;
    if( entry->active == true ) {
        if (!entry->valid) { cJSON_Delete(obj); return; }
        if (entry->trackerSleep) { cJSON_Delete(obj); return; }
    }
    const char* label = NiceName(entry->class_name);
    if (!label) { cJSON_Delete(obj); return; }
    if (ObjectDetection_Blacklisted(label)) { cJSON_Delete(obj); return; }

    cJSON_AddStringToObject(obj, "class", label);
    cJSON_AddNumberToObject(obj, "confidence", entry->confidence);
    cJSON_AddNumberToObject(obj, "age", entry->age);
    cJSON_AddNumberToObject(obj, "distance", entry->distance/10);
    cJSON_AddNumberToObject(obj, "directions", entry->directions);
    cJSON_AddNumberToObject(obj, "x", entry->x);
    cJSON_AddNumberToObject(obj, "y", entry->y);
    cJSON_AddNumberToObject(obj, "w", entry->w);
    cJSON_AddNumberToObject(obj, "h", entry->h);
    cJSON_AddNumberToObject(obj, "cx", entry->cx);
    cJSON_AddNumberToObject(obj, "cy", entry->cy);
    cJSON_AddNumberToObject(obj, "dx", entry->dx);
    cJSON_AddNumberToObject(obj, "dy", entry->dy);
    cJSON_AddNumberToObject(obj, "bx", entry->bx);
    cJSON_AddNumberToObject(obj, "by", entry->by);
    cJSON_AddNumberToObject(obj, "timestamp", entry->timestamp);
    cJSON_AddNumberToObject(obj, "birth", entry->birthTime);
    cJSON_AddNumberToObject(obj, "idle", entry->idle_duration);	
    cJSON_AddNumberToObject(obj, "maxIdle", entry->max_idle_duration);	
	cJSON_AddNumberToObject(obj, "speed", entry->speed);
	cJSON_AddNumberToObject(obj, "maxSpeed", entry->maxSpeed);
    cJSON_AddStringToObject(obj, "id", entry->id);
    if( entry->sleep && !entry->trackerSleep) {
        cJSON_AddBoolToObject(obj, "active", 0);
        entry->trackerSleep = 1;
    } else {
        cJSON_AddBoolToObject(obj, "active", entry->active);
    }
    for (size_t a = 0; a < entry->num_attributes; ++a) {
        if (strlen(entry->attributes[a].name) && strlen(entry->attributes[a].value)) {
            const char* aKey = entry->attributes[a].name;
            const char* aValue = entry->attributes[a].value;
            if( strcmp("vehicle_type",aKey) == 0 ) {
                cJSON_ReplaceItemInObject(obj,"class",cJSON_CreateString(NiceName(aValue)));
                aKey = 0; aValue = 0;
            }
            if( aKey && strcmp("vehicle_color",aKey) == 0 ) {
                cJSON_AddStringToObject(obj,"color",NiceName(aValue));
                aKey = 0; aValue = 0;
            }
            if( aKey && strcmp("clothing_upper_color",aKey) == 0) {
                cJSON_AddStringToObject(obj,"color",NiceName(aValue));
                aKey = 0; aValue = 0;
            }
            if( aKey && strcmp("clothing_lower_color",aKey) == 0 ) {
                cJSON_AddStringToObject(obj,"color2",NiceName(aValue));
                aKey = 0; aValue = 0;
            }
            if( aKey && strcmp("hat_type",aKey) == 0 ) {
                if( strcmp("no_hat", aValue ) != 0 )
                    cJSON_AddStringToObject(obj,"hat",NiceName(aValue));
                aKey = 0; aValue = 0;
            }
            if( aKey && strcmp("bag_type",aKey) == 0) {
                cJSON_AddStringToObject(obj,"bag",NiceName(aValue));
                aKey = 0; aValue = 0;
            }
            if( aKey && strcmp("human_face_visibility",aKey) == 0) {
                if( strcmp( aValue,"face_non_visible") == 0 )
                    cJSON_AddFalseToObject(obj,"face");
                else
                    cJSON_AddTrueToObject(obj,"face");
                aKey = 0; aValue = 0;
            }
            if( aKey && strcmp("human_pose",aKey) == 0 ) {
                if( strcmp( aValue,"down") == 0 )
                    cJSON_AddTrueToObject(obj,"isOnGround");
                else
                    cJSON_AddFalseToObject(obj,"isOnGround");
                aKey = 0; aValue = 0;
            }
            if( aKey && aValue )
                cJSON_AddStringToObject(obj, entry->attributes[a].name, entry->attributes[a].value);
        }
    }
    entry->last_published_tracker = get_epoch_ms();
    cJSON* payload = cJSON_Duplicate(obj,1);
    if( trackerCallback )
        trackerCallback(payload, timer);
    cJSON_Delete(obj);
}

static void publish_detections(GHashTable *cache) {
    cJSON *arr = cJSON_CreateArray();
    if (!arr) return;
    GHashTableIter iter;
    gpointer key, value;
    g_hash_table_iter_init(&iter, cache);
    while (g_hash_table_iter_next(&iter, &key, &value)) {
        detection_cache_entry_t *entry = (detection_cache_entry_t*)value;
        cJSON *obj = cJSON_CreateObject();
        if (!obj || !entry || !entry->id) continue;
        if (!entry->valid ) { cJSON_Delete(obj); continue; }
        if( entry->active == true && entry->sleep) { cJSON_Delete(obj); continue; }
        const char* label = NiceName(entry->class_name);
        if (!label) { cJSON_Delete(obj); continue; }
        if (ObjectDetection_Blacklisted(label)) { cJSON_Delete(obj); continue; }
        cJSON_AddStringToObject(obj, "class", label);
        cJSON_AddNumberToObject(obj, "confidence", entry->confidence);
        cJSON_AddNumberToObject(obj, "age", entry->age);
        cJSON_AddNumberToObject(obj, "distance", entry->distance/10);
        cJSON_AddNumberToObject(obj, "x", entry->x);
        cJSON_AddNumberToObject(obj, "y", entry->y);
        cJSON_AddNumberToObject(obj, "w", entry->w);
        cJSON_AddNumberToObject(obj, "h", entry->h);
        cJSON_AddNumberToObject(obj, "cx", entry->cx);
        cJSON_AddNumberToObject(obj, "cy", entry->cy);
        cJSON_AddNumberToObject(obj, "dx", entry->dx);
        cJSON_AddNumberToObject(obj, "dy", entry->dy);
        cJSON_AddNumberToObject(obj, "timestamp", entry->timestamp);
        cJSON_AddNumberToObject(obj, "idle", entry->idle_duration);
        cJSON_AddStringToObject(obj, "id", entry->id);
        if( config_max_idle && entry->idle_duration > config_max_idle && entry->active ) {
            cJSON_AddBoolToObject(obj, "active", 0);
            entry->sleep = true;
        } else {			
            cJSON_AddBoolToObject(obj, "active", entry->active);
        }
        if( entry->active == false ) {
            publish_tracker(entry, 0);
        }
        for (size_t a = 0; a < entry->num_attributes; ++a) {
            if (strlen(entry->attributes[a].name) && strlen(entry->attributes[a].value)) {
                const char* aKey = entry->attributes[a].name;
                const char* aValue = entry->attributes[a].value;
                if( strcmp("vehicle_type",aKey) == 0 ) {
                    cJSON_ReplaceItemInObject(obj,"class",cJSON_CreateString(NiceName(aValue)));
                    aKey = 0; aValue = 0;
                }
                if( aKey && strcmp("vehicle_color",aKey) == 0) {
                    cJSON_AddStringToObject(obj,"color",NiceName(aValue));
                    aKey = 0; aValue = 0;
                }
                if( aKey && strcmp("clothing_upper_color",aKey) == 0) {
                    cJSON_AddStringToObject(obj,"color",NiceName(aValue));
                    aKey = 0; aValue = 0;
                }
                if( aKey && strcmp("clothing_lower_color",aKey) == 0) {
                    cJSON_AddStringToObject(obj,"color2",NiceName(aValue));
                    aKey = 0; aValue = 0;
                }
                if( aKey && strcmp("hat_type",aKey) == 0) {
                    if( strcmp("no_hat", aValue ) != 0 )
                        cJSON_AddStringToObject(obj,"type",NiceName(aValue));
                    aKey = 0; aValue = 0;
                }
                if( aKey && strcmp("bag_type",aKey) == 0) {
                    cJSON_AddStringToObject(obj,"type",NiceName(aValue));
                    aKey = 0; aValue = 0;
                }
                if( aKey && strcmp("human_face_visibility",aKey) == 0 ) {
                    if( strcmp( aValue,"face_non_visible") == 0 )
                        cJSON_AddFalseToObject(obj,"face");
                    else
                        cJSON_AddTrueToObject(obj,"face");
                    aKey = 0; aValue = 0;
                }
                if( aKey && strcmp("human_pose",aKey) == 0 ) {
                    if( strcmp( aValue,"down") == 0 )
                        cJSON_AddTrueToObject(obj,"isOnGround");
                    else
                        cJSON_AddFalseToObject(obj,"isOnGround");
                    aKey = 0; aValue = 0;
                }
                if( aKey && aValue )
                    cJSON_AddStringToObject(obj, entry->attributes[a].name, entry->attributes[a].value);
            } else {
                LOG_WARN("%s: Invalid attribute or value\n", __func__);
            }
        }
        cJSON_AddItemToArray(arr, obj);
    }
    cJSON* payload = cJSON_Duplicate(arr,1);
    cJSON_Delete(arr);
    if( detectionsCallback )
        detectionsCallback(payload);
}

static od_attribute_t *clone_attributes(const vod_attribute_t *src, size_t num, size_t *out_num) {
    if (!src || num == 0) {
        if (out_num) *out_num = 0;
        return NULL;
    }
    od_attribute_t *dst = calloc(num, sizeof(od_attribute_t));
    if (!dst) {
        if (out_num) *out_num = 0;
        return NULL;
    }
    for (size_t i = 0, j = 0; i < num; ++i) {
        if (!src[i].name || !src[i].value) continue;
        if (strlen(src[i].name) == 0 || strlen(src[i].value) == 0) continue;
        strncpy(dst[j].name, src[i].name, sizeof(dst[j].name) - 1);
        dst[j].name[sizeof(dst[j].name) - 1] = '\0';
        strncpy(dst[j].value, src[i].value, sizeof(dst[j].value) - 1);
        dst[j].value[sizeof(dst[j].value) - 1] = '\0';
        ++j;
        if (out_num) *out_num = j;
    }
    return dst;
}

void
Adjust_For_VehicleType(detection_cache_entry_t *entry) {
    if (!entry || !entry->attributes || entry->num_attributes == 0)
        return;

    for (size_t i = 0; i < entry->num_attributes; ++i) {
        if (strcmp(entry->attributes[i].name, "vehicle_type") == 0) {
            // Set class_name to value, making sure to fit the buffer
            strncpy(entry->class_name, entry->attributes[i].value, sizeof(entry->class_name) - 1);
            entry->class_name[sizeof(entry->class_name) - 1] = '\0';

            // Remove the attribute by shifting the others
            for (size_t j = i; j + 1 < entry->num_attributes; ++j) {
                entry->attributes[j] = entry->attributes[j + 1];
            }
            entry->num_attributes--;
            // Optionally zero out the last slot
            if (entry->num_attributes > 0) {
                entry->attributes[entry->num_attributes].name[0] = '\0';
                entry->attributes[entry->num_attributes].value[0] = '\0';
            }
            break; // Remove only the first "vehicle_type"
        }
    }
}

void distinct_direction_change(detection_cache_entry_t *entry, int cx, int cy) {
    float dx = cx - entry->prev_cx;
    float dy = cy - entry->prev_cy;
    float norm = sqrtf(dx * dx + dy * dy);
    if (norm < 1e-3) {
        // No significant movement, do nothing
        return;
    }
    double cur_angle = atan2(dy, dx); // returns in radians, full [-pi, pi]

    if (isnan(entry->prev_angle)) {
        // First entry—just set angle but don't increment
        entry->prev_angle = cur_angle;
        return;
    }
    double delta = fabs(cur_angle - entry->prev_angle);
    // Normalize to [0,pi]
    if (delta > M_PI)
        delta = 2 * M_PI - delta;
    if (delta > DIRECTION_CHANGE_THRESHOLD_RAD) {
        entry->directions += 1;
    }
    entry->prev_angle = cur_angle;
}

static void VOD_Data(const vod_object_t *objects, size_t num_objects, void *user_data) {
    g_mutex_lock(&detection_mutex);
	LOG_DEEP("<ObjectDection:%s",__func__);
    if (!detectionCache) {
        detectionCache = g_hash_table_new_full(g_str_hash, g_str_equal, free, free_detection_cache_entry);
        if (!detectionCache) {
            LOG_WARN("Failed to allocate detectionCache");
            g_mutex_unlock(&detection_mutex);
			LOG_DEEP("Error %s:ObjectDetection>", __func__);
            return;
        }
    }
    double now = get_epoch_ms();
    GHashTableIter iter;
    gpointer key, value;
    for (size_t i = 0; i < num_objects; ++i) {
        const vod_object_t *obj = &objects[i];
        if (!obj || !obj->class_name) continue;
        int rx, ry, rw, rh;
        rotate_bbox(obj->x, obj->y, obj->w, obj->h, &rx, &ry, &rw, &rh, config_rotation);
        int cx, cy;
        if (config_cog == 0) {
            cx = rx + rw / 2;
            cy = ry + rh / 2;
        } else {
            cx = rx + rw / 2;
            cy = ry + rh;
        }
        bool valid = true;
        if (obj->confidence < config_min_confidence) valid = false;
        if (cx < config_x1 || cx > config_x2) valid = false;
        if (cy < config_y1 || cy > config_y2) valid = false;
        if (rw < 5 || rh < 5) valid = false;
        if (ObjectDetection_Blacklisted(obj->class_name)) valid = false;
        if (rw < config_min_width || rh < config_min_height) valid = false;
        if (rw > config_max_width || rh > config_max_height) valid = false;
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
            strncpy(entry->class_name, obj->class_name, sizeof(entry->class_name)-1);
            entry->class_name[sizeof(entry->class_name) - 1] = '\0';
            entry->confidence = obj->confidence;
            entry->x = rx;
            entry->y = ry;
            entry->w = rw;
            entry->h = rh;
            entry->cx = cx;
            entry->cy = cy;
            entry->dx = 0;
            entry->dy = 0;
            entry->bx = cx;
            entry->by = cy;
            entry->birthTime = now;
            entry->prev_cx = cx;
            entry->prev_cy = cy;
			entry->prev_angle = NAN; // or -1000.0, but NAN is preferred for clarity
			entry->directions = 0;			
            entry->age = 0.0f;
            entry->max_idle_duration = 0;
            entry->valid = valid;
            entry->idle = false;
			entry->speed = 0;
			entry->maxSpeed = 0;
			entry->sleep = false;
            entry->trackerSleep = false;
            entry->idle_duration = 0.0f;
            entry->idle_start_time = now;
            entry->attributes = clone_attributes(obj->attributes, obj->num_attributes, &entry->num_attributes);
            entry->active = obj->active;
            char *keycopy = strdup(entry->id);
            if (!keycopy) {
                free_detection_cache_entry(entry);
                continue;
            }
			Adjust_For_VehicleType( entry);
			if( valid )
				publish_tracker( entry, 0 );
            g_hash_table_insert(detectionCache, keycopy, entry);
			
        } else {
            float dist = calc_distance(entry->prev_cx, entry->prev_cy, cx, cy);
            if (dist < IDLE_THRESHOLD_PCT) {
                if (!entry->idle) {
                    entry->idle = true;
                    entry->idle_start_time = now;
                }
                entry->idle_duration = (now - entry->idle_start_time) / 1000.0f;
                if( entry->idle_duration > entry->max_idle_duration )
                    entry->max_idle_duration = floor((entry->idle_duration*10)+0.5) / 10.0;
            } else {
				distinct_direction_change(entry, cx, cy);
                if( entry->sleep ) {
                    entry->sleep = false;
                    entry->bx = cx;
                    entry->by = cy;
                    entry->dx = 0;
                    entry->dy = 0;
                    entry->birthTime = now;
                    entry->prev_cx = cx;
                    entry->prev_cy = cy;
                    entry->age = 0.0f;
                    entry->max_idle_duration = 0;
                    entry->distance = 0;
                    dist = 0;
                }
                entry->trackerSleep = false;
                entry->distance += dist;
				if( entry->age > 1)
					entry->speed = floor( (entry->distance/entry->age) + 0.5);
				if( entry->speed > entry->maxSpeed )
					entry->maxSpeed = entry->speed;
                entry->idle = false;
                publish_tracker( entry, 0 );
                entry->idle_duration = 0.0f;
                entry->idle_start_time = now;
                entry->prev_cx = cx;
                entry->prev_cy = cy;
            }
            entry->confidence = obj->confidence;
            entry->timestamp = now;
            entry->x = rx;
            entry->y = ry;
            entry->w = rw;
            entry->h = rh;
            entry->cx = cx;
            entry->cy = cy;
            entry->dx = cx - entry->bx;
            entry->dy = cy - entry->by;
            entry->age = round(10.0 * ((now - entry->birthTime) / 1000.0)) / 10.0;
            if( entry->valid == false && valid == true)
                entry->valid = true;
            if (entry->attributes) {
                free(entry->attributes);
                entry->attributes = NULL;
                entry->num_attributes = 0;
            }
            entry->attributes = clone_attributes(obj->attributes, obj->num_attributes, &entry->num_attributes);
            entry->active = obj->active;
			Adjust_For_VehicleType(entry);
        }
    }
    publish_detections(detectionCache);
    g_hash_table_iter_init(&iter, detectionCache);
    GList *remove_list = NULL;
    while (g_hash_table_iter_next(&iter, &key, &value)) {
        detection_cache_entry_t *entry = (detection_cache_entry_t*)value;
        if (!entry->active) {
            remove_list = g_list_prepend(remove_list, key);
        }
    }
    for (GList *l = remove_list; l != NULL; l = l->next) {
        g_hash_table_remove(detectionCache, l->data);
    }
    g_list_free(remove_list);
	LOG_DEEP("%s:ObjectDetection>",__func__);
    g_mutex_unlock(&detection_mutex);
}

void ObjectDetection_Reset() {
    g_mutex_lock(&detection_mutex);
    LOG_DEEP("%s: Object cache reset",__func__);

    if (detectionCache) {
        // Temporary hash table to hold valid objects for publishing
        GHashTable *validCache = g_hash_table_new_full(g_str_hash, g_str_equal, NULL, NULL);

        GHashTableIter iter;
        gpointer key, value;
        g_hash_table_iter_init(&iter, detectionCache);
        while (g_hash_table_iter_next(&iter, &key, &value)) {
            detection_cache_entry_t *entry = (detection_cache_entry_t*)value;
            if (entry->valid) {
                // Set active to false before publishing
                entry->active = false;
                // Insert into validCache for publishing
                g_hash_table_insert(validCache, key, entry);
            }
        }

        // Publish trackers for valid objects
        GHashTableIter validIter;
        gpointer vkey, vvalue;
        g_hash_table_iter_init(&validIter, validCache);
        while (g_hash_table_iter_next(&validIter, &vkey, &vvalue)) {
            detection_cache_entry_t *entry = (detection_cache_entry_t*)vvalue;
            publish_tracker(entry ,0);
        }

        // Publish detections for valid objects
        publish_detections(validCache);

        // Destroy the old cache
        g_hash_table_destroy(detectionCache);
        detectionCache = NULL;

        // Destroy the temporary validCache (does not free keys/values)
        g_hash_table_destroy(validCache);
    }

    // Create a new empty cache
    detectionCache = g_hash_table_new_full(g_str_hash, g_str_equal, free, free_detection_cache_entry);

    g_mutex_unlock(&detection_mutex);
}

gboolean update_trackers(gpointer user_data) {
    g_mutex_lock(&detection_mutex);
    if( !detectionCache ) {
        g_mutex_unlock(&detection_mutex);
        return TRUE;
    }
    double now = get_epoch_ms();
    GHashTableIter iter;
    gpointer key, value;
    g_hash_table_iter_init(&iter, detectionCache);
    while (g_hash_table_iter_next(&iter, &key, &value)) {
        detection_cache_entry_t *entry = (detection_cache_entry_t*)value;
        if( entry->active == true && now - entry->last_published_tracker > 1500 )
            publish_tracker(entry, 1);
    }
    g_mutex_unlock(&detection_mutex);
    return TRUE; 
}

int ObjectDetection_Init(ObjectDetection_Callback detections, TrackerDetection_Callback tracker) {
    g_mutex_lock(&detection_mutex);
    LOG_TRACE("%s: Entry\n",__func__);
    detectionsCallback = detections;
    trackerCallback = tracker;
    if (!detectionCache)
        detectionCache = g_hash_table_new_full(g_str_hash, g_str_equal, free, free_detection_cache_entry);
	int allow_predictions = 0;
	cJSON* settings = ACAP_Get_Config("settings");
	cJSON* scene = settings?cJSON_GetObjectItem(settings,"scene"):0;
	cJSON* tracker_confidence = scene?cJSON_GetObjectItem(scene,"tracker_confidence"):0;
	if( tracker_confidence && tracker_confidence->type==cJSON_False)
		allow_predictions = 1;

    if (VOD_Init(0, VOD_Data, NULL, allow_predictions) != 0) {
        LOG_WARN("%s: Object detection service failed\n", __func__);
        LOG_TRACE("%s: Exit\n",__func__);
        g_mutex_unlock(&detection_mutex);
        return 0;
    }

    cJSON* list = VOD_Label_List();
    cJSON* labels = cJSON_CreateArray();
    cJSON* item = list->child;
    while(item) {
        const char* label = NiceName(item->valuestring);
        if( label )
            cJSON_AddItemToArray(labels,cJSON_CreateString(label));
        item = item->next;
    }


	cJSON_Delete(list);
    ACAP_STATUS_SetObject("detections", "labels", labels);
    g_timeout_add_seconds(1, update_trackers, NULL);	
    LOG_TRACE("%s: Exit\n",__func__);
    g_mutex_unlock(&detection_mutex);

    return 1;
}