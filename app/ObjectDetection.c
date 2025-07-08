/*
 * ObjectDetection.c
 * Complete, robust, multi-firmware attribute support, 2025
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <syslog.h>
#include <glib.h>
#include "ObjectDetection.h"
#include "cJSON.h"
#include "ACAP.h"
#include "video_object_detection_subscriber.h"
#include "video_object_detection.pb-c.h"

#define LOG(fmt, args...)    { syslog(LOG_INFO, fmt, ## args);}
#define LOG_WARN(fmt, args...)    { syslog(LOG_WARNING, fmt, ## args);}
#define LOG_TRACE(fmt, args...)    {}

#define SIGNIFICANT_MOVE_THRESHOLD 50  // 5% of [0-1000] view

// --- Attribute value arrays (capitalized) ---
static const char* vehicle_color_vals[] = {"White", "Gray", "Black", "Red", "Blue", "Green", "Yellow", "Unknown"};
static const char* clothing_upper_vals[] = {"White", "Gray", "Black", "Red", "Blue", "Green", "Yellow", "Beige", "Unknown"};
static const char* clothing_lower_vals[] = {"White", "Gray", "Black", "Red", "Blue", "Green", "Beige", "Unknown"};
static const char* bag_type_vals[] = {"Bag", "Backpack", "Suitcase"};
static const char* hat_type_vals[] = {"", "Hat", "Helmet"};
static const char* human_face_visibility_vals[] = {"Face", "Head"};
static const char* vehicle_type_vals[] = {"Car", "Truck", "Bus", "Vehicle"};
static const char* human_pose_vals[] = {"Down", ""};

// --- Attribute definition structures ---
typedef struct {
    int type;
    const char* name;
    const char** values;
    int value_count;
} AttributeDef;

typedef struct {
    int version_id; // 1, 2, or 3
    const AttributeDef* attributes;
    int attribute_count;
} AttributeDefinitionSet;

// --- Attribute definitions for each firmware version ---
static const AttributeDef attr_defs_1[] = { // Firmware < 12.0.0
    {2, "vehicle_color", vehicle_color_vals, 8},
    {3, "clothing_upper_color", clothing_upper_vals, 9},
    {4, "clothing_lower_color", clothing_lower_vals, 8},
    {5, "bag_type", bag_type_vals, 3},
    {6, "hat_type", hat_type_vals, 3},
    {7, "human_face_visibility", human_face_visibility_vals, 2}
};
static const AttributeDef attr_defs_2[] = { // Firmware 12.0.0 - 12.4.99
    {0, "vehicle_type", vehicle_type_vals, 4},
    {1, "bag_type", bag_type_vals, 3},
    {2, "clothing_upper_color", clothing_upper_vals, 9},
    {3, "clothing_lower_color", clothing_lower_vals, 8},
    {5, "human_face_visibility", human_face_visibility_vals, 2},
    {6, "hat_type", hat_type_vals, 3},
    {7, "vehicle_color", vehicle_color_vals, 8}
};
static const AttributeDef attr_defs_3[] = { // Firmware 12.5.0+
    {2, "vehicle_color", vehicle_color_vals, 8},
    {3, "clothing_upper_color", clothing_upper_vals, 9},
    {4, "clothing_lower_color", clothing_lower_vals, 8},
    {5, "vehicle_type", vehicle_type_vals, 4},
    {6, "hat_type", hat_type_vals, 3},
    {7, "bag_type", bag_type_vals, 3},
    {8, "human_face_visibility", human_face_visibility_vals, 2},
    {9, "human_pose", human_pose_vals, 2}
};
static const AttributeDefinitionSet attribute_sets[] = {
    {1, attr_defs_1, sizeof(attr_defs_1)/sizeof(attr_defs_1[0])},
    {2, attr_defs_2, sizeof(attr_defs_2)/sizeof(attr_defs_2[0])},
    {3, attr_defs_3, sizeof(attr_defs_3)/sizeof(attr_defs_3[0])}
};
static const AttributeDefinitionSet* g_active_attr_set = NULL;

// --- Firmware version detection and attribute definition selection ---
static int attribute_definition() {
    cJSON* device = ACAP_Get_Config("device");
    if (!device) return 1;
    cJSON* fw = cJSON_GetObjectItem(device, "firmware");
    if (!fw || !cJSON_IsString(fw) || !fw->valuestring) return 1;
    int maj=0, min=0, pat=0;
    sscanf(fw->valuestring, "%d.%d.%d", &maj, &min, &pat);
    if (maj < 12) return 1;
    if (maj == 12 && min < 5) return 2;
    return 3;
}
void ObjectDetection_InitAttributeDefs() {
    int def = attribute_definition();
    for (int i = 0; i < sizeof(attribute_sets)/sizeof(attribute_sets[0]); ++i) {
        if (attribute_sets[i].version_id == def) {
            g_active_attr_set = &attribute_sets[i];
            break;
        }
    }
}

// --- Attribute value lookup ---
static const char* attribute_value_string(int type, int value) {
    if (!g_active_attr_set) return NULL;
    for (int i = 0; i < g_active_attr_set->attribute_count; ++i) {
        if (g_active_attr_set->attributes[i].type == type) {
            if (value >= 0 && value < g_active_attr_set->attributes[i].value_count)
                return g_active_attr_set->attributes[i].values[value];
        }
    }
    return NULL;
}

// --- Class name translation ---
static const char* names_map[][2] = {
    {"vehicle", "Vehicle"}, {"car", "Car"}, {"truck", "Truck"}, {"bus", "Bus"},
    {"human", "Human"}, {"human_head", "Head"}, {"motorcycle_bicycle", "Bike"},
    {"bag", "Bag"}, {"animal", "Animal"}, {"face", "Head"}, {"human_face", "Head"},
    {"license_plate", "LicensePlate"}
};
static const int names_map_len = sizeof(names_map)/sizeof(names_map[0]);
static const char* translate_class(const char* raw) {
    for (int i = 0; i < names_map_len; ++i)
        if (strcmp(names_map[i][0], raw) == 0)
            return names_map[i][1];
    return raw;
}

// --- Attribute tracking ---
typedef struct { int value; int score; } AttrVal;
static void attr_set(GHashTable* map, int type, int value, int score) {
    int* key = malloc(sizeof(int)); *key = type;
    AttrVal* av = g_hash_table_lookup(map, key);
    if (!av) {
        av = malloc(sizeof(AttrVal)); av->value = value; av->score = score;
        g_hash_table_insert(map, key, av);
    } else if (score > av->score) {
        av->value = value; av->score = score;
        free(key);
    } else {
        free(key);
    }
}
static AttrVal* attr_get(GHashTable* map, int type) {
    int key = type;
    return g_hash_table_lookup(map, &key);
}
static void attr_map_free(gpointer data) { free(data); }
static void attr_map_key_free(gpointer data) { free(data); }

// --- Tracked object structure ---
typedef struct {
    char id[32];
    int class_id;
    char class_raw[32];
    char class_name[32];
    int class_score;
    int confidence;
    int x, y, w, h, cx, cy;
    int bx, by;
    int dx, dy;
    double birth_ts;
    double last_ts;
    double age;
    double idle_start_ts;
    double idle_accum_ms;
    bool active;
    bool suppressed;
    bool was_idle;
    bool significantMovement;
    bool just_created;
    bool pending_removal;
    bool valid;
    float distance;
    int last_sig_cx, last_sig_cy;
    // Attribute tracking
    int color_type; int color_score; int color_value;
    int color2_type; int color2_score; int color2_value;
    int hat_type; int hat_score; int hat_value;
    int bag_type; int bag_score; int bag_value;
    int face_type; int face_score; int face_value;
    int vehicle_type_type; int vehicle_type_score; int vehicle_type_value;
    GHashTable* attr_map;
} TrackedObject;

// --- State ---
typedef struct {
    GHashTable* objects;
    int max_idle_ms;
    int min_confidence;
    int min_width, max_width, min_height, max_height;
    int x1, x2, y1, y2;
    int cog;
    int rotation;
    ObjectDetection_Callback user_cb;
    video_object_detection_subscriber_t* vod_handler;
    video_object_detection_subscriber_class_t** classifications;
} ObjectDetectionState;

static ObjectDetectionState g_state = {0};

/* --- Utility Functions --- */

static double now_ms() { return ACAP_DEVICE_Timestamp(); }
static int clamp(int v, int min, int max) { if (v < min) return min; if (v > max) return max; return v; }

/* --- Tracked Object Management --- */

static TrackedObject* tracked_object_new(const char* id, int class_id, const char* class_raw, int class_score, int cx, int cy, double ts) {
    TrackedObject* obj = calloc(1, sizeof(TrackedObject));
    strncpy(obj->id, id, sizeof(obj->id)-1);
    obj->class_id = class_id;
    strncpy(obj->class_raw, class_raw, sizeof(obj->class_raw)-1);
    strncpy(obj->class_name, translate_class(class_raw), sizeof(obj->class_name)-1);
    obj->class_score = class_score;
    obj->confidence = class_score;
    obj->bx = cx; obj->by = cy;           // Set birth position ONCE
    obj->birth_ts = ts;                   // Set birth time ONCE
    obj->last_ts = ts;
    obj->active = true;
    obj->significantMovement = true;
    obj->just_created = true;
    obj->pending_removal = false;
    obj->valid = false;
    obj->distance = 0.0f;                 // Set distance ONCE
    obj->last_sig_cx = cx; obj->last_sig_cy = cy; // Set last sig move ONCE
    obj->attr_map = g_hash_table_new_full(g_int_hash, g_int_equal, attr_map_key_free, attr_map_free);
    obj->color_score = -1; obj->color2_score = -1; obj->hat_score = -1; obj->bag_score = -1; obj->face_score = -1; obj->vehicle_type_score = -1;
    return obj;
}

static void tracked_object_free(TrackedObject* obj) {
    if (!obj) return;
    if (obj->attr_map) g_hash_table_destroy(obj->attr_map);
    free(obj);
}
static void tracked_object_reset_movement(TrackedObject* obj, double ts, int cx, int cy) {
    obj->last_sig_cx = cx;
    obj->last_sig_cy = cy;
    obj->dx = 0; obj->dy = 0;
    obj->significantMovement = true;
}

/* --- Idle and Movement Logic --- */

static bool is_significant_move(TrackedObject* obj, int new_cx, int new_cy) {
    int dx = abs(new_cx - obj->last_sig_cx), dy = abs(new_cy - obj->last_sig_cy);
    return (dx >= SIGNIFICANT_MOVE_THRESHOLD) || (dy >= SIGNIFICANT_MOVE_THRESHOLD);
}

static void update_idle_state(TrackedObject* obj, int new_cx, int new_cy, double ts) {
    bool moved = is_significant_move(obj, new_cx, new_cy);
    if (moved) {
        // --- DISTANCE ---
        int dx = new_cx - obj->last_sig_cx;
        int dy = new_cy - obj->last_sig_cy;
        float delta_distance = sqrtf(dx*dx + dy*dy) / 10.0f;
        obj->distance += delta_distance;
        obj->dx = dx;
        obj->dy = dy;
        obj->last_sig_cx = new_cx;
        obj->last_sig_cy = new_cy;

        obj->significantMovement = true;
        if (obj->was_idle || obj->suppressed) {
            obj->active = true;
            obj->suppressed = false;
            obj->was_idle = false;
        }
        obj->idle_start_ts = ts;
        obj->idle_accum_ms = 0;
    } else {
        obj->dx = new_cx - obj->last_sig_cx;
        obj->dy = new_cy - obj->last_sig_cy;
        if (!obj->was_idle) {
            obj->idle_start_ts = ts;
            obj->idle_accum_ms = 0;
            obj->was_idle = true;
        } else {
            obj->idle_accum_ms += (ts - obj->last_ts);
        }
        obj->significantMovement = false;
        if (g_state.max_idle_ms > 0 && obj->idle_accum_ms >= g_state.max_idle_ms && !obj->suppressed) {
            obj->active = false;
            obj->suppressed = true;
        }
    }
}

/* --- Attribute processing --- */

static void process_attributes(TrackedObject* obj, VOD__Detection* det) {
    for (unsigned int i = 0; i < det->n_attributes; ++i) {
        VOD__Attribute* attr = det->attributes[i];
        int type = attr->type;
        int value = attr->has_class_case ? attr->attr_class : -1;
        int score = attr->has_score_case ? attr->score : -1;
        if (score < 0 || value < 0) continue;
        AttrVal* av = attr_get(obj->attr_map, type);
        if (!av || score > av->score) attr_set(obj->attr_map, type, value, score);

        // Table-driven: update tracked fields for known types
        const char* valstr = attribute_value_string(type, value);
        // Vehicle type: update class string if score is higher
        if ((g_active_attr_set->version_id == 2 && type == 0) // Definition 2: vehicle_type
            || (g_active_attr_set->version_id == 3 && type == 5)) { // Definition 3: vehicle_type
            if (score > obj->vehicle_type_score) {
                obj->vehicle_type_type = type;
                obj->vehicle_type_value = value;
                obj->vehicle_type_score = score;
                if (valstr) {
                    strncpy(obj->class_name, valstr, sizeof(obj->class_name)-1);
                    obj->class_name[sizeof(obj->class_name)-1] = '\0';
                }
            }
        }
        // Bag type
        if ((g_active_attr_set->version_id == 1 && type == 5) // Def 1: bag_type
            || (g_active_attr_set->version_id == 2 && type == 1) // Def 2: bag_type
            || (g_active_attr_set->version_id == 3 && type == 7)) { // Def 3: bag_type
            if (score > obj->bag_score) {
                obj->bag_type = type; obj->bag_value = value; obj->bag_score = score;
            }
        }
        // Upper/lower color
        if ((g_active_attr_set->version_id == 1 && type == 3) // Def 1: clothing_upper_color
            || (g_active_attr_set->version_id == 2 && type == 2) // Def 2: clothing_upper_color
            || (g_active_attr_set->version_id == 3 && type == 3)) { // Def 3: clothing_upper_color
            if (score > obj->color_score) { obj->color_type = type; obj->color_value = value; obj->color_score = score; }
        }
        if ((g_active_attr_set->version_id == 1 && type == 4) // Def 1: clothing_lower_color
            || (g_active_attr_set->version_id == 2 && type == 3) // Def 2: clothing_lower_color
            || (g_active_attr_set->version_id == 3 && type == 4)) { // Def 3: clothing_lower_color
            if (score > obj->color2_score) { obj->color2_type = type; obj->color2_value = value; obj->color2_score = score; }
        }
        // Vehicle color
        if ((g_active_attr_set->version_id == 1 && type == 2) // Def 1: vehicle_color
            || (g_active_attr_set->version_id == 2 && type == 7) // Def 2: vehicle_color
            || (g_active_attr_set->version_id == 3 && type == 2)) { // Def 3: vehicle_color
            if (score > obj->color_score) { obj->color_type = type; obj->color_value = value; obj->color_score = score; }
        }
        // Hat type
        if ((g_active_attr_set->version_id == 1 && type == 6)
            || (g_active_attr_set->version_id == 2 && type == 6)
            || (g_active_attr_set->version_id == 3 && type == 6)) {
            if (score > obj->hat_score) { obj->hat_type = type; obj->hat_value = value; obj->hat_score = score; }
        }
        // Face visibility
        if ((g_active_attr_set->version_id == 1 && type == 7)
            || (g_active_attr_set->version_id == 2 && type == 5)
            || (g_active_attr_set->version_id == 3 && type == 8)) {
            if (score > obj->face_score) { obj->face_type = type; obj->face_value = value; obj->face_score = score; }
        }
    }
}

/* --- Main Detection Update Logic --- */

static void update_tracked_object(TrackedObject* obj, VOD__Detection* det, double ts, int attr_def) {
    // Calculate bounding box and center
    int x1 = (det->left * 4096) + 4096;
    int y1 = 4096 - (det->top * 4096);
    int x2 = (det->right * 4096) + 4096;
    int y2 = 4096 - (det->bottom * 4096);
    int x = (x1 * 1000) / 8192;
    int y = (y1 * 1000) / 8192;
    int w = ((x2 - x1) * 1000) / 8192;
    int h = ((y2 - y1) * 1000) / 8192;

    // Apply rotation
    if (g_state.rotation == 180) {
        x = 1000 - x - w;
        y = 1000 - y - h;
    } else if (g_state.rotation == 90) {
        int t = h; h = w; w = t;
        t = x; x = 1000 - y - w; y = t;
    } else if (g_state.rotation == 270) {
        int t = h; h = w; w = t;
        t = y; y = 1000 - x - h; x = t;
    }

    int cx = x + (w / 2);
    int cy = y + (h / 2);
    if (g_state.cog == 1)
        cy = y + h;

    obj->x = x; obj->y = y; obj->w = w; obj->h = h;
    obj->cx = cx; obj->cy = cy;

    // Update main class confidence (stable/highest)
    int new_score = (unsigned)(det->score);
    if (new_score > obj->class_score) {
        obj->class_score = new_score;
        obj->class_id = (int)det->det_class;
        strncpy(obj->class_raw, video_object_detection_subscriber_det_class_name(g_state.classifications[obj->class_id]), sizeof(obj->class_raw) - 1);
        strncpy(obj->class_name, translate_class(obj->class_raw), sizeof(obj->class_name) - 1);
    }
    obj->confidence = obj->class_score;
    obj->last_ts = ts;
    obj->age = ts - obj->birth_ts;

    // Validation: only update if valid, or if just became valid
    if (!obj->valid) {
        if (cx >= g_state.x1 && cx <= g_state.x2 &&
            cy >= g_state.y1 && cy <= g_state.y2 &&
            obj->confidence > g_state.min_confidence &&
            w > g_state.min_width && w < g_state.max_width &&
            h > g_state.min_height && h < g_state.max_height) {
            obj->valid = true;
            obj->last_sig_cx = cx;
            obj->last_sig_cy = cy;
            obj->distance = 0.0f;
        } else {
            // Not valid yet; do not update further
            return;
        }
    }

    // On first detection, keep significantMovement true, then clear
    if (obj->just_created) {
        obj->just_created = false;
        obj->significantMovement = true;
    } else {
        update_idle_state(obj, cx, cy, ts);
    }

    // Distance update on significant movement
    if (obj->significantMovement) {
        int dx = cx - obj->last_sig_cx;
        int dy = cy - obj->last_sig_cy;
        float delta_distance = sqrtf(dx * dx + dy * dy) / 10.0f;
        obj->distance += delta_distance;
        obj->last_sig_cx = cx;
        obj->last_sig_cy = cy;
        obj->dx = dx;
        obj->dy = dy;
    } else {
        obj->dx = cx - obj->last_sig_cx;
        obj->dy = cy - obj->last_sig_cy;
    }

    // Attribute processing (version-aware)
    process_attributes(obj, det, attr_def);
}

/* --- Subscriber Notification --- */

static void notify_subscribers() {
    cJSON* list = cJSON_CreateArray();
    GHashTableIter iter;
    gpointer key, value;
    g_hash_table_iter_init(&iter, g_state.objects);
    while (g_hash_table_iter_next(&iter, &key, &value)) {
        TrackedObject* obj = (TrackedObject*)value;
        if (obj->suppressed || !obj->valid) continue;
        cJSON* item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "id", obj->id);
        cJSON_AddBoolToObject(item, "active", obj->active);
        cJSON_AddNumberToObject(item, "type", obj->class_id);
        cJSON_AddStringToObject(item, "class", obj->class_name);
        cJSON_AddNumberToObject(item, "age", obj->age / 1000);  // seconds
        cJSON_AddNumberToObject(item, "distance", (int)(obj->distance));
        cJSON_AddNumberToObject(item, "x", (int)obj->x);
        cJSON_AddNumberToObject(item, "y", (int)obj->y);
        cJSON_AddNumberToObject(item, "w", (int)obj->w);
        cJSON_AddNumberToObject(item, "h", obj->h);
        cJSON_AddNumberToObject(item, "cx", obj->cx);
        cJSON_AddNumberToObject(item, "cy", obj->cy);
        cJSON_AddNumberToObject(item, "bx", obj->bx);
        cJSON_AddNumberToObject(item, "by", obj->by);
        cJSON_AddNumberToObject(item, "dx", obj->dx);
        cJSON_AddNumberToObject(item, "dy", obj->dy);
        cJSON_AddNumberToObject(item, "confidence", obj->confidence);
        cJSON_AddNumberToObject(item, "idle", obj->idle_accum_ms);
        cJSON_AddBoolToObject(item, "significantMovement", obj->significantMovement);

        // Output attributes as human-readable strings
        // Color (vehicle or clothing upper)
        const char* color = attribute_value_string(obj->color_type, obj->color_value);
        if (color && color[0]) cJSON_AddStringToObject(item, "color", color);
        // Color2 (clothing lower)
        const char* color2 = attribute_value_string(obj->color2_type, obj->color2_value);
        if (color2 && color2[0]) cJSON_AddStringToObject(item, "color2", color2);
        // Hat
        const char* hat = attribute_value_string(obj->hat_type, obj->hat_value);
        if (hat && hat[0]) cJSON_AddStringToObject(item, "hat", hat);
        // Bag
        const char* bag = attribute_value_string(obj->bag_type, obj->bag_value);
        if (bag && bag[0]) cJSON_AddStringToObject(item, "bag", bag);
        // Face
        if (obj->face_score >= 0) {
            const char* face = attribute_value_string(obj->face_type, obj->face_value);
            if (face && strcmp(face, "Face") == 0)
                cJSON_AddBoolToObject(item, "face", true);
            else if (face && strcmp(face, "Head") == 0)
                cJSON_AddBoolToObject(item, "face", false);
        }
        // attributes array
        cJSON* attr_arr = cJSON_CreateArray();
        GHashTableIter aiter;
        gpointer akey, aval;
        g_hash_table_iter_init(&aiter, obj->attr_map);
        while (g_hash_table_iter_next(&aiter, &akey, &aval)) {
            int type = *(int*)akey;
            AttrVal* av = (AttrVal*)aval;
            cJSON* aitem = cJSON_CreateObject();
            cJSON_AddNumberToObject(aitem, "type", type);
            cJSON_AddNumberToObject(aitem, "value", av->value);
            cJSON_AddNumberToObject(aitem, "score", av->score);
            cJSON_AddItemToArray(attr_arr, aitem);
        }
        cJSON_AddItemToObject(item, "attributes", attr_arr);

        cJSON_AddItemToArray(list, item);
        obj->significantMovement = false;
    }
    if (g_state.user_cb) g_state.user_cb(list);
    cJSON_Delete(list);
}

/* --- Main Detection Callback --- */

static void ObjectDetection_Scene_Callback(const uint8_t *detection_data, size_t data_size, void *user_data) {
    if (!detection_data || !data_size) return;
    double ts = now_ms();

    VOD__Scene* recv_scene = vod__scene__unpack(NULL, data_size, detection_data);
    if (!recv_scene) return;

    // Process detections: update or create objects, set active = true
    for (unsigned int i = 0; i < recv_scene->n_detections; i++) {
        VOD__Detection* det = recv_scene->detections[i];
        int x1 = (det->left * 4096) + 4096;
        int y1 = 4096 - (det->top * 4096);
        int x2 = (det->right * 4096) + 4096;
        int y2 = 4096 - (det->bottom * 4096);
        int x = (x1 * 1000) / 8192;
        int y = (y1 * 1000) / 8192;
        int w = ((x2 - x1) * 1000) / 8192;
        int h = ((y2 - y1) * 1000) / 8192;
        int cx = x + (w/2);
        int cy = y + (h/2);
        if (g_state.cog == 1)
            cy = y + h;

        char id_str[32];
        snprintf(id_str, sizeof(id_str), "%d", (int)det->id);

        TrackedObject* obj = g_hash_table_lookup(g_state.objects, id_str);
        if (!obj) {
            int class_id = (int)det->det_class;
            const char* class_raw = video_object_detection_subscriber_det_class_name(g_state.classifications[class_id]);
            obj = tracked_object_new(id_str, class_id, class_raw, (unsigned)det->score, cx, cy, ts);
            g_hash_table_insert(g_state.objects, g_strdup(id_str), obj);
        }
        update_tracked_object(obj, det, ts);
        obj->active = true;
        obj->pending_removal = false;
        if (g_state.max_idle_ms > 0 && obj->idle_accum_ms >= g_state.max_idle_ms && !obj->suppressed) {
            obj->active = false;
            obj->suppressed = true;
        }
    }

    // Process events (object lost)
    for (unsigned int i = 0; i < recv_scene->n_events; i++) {
        VOD__Event* ev = recv_scene->events[i];
        if (ev->action == 0) { // Lost
            char id_str[32];
            snprintf(id_str, sizeof(id_str), "%d", (int)ev->object_id);
            TrackedObject* obj = g_hash_table_lookup(g_state.objects, id_str);
            if (obj) {
                if (obj->suppressed) {
                    g_hash_table_remove(g_state.objects, id_str);
                } else if (obj->active) {
                    obj->active = false;
                    obj->pending_removal = true;
                }
            }
        }
    }

    notify_subscribers();

    // Remove objects marked for removal (lost and not suppressed)
    GList* keys = g_hash_table_get_keys(g_state.objects);
    for (GList* l = keys; l != NULL; l = l->next) {
        const char* id = (const char*)l->data;
        TrackedObject* obj = g_hash_table_lookup(g_state.objects, id);
        if (obj && obj->pending_removal) {
            g_hash_table_remove(g_state.objects, id);
        }
    }
    g_list_free(keys);

    vod__scene__free_unpacked(recv_scene, NULL);
}

/* --- Initialization and Config --- */

static void object_detection_state_clear() {
    if (g_state.objects) {
        g_hash_table_destroy(g_state.objects);
        g_state.objects = NULL;
    }
    g_state.objects = g_hash_table_new_full(g_str_hash, g_str_equal, free, (GDestroyNotify)tracked_object_free);
}

int ObjectDetection_Init(ObjectDetection_Callback callback) {
    if (!callback) return 0;
    g_state.user_cb = callback;
    object_detection_state_clear();

    if (video_object_detection_subscriber_create(&g_state.vod_handler, NULL, 0) != 0) {
        LOG_WARN("Cannot open channel to ObjectDetection\n");
        return 0;
    }
    int num_classes = video_object_detection_subscriber_det_classes_get(&g_state.classifications);
    for (int i = 0; i < num_classes; i++) {
        LOG("%d: %s\n", i, video_object_detection_subscriber_det_class_name(g_state.classifications[i]));
    }
    if (video_object_detection_subscriber_set_get_detection_callback(g_state.vod_handler, ObjectDetection_Scene_Callback) != 0) {
        LOG_WARN("Could not set object detection callback\n");
        return 0;
    }
    video_object_detection_subscriber_set_receive_empty_hits(g_state.vod_handler, 1);
    int status = video_object_detection_subscriber_subscribe(g_state.vod_handler);
    if (status != 0) {
        LOG_WARN("Object detection subscription failed. Error Code: %d\n", status);
        return 0;
    }
    ObjectDetection_InitAttributeDefs();
    return 1;
}

void ObjectDetection_Config(cJSON* data) {
    if (!data) return;
    g_state.min_confidence = cJSON_GetObjectItem(data, "confidence") ? cJSON_GetObjectItem(data, "confidence")->valueint : 40;
    g_state.cog = cJSON_GetObjectItem(data, "cog") ? cJSON_GetObjectItem(data, "cog")->valueint : 0;
    g_state.rotation = cJSON_GetObjectItem(data, "rotation") ? cJSON_GetObjectItem(data, "rotation")->valueint : 0;
    g_state.max_idle_ms = cJSON_GetObjectItem(data, "maxIdle") ? cJSON_GetObjectItem(data, "maxIdle")->valueint : 0;
    g_state.min_height = cJSON_GetObjectItem(data, "minHeight") ? cJSON_GetObjectItem(data, "minHeight")->valueint : 10;
    g_state.max_height = cJSON_GetObjectItem(data, "maxHeight") ? cJSON_GetObjectItem(data, "maxHeight")->valueint : 800;
    g_state.min_width = cJSON_GetObjectItem(data, "minWidth") ? cJSON_GetObjectItem(data, "minWidth")->valueint : 10;
    g_state.max_width = cJSON_GetObjectItem(data, "maxWidth") ? cJSON_GetObjectItem(data, "maxWidth")->valueint : 800;
    cJSON* aoi = cJSON_GetObjectItem(data, "aoi");
    if (aoi) {
        g_state.x1 = cJSON_GetObjectItem(aoi, "x1") ? cJSON_GetObjectItem(aoi, "x1")->valueint : 0;
        g_state.x2 = cJSON_GetObjectItem(aoi, "x2") ? cJSON_GetObjectItem(aoi, "x2")->valueint : 1000;
        g_state.y1 = cJSON_GetObjectItem(aoi, "y1") ? cJSON_GetObjectItem(aoi, "y1")->valueint : 0;
        g_state.y2 = cJSON_GetObjectItem(aoi, "y2") ? cJSON_GetObjectItem(aoi, "y2")->valueint : 1000;
    }
    object_detection_state_clear();
    ObjectDetection_InitAttributeDefs();
}

void ObjectDetection_Reset() {
    object_detection_state_clear();
}

int ObjectDetection_CacheSize() {
    return g_state.objects ? g_hash_table_size(g_state.objects) : 0;
}
