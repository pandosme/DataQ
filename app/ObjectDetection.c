/**
 * ObjectDetection.c
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
#include "ObjectDetection.h"
#include "cJSON.h"
#include "ACAP.h"
#include <mdb/connection.h>
#include <mdb/error.h>
#include <mdb/subscriber.h>

#define LOG(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args);}
#define LOG_WARN(fmt, args...)    { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args);}
//#define LOG_TRACE(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_TRACE(fmt, args...)    {}

typedef struct channel_identifier {
    char* topic;
    char* source;
} channel_identifier_t;

// Global variables
static mdb_error_t* error = NULL;
static mdb_subscriber_config_t* subscriber_config = NULL;
static mdb_subscriber_t* subscriber = NULL;
static ObjectDetection_Callback ObjectDetection_UserCallback = NULL;

// Three-cache system for different object states
static cJSON* validCache = NULL;      // Active, moving objects
static cJSON* nonValidCache = NULL;   // Objects that haven't passed initial validation
static cJSON* idleCache = NULL;       // Objects that exceeded maxIdle time but are still monitored
static cJSON* detectionCache = NULL;  // Output cache for consumers
static cJSON* config = NULL;

static int lastDetectionWasEmpty = 0;

// Configuration values
static int config_confidence = 30;
static int config_cog = 1;
static int config_maxIdle = 0;
static int config_minWidth = 10;
static int config_minHeight = 10;
static int config_maxWidth = 800;
static int config_maxHeight = 800;
static int config_aoi_x1 = 50;
static int config_aoi_y1 = 50;
static int config_aoi_x2 = 950;
static int config_aoi_y2 = 950;

// Thread safety
static GMutex cache_mutex;

// Timer
static guint timer_id = 0;

// Statistics
static int stats_total_processed = 0;
static int stats_total_filtered = 0;

// Function prototypes
static void process_frame_data(cJSON* frame);
static void process_observations(cJSON* observations);
static void process_operations(cJSON* operations);
static void process_idle_objects_for_movement(cJSON* observations);
static int validate_object(cJSON* observation);
static void update_object(cJSON* cached_object, cJSON* observation);
static void create_new_object(cJSON* observation, const char* track_id);
static void move_to_valid_cache(const char* track_id);
static void move_to_idle_cache(const char* track_id);
static void reactivate_from_idle(const char* track_id);
static cJSON* find_object_in_any_cache(const char* track_id);
static double calculate_distance(int x1, int y1, int x2, int y2);
static void build_detection_cache();
static gboolean timer_callback(gpointer user_data);

void
ObjectDetection_Config(cJSON* data) {
    g_mutex_lock(&cache_mutex);
    
    if (config) {
        cJSON_Delete(config);
    }
    config = cJSON_Duplicate(data, 1);
    
    // Extract configuration values
    cJSON* item = cJSON_GetObjectItem(config, "confidence");
    if (item && cJSON_IsNumber(item)) {
        config_confidence = item->valueint;
    }
    
    item = cJSON_GetObjectItem(config, "cog");
    if (item && cJSON_IsNumber(item)) {
        config_cog = item->valueint;
    }
    
    item = cJSON_GetObjectItem(config, "maxIdle");
    if (item && cJSON_IsNumber(item)) {
        config_maxIdle = item->valueint;
    }
    
    item = cJSON_GetObjectItem(config, "minWidth");
    if (item && cJSON_IsNumber(item)) {
        config_minWidth = item->valueint;
    }
    
    item = cJSON_GetObjectItem(config, "minHeight");
    if (item && cJSON_IsNumber(item)) {
        config_minHeight = item->valueint;
    }
    
    item = cJSON_GetObjectItem(config, "maxWidth");
    if (item && cJSON_IsNumber(item)) {
        config_maxWidth = item->valueint;
    }
    
    item = cJSON_GetObjectItem(config, "maxHeight");
    if (item && cJSON_IsNumber(item)) {
        config_maxHeight = item->valueint;
    }
    
    cJSON* aoi = cJSON_GetObjectItem(config, "aoi");
    if (aoi) {
        item = cJSON_GetObjectItem(aoi, "x1");
        if (item && cJSON_IsNumber(item)) config_aoi_x1 = item->valueint;
        
        item = cJSON_GetObjectItem(aoi, "y1");
        if (item && cJSON_IsNumber(item)) config_aoi_y1 = item->valueint;
        
        item = cJSON_GetObjectItem(aoi, "x2");
        if (item && cJSON_IsNumber(item)) config_aoi_x2 = item->valueint;
        
        item = cJSON_GetObjectItem(aoi, "y2");
        if (item && cJSON_IsNumber(item)) config_aoi_y2 = item->valueint;
    }
    
//    config_blacklist = cJSON_GetObjectItem(config, "ignoreClass");
    
    g_mutex_unlock(&cache_mutex);
}

void
ObjectDetection_Reset() {
    g_mutex_lock(&cache_mutex);
    
    if (validCache) {
        cJSON_Delete(validCache);
    }
    validCache = cJSON_CreateObject();
    
    if (nonValidCache) {
        cJSON_Delete(nonValidCache);
    }
    nonValidCache = cJSON_CreateObject();
    
    if (idleCache) {
        cJSON_Delete(idleCache);
    }
    idleCache = cJSON_CreateObject();
    
    if (detectionCache) {
        cJSON_Delete(detectionCache);
    }
    detectionCache = cJSON_CreateArray();
    
    // Reset statistics
    stats_total_processed = 0;
    stats_total_filtered = 0;
    
    g_mutex_unlock(&cache_mutex);
}

int
ObjectDetection_CacheSize() {
    g_mutex_lock(&cache_mutex);
    int size = cJSON_GetArraySize(validCache) + cJSON_GetArraySize(nonValidCache) + cJSON_GetArraySize(idleCache);
    g_mutex_unlock(&cache_mutex);
    return size;
}

static void
MessageBroker_Connection_Error(const mdb_error_t* error, void* user_data) {
    (void)user_data;
    LOG_WARN("%s: %s\n", __func__, error->message);
}

static void
MessageBroker_Data(const mdb_message_t* message, void* user_data) {
    const mdb_message_payload_t* payload = mdb_message_get_payload(message);
    
    cJSON* data = cJSON_Parse((char*)payload->data);
    if (!data) {
        LOG_TRACE("%s: Invalid JSON data\n", __func__);
        return;
    }

    cJSON* frame = cJSON_GetObjectItem(data, "frame");
    if (!frame) {
        cJSON_Delete(data);
        return;
    }
    
    g_mutex_lock(&cache_mutex);
    
    // Process frame data
    process_frame_data(frame);
    
    // Build detection cache for callback
    build_detection_cache();
    
    // Send to consumers with empty detection suppression
    if (cJSON_GetArraySize(detectionCache) > 0) {
        lastDetectionWasEmpty = 0;
        g_mutex_unlock(&cache_mutex);
        ObjectDetection_UserCallback(detectionCache);
        g_mutex_lock(&cache_mutex);
    } else {
        if (lastDetectionWasEmpty == 0) {
            g_mutex_unlock(&cache_mutex);
            ObjectDetection_UserCallback(detectionCache);
            g_mutex_lock(&cache_mutex);
        }
        lastDetectionWasEmpty = 1;
    }
    
    g_mutex_unlock(&cache_mutex);
    
    cJSON_Delete(data);
}

static void
process_frame_data(cJSON* frame) {
    // Process observations
    cJSON* observations = cJSON_GetObjectItem(frame, "observations");
    if (observations && cJSON_IsArray(observations)) {
        // First check idle objects for movement
        process_idle_objects_for_movement(observations);
        // Then process all observations
        process_observations(observations);
    }
    
    // Process operations
    cJSON* operations = cJSON_GetObjectItem(frame, "operations");
    if (operations && cJSON_IsArray(operations)) {
        process_operations(operations);
    }
}

static void
process_idle_objects_for_movement(cJSON* observations) {
    cJSON* observation = NULL;
    
    cJSON_ArrayForEach(observation, observations) {
        cJSON* track_id_item = cJSON_GetObjectItem(observation, "track_id");
        if (!track_id_item || !cJSON_IsString(track_id_item)) {
            continue;
        }
        
        const char* track_id = track_id_item->valuestring;
        
        // Check if this object is in idle cache
        cJSON* idle_object = cJSON_GetObjectItem(idleCache, track_id);
        if (idle_object) {
            // Calculate movement from current position
            cJSON* bbox = cJSON_GetObjectItem(observation, "bounding_box");
            if (bbox) {
                cJSON* left = cJSON_GetObjectItem(bbox, "left");
                cJSON* top = cJSON_GetObjectItem(bbox, "top");
                cJSON* right = cJSON_GetObjectItem(bbox, "right");
                cJSON* bottom = cJSON_GetObjectItem(bbox, "bottom");
                
                if (left && top && right && bottom) {
                    int x = (int)(left->valuedouble * 1000);
                    int y = (int)(top->valuedouble * 1000);
                    int w = (int)((right->valuedouble - left->valuedouble) * 1000);
                    int h = (int)((bottom->valuedouble - top->valuedouble) * 1000);
                    
                    // Calculate center of gravity
                    int cx, cy;
                    if (config_cog == 1) {
                        cx = x + w / 2;
                        cy = y + h;
                    } else {
                        cx = x + w / 2;
                        cy = y + h / 2;
                    }
                    
                    // Get previous position (use px, py)
                    cJSON* prev_px = cJSON_GetObjectItem(idle_object, "px");
                    cJSON* prev_py = cJSON_GetObjectItem(idle_object, "py");
                    
                    if (prev_px && prev_py) {
                        double movement = calculate_distance(cx, cy, prev_px->valueint, prev_py->valueint);
                        
                        if (movement > 50.0) { // Object started moving again
                            reactivate_from_idle(track_id);
                        }
                    }
                }
            }
        }
    }
}

static void
process_observations(cJSON* observations) {
    cJSON* observation = NULL;
    
    cJSON_ArrayForEach(observation, observations) {
        stats_total_processed++;
        
        cJSON* track_id_item = cJSON_GetObjectItem(observation, "track_id");
        if (!track_id_item || !cJSON_IsString(track_id_item)) {
            continue;
        }
        
        const char* track_id = track_id_item->valuestring;
        
        // Check if object exists in valid cache - O(1) lookup
        cJSON* cached_object = cJSON_GetObjectItem(validCache, track_id);
        if (cached_object) {
            update_object(cached_object, observation);
            continue;
        }
        
        // Check if object exists in idle cache - O(1) lookup
        cached_object = cJSON_GetObjectItem(idleCache, track_id);
        if (cached_object) {
            // Update idle object but don't move it unless it's moving (handled in process_idle_objects_for_movement)
            update_object(cached_object, observation);
            continue;
        }
        
        // Check if object exists in non-valid cache - O(1) lookup
        cached_object = cJSON_GetObjectItem(nonValidCache, track_id);
        if (cached_object) {
            update_object(cached_object, observation);
            // Check if it now passes validation
            if (validate_object(observation)) {
                move_to_valid_cache(track_id);
            }
            continue;
        }
        
        // New object - validate and add to appropriate cache
        if (validate_object(observation)) {
            create_new_object(observation, track_id);
            cJSON* new_object = cJSON_GetObjectItem(nonValidCache, track_id);
            if (new_object) {
                // Move to valid cache
                cJSON_DetachItemFromObject(nonValidCache, track_id);
                cJSON_AddItemToObject(validCache, track_id, new_object);
            }
        } else {
            create_new_object(observation, track_id);
            stats_total_filtered++;
        }
    }
}

static void
process_operations(cJSON* operations) {
    cJSON* operation = NULL;
    
    cJSON_ArrayForEach(operation, operations) {
        cJSON* type_item = cJSON_GetObjectItem(operation, "type");
        cJSON* id_item = cJSON_GetObjectItem(operation, "id");
        
        if (!type_item || !id_item || !cJSON_IsString(type_item) || !cJSON_IsString(id_item)) {
            continue;
        }
        
        if (strcmp(type_item->valuestring, "DeleteOperation") == 0) {
            const char* object_id = id_item->valuestring;
            
            // Find and mark object as inactive in valid cache - O(1) lookup
            cJSON* cached_object = cJSON_GetObjectItem(validCache, object_id);
            if (cached_object) {
                cJSON* active_item = cJSON_GetObjectItem(cached_object, "active");
                if (active_item) {
                    cJSON_SetBoolValue(active_item, false);
                }
            }
            
            // Find and mark object as inactive in idle cache - O(1) lookup
            cached_object = cJSON_GetObjectItem(idleCache, object_id);
            if (cached_object) {
                cJSON* active_item = cJSON_GetObjectItem(cached_object, "active");
                if (active_item) {
                    cJSON_SetBoolValue(active_item, false);
                }
            }
            
            // Remove from non-valid cache if present - O(1) removal
            cJSON_DeleteItemFromObject(nonValidCache, object_id);
        }
    }
}

static int
validate_object(cJSON* observation) {
    // Get bounding box
    cJSON* bbox = cJSON_GetObjectItem(observation, "bounding_box");
    if (!bbox) return 0;
    
    cJSON* left = cJSON_GetObjectItem(bbox, "left");
    cJSON* top = cJSON_GetObjectItem(bbox, "top");
    cJSON* right = cJSON_GetObjectItem(bbox, "right");
    cJSON* bottom = cJSON_GetObjectItem(bbox, "bottom");
    
    if (!left || !top || !right || !bottom) return 0;
    
    // Convert to [0-1000] coordinates
    int x = (int)(left->valuedouble * 1000);
    int y = (int)(top->valuedouble * 1000);
    int w = (int)((right->valuedouble - left->valuedouble) * 1000);
    int h = (int)((bottom->valuedouble - top->valuedouble) * 1000);
    
    // Calculate center of gravity
    int cx, cy;
    if (config_cog == 1) {
        // Bottom middle
        cx = x + w / 2;
        cy = y + h;
    } else {
        // Center of box
        cx = x + w / 2;
        cy = y + h / 2;
    }
    
    cJSON* class_obj = cJSON_GetObjectItem(observation, "class");
    if (class_obj) {
        cJSON* score_item = cJSON_GetObjectItem(class_obj, "score");
        if (score_item && cJSON_IsNumber(score_item)) {
            int confidence = (int)(score_item->valuedouble * 100);
            if (confidence < config_confidence) {
                return 0;
            }
        }
    }
    
    // Check dimensions
    if (w < config_minWidth || h < config_minHeight || 
        w > config_maxWidth || h > config_maxHeight) {
        return 0;
    }
    
    // Check AOI
    if (cx < config_aoi_x1 || cx > config_aoi_x2 || 
        cy < config_aoi_y1 || cy > config_aoi_y2) {
        return 0;
    }
    
    return 1;
}

static void
create_new_object(cJSON* observation, const char* track_id) {
    cJSON* new_object = cJSON_CreateObject();
    
    // Set ID
    cJSON_AddStringToObject(new_object, "id", track_id);
    
    // Set initial values
    cJSON_AddBoolToObject(new_object, "active", true);
    
    // Get class information
    cJSON* class_obj = cJSON_GetObjectItem(observation, "class");
    if (class_obj) {
        cJSON* type_item = cJSON_GetObjectItem(class_obj, "type");
        if (type_item && cJSON_IsString(type_item)) {
            cJSON_AddStringToObject(new_object, "class", type_item->valuestring);
        } else {
            cJSON_AddStringToObject(new_object, "class", "Object");
        }
        
        cJSON* score_item = cJSON_GetObjectItem(class_obj, "score");
        if (score_item && cJSON_IsNumber(score_item)) {
            int confidence = (int)(score_item->valuedouble * 100);
            cJSON_AddNumberToObject(new_object, "confidence", confidence);
        } else {
            cJSON_AddNumberToObject(new_object, "confidence", 10);
        }

        
        // Handle colors
		cJSON_AddNumberToObject(new_object, "colorConfidence", 0);
		cJSON_AddNumberToObject(new_object, "color2Confidence", 0);
        cJSON* colors = cJSON_GetObjectItem(class_obj, "colors");
        if (colors && cJSON_IsArray(colors) && cJSON_GetArraySize(colors) > 0) {
            cJSON* color_item = cJSON_GetArrayItem(colors, 0);
            cJSON* color_name = cJSON_GetObjectItem(color_item, "name");
            cJSON* color_score = cJSON_GetObjectItem(color_item, "score");
            if (color_name && cJSON_IsString(color_name)) {
                cJSON_AddStringToObject(new_object, "color", color_name->valuestring);
            }
            if (color_score)
				cJSON_ReplaceItemInObject(new_object,"colorConfidence",cJSON_CreateNumber(color_score->valuedouble));
        } else {
            cJSON* upper_colors = cJSON_GetObjectItem(class_obj, "upper_clothing_colors");
            if (upper_colors && cJSON_IsArray(upper_colors) && cJSON_GetArraySize(upper_colors) > 0) {
                cJSON* color_item = cJSON_GetArrayItem(upper_colors, 0);
                cJSON* color_name = cJSON_GetObjectItem(color_item, "name");
				cJSON* color_score = cJSON_GetObjectItem(color_item, "score");
                if (color_name && cJSON_IsString(color_name)) {
                    cJSON_AddStringToObject(new_object, "color", color_name->valuestring);
                }
				if (color_score)
					cJSON_ReplaceItemInObject(new_object,"colorConfidence",cJSON_CreateNumber(color_score->valuedouble));
            } else {
                cJSON_AddStringToObject(new_object, "color", "");
            }
        }
        
        // Handle color2 (lower clothing)
        cJSON* lower_colors = cJSON_GetObjectItem(class_obj, "lower_clothing_colors");
        if (lower_colors && cJSON_IsArray(lower_colors) && cJSON_GetArraySize(lower_colors) > 0) {
            cJSON* color_item = cJSON_GetArrayItem(lower_colors, 0);
            cJSON* color_name = cJSON_GetObjectItem(color_item, "name");
			cJSON* color_score = cJSON_GetObjectItem(color_item, "score");
            if (color_name && cJSON_IsString(color_name)) {
                cJSON_AddStringToObject(new_object, "color2", color_name->valuestring);
            }
			if (color_score)
				cJSON_ReplaceItemInObject(new_object,"color2Confidence",cJSON_CreateNumber(color_score->valuedouble));
        } else {
            cJSON_AddStringToObject(new_object, "color2", "");
        }
    } else {
        cJSON_AddStringToObject(new_object, "class", "Object");
        cJSON_AddNumberToObject(new_object, "confidence", 10);
        cJSON_AddStringToObject(new_object, "color", "");
        cJSON_AddStringToObject(new_object, "color2", "");
    }
    
    // Calculate position and dimensions
    cJSON* bbox = cJSON_GetObjectItem(observation, "bounding_box");
    if (bbox) {
        cJSON* left = cJSON_GetObjectItem(bbox, "left");
        cJSON* top = cJSON_GetObjectItem(bbox, "top");
        cJSON* right = cJSON_GetObjectItem(bbox, "right");
        cJSON* bottom = cJSON_GetObjectItem(bbox, "bottom");
        
        if (left && top && right && bottom) {
            int x = (int)(left->valuedouble * 1000);
            int y = (int)(top->valuedouble * 1000);
            int w = (int)((right->valuedouble - left->valuedouble) * 1000);
            int h = (int)((bottom->valuedouble - top->valuedouble) * 1000);
            
            // Calculate center of gravity
            int cx, cy;
            if (config_cog == 1) {
                cx = x + w / 2;
                cy = y + h;
            } else {
                cx = x + w / 2;
                cy = y + h / 2;
            }
            
            cJSON_AddNumberToObject(new_object, "x", x);
            cJSON_AddNumberToObject(new_object, "y", y);
            cJSON_AddNumberToObject(new_object, "w", w);
            cJSON_AddNumberToObject(new_object, "h", h);
            cJSON_AddNumberToObject(new_object, "cx", cx);
            cJSON_AddNumberToObject(new_object, "cy", cy);
            cJSON_AddNumberToObject(new_object, "bx", cx);
            cJSON_AddNumberToObject(new_object, "by", cy);
            cJSON_AddNumberToObject(new_object, "px", cx);  // Previous x = current x initially
            cJSON_AddNumberToObject(new_object, "py", cy);  // Previous y = current y initially
            cJSON_AddNumberToObject(new_object, "dx", 0);
            cJSON_AddNumberToObject(new_object, "dy", 0);
            cJSON_AddNumberToObject(new_object, "distance", 0);
        }
    }
    
    // Set timestamps
    double current_time = ACAP_DEVICE_Timestamp();
    cJSON_AddNumberToObject(new_object, "birth", current_time);
    cJSON_AddNumberToObject(new_object, "age", 0.0);
    cJSON_AddNumberToObject(new_object, "idle", 0);
    cJSON_AddNumberToObject(new_object, "timestamp", current_time);
    
    // Add velocity tracking
    cJSON_AddNumberToObject(new_object, "velocity", 0.0);
    cJSON_AddNumberToObject(new_object, "topVelocity", 0.0);
    cJSON_AddNumberToObject(new_object, "last_update", current_time);
    
    // Add to non-valid cache initially (will be moved to valid if it passes validation)
    cJSON_AddItemToObject(nonValidCache, track_id, new_object);
}

static void
update_object(cJSON* cached_object, cJSON* observation) {
    double current_time = ACAP_DEVICE_Timestamp();
    
    // Update confidence and class if higher
    cJSON* class_obj = cJSON_GetObjectItem(observation, "class");
    if (class_obj) {
        cJSON* score_item = cJSON_GetObjectItem(class_obj, "score");
        if (score_item && cJSON_IsNumber(score_item)) {
            int new_confidence = (int)(score_item->valuedouble * 100);
            cJSON* cached_confidence = cJSON_GetObjectItem(cached_object, "confidence");
            if (cached_confidence && new_confidence > cached_confidence->valueint) {
                // Update confidence value
                cJSON_SetIntValue(cached_confidence, new_confidence);
                
                // Update class name
                cJSON* type_item = cJSON_GetObjectItem(class_obj, "type");
                if (type_item && cJSON_IsString(type_item)) {
                    cJSON* cached_class = cJSON_GetObjectItem(cached_object, "class");
                    if (cached_class) {
                        cJSON_SetValuestring(cached_class, type_item->valuestring);
                    }
                }
            }
        }
        
        // Handle color (vehicles or human upper clothing)
        cJSON* colors = cJSON_GetObjectItem(class_obj, "colors");
        if (colors && cJSON_IsArray(colors) && cJSON_GetArraySize(colors) > 0) {
            cJSON* color_item = cJSON_GetArrayItem(colors, 0);
            cJSON* color_name = cJSON_GetObjectItem(color_item, "name");
            cJSON* color_score = cJSON_GetObjectItem(color_item, "score");
            if (color_name && cJSON_IsString(color_name) && color_score && cJSON_IsNumber(color_score)) {
                cJSON* stored_confidence = cJSON_GetObjectItem(cached_object, "colorConfidence");
                if (!stored_confidence || color_score->valuedouble > stored_confidence->valuedouble) {
                    // Update color with higher confidence
                    cJSON* cached_color = cJSON_GetObjectItem(cached_object, "color");
                    if (cached_color) {
                        cJSON_SetValuestring(cached_color, color_name->valuestring);
                    } else {
                        cJSON_AddStringToObject(cached_object, "color", color_name->valuestring);
                    }
                    
                    // Update stored confidence
                    if (stored_confidence) {
                        cJSON_SetNumberValue(stored_confidence, color_score->valuedouble);
                    } else {
                        cJSON_AddNumberToObject(cached_object, "colorConfidence", color_score->valuedouble);
                    }
                }
            }
        } else {
            // For humans, check upper_clothing_colors
            cJSON* upper_colors = cJSON_GetObjectItem(class_obj, "upper_clothing_colors");
            if (upper_colors && cJSON_IsArray(upper_colors) && cJSON_GetArraySize(upper_colors) > 0) {
                cJSON* color_item = cJSON_GetArrayItem(upper_colors, 0);
                cJSON* color_name = cJSON_GetObjectItem(color_item, "name");
                cJSON* color_score = cJSON_GetObjectItem(color_item, "score");
                if (color_name && cJSON_IsString(color_name) && color_score && cJSON_IsNumber(color_score)) {
                    cJSON* stored_confidence = cJSON_GetObjectItem(cached_object, "colorConfidence");
                    if (!stored_confidence || color_score->valuedouble > stored_confidence->valuedouble) {
                        // Update color with higher confidence
                        cJSON* cached_color = cJSON_GetObjectItem(cached_object, "color");
                        if (cached_color) {
                            cJSON_SetValuestring(cached_color, color_name->valuestring);
                        } else {
                            cJSON_AddStringToObject(cached_object, "color", color_name->valuestring);
                        }
                        
                        // Update stored confidence
                        if (stored_confidence) {
                            cJSON_SetNumberValue(stored_confidence, color_score->valuedouble);
                        } else {
                            cJSON_AddNumberToObject(cached_object, "colorConfidence", color_score->valuedouble);
                        }
                    }
                }
            }
        }
        
        // Handle color2 (lower clothing for humans)
        cJSON* lower_colors = cJSON_GetObjectItem(class_obj, "lower_clothing_colors");
        if (lower_colors && cJSON_IsArray(lower_colors) && cJSON_GetArraySize(lower_colors) > 0) {
            cJSON* color_item = cJSON_GetArrayItem(lower_colors, 0);
            cJSON* color_name = cJSON_GetObjectItem(color_item, "name");
            cJSON* color_score = cJSON_GetObjectItem(color_item, "score");
            if (color_name && cJSON_IsString(color_name) && color_score && cJSON_IsNumber(color_score)) {
                cJSON* stored_confidence2 = cJSON_GetObjectItem(cached_object, "color2Confidence");
                if (!stored_confidence2 || color_score->valuedouble > stored_confidence2->valuedouble) {
                    // Update color2 with higher confidence
                    cJSON* cached_color2 = cJSON_GetObjectItem(cached_object, "color2");
                    if (cached_color2) {
                        cJSON_SetValuestring(cached_color2, color_name->valuestring);
                    } else {
                        cJSON_AddStringToObject(cached_object, "color2", color_name->valuestring);
                    }
                    
                    // Update stored confidence
                    if (stored_confidence2) {
                        cJSON_SetNumberValue(stored_confidence2, color_score->valuedouble);
                    } else {
                        cJSON_AddNumberToObject(cached_object, "color2Confidence", color_score->valuedouble);
                    }
                }
            }
        }
    }
    
    // Update position
    cJSON* bbox = cJSON_GetObjectItem(observation, "bounding_box");
    if (bbox) {
        cJSON* left = cJSON_GetObjectItem(bbox, "left");
        cJSON* top = cJSON_GetObjectItem(bbox, "top");
        cJSON* right = cJSON_GetObjectItem(bbox, "right");
        cJSON* bottom = cJSON_GetObjectItem(bbox, "bottom");
        
        if (left && top && right && bottom) {
            int x = (int)(left->valuedouble * 1000);
            int y = (int)(top->valuedouble * 1000);
            int w = (int)((right->valuedouble - left->valuedouble) * 1000);
            int h = (int)((bottom->valuedouble - top->valuedouble) * 1000);
            
            // Calculate center of gravity
            int cx, cy;
            if (config_cog == 1) {
                cx = x + w / 2;
                cy = y + h;
            } else {
                cx = x + w / 2;
                cy = y + h / 2;
            }
            
            // Get previous position for movement calculation (using px, py)
            cJSON* prev_px = cJSON_GetObjectItem(cached_object, "px");
            cJSON* prev_py = cJSON_GetObjectItem(cached_object, "py");
            cJSON* birth_x = cJSON_GetObjectItem(cached_object, "bx");
            cJSON* birth_y = cJSON_GetObjectItem(cached_object, "by");
            cJSON* last_update = cJSON_GetObjectItem(cached_object, "last_update");
            
            if (prev_px && prev_py && birth_x && birth_y && last_update) {
                // Calculate movement distance using px, py (previous position)
                double movement = calculate_distance(cx, cy, prev_px->valueint, prev_py->valueint);
                
                cJSON* idle_item = cJSON_GetObjectItem(cached_object, "idle");
                
                if (movement > 50.0) { // 5% movement threshold with double precision
                    // Object moved significantly
                    if (idle_item) cJSON_SetIntValue(idle_item, 0);  // Reset idle time
                    
                    // Update last_update timestamp
                    cJSON_SetNumberValue(cJSON_GetObjectItem(cached_object, "last_update"), current_time);
                    
                    // Add movement to total distance
                    cJSON* distance_item = cJSON_GetObjectItem(cached_object, "distance");
                    if (distance_item) {
                        double current_distance = distance_item->valuedouble;
                        cJSON_SetNumberValue(distance_item, current_distance + (movement / 10.0));
                    }
                    
                    // Calculate velocity using time since last movement
                    double time_diff_sec = (current_time - last_update->valuedouble) / 1000.0;
                    if (time_diff_sec > 0) {
                        double velocity = (movement / 10.0) / time_diff_sec; // %/second
                        
                        cJSON* velocity_item = cJSON_GetObjectItem(cached_object, "velocity");
                        cJSON* topVelocity_item = cJSON_GetObjectItem(cached_object, "topVelocity");
                        
                        if (velocity_item) cJSON_SetNumberValue(velocity_item, velocity);
                        if (topVelocity_item && velocity > topVelocity_item->valuedouble) {
                            cJSON_SetNumberValue(topVelocity_item, velocity);
                        }
                    }
                    
                    // Set px = cx, py = cy (update previous position)
                    cJSON_SetIntValue(cJSON_GetObjectItem(cached_object, "px"), cx);
                    cJSON_SetIntValue(cJSON_GetObjectItem(cached_object, "py"), cy);
                    
                } else {
                    // Object hasn't moved significantly, add time to idle
                    double time_diff = current_time - last_update->valuedouble;
                    if (idle_item) {
                        cJSON_SetIntValue(idle_item, idle_item->valueint + (int)time_diff);
                    }
                }
                
                // Update displacement (always based on birth position)
                cJSON* dx_item = cJSON_GetObjectItem(cached_object, "dx");
                cJSON* dy_item = cJSON_GetObjectItem(cached_object, "dy");
                if (dx_item && dy_item) {
                    cJSON_SetIntValue(dx_item, cx - birth_x->valueint);
                    cJSON_SetIntValue(dy_item, cy - birth_y->valueint);
                }
            }
            
            // Update position values
            cJSON_SetIntValue(cJSON_GetObjectItem(cached_object, "x"), x);
            cJSON_SetIntValue(cJSON_GetObjectItem(cached_object, "y"), y);
            cJSON_SetIntValue(cJSON_GetObjectItem(cached_object, "w"), w);
            cJSON_SetIntValue(cJSON_GetObjectItem(cached_object, "h"), h);
            cJSON_SetIntValue(cJSON_GetObjectItem(cached_object, "cx"), cx);
            cJSON_SetIntValue(cJSON_GetObjectItem(cached_object, "cy"), cy);
        }
    }
    
    // Update timestamp
    cJSON_SetNumberValue(cJSON_GetObjectItem(cached_object, "timestamp"), current_time);
}

static void
move_to_valid_cache(const char* track_id) {
    // Get object from non-valid cache - O(1) lookup
    cJSON* object = cJSON_GetObjectItem(nonValidCache, track_id);
    if (!object) return;
    
    // Detach from non-valid cache and add to valid cache - O(1) operations
    cJSON_DetachItemFromObject(nonValidCache, track_id);
    cJSON_AddItemToObject(validCache, track_id, object);
}

static void
move_to_idle_cache(const char* track_id) {
    // Get object from valid cache - O(1) lookup
    cJSON* object = cJSON_GetObjectItem(validCache, track_id);
    if (!object) return;
    
    // Set active to false
    cJSON* active_item = cJSON_GetObjectItem(object, "active");
    if (active_item) {
        cJSON_SetBoolValue(active_item, false);
    }
    
    // Add flag to indicate this object needs to be sent once
    cJSON_AddBoolToObject(object, "needs_notification", true);
    
    // Detach from valid cache and add to idle cache - O(1) operations
    cJSON_DetachItemFromObject(validCache, track_id);
    cJSON_AddItemToObject(idleCache, track_id, object);
}

static void
reactivate_from_idle(const char* track_id) {
    // Get object from idle cache - O(1) lookup
    cJSON* object = cJSON_GetObjectItem(idleCache, track_id);
    if (!object) return;
    
    // Reset object as new detection - preserve ID but reset tracking data
    double current_time = ACAP_DEVICE_Timestamp();
    
    // Get current position to set as new birth position
    cJSON* cx_item = cJSON_GetObjectItem(object, "cx");
    cJSON* cy_item = cJSON_GetObjectItem(object, "cy");
    
    if (cx_item && cy_item) {
        cJSON_SetIntValue(cJSON_GetObjectItem(object, "bx"), cx_item->valueint);
        cJSON_SetIntValue(cJSON_GetObjectItem(object, "by"), cy_item->valueint);
        // Set px = cx, py = cy for reactivated object
        cJSON_SetIntValue(cJSON_GetObjectItem(object, "px"), cx_item->valueint);
        cJSON_SetIntValue(cJSON_GetObjectItem(object, "py"), cy_item->valueint);
    }
    
    // Reset tracking values
    cJSON_SetNumberValue(cJSON_GetObjectItem(object, "birth"), current_time);
    cJSON_SetNumberValue(cJSON_GetObjectItem(object, "age"), 0.0);
    cJSON_SetIntValue(cJSON_GetObjectItem(object, "idle"), 0);
    cJSON_SetIntValue(cJSON_GetObjectItem(object, "dx"), 0);
    cJSON_SetIntValue(cJSON_GetObjectItem(object, "dy"), 0);
    cJSON_SetNumberValue(cJSON_GetObjectItem(object, "distance"), 0);
    cJSON_SetNumberValue(cJSON_GetObjectItem(object, "velocity"), 0.0);
    cJSON_SetNumberValue(cJSON_GetObjectItem(object, "topVelocity"), 0.0);
    cJSON_SetNumberValue(cJSON_GetObjectItem(object, "last_update"), current_time);
    
    // Set active to true
    cJSON* active_item = cJSON_GetObjectItem(object, "active");
    if (active_item) {
        cJSON_SetBoolValue(active_item, true);
    }
    
    // Detach from idle cache and add to valid cache - O(1) operations
    cJSON_DetachItemFromObject(idleCache, track_id);
    cJSON_AddItemToObject(validCache, track_id, object);
}

static cJSON*
find_object_in_any_cache(const char* track_id) {
    cJSON* object = cJSON_GetObjectItem(validCache, track_id);
    if (object) return object;
    
    object = cJSON_GetObjectItem(idleCache, track_id);
    if (object) return object;
    
    object = cJSON_GetObjectItem(nonValidCache, track_id);
    return object;
}

static double
calculate_distance(int x1, int y1, int x2, int y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

static void
build_detection_cache() {
    // Clear detection cache
    cJSON_Delete(detectionCache);
    detectionCache = cJSON_CreateArray();
    
    // Add all valid objects to detection cache
    cJSON* item = validCache->child;
    while (item) {
        cJSON* class_item = cJSON_GetObjectItem(item, "class");
        if (class_item && cJSON_IsString(class_item) ) {
            cJSON* object_copy = cJSON_Duplicate(item, 1);
            cJSON_AddItemToArray(detectionCache, object_copy);
        }
        item = item->next;
    }
    
    // NEW: Process idleCache for objects that need notification
    item = idleCache->child;
    while (item) {
        cJSON* needs_notification = cJSON_GetObjectItem(item, "needs_notification");
        if (needs_notification && cJSON_IsTrue(needs_notification)) {
            // Send this inactive object once
            cJSON* class_item = cJSON_GetObjectItem(item, "class");
            if (class_item && cJSON_IsString(class_item) ) {
                cJSON* object_copy = cJSON_Duplicate(item, 1);
                // Remove the internal flag from the copy sent to consumer
                cJSON_DeleteItemFromObject(object_copy, "needs_notification");
                cJSON_AddItemToArray(detectionCache, object_copy);
            }
            
            // Mark as sent (remove flag from original)
            cJSON_DeleteItemFromObject(item, "needs_notification");
        }
        item = item->next;
    }
    
    // Remove inactive objects from valid cache after adding to detection cache
    item = validCache->child;
    while (item) {
        cJSON* next = item->next;
        cJSON* active = cJSON_GetObjectItem(item, "active");
        if (active && !cJSON_IsTrue(active)) {
            // Remove inactive object - O(1) removal by name
            cJSON_DeleteItemFromObject(validCache, item->string);
        }
        item = next;
    }
    
    // Remove inactive objects from idle cache after adding to detection cache
    item = idleCache->child;
    while (item) {
        cJSON* next = item->next;
        cJSON* active = cJSON_GetObjectItem(item, "active");
        if (active && !cJSON_IsTrue(active)) {
            // Remove inactive object - O(1) removal by name
            cJSON_DeleteItemFromObject(idleCache, item->string);
        }
        item = next;
    }
}

static gboolean
timer_callback(gpointer user_data) {
    (void)user_data;
    
    g_mutex_lock(&cache_mutex);
    
    double current_time = ACAP_DEVICE_Timestamp();
    
    // Update age and idle time for all valid objects
    cJSON* item = validCache->child;
    while (item) {
        cJSON* next = item->next;
        
        cJSON* birth = cJSON_GetObjectItem(item, "birth");
        cJSON* age = cJSON_GetObjectItem(item, "age");
        cJSON* idle = cJSON_GetObjectItem(item, "idle");
        cJSON* last_update = cJSON_GetObjectItem(item, "last_update");
        
        if (birth && age) {
            double age_seconds = (current_time - birth->valuedouble) / 1000.0;
            cJSON_SetNumberValue(age, age_seconds);
        }
        
        // Update idle time if object hasn't moved
        if (idle && last_update) {
            double time_since_update = current_time - last_update->valuedouble;
            if (time_since_update > 1000) { // More than 1 second since last update
                cJSON_SetIntValue(idle, idle->valueint + 1000);
                
                // Check if object should be moved to idle cache
                if (config_maxIdle > 0 && idle->valueint >= config_maxIdle) {
                    const char* track_id = item->string;
                    move_to_idle_cache(track_id);
                    // Note: item is now invalid, but next was already set
                }
            }
        }
        
        item = next;
    }
    
    // Update age for all idle objects
    item = idleCache->child;
    while (item) {
        cJSON* birth = cJSON_GetObjectItem(item, "birth");
        cJSON* age = cJSON_GetObjectItem(item, "age");
        
        if (birth && age) {
            double age_seconds = (current_time - birth->valuedouble) / 1000.0;
            cJSON_SetNumberValue(age, age_seconds);
        }
        
        item = item->next;
    }
    
    // Log statistics
    int a = cJSON_GetArraySize(validCache);
    int b = cJSON_GetArraySize(idleCache);
    int c = cJSON_GetArraySize(nonValidCache);
    if (a > 20 || b > 20 || c > 20) {
        LOG_TRACE("%s: Cache stats - Valid: %d, Idle: %d, NonValid: %d, Processed: %d, Filtered: %d\n", 
                  __func__, a, b, c, stats_total_processed, stats_total_filtered);
    }
    
    g_mutex_unlock(&cache_mutex);
    
    return G_SOURCE_CONTINUE;
}

static void
MessageBroker_Subscription_Status(const mdb_error_t* error, void* user_data) {
    if (error != NULL) {
        LOG_WARN("%s: %s\n", __func__, error->message);
        return;
    }
}

int
ObjectDetection_Init(ObjectDetection_Callback callback) {
    if (callback == NULL) {
        LOG_WARN("%s: Invalid callback when configuring object detection\n", __func__);
        return 0;
    }
    
    if (ObjectDetection_UserCallback) {
        return 1;
    }
    
    ObjectDetection_UserCallback = callback;
    
    // Initialize mutex
    g_mutex_init(&cache_mutex);
    
    // Initialize three-cache system for different object states
    validCache = cJSON_CreateObject();      // Active, moving objects
    nonValidCache = cJSON_CreateObject();   // Objects that haven't passed initial validation
    idleCache = cJSON_CreateObject();       // Objects that exceeded maxIdle time but are still monitored
    detectionCache = cJSON_CreateArray();   // Output cache for consumers
    
    // Start timer for age/idle updates
    timer_id = g_timeout_add_seconds(1, timer_callback, NULL);
    
    channel_identifier_t channel_identifier = {
        .topic = "com.axis.analytics_scene_description.v0.beta",
        .source = "1"
    };
    
    mdb_connection_t* connection = mdb_connection_create(MessageBroker_Connection_Error, NULL, &error);
    if (error != NULL) {
        LOG_WARN("%s: Connection error. %s\n", __func__, error->message);
        mdb_error_destroy(&error);
        return 0;
    }
    
    subscriber_config = mdb_subscriber_config_create(channel_identifier.topic,
                                                     channel_identifier.source,
                                                     MessageBroker_Data,
                                                     &channel_identifier,
                                                     &error);
    if (error != NULL) {
        LOG_WARN("%s: Config error. %s\n", __func__, error->message);
        mdb_error_destroy(&error);
        mdb_connection_destroy(&connection);
        return 0;
    }
    
    subscriber = mdb_subscriber_create_async(connection,
                                             subscriber_config,
                                             MessageBroker_Subscription_Status,
                                             &channel_identifier,
                                             &error);
    
    if (error != NULL) {
        LOG_WARN("%s: Subscription error. %s\n", __func__, error->message);
        mdb_error_destroy(&error);
        mdb_connection_destroy(&connection);
        mdb_subscriber_config_destroy(&subscriber_config);
        return 0;
    }
    
    return 1;
}
