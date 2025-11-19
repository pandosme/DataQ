/*
 * main.c
 * ACAP Object Detection Application Main Entry Point
 * 
 * Integrates VOD, ObjectDetection, MQTT, and ACAP for real-time detection,
 * tracking, and event publishing.
 * 
 * Author: Fred Juhlin (2025)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <glib.h>
#include <time.h>
#include <glib-unix.h>
#include <signal.h>
#include <math.h>

#include "cJSON.h"
#include "ACAP.h"
#include "MQTT.h"
#include "ObjectDetection.h"
#include "GeoSpace.h"
#include "Stitch.h"

#define APP_PACKAGE "DataQ"

#define LOG(fmt, args...)      { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_WARN(fmt, args...) { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args); }
#define LOG_TRACE(fmt, args...) {}

cJSON* activeTrackers = 0;
cJSON* PreviousPosition = 0;
cJSON* lastPublishedTracker = 0;
cJSON* PathCache = 0;
cJSON* occupancyDetectionCounter = 0;
cJSON* classCounterArrays = 0;
cJSON* previousOccupancy = 0;
int lastDetectionListWasEmpty = 0;
int publishEvents = 1;
int publishDetections = 1;
int publishAnomaly = 0;
int publishTracker = 1;
int publishPath = 1;
int publishOccupancy = 0;
int publishStatus = 1;
int publishGeospace = 0;
int shouldReset = 0;

#define MAX_OCCUPANCY_HISTORY 64  // Adjust if your max burst rate is super high

typedef struct {
    double timestamp_ms; ///< in seconds (use e.g. gettimeofday or monotonic)
    cJSON* counter;   ///< cJSON object: { "Human":3, "Car":5, ... }
} OccupancySample;

OccupancySample occupancy_history[MAX_OCCUPANCY_HISTORY];
int occupancy_history_start = 0;
int occupancy_history_count = 0;


cJSON* ProcessPaths(cJSON* tracker) {
    if (!PathCache)
        PathCache = cJSON_CreateObject();
    
    const char* id = cJSON_GetObjectItem(tracker, "id") ? 
                     cJSON_GetObjectItem(tracker, "id")->valuestring : 0;
    if (!id) return 0;

    const char* class = cJSON_GetObjectItem(tracker, "class") ? 
                        cJSON_GetObjectItem(tracker, "class")->valuestring : 0;
    if (!class) return 0;

    int active = cJSON_GetObjectItem(tracker, "active") ? 
                 cJSON_GetObjectItem(tracker, "active")->type == cJSON_True : 0;

    int confidence = cJSON_GetObjectItem(tracker, "confidence") ? 
                     cJSON_GetObjectItem(tracker, "confidence")->valueint : 0;
    if (!confidence) return 0;

    double age = cJSON_GetObjectItem(tracker, "age") ? 
                 cJSON_GetObjectItem(tracker, "age")->valuedouble : 0;
    if (!age) return 0;

    double distance = cJSON_GetObjectItem(tracker, "distance") ? 
                      cJSON_GetObjectItem(tracker, "distance")->valuedouble : 0;
    if (!distance) return 0;

    // Get timestamps from tracker (passed from ObjectDetection.c)
    double currentTimestamp = cJSON_GetObjectItem(tracker, "timestamp") ? 
                              cJSON_GetObjectItem(tracker, "timestamp")->valuedouble : 0;
    
    double previousTimestamp = cJSON_GetObjectItem(tracker, "previousTimestamp") ? 
                               cJSON_GetObjectItem(tracker, "previousTimestamp")->valuedouble : currentTimestamp;

    cJSON* path = cJSON_GetObjectItem(PathCache, id);
    
    if (!path && active) {
        // ============================================================
        // NEW PATH CREATION - First time seeing this tracker
        // ============================================================
        path = cJSON_CreateObject();
        cJSON_AddStringToObject(path, "class", class);
        cJSON_AddNumberToObject(path, "confidence", confidence);
        cJSON_AddNumberToObject(path, "age", age);
        cJSON_AddNumberToObject(path, "distance", distance);
        
        if (cJSON_GetObjectItem(tracker, "color"))
            cJSON_AddStringToObject(path, "color", cJSON_GetObjectItem(tracker, "color")->valuestring);
        if (cJSON_GetObjectItem(tracker, "color2"))
            cJSON_AddStringToObject(path, "color2", cJSON_GetObjectItem(tracker, "color2")->valuestring);
        
        cJSON_AddNumberToObject(path, "dx", cJSON_GetObjectItem(tracker, "dx")->valuedouble);
        cJSON_AddNumberToObject(path, "dy", cJSON_GetObjectItem(tracker, "dy")->valuedouble);
        cJSON_AddNumberToObject(path, "bx", cJSON_GetObjectItem(tracker, "bx")->valuedouble);
        cJSON_AddNumberToObject(path, "by", cJSON_GetObjectItem(tracker, "by")->valuedouble);
        
        double birthTime = cJSON_GetObjectItem(tracker, "birth") ? 
                          cJSON_GetObjectItem(tracker, "birth")->valuedouble : currentTimestamp;
        cJSON_AddNumberToObject(path, "timestamp", currentTimestamp);
        cJSON_AddNumberToObject(path, "dwell", 0);
        cJSON_AddStringToObject(path, "id", id);

        cJSON* face = cJSON_GetObjectItem(tracker, "face");
        if (face) cJSON_AddBoolToObject(path, "face", cJSON_IsTrue(face));
        cJSON* hat = cJSON_GetObjectItem(tracker, "hat");
        if (hat) cJSON_AddStringToObject(path, "hat", hat->valuestring);

        double lat = cJSON_GetObjectItem(tracker, "lat") ? 
                     cJSON_GetObjectItem(tracker, "lat")->valuedouble : 0;
        double lon = cJSON_GetObjectItem(tracker, "lon") ? 
                     cJSON_GetObjectItem(tracker, "lon")->valuedouble : 0;
        double blat = 0, blon = 0;
        if (lat && lon)
            GeoSpace_transform(cJSON_GetObjectItem(tracker, "bx")->valueint, 
                             cJSON_GetObjectItem(tracker, "by")->valueint, &blat, &blon);

        cJSON* pathArr = cJSON_CreateArray();
        
        // Position 0: Birth position (bx, by)
        cJSON* pos1 = cJSON_CreateObject();
        cJSON_AddNumberToObject(pos1, "x", cJSON_GetObjectItem(tracker, "bx")->valuedouble);
        cJSON_AddNumberToObject(pos1, "y", cJSON_GetObjectItem(tracker, "by")->valuedouble);
        cJSON_AddNumberToObject(pos1, "d", 0);  // Will be updated on first tracker update
        if (blat && blon) {
            cJSON_AddNumberToObject(pos1, "lat", blat);
            cJSON_AddNumberToObject(pos1, "lon", blon);
        }
        cJSON_AddItemToArray(pathArr, pos1);

        // Position 1: Current position (cx, cy)
        cJSON* pos2 = cJSON_CreateObject();
        cJSON_AddNumberToObject(pos2, "x", cJSON_GetObjectItem(tracker, "cx")->valuedouble);
        cJSON_AddNumberToObject(pos2, "y", cJSON_GetObjectItem(tracker, "cy")->valuedouble);
        cJSON_AddNumberToObject(pos2, "d", 0);  // Will be updated on next tracker update
        if (lat && lon) {
            cJSON_AddNumberToObject(pos2, "lat", lat);
            cJSON_AddNumberToObject(pos2, "lon", lon);
        }
        cJSON_AddItemToArray(pathArr, pos2);

        cJSON_AddItemToObject(path, "path", pathArr);
        cJSON_AddItemToObject(PathCache, id, path);
        
        // NO PreviousTimestamp cache operations needed anymore!
        
        return 0;
    }
    
    if (path && active) {
        // ============================================================
        // UPDATE EXISTING PATH - Tracker still active
        // ============================================================
        cJSON_ReplaceItemInObject(path, "class", cJSON_Duplicate(cJSON_GetObjectItem(tracker, "class"), 1));
        cJSON_ReplaceItemInObject(path, "confidence", cJSON_Duplicate(cJSON_GetObjectItem(tracker, "confidence"), 1));
        cJSON_ReplaceItemInObject(path, "age", cJSON_Duplicate(cJSON_GetObjectItem(tracker, "age"), 1));
        cJSON_ReplaceItemInObject(path, "distance", cJSON_Duplicate(cJSON_GetObjectItem(tracker, "distance"), 1));
        
        if (cJSON_GetObjectItem(tracker, "color"))
            cJSON_ReplaceItemInObject(path, "color", cJSON_Duplicate(cJSON_GetObjectItem(tracker, "color"), 1));
        if (cJSON_GetObjectItem(tracker, "color2"))
            cJSON_ReplaceItemInObject(path, "color2", cJSON_Duplicate(cJSON_GetObjectItem(tracker, "color2"), 1));
        
        cJSON_ReplaceItemInObject(path, "dx", cJSON_Duplicate(cJSON_GetObjectItem(tracker, "dx"), 1));
        cJSON_ReplaceItemInObject(path, "dy", cJSON_Duplicate(cJSON_GetObjectItem(tracker, "dy"), 1));
        cJSON_ReplaceItemInObject(path, "timestamp", cJSON_CreateNumber(currentTimestamp));

        cJSON* face = cJSON_GetObjectItem(tracker, "face");
        if (face) cJSON_ReplaceItemInObject(path, "face", cJSON_Duplicate(face, 1));
        cJSON* hat = cJSON_GetObjectItem(tracker, "hat");
        if (hat) cJSON_ReplaceItemInObject(path, "hat", cJSON_Duplicate(hat, 1));

        double idle = cJSON_GetObjectItem(tracker, "idle")->valuedouble;
        cJSON* dwell_item = cJSON_GetObjectItem(path, "dwell");
        if (idle > dwell_item->valuedouble)
            cJSON_SetNumberValue(dwell_item, idle);

        cJSON* pathArr = cJSON_GetObjectItem(path, "path");
        int pathLen = cJSON_GetArraySize(pathArr);
        
        if (pathLen >= 2) {
            // FIX #1: Update SECOND-TO-LAST position's duration
            // This is position[pathLen-2], which represents where the object WAS
            cJSON* secondToLast = cJSON_GetArrayItem(pathArr, pathLen - 2);
            
            // Calculate duration: time object spent at that position
            double duration = (currentTimestamp - previousTimestamp) / 1000.0; // Convert ms to seconds
            cJSON_ReplaceItemInObject(secondToLast, "d", cJSON_CreateNumber(duration));
        }

        // Add NEW position with d=0 (will be calculated on next update or exit)
        cJSON* pos = cJSON_CreateObject();
        cJSON_AddNumberToObject(pos, "x", cJSON_GetObjectItem(tracker, "cx")->valuedouble);
        cJSON_AddNumberToObject(pos, "y", cJSON_GetObjectItem(tracker, "cy")->valuedouble);
        cJSON_AddNumberToObject(pos, "d", 0);  // Always 0 for newest position
        
        double lat = cJSON_GetObjectItem(tracker, "lat") ? 
                     cJSON_GetObjectItem(tracker, "lat")->valuedouble : 0;
        double lon = cJSON_GetObjectItem(tracker, "lon") ? 
                     cJSON_GetObjectItem(tracker, "lon")->valuedouble : 0;
        if (lat && lon) {
            cJSON_AddNumberToObject(pos, "lat", lat);
            cJSON_AddNumberToObject(pos, "lon", lon);
        }
        cJSON_AddItemToArray(pathArr, pos);
        
        // NO PreviousTimestamp cache operations needed anymore!
        
        return 0;
    }
    
    if (path && !active) {
        // ============================================================
        // FINALIZE PATH - Object exited scene
        // ============================================================
        cJSON* pathArr = cJSON_GetObjectItem(path, "path");
        int pathLen = cJSON_GetArraySize(pathArr);
        
        if (pathLen > 1) {
            // FIX #1: Calculate LAST position's duration
            // This is the final position where object was when it exited
            cJSON* last = cJSON_GetArrayItem(pathArr, pathLen - 1);
            double finalDuration = (currentTimestamp - previousTimestamp) / 1000.0; // Convert ms to seconds
            cJSON_ReplaceItemInObject(last, "d", cJSON_CreateNumber(finalDuration));
            
            cJSON_DetachItemFromObject(PathCache, id);
            
            // FIX #2: Update final age and distance from tracker
            // Ensures path properties match final tracker state
            cJSON_ReplaceItemInObject(path, "age", cJSON_CreateNumber(age));
            cJSON_ReplaceItemInObject(path, "distance", cJSON_CreateNumber(distance));
            
            // Update other final metadata
            cJSON_AddNumberToObject(path, "directions", cJSON_GetObjectItem(tracker, "directions")->valueint);
            if (cJSON_GetObjectItem(tracker, "anomaly"))
                cJSON_AddStringToObject(path, "anomaly", cJSON_GetObjectItem(tracker, "anomaly")->valuestring);
            if (cJSON_GetObjectItem(tracker, "maxSpeed"))
                cJSON_AddNumberToObject(path, "maxSpeed", cJSON_GetObjectItem(tracker, "maxSpeed")->valuedouble);
            
            cJSON_ReplaceItemInObject(path, "timestamp", cJSON_CreateNumber(currentTimestamp));
            
            // NO PreviousTimestamp cache cleanup needed anymore!
            
            return path;
        }
        
        cJSON_DetachItemFromObject(PathCache, id);
        // NO PreviousTimestamp cache cleanup needed anymore!
        cJSON_Delete(path);
        return 0;
    }
    
    return 0;
}


static guint anomaly_timeout_id = 0;

static gboolean
Clear_Anomaly(gpointer user_data) {
    ACAP_EVENTS_Fire_State("anomaly", 0);
    anomaly_timeout_id = 0; // Reset timeout id
    return FALSE; // Do not reschedule
}

void
Fire_Anomaly() {
    ACAP_EVENTS_Fire_State("anomaly", 1);

    // Cancel any pending timeout
    if (anomaly_timeout_id != 0) {
        g_source_remove(anomaly_timeout_id);
        anomaly_timeout_id = 0;
    }
    // Set a new 2-second timeout (2000 ms)
    anomaly_timeout_id = g_timeout_add(4000, Clear_Anomaly, NULL);
}


void
Check_Anomaly(cJSON* tracker) {

    int is_human = 0, is_vehicle = 0;
    const char* label = cJSON_GetObjectItem(tracker, "class")->valuestring;
    if (strcmp("Human", label) == 0) is_human = 1;
    // If vehicle-like class
    if (strcmp("Car", label) == 0 || strcmp("Truck", label) == 0 ||
        strcmp("Bus", label) == 0 || strcmp("Bike", label) == 0 ||
        strcmp("Other", label) == 0 || strcmp("Veicle", label) == 0)
        is_vehicle = 1;

    // Save stats
    if (cJSON_GetObjectItem(tracker,"active")->type == cJSON_False) {
        char* group_label = is_human ? "humans" : "vehicles";
        cJSON* stats;

        stats = ACAP_STATUS_Object(group_label, "directions");
        if(!stats) stats = cJSON_CreateArray();
        while( cJSON_GetArraySize(stats) > 200 )
            cJSON_DeleteItemFromArray(stats, 0);
        cJSON_AddItemToArray(stats, cJSON_CreateNumber(cJSON_GetObjectItem(tracker, "directions")->valueint));
        ACAP_STATUS_SetObject(group_label, "directions", stats);

        stats = ACAP_STATUS_Object(group_label, "age");
        if(!stats) stats = cJSON_CreateArray();
        cJSON_AddItemToArray(stats, cJSON_CreateNumber(cJSON_GetObjectItem(tracker, "age")->valuedouble));
        while( cJSON_GetArraySize(stats) > 200 )
            cJSON_DeleteItemFromArray(stats, 0);
        ACAP_STATUS_SetObject(group_label, "age", stats);

        stats = ACAP_STATUS_Object(group_label, "idle");
        if(!stats) stats = cJSON_CreateArray();
        cJSON_AddItemToArray(stats, cJSON_CreateNumber(cJSON_GetObjectItem(tracker, "maxIdle")->valuedouble));
        while( cJSON_GetArraySize(stats) > 200 )
            cJSON_DeleteItemFromArray(stats, 0);
        ACAP_STATUS_SetObject(group_label, "idle", stats);

        stats = ACAP_STATUS_Object(group_label, "speed");
        if(!stats) stats = cJSON_CreateArray();
        double speed = cJSON_GetObjectItem(tracker, "maxSpeed")->valuedouble;
        if (speed > 0) {
            cJSON_AddItemToArray(stats, cJSON_CreateNumber(speed));
            while( cJSON_GetArraySize(stats) > 200 )
                cJSON_DeleteItemFromArray(stats, 0);
            ACAP_STATUS_SetObject(group_label, "speed", stats);
        }

        stats = ACAP_STATUS_Object(group_label, "horizontal");
        if(!stats) stats = cJSON_CreateArray();
        int dx = cJSON_GetObjectItem(tracker, "dx")->valueint;
        cJSON_AddItemToArray(stats, cJSON_CreateNumber(dx));
        while( cJSON_GetArraySize(stats) > 200 )
            cJSON_DeleteItemFromArray(stats, 0);
        ACAP_STATUS_SetObject(group_label, "horizontal", stats);

        stats = ACAP_STATUS_Object(group_label, "vertical");
        if(!stats) stats = cJSON_CreateArray();
        int dy = cJSON_GetObjectItem(tracker, "dy")->valueint;
        cJSON_AddItemToArray(stats, cJSON_CreateNumber(dy));
        while( cJSON_GetArraySize(stats) > 200 )
            cJSON_DeleteItemFromArray(stats, 0);
        ACAP_STATUS_SetObject(group_label, "vertical", stats);
    }

	if(!publishAnomaly) return;
    cJSON* settings = ACAP_Get_Config("settings");
    if (!settings) return;
    cJSON* anomaly = cJSON_GetObjectItem(settings, "anomaly");
    if (!anomaly) return;
    if (!anomaly->child) return;
    cJSON* group = NULL;
    if (is_human)
		group = cJSON_GetObjectItem(anomaly, "humans");
    if (is_vehicle)
		group = cJSON_GetObjectItem(anomaly, "vehicles");
    if (!group) return;


    // Extract area arrays for the current group
    cJSON* common = cJSON_GetObjectItem(group, "common");
    cJSON* restricted = cJSON_GetObjectItem(group, "restricted");

    int cx = cJSON_GetObjectItem(tracker, "cx")->valueint;
    int cy = cJSON_GetObjectItem(tracker, "cy")->valueint;
    int bx = cJSON_GetObjectItem(tracker, "bx")->valueint;
    int by = cJSON_GetObjectItem(tracker, "by")->valueint;

    // AREA VALIDATION

    // Check common entry/exit (use "bx/by" and "cx/cy" as your logic needs, typically both should match one common area)
    int found_common = 0;
    if (common && cJSON_GetArraySize(common)) {
        cJSON* item = common->child;
        while(item && !found_common) {
            int x1 = cJSON_GetObjectItem(item,"x1")->valueint;
            int x2 = cJSON_GetObjectItem(item,"x2")->valueint;
            int y1 = cJSON_GetObjectItem(item,"y1")->valueint;
            int y2 = cJSON_GetObjectItem(item,"y2")->valueint;
            if (bx > x1 && bx < x2 && by > y1 && by < y2) {
                found_common = 1;
            }
            item = item->next;
        }
        if (!found_common) {
            LOG("Invalid entry");
            Fire_Anomaly();
            cJSON_AddStringToObject(tracker, "anomaly", "Invalid entry");
            return;
        }
    }

    found_common = 0;
    if (common && cJSON_GetArraySize(common) && cJSON_GetObjectItem(tracker,"active")->type == cJSON_False) {
        cJSON* item = common->child;
        while(item && !found_common) {
            int x1 = cJSON_GetObjectItem(item,"x1")->valueint;
            int x2 = cJSON_GetObjectItem(item,"x2")->valueint;
            int y1 = cJSON_GetObjectItem(item,"y1")->valueint;
            int y2 = cJSON_GetObjectItem(item,"y2")->valueint;
            if (cx > x1 && cx < x2 && cy > y1 && cy < y2) {
                found_common = 1;
            }
            item = item->next;
        }
        if (!found_common) {
            //LOG("Invalid exit");
            Fire_Anomaly();
            cJSON_AddStringToObject(tracker, "anomaly", "Invalid exit");
            return;
        }
    }

    // Check restricted area (using "cx/cy" typically)
    int found_restricted = 0;
    if (restricted && cJSON_GetArraySize(restricted)) {
        cJSON* item = restricted->child;
		if( cJSON_GetObjectItem(item,"distance")->valueint < 20 )
			item = 0;
        while(item && !found_restricted) {
            int x1 = cJSON_GetObjectItem(item,"x1")->valueint;
            int x2 = cJSON_GetObjectItem(item,"x2")->valueint;
            int y1 = cJSON_GetObjectItem(item,"y1")->valueint;
            int y2 = cJSON_GetObjectItem(item,"y2")->valueint;
            if (cx > x1 && cx < x2 && cy > y1 && cy < y2) {
                found_restricted = 1;
            }
            item = item->next;
        }
        if (!found_restricted) {
            Fire_Anomaly();
            cJSON_AddStringToObject(tracker, "anomaly", "Restricted Area");
            return;
        }
    }

    // GET settings block for the group
    cJSON* normal = cJSON_GetObjectItem(group, "settings");
    if (!normal) return;
	char text[64];

    int maxDirections = cJSON_GetObjectItem(normal, "directions") ? cJSON_GetObjectItem(normal, "directions")->valueint : 0;
	int directions = cJSON_GetObjectItem(tracker, "directions")->valueint;
    if (maxDirections && directions > maxDirections) {
		sprintf(text,"Directions: %d > %d", directions , maxDirections);
		//LOG("%s",text);
        cJSON_AddStringToObject(tracker, "anomaly", text);
        Fire_Anomaly();
        return;
    }

    float maxAge = cJSON_GetObjectItem(normal, "age") ? cJSON_GetObjectItem(normal, "age")->valuedouble : 0;
    float age = cJSON_GetObjectItem(tracker, "age")->valuedouble;
    if (maxAge && age > maxAge) {
		sprintf(text,"Age: %d>%d", (int)age , (int)maxAge);
		//LOG("%s",text);		
        cJSON_AddStringToObject(tracker, "anomaly", text);
        Fire_Anomaly();
        return;
    }

    float maxIdle = cJSON_GetObjectItem(normal, "idle") ? cJSON_GetObjectItem(normal, "idle")->valuedouble : 0;
    float idle = cJSON_GetObjectItem(tracker, "maxIdle")->valuedouble;
    if (maxIdle && idle > maxIdle) {
		sprintf(text,"Idle: %d>%d", (int)idle , (int)maxIdle);
		//LOG("%s",text);
        cJSON_AddStringToObject(tracker, "anomaly", text);
        Fire_Anomaly();
        return;
    }

    float speedLimit = cJSON_GetObjectItem(normal, "maxSpeed") ? cJSON_GetObjectItem(normal, "maxSpeed")->valuedouble : 0;
    float maxSpeed = cJSON_GetObjectItem(tracker, "maxSpeed")->valuedouble;
    if (speedLimit && maxSpeed > speedLimit) {
		sprintf(text,"Speed: %d>%d", (int)maxSpeed , (int)speedLimit);
		//LOG("%s",text);
        cJSON_AddStringToObject(tracker, "anomaly", text);
        Fire_Anomaly();
        return;
    }

    // Direction checks (horizontal/vertical)
    char* horizontal = cJSON_GetObjectItem(normal, "horizontal") ? cJSON_GetObjectItem(normal, "horizontal")->valuestring : NULL;
    int dx = cJSON_GetObjectItem(tracker, "dx") ? cJSON_GetObjectItem(tracker, "dx")->valueint : 0;
    if (horizontal && strcmp(horizontal, "Left") == 0 && dx > 0) {
		//LOG("%s",text);
        Fire_Anomaly();
        cJSON_AddStringToObject(tracker, "anomaly", "Wrong way");
        return;
    }
    if (horizontal && strcmp(horizontal, "Right") == 0 && dx < 0) {
		//LOG("%s",text);
        Fire_Anomaly();
        cJSON_AddStringToObject(tracker, "anomaly", "Wrong way");
        return;
    }

    char* vertical = cJSON_GetObjectItem(normal, "vertical") ? cJSON_GetObjectItem(normal, "vertical")->valuestring : NULL;
    int dy = cJSON_GetObjectItem(tracker, "dy") ? cJSON_GetObjectItem(tracker, "dy")->valueint : 0;
    if (vertical && strcmp(vertical, "Up") == 0 && dy > 0) {
        //LOG("Wrong way: Down");
        Fire_Anomaly();
        cJSON_AddStringToObject(tracker, "anomaly", "Wrong way");
        return;
    }
    if (vertical && strcmp(vertical, "Down") == 0 && dy < 0) {
        //LOG("Wrong way: Up");
        Fire_Anomaly();
        cJSON_AddStringToObject(tracker, "anomaly", "Wrong way");
        return;
    }
}

void Tracker_Data(cJSON *tracker, int timer) {
    char topic[128];

	Check_Anomaly( tracker );

    if (publishPath && !timer && tracker)
		STICH_Path(ProcessPaths(tracker));

    cJSON_DeleteItemFromObject(tracker,"previousTimestamp");

    if (publishTracker) {
        sprintf(topic, "tracker/%s", ACAP_DEVICE_Prop("serial"));
        MQTT_Publish_JSON(topic, tracker, 0, 0);
    }

    if (publishGeospace && ACAP_STATUS_Bool("geospace", "active")) {
        double lat = 0, lon = 0;
        GeoSpace_transform(cJSON_GetObjectItem(tracker, "cx")->valueint, cJSON_GetObjectItem(tracker, "cy")->valueint, &lat, &lon);
        if (lat && lon) {
            cJSON_AddNumberToObject(tracker, "lat", lat);
            cJSON_AddNumberToObject(tracker, "lon", lon);
            cJSON* geospaceObject = cJSON_CreateObject();
            cJSON_AddStringToObject(geospaceObject, "class", cJSON_GetObjectItem(tracker, "class")->valuestring);
            cJSON_AddNumberToObject(geospaceObject, "lat", lat);
            cJSON_AddNumberToObject(geospaceObject, "lon", lon);
            cJSON_AddNumberToObject(geospaceObject, "age", cJSON_GetObjectItem(tracker, "age")->valuedouble);
            cJSON_AddNumberToObject(geospaceObject, "idle", cJSON_GetObjectItem(tracker, "idle")->valuedouble);
            cJSON_AddStringToObject(geospaceObject, "id", cJSON_GetObjectItem(tracker, "id")->valuestring);
			cJSON_AddItemToObject(geospaceObject, "active", cJSON_Duplicate(cJSON_GetObjectItem(tracker,"active"), 1));
            sprintf(topic, "geospace/%s", ACAP_DEVICE_Prop("serial"));
            MQTT_Publish_JSON(topic, geospaceObject, 0, 0);
            cJSON_Delete(geospaceObject);
        }
    }

    cJSON_Delete(tracker);
}

void Publish_Path( cJSON* path ){
	if( !path ) return;
    char topic[128];
	sprintf(topic, "path/%s", ACAP_DEVICE_Prop("serial"));
	MQTT_Publish_JSON(topic, path, 0, 0);
	
    cJSON* statusPaths = ACAP_STATUS_Object("detections", "paths");
	
    cJSON_AddItemToArray(statusPaths, cJSON_Duplicate(path, 1));
    while (cJSON_GetArraySize(statusPaths) > 10)
        cJSON_DeleteItemFromArray(statusPaths, 0);
	cJSON_Delete(path);
}

static cJSON* last_occupancy = NULL;

int Check_Counters_Equal(cJSON* counter1, cJSON* counter2) {
    if (counter1 == NULL && counter2 == NULL)
        return 1;
    if (counter1 == NULL || counter2 == NULL)
        return 0;
    if (cJSON_GetArraySize(counter1) != cJSON_GetArraySize(counter2))
        return 0;
    cJSON *item = counter1->child;
    while (item) {
        cJSON* other = cJSON_GetObjectItem(counter2, item->string);
        if (!other)
            return 0;
        if (item->valuedouble != other->valuedouble)
            return 0;
        item = item->next;
    }
    item = counter2->child;
    while (item) {
        cJSON* other = cJSON_GetObjectItem(counter1, item->string);
        if (!other)
            return 0;
        if (item->valuedouble != other->valuedouble)
            return 0;
        item = item->next;
    }
    return 1;
}

static void push_occupancy_sample(double timestamp_ms, cJSON* counter) {
    // Remove oldest if buffer is full
    if (occupancy_history_count == MAX_OCCUPANCY_HISTORY) {
        cJSON_Delete(occupancy_history[occupancy_history_start].counter);
        occupancy_history_start = (occupancy_history_start + 1) % MAX_OCCUPANCY_HISTORY;
        occupancy_history_count--;
    }
    int idx = (occupancy_history_start + occupancy_history_count) % MAX_OCCUPANCY_HISTORY;
    occupancy_history[idx].timestamp_ms = timestamp_ms;
    occupancy_history[idx].counter = cJSON_Duplicate(counter, 1);
    occupancy_history_count++;
}

static void push_occupancy_zero_sample(double timestamp_ms) {
    cJSON* empty = cJSON_CreateObject();
    push_occupancy_sample(timestamp_ms, empty);
    cJSON_Delete(empty);
}

cJSON*
compute_stabilized_occupancy(double now, double integration_time) {
    // Remove outdated entries
    while (occupancy_history_count > 0 &&
           occupancy_history[occupancy_history_start].timestamp_ms < now - integration_time) {
        cJSON_Delete(occupancy_history[occupancy_history_start].counter);
        occupancy_history_start = (occupancy_history_start + 1) % MAX_OCCUPANCY_HISTORY;
        occupancy_history_count--;
    }
    // Aggregate all counters
    cJSON* result = cJSON_CreateObject();
    int n = 0;
    for (int i = 0, idx = occupancy_history_start; i < occupancy_history_count; ++i, idx = (idx + 1) % MAX_OCCUPANCY_HISTORY) {
        cJSON* counter = occupancy_history[idx].counter;
        cJSON* item = counter->child;
        while (item) {
            cJSON* sum = cJSON_GetObjectItem(result, item->string);
            if (sum) {
                sum->valuedouble += item->valuedouble;
                sum->valueint = (int)sum->valuedouble;
            } else {
                cJSON_AddNumberToObject(result, item->string, item->valuedouble);
            }
            item = item->next;
        }
        n++;
    }
    // Average for each class
    cJSON* item = result->child;
    while (item) {
        if (n > 0) item->valuedouble /= n;
        item->valueint = (int)(item->valuedouble + 0.5); // Round
        item = item->next;
    }
    return result;
}

cJSON* ProcessOccupancy(cJSON* list) {
    cJSON* settings = ACAP_Get_Config("settings");
    if (!settings)
        return 0;
    cJSON* occupancy = cJSON_GetObjectItem(settings, "occupancy");
    if (!occupancy)
        return 0;
    int stationary = cJSON_GetObjectItem(occupancy, "stationary") ? cJSON_GetObjectItem(occupancy, "stationary")->type == cJSON_True : 0;
    int moving = cJSON_GetObjectItem(occupancy, "moving") ? cJSON_GetObjectItem(occupancy, "moving")->type == cJSON_True : 0;
    double ageThreshold = cJSON_GetObjectItem(occupancy, "ageThreshold") ? cJSON_GetObjectItem(occupancy, "ageThreshold")->valuedouble : 2.0;
    double idleThreshold = cJSON_GetObjectItem(occupancy, "idleThreshold") ? cJSON_GetObjectItem(occupancy, "idleThreshold")->valuedouble : 3.0;
    int listSize = cJSON_GetArraySize(list);

    // Handle empty list: only broadcast on change (normalized empty object)
    if (listSize == 0) {
        cJSON* empty = cJSON_CreateObject();
        int changed = 1;
        if (last_occupancy && Check_Counters_Equal(last_occupancy, empty))
            changed = 0;
        if (!changed) {
            cJSON_Delete(empty);
            return 0;
        }
        // Only add to history and report on change
        double now = ACAP_DEVICE_Timestamp();
        push_occupancy_zero_sample(now);
        if (last_occupancy)
            cJSON_Delete(last_occupancy);
        last_occupancy = cJSON_Duplicate(empty, 1);
        return empty;
    }

    // Build counters object (class counts)
    cJSON* counters = cJSON_CreateObject();
    for (int i = 0; i < listSize; ++i) {
        cJSON* det = cJSON_GetArrayItem(list, i);
        const char* cls = cJSON_GetObjectItem(det, "class")->valuestring;
        double age = cJSON_GetObjectItem(det, "age")->valuedouble;
        double idle = cJSON_GetObjectItem(det, "idle")->valuedouble;
        int is_moving = (age >= ageThreshold) && (idle < idleThreshold);
        int is_stationary = (age >= ageThreshold) && (idle >= idleThreshold);
        if ((moving && is_moving) || (stationary && is_stationary)) {
            cJSON* curr = cJSON_GetObjectItem(counters, cls);
            if (curr) {
                curr->valuedouble += 1.0;
            } else {
                cJSON_AddNumberToObject(counters, cls, 1.0);
            }
        }
    }

    // Normalize and round counters before comparison
    cJSON* item = counters->child;
    while (item) {
        item->valueint = (int)(item->valuedouble + 0.5);
        item->valuedouble = (double)item->valueint;
        item = item->next;
    }

    // Remove zero-count classes
    item = counters->child;
    while (item) {
        cJSON* next = item->next;
        if (item->valueint == 0)
            cJSON_DeleteItemFromObjectCaseSensitive(counters, item->string);
        item = next;
    }

    // Compare with the last reported occupancy before history/report
    if (last_occupancy && Check_Counters_Equal(last_occupancy, counters)) {
        cJSON_Delete(counters); // Clean up if not sending
        return 0;
    }

    // Only now do we push to history and report
    double now = ACAP_DEVICE_Timestamp();
    push_occupancy_sample(now, counters);

    if (last_occupancy)
        cJSON_Delete(last_occupancy);
    last_occupancy = cJSON_Duplicate(counters, 1);
    ACAP_STATUS_SetObject("occupancy", "counter", counters);

    return counters;
}

int lasty_detections_was_empty = 0;

void Detections_Data(cJSON *list) {
    char topic[128];
    if (publishDetections) {
        if (cJSON_GetArraySize(list) > 0 || !lasty_detections_was_empty) {
            sprintf(topic, "detections/%s", ACAP_DEVICE_Prop("serial"));
            cJSON* payload = cJSON_CreateObject();
            cJSON_AddItemReferenceToObject(payload, "list", list);
            MQTT_Publish_JSON(topic, payload, 0, 0);
            cJSON_Delete(payload);
        }
    }
    lasty_detections_was_empty = cJSON_GetArraySize(list) == 0;

    if (publishOccupancy) {
        // Process actual occupancy, update buffers
        cJSON* counter = ProcessOccupancy(list);
		if(counter ) {
			double now = ACAP_DEVICE_Timestamp();

			// Fetch settings (allow fallback if not found)
			cJSON* settings = ACAP_Get_Config("settings");
			cJSON* occupancy = settings ? cJSON_GetObjectItem(settings, "occupancy") : NULL;
			double integrationTime = 2.0; // Default integration window
			if (occupancy && cJSON_GetObjectItem(occupancy, "integrationTime")) {
				integrationTime = cJSON_GetObjectItem(occupancy, "integrationTime")->valuedouble;
			}

			// Compute stabilized output
			cJSON* stabilized = compute_stabilized_occupancy(now, integrationTime);
			if (stabilized) {
				cJSON* payload = cJSON_CreateObject();
				cJSON_AddItemToObject(payload, "occupancy", stabilized);
				cJSON_AddNumberToObject(payload, "timestamp", now);
				sprintf(topic, "occupancy/%s", ACAP_DEVICE_Prop("serial"));
				MQTT_Publish_JSON(topic, payload, 0, 0);
				cJSON_Delete(payload);
			}
			cJSON_Delete(counter); // Unstabilized, only for transition/compat use
		}
    }
    cJSON_Delete(list);
}

void Event_Callback(cJSON *event, void* userdata) {
    if (!event)
        return;

    cJSON* settings = ACAP_Get_Config("settings");
    if (!settings)
        return;
    cJSON* publish = cJSON_GetObjectItem(settings, "publish");
    if (!publish)
        return;

    if (!cJSON_GetObjectItem(publish, "events") || cJSON_GetObjectItem(publish, "events")->type != cJSON_True)
        return;

    cJSON* eventTopic = cJSON_DetachItemFromObject(event, "event");
    if (!eventTopic)
        return;

    int ignore = 0;
    if (!ignore && strstr(eventTopic->valuestring, "HardwareFailure")) ignore = 1;
    if (!ignore && strstr(eventTopic->valuestring, "SystemReady")) ignore = 1;
    if (!ignore && strstr(eventTopic->valuestring, "ClientStatus")) ignore = 1;
    if (!ignore && strstr(eventTopic->valuestring, "RingPowerLimitExceeded")) ignore = 1;
    if (!ignore && strstr(eventTopic->valuestring, "PTZPowerFailure")) ignore = 1;
    if (!ignore && strstr(eventTopic->valuestring, "SystemInitializing")) ignore = 1;
    if (!ignore && strstr(eventTopic->valuestring, "Network")) ignore = 1;
    if (!ignore && strstr(eventTopic->valuestring, "xinternal_data")) ignore = 1;
    if (!ignore && strstr(eventTopic->valuestring, "xinternal_data")) ignore = 1;	
    if (ignore) {
        cJSON_Delete(eventTopic);
        return;
    }

    cJSON* eventFilter = cJSON_GetObjectItem(settings, "eventTopics") ? cJSON_GetObjectItem(settings, "eventTopics")->child : 0;
    while (eventFilter) {
        if (cJSON_GetObjectItem(eventFilter, "enabled")->type == cJSON_False) {
            const char* ignoreTopic = cJSON_GetObjectItem(eventFilter, "topic") ? cJSON_GetObjectItem(eventFilter, "topic")->valuestring : 0;
            if (ignoreTopic && strstr(eventTopic->valuestring, ignoreTopic)) {
                cJSON_Delete(eventTopic);
                return;
            }
        }
        eventFilter = eventFilter->next;
    }

    char topic[256];
    sprintf(topic, "event/%s/%s", ACAP_DEVICE_Prop("serial"), eventTopic->valuestring);
    cJSON_Delete(eventTopic);
    MQTT_Publish_JSON(topic, event, 0, 0);
}

static gboolean MQTT_Publish_Device_Status(gpointer user_data) {
    if (!publishStatus)
        return G_SOURCE_CONTINUE;

    cJSON* payload = cJSON_CreateObject();
    cJSON_AddNumberToObject(payload, "Network_Kbps", (int)ACAP_DEVICE_Network_Average());
    cJSON_AddNumberToObject(payload, "CPU_average", (int)(ACAP_DEVICE_CPU_Average() * 100));
    cJSON_AddNumberToObject(payload, "Uptime_Hours", (int)(ACAP_DEVICE_Uptime() / 3600));

    char topic[256];
    sprintf(topic, "status/%s", ACAP_DEVICE_Prop("serial"));
    MQTT_Publish_JSON(topic, payload, 0, 0);
    cJSON_Delete(payload);

    return G_SOURCE_CONTINUE;
}

void Main_MQTT_Status(int state) {
    char topic[64];
    cJSON* message = 0;

    switch (state) {
        case MQTT_INITIALIZING:
            LOG("%s: Initializing\n", __func__);
            break;
        case MQTT_CONNECTING:
            LOG("%s: Connecting\n", __func__);
            break;
        case MQTT_CONNECTED:
            LOG("%s: Connected\n", __func__);
            sprintf(topic, "connect/%s", ACAP_DEVICE_Prop("serial"));
            message = cJSON_CreateObject();
            cJSON_AddTrueToObject(message, "connected");
            cJSON_AddStringToObject(message, "address", ACAP_DEVICE_Prop("IPv4"));
            MQTT_Publish_JSON(topic, message, 0, 1);
            cJSON_Delete(message);
			MQTT_Publish_Device_Status(0);
            break;
        case MQTT_DISCONNECTING:
            sprintf(topic, "connect/%s", ACAP_DEVICE_Prop("serial"));
            message = cJSON_CreateObject();
            cJSON_AddFalseToObject(message, "connected");
            cJSON_AddStringToObject(message, "address", ACAP_DEVICE_Prop("IPv4"));
            MQTT_Publish_JSON(topic, message, 0, 1);
            cJSON_Delete(message);
            break;
        case MQTT_RECONNECTED:
            LOG("%s: Reconnected\n", __func__);
            break;
        case MQTT_DISCONNECTED:
            LOG("%s: Disconnect\n", __func__);
            break;
    }
}

void Main_MQTT_Subscription_Message(const char *topic, const char *payload) {
    LOG("Message arrived: %s %s\n", topic, payload);
}

static GMainLoop *main_loop = NULL;

static gboolean signal_handler(gpointer user_data) {
    LOG("Received SIGTERM, initiating shutdown\n");
    if (main_loop && g_main_loop_is_running(main_loop)) {
        g_main_loop_quit(main_loop);
    }
    return G_SOURCE_REMOVE;
}

void Settings_Updated_Callback(const char* service, cJSON* data) {

    if (strcmp(service, "publish") == 0) {
        publishEvents = cJSON_IsTrue(cJSON_GetObjectItem(data, "events"));
        publishDetections = cJSON_IsTrue(cJSON_GetObjectItem(data, "detections"));
        publishTracker = cJSON_IsTrue(cJSON_GetObjectItem(data, "tracker"));
        publishPath = cJSON_IsTrue(cJSON_GetObjectItem(data, "path"));
        publishOccupancy = cJSON_IsTrue(cJSON_GetObjectItem(data, "occupancy"));
        publishStatus = cJSON_IsTrue(cJSON_GetObjectItem(data, "status"));
        publishGeospace = cJSON_IsTrue(cJSON_GetObjectItem(data, "geospace"));
        publishAnomaly = cJSON_IsTrue(cJSON_GetObjectItem(data, "anomaly"));
    }

    if (strcmp(service, "scene") == 0)
        ObjectDetection_Config(data);

    if (strcmp(service, "matrix") == 0)
        GeoSpace_Matrix(data);

    if (strcmp(service, "stitch") == 0)
        GeoSpace_Matrix(data);	
}

void HandleVersionUpdateConfigurations(cJSON* settings) {
    cJSON* scene = cJSON_GetObjectItem(settings, "scene");
    if (!cJSON_GetObjectItem(scene, "maxIdle"))
        cJSON_AddNumberToObject(scene, "maxIdle", 0);
    if (!cJSON_GetObjectItem(scene, "tracker_confidence"))
        cJSON_AddTrueToObject(scene, "tracker_confidence");
    if (!cJSON_GetObjectItem(scene, "hanging_objects"))
        cJSON_AddNumberToObject(scene, "hanging_objects", 5);

    if (!cJSON_GetObjectItem(scene, "minWidth"))
        cJSON_AddNumberToObject(scene, "minWidth", 10);
    if (!cJSON_GetObjectItem(scene, "minHeight"))
        cJSON_AddNumberToObject(scene, "minHeight", 10);
    if (!cJSON_GetObjectItem(scene, "maxWidth"))
        cJSON_AddNumberToObject(scene, "maxWidth", 800);
    if (!cJSON_GetObjectItem(scene, "maxHeight"))
        cJSON_AddNumberToObject(scene, "maxHeight", 10);
    if (!cJSON_GetObjectItem(scene, "aoi")) {
        cJSON* aoi = cJSON_CreateObject();
        cJSON_AddNumberToObject(aoi, "x1", 50);
        cJSON_AddNumberToObject(aoi, "x2", 950);
        cJSON_AddNumberToObject(aoi, "y1", 50);
        cJSON_AddNumberToObject(aoi, "y2", 950);
        cJSON_AddItemToObject(scene, "aoi", aoi);
    }
    if (!cJSON_GetObjectItem(scene, "ignoreClass"))
        cJSON_AddArrayToObject(scene, "ignoreClass");

    cJSON* publish = cJSON_GetObjectItem(settings, "publish");
    if (!cJSON_GetObjectItem(publish, "geospace"))
        cJSON_AddFalseToObject(publish, "geospace");
}

int main(void) {
    openlog(APP_PACKAGE, LOG_PID | LOG_CONS, LOG_USER);
    LOG("------ Starting ACAP Service ------\n");

    cJSON* settings = ACAP(APP_PACKAGE, Settings_Updated_Callback);
    HandleVersionUpdateConfigurations(settings);

    MQTT_Init(Main_MQTT_Status, Main_MQTT_Subscription_Message);
    ACAP_Set_Config("mqtt", MQTT_Settings());
    ACAP_STATUS_SetObject("detections", "paths", cJSON_CreateArray());

    ACAP_EVENTS_SetCallback(Event_Callback);

    cJSON* eventSubscriptions = ACAP_FILE_Read("settings/subscriptions.json");
    cJSON* subscription = eventSubscriptions ? eventSubscriptions->child : 0;
    while (subscription) {
        ACAP_EVENTS_Subscribe(subscription, NULL);
        subscription = subscription->next;
    }

    if (ObjectDetection_Init(Detections_Data, Tracker_Data)) {
        ACAP_STATUS_SetBool("objectdetection", "connected", 1);
        ACAP_STATUS_SetString("objectdetection", "status", "OK");
    } else {
        ACAP_STATUS_SetBool("objectdetection", "connected", 0);
        ACAP_STATUS_SetString("objectdetection", "status", "Object detection is not available");
    }

    GeoSpace_Init();
    g_timeout_add_seconds(15 * 60, MQTT_Publish_Device_Status, NULL);

	STICH_Init(Publish_Path);



	ACAP_EVENTS_Add_Event("anomaly", "DataQ: Anomlay", 1);
    main_loop = g_main_loop_new(NULL, FALSE);
    GSource *signal_source = g_unix_signal_source_new(SIGTERM);
    if (signal_source) {
        g_source_set_callback(signal_source, signal_handler, NULL, NULL);
        g_source_attach(signal_source, NULL);
    } else {
        LOG_WARN("Signal detection failed");
    }

    g_main_loop_run(main_loop);

    LOG("Terminating and cleaning up %s\n", APP_PACKAGE);
    Main_MQTT_Status(MQTT_DISCONNECTING);
    MQTT_Cleanup();
    ACAP_Cleanup();
    closelog();
    return 0;
}
