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
#include <stdint.h>
#include <syslog.h>
#include <glib.h>
#include <time.h>
#include <glib-unix.h>
#include <signal.h>
#include <math.h>

#include <vdo/vdo-stream.h>
#include <vdo/vdo-buffer.h>
#include <vdo/vdo-map.h>
#include <vdo/vdo-channel.h>
#include <vdo/vdo-frame.h>
#include <vdo/vdo-error.h>

#include "cJSON.h"
#include "ACAP.h"
#include "MQTT.h"
#include "ObjectDetection.h"
#include "GeoSpace.h"
#include "Stitch.h"
// VOD.h removed - label list sourced from ObjectDetection_Labels()

#define APP_PACKAGE "dataq"

#define LOG(fmt, args...)      { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_WARN(fmt, args...) { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args); }
//#define LOG_TRACE(fmt, args...) { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args); }
#define LOG_TRACE(fmt, args...) {}

cJSON* activeTrackers = 0;
cJSON* PreviousPosition = 0;
cJSON* lastPublishedTracker = 0;
cJSON* PathCache = 0;
int lastDetectionListWasEmpty = 0;
int publishEvents = 1;
int publishDetections = 1;
int publishAnomaly = 0;
int publishTracker = 1;
int publishPath = 1;
int publishOccupancy = 0;
int publishStatus = 1;
int publishGeospace = 0;
int publishImage = 0;
int mqttConnected = 0;
static guint image_noon_timer_id = 0;
int shouldReset = 0;

/* ── Multi-area occupancy ─────────────────────────────────────────── */
#define MAX_AREAS         8
#define MAX_POLY_VERTICES 32

typedef struct {
    int    id;
    int    active;
    char   name[64];
    int    polygon_count;
    int    poly_x[MAX_POLY_VERTICES];
    int    poly_y[MAX_POLY_VERTICES];
    cJSON* last_published;   /* full counter last sent  { "Human":1, "Car":0, … } */
    cJSON* pending_counter;  /* pending decrease (NULL = none)                     */
    double decrease_due_at;  /* monotonic timestamp when pending fires              */
    cJSON* seen_labels;      /* object: keys = labels ever counted >=1 for this area */
    cJSON* acc_sum;          /* running label sums for periodic publish               */
    int    acc_count;        /* number of detection-cycle samples accumulated         */
} OccupancyArea;

static OccupancyArea g_areas[MAX_AREAS];
static int           g_area_count = 0;

/* Whole-frame fallback state (used when no areas are defined) */
static cJSON*  g_fallback_last    = NULL;
static cJSON*  g_fallback_pending = NULL;
static double  g_fallback_due     = 0.0;
static cJSON*  g_fallback_seen    = NULL;

/* Periodic publish state */
static cJSON*  g_fallback_acc_sum   = NULL;
static int     g_fallback_acc_count = 0;
static guint   g_periodic_timer_id  = 0;
static int     g_occ_periodic       = 0;     /* 0 = on_change, 1 = periodic */
static int     g_occ_interval_sec   = 300;   /* default 5 min               */


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

        cJSON* dxItem = cJSON_GetObjectItem(tracker, "dx");
        cJSON* dyItem = cJSON_GetObjectItem(tracker, "dy");
        cJSON* bxItem = cJSON_GetObjectItem(tracker, "bx");
        cJSON* byItem = cJSON_GetObjectItem(tracker, "by");

        cJSON_AddNumberToObject(path, "dx", dxItem ? dxItem->valuedouble : 0);
        cJSON_AddNumberToObject(path, "dy", dyItem ? dyItem->valuedouble : 0);
        cJSON_AddNumberToObject(path, "bx", bxItem ? bxItem->valuedouble : 0);
        cJSON_AddNumberToObject(path, "by", byItem ? byItem->valuedouble : 0);

        double birthTime = cJSON_GetObjectItem(tracker, "birth") ?
                          cJSON_GetObjectItem(tracker, "birth")->valuedouble : currentTimestamp;
        cJSON_AddNumberToObject(path, "timestamp", birthTime);
        cJSON_AddNumberToObject(path, "dwell", 0);
        cJSON_AddStringToObject(path, "id", id);

        cJSON* face = cJSON_GetObjectItem(tracker, "face");
        if (face) cJSON_AddBoolToObject(path, "face", cJSON_IsTrue(face));
        cJSON* hat = cJSON_GetObjectItem(tracker, "hat");
        if (hat) cJSON_AddStringToObject(path, "hat", hat->valuestring);

        double blat = 0, blon = 0;
        int geo_success_birth = 0;
        if (bxItem && byItem)
            geo_success_birth = GeoSpace_transform(bxItem->valueint, byItem->valueint, &blat, &blon);

        cJSON* pathArr = cJSON_CreateArray();

        // Position 0: Birth position (bx, by)
        cJSON* pos1 = cJSON_CreateObject();
        cJSON_AddNumberToObject(pos1, "x", bxItem ? bxItem->valuedouble : 0);
        cJSON_AddNumberToObject(pos1, "y", byItem ? byItem->valuedouble : 0);
        cJSON_AddNumberToObject(pos1, "d", 0);  // Will be updated on first tracker update
        cJSON_AddNumberToObject(pos1, "t", birthTime / 1000.0);  // Epoch seconds for stitch matching
        if (geo_success_birth) {
            cJSON_AddNumberToObject(pos1, "lat", round(blat * 1e6) / 1e6);
            cJSON_AddNumberToObject(pos1, "lon", round(blon * 1e6) / 1e6);
        }
        cJSON_AddItemToArray(pathArr, pos1);

        // Position 1: Current position (cx, cy)
        cJSON* cxNew = cJSON_GetObjectItem(tracker, "cx");
        cJSON* cyNew = cJSON_GetObjectItem(tracker, "cy");
        if (!cxNew || !cyNew) return 0;
        double clat = 0, clon = 0;
        int geo_success_current = GeoSpace_transform(cxNew->valueint, cyNew->valueint, &clat, &clon);
        cJSON* pos2 = cJSON_CreateObject();
        cJSON_AddNumberToObject(pos2, "x", cxNew->valuedouble);
        cJSON_AddNumberToObject(pos2, "y", cyNew->valuedouble);
        cJSON_AddNumberToObject(pos2, "d", 0);  // Will be updated on next tracker update
        cJSON_AddNumberToObject(pos2, "t", currentTimestamp / 1000.0);  // Epoch seconds for stitch matching
        if (geo_success_current) {
            cJSON_AddNumberToObject(pos2, "lat", round(clat * 1e6) / 1e6);
            cJSON_AddNumberToObject(pos2, "lon", round(clon * 1e6) / 1e6);
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

        cJSON* face = cJSON_GetObjectItem(tracker, "face");
        if (face) cJSON_ReplaceItemInObject(path, "face", cJSON_Duplicate(face, 1));
        cJSON* hat = cJSON_GetObjectItem(tracker, "hat");
        if (hat) cJSON_ReplaceItemInObject(path, "hat", cJSON_Duplicate(hat, 1));

        cJSON* pathArr = cJSON_GetObjectItem(path, "path");
        int pathLen = cJSON_GetArraySize(pathArr);

        if (pathLen >= 2) {
            // Update the LAST position's duration — this is where the object WAS
            // between the previous tracker event and this new movement event
            cJSON* lastPos = cJSON_GetArrayItem(pathArr, pathLen - 1);

            // Calculate duration: time object spent at that position
            double duration = (currentTimestamp - previousTimestamp) / 1000.0; // Convert ms to seconds
            cJSON_ReplaceItemInObject(lastPos, "d", cJSON_CreateNumber(duration));
        }

        // Update dwell = max time spent at any single sampled position
        double maxDwell = 0.0;
        for (int i = 0; i < pathLen; ++i) {
            cJSON* pos = cJSON_GetArrayItem(pathArr, i);
            cJSON* d_item = cJSON_GetObjectItem(pos, "d");
            if (d_item && d_item->valuedouble > maxDwell)
                maxDwell = d_item->valuedouble;
        }
        cJSON* dwell_item = cJSON_GetObjectItem(path, "dwell");
        if (dwell_item)
            cJSON_SetNumberValue(dwell_item, maxDwell);

        // Add NEW position with d=0 (will be calculated on next update or exit)
        cJSON* cxUpd = cJSON_GetObjectItem(tracker, "cx");
        cJSON* cyUpd = cJSON_GetObjectItem(tracker, "cy");
        if (!cxUpd || !cyUpd) return 0;
        double lat = 0, lon = 0;
        int geo_success_upd = GeoSpace_transform(cxUpd->valueint, cyUpd->valueint, &lat, &lon);
        cJSON* pos = cJSON_CreateObject();
        cJSON_AddNumberToObject(pos, "x", cxUpd->valuedouble);
        cJSON_AddNumberToObject(pos, "y", cyUpd->valuedouble);
        cJSON_AddNumberToObject(pos, "d", 0);  // Always 0 for newest position
        cJSON_AddNumberToObject(pos, "t", currentTimestamp / 1000.0);  // Epoch seconds for stitch matching
        if (geo_success_upd) {
            cJSON_AddNumberToObject(pos, "lat", round(lat * 1e6) / 1e6);
            cJSON_AddNumberToObject(pos, "lon", round(lon * 1e6) / 1e6);
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

            // Calculate final dwell = max time spent at any single sampled position
            double maxDwell = 0.0;
            for (int i = 0; i < pathLen; ++i) {
                cJSON* pos = cJSON_GetArrayItem(pathArr, i);
                cJSON* d_item = cJSON_GetObjectItem(pos, "d");
                if (d_item && d_item->valuedouble > maxDwell)
                    maxDwell = d_item->valuedouble;
            }
            cJSON_ReplaceItemInObject(path, "dwell", cJSON_CreateNumber(maxDwell));

            cJSON_DetachItemFromObject(PathCache, id);

            // FIX #2: Update final age and distance from tracker
            // Ensures path properties match final tracker state
            cJSON_ReplaceItemInObject(path, "age", cJSON_CreateNumber(age));
            cJSON_ReplaceItemInObject(path, "distance", cJSON_CreateNumber(distance));

            // Update other final metadata
            cJSON* pathAnomalyItem = cJSON_GetObjectItem(tracker, "anomaly");
            if (pathAnomalyItem && pathAnomalyItem->valuestring)
                cJSON_AddStringToObject(path, "anomaly", pathAnomalyItem->valuestring);
            cJSON* pathMaxSpeedItem = cJSON_GetObjectItem(tracker, "maxSpeed");
            if (pathMaxSpeedItem)
                cJSON_AddNumberToObject(path, "maxSpeed", pathMaxSpeedItem->valuedouble);
            // maxIdle is redundant — dwell already holds that value

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
    cJSON* classItem = cJSON_GetObjectItem(tracker, "class");
    if (!classItem || !classItem->valuestring) return;
    const char* label = classItem->valuestring;
    if (strcmp("Human", label) == 0) is_human = 1;
    // If vehicle-like class
    if (strcmp("Car", label) == 0 || strcmp("Truck", label) == 0 ||
        strcmp("Bus", label) == 0 || strcmp("Bike", label) == 0 ||
        strcmp("Other", label) == 0 || strcmp("Veicle", label) == 0)
        is_vehicle = 1;

    // Save stats
    cJSON* activeCheck = cJSON_GetObjectItem(tracker, "active");
    if (activeCheck && activeCheck->type == cJSON_False) {
        char* group_label = is_human ? "humans" : "vehicles";
        cJSON* stats;

        stats = ACAP_STATUS_Object(group_label, "directions");
        if(!stats) stats = cJSON_CreateArray();
        while( cJSON_GetArraySize(stats) > 200 )
            cJSON_DeleteItemFromArray(stats, 0);
        cJSON* dirItem = cJSON_GetObjectItem(tracker, "directions");
        cJSON_AddItemToArray(stats, cJSON_CreateNumber(dirItem ? dirItem->valueint : 0));
        ACAP_STATUS_SetObject(group_label, "directions", stats);

        stats = ACAP_STATUS_Object(group_label, "age");
        if(!stats) stats = cJSON_CreateArray();
        cJSON* ageStatItem = cJSON_GetObjectItem(tracker, "age");
        cJSON_AddItemToArray(stats, cJSON_CreateNumber(ageStatItem ? ageStatItem->valuedouble : 0));
        while( cJSON_GetArraySize(stats) > 200 )
            cJSON_DeleteItemFromArray(stats, 0);
        ACAP_STATUS_SetObject(group_label, "age", stats);

        stats = ACAP_STATUS_Object(group_label, "idle");
        if(!stats) stats = cJSON_CreateArray();
        cJSON* maxIdleStatItem = cJSON_GetObjectItem(tracker, "maxIdle");
        cJSON_AddItemToArray(stats, cJSON_CreateNumber(maxIdleStatItem ? maxIdleStatItem->valuedouble : 0));
        while( cJSON_GetArraySize(stats) > 200 )
            cJSON_DeleteItemFromArray(stats, 0);
        ACAP_STATUS_SetObject(group_label, "idle", stats);

        stats = ACAP_STATUS_Object(group_label, "speed");
        if(!stats) stats = cJSON_CreateArray();
        cJSON* maxSpeedStatItem = cJSON_GetObjectItem(tracker, "maxSpeed");
        double speed = maxSpeedStatItem ? maxSpeedStatItem->valuedouble : 0;
        if (speed > 0) {
            cJSON_AddItemToArray(stats, cJSON_CreateNumber(speed));
            while( cJSON_GetArraySize(stats) > 200 )
                cJSON_DeleteItemFromArray(stats, 0);
            ACAP_STATUS_SetObject(group_label, "speed", stats);
        }

        stats = ACAP_STATUS_Object(group_label, "horizontal");
        if(!stats) stats = cJSON_CreateArray();
        cJSON* dxStatItem = cJSON_GetObjectItem(tracker, "dx");
        int dx = dxStatItem ? dxStatItem->valueint : 0;
        cJSON_AddItemToArray(stats, cJSON_CreateNumber(dx));
        while( cJSON_GetArraySize(stats) > 200 )
            cJSON_DeleteItemFromArray(stats, 0);
        ACAP_STATUS_SetObject(group_label, "horizontal", stats);

        stats = ACAP_STATUS_Object(group_label, "vertical");
        if(!stats) stats = cJSON_CreateArray();
        cJSON* dyStatItem = cJSON_GetObjectItem(tracker, "dy");
        int dy = dyStatItem ? dyStatItem->valueint : 0;
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

    cJSON* cxItem_a = cJSON_GetObjectItem(tracker, "cx");
    cJSON* cyItem_a = cJSON_GetObjectItem(tracker, "cy");
    cJSON* bxItem_a = cJSON_GetObjectItem(tracker, "bx");
    cJSON* byItem_a = cJSON_GetObjectItem(tracker, "by");
    if (!cxItem_a || !cyItem_a || !bxItem_a || !byItem_a) return;
    int cx = cxItem_a->valueint;
    int cy = cyItem_a->valueint;
    int bx = bxItem_a->valueint;
    int by = byItem_a->valueint;

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
            LOG_WARN("%s: Invalid entry", __func__);
            Fire_Anomaly();
            cJSON_AddStringToObject(tracker, "anomaly", "Invalid entry");
            return;
        }
    }

    found_common = 0;
    cJSON* activeExitCheck = cJSON_GetObjectItem(tracker, "active");
    if (common && cJSON_GetArraySize(common) && activeExitCheck && activeExitCheck->type == cJSON_False) {
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
        cJSON* distCheck = item ? cJSON_GetObjectItem(item, "distance") : NULL;
        if (distCheck && distCheck->valueint < 20)
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
	cJSON* directionsItem = cJSON_GetObjectItem(tracker, "directions");
	int directions = directionsItem ? directionsItem->valueint : 0;
    if (maxDirections && directions > maxDirections) {
		snprintf(text, sizeof(text), "Directions: %d > %d", directions , maxDirections);
		//LOG("%s",text);
        cJSON_AddStringToObject(tracker, "anomaly", text);
        Fire_Anomaly();
        return;
    }

    float maxAge = cJSON_GetObjectItem(normal, "age") ? cJSON_GetObjectItem(normal, "age")->valuedouble : 0;
    cJSON* ageItem = cJSON_GetObjectItem(tracker, "age");
    float age = ageItem ? ageItem->valuedouble : 0;
    if (maxAge && age > maxAge) {
		snprintf(text, sizeof(text), "Age: %d>%d", (int)age , (int)maxAge);
		//LOG("%s",text);
        cJSON_AddStringToObject(tracker, "anomaly", text);
        Fire_Anomaly();
        return;
    }

    float maxIdle = cJSON_GetObjectItem(normal, "idle") ? cJSON_GetObjectItem(normal, "idle")->valuedouble : 0;
    cJSON* idleItem = cJSON_GetObjectItem(tracker, "maxIdle");
    float idle = idleItem ? idleItem->valuedouble : 0;
    if (maxIdle && idle > maxIdle) {
		snprintf(text, sizeof(text), "Idle: %d>%d", (int)idle , (int)maxIdle);
		//LOG("%s",text);
        cJSON_AddStringToObject(tracker, "anomaly", text);
        Fire_Anomaly();
        return;
    }

    float speedLimit = cJSON_GetObjectItem(normal, "maxSpeed") ? cJSON_GetObjectItem(normal, "maxSpeed")->valuedouble : 0;
    cJSON* maxSpeedItem = cJSON_GetObjectItem(tracker, "maxSpeed");
    float maxSpeed = maxSpeedItem ? maxSpeedItem->valuedouble : 0;
    if (speedLimit && maxSpeed > speedLimit) {
		snprintf(text, sizeof(text), "Speed: %d>%d", (int)maxSpeed , (int)speedLimit);
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
    if (!tracker) return;
    char topic[128];

	Check_Anomaly( tracker );

    if (publishPath && !timer && tracker)
		Stitch_Path(ProcessPaths(tracker));

    cJSON_DeleteItemFromObject(tracker, "previousTimestamp");
    cJSON_DeleteItemFromObject(tracker, "maxIdle");

    if (publishTracker) {
        snprintf(topic, sizeof(topic), "tracker/%s", ACAP_DEVICE_Prop("serial"));
        MQTT_Publish_JSON(topic, tracker, 0, 0);
    }

    if (publishGeospace && ACAP_STATUS_Bool("geospace", "active")) {
        cJSON* geoCx = cJSON_GetObjectItem(tracker, "cx");
        cJSON* geoCy = cJSON_GetObjectItem(tracker, "cy");
        if (!geoCx || !geoCy) goto skip_geospace;
        double lat = 0, lon = 0;
        cJSON* geoActive = cJSON_GetObjectItem(tracker, "active");
        int is_active = geoActive && cJSON_IsTrue(geoActive);
        int geo_ok = GeoSpace_transform(geoCx->valueint, geoCy->valueint, &lat, &lon);
        // Publish if transform succeeded (active objects) OR always when object leaves (to clean up map markers)
        if (geo_ok || !is_active) {
            cJSON* geospaceObject = cJSON_CreateObject();
            cJSON* geoId = cJSON_GetObjectItem(tracker, "id");
            if (geoId && geoId->valuestring) cJSON_AddStringToObject(geospaceObject, "id", geoId->valuestring);
            if (geoActive) cJSON_AddItemToObject(geospaceObject, "active", cJSON_Duplicate(geoActive, 1));
            if (geo_ok) {
                cJSON_AddNumberToObject(geospaceObject, "lat", round(lat * 1e6) / 1e6);
                cJSON_AddNumberToObject(geospaceObject, "lon", round(lon * 1e6) / 1e6);
                cJSON* geoClass = cJSON_GetObjectItem(tracker, "class");
                if (geoClass && geoClass->valuestring)
                    cJSON_AddStringToObject(geospaceObject, "class", geoClass->valuestring);
                cJSON* geoAge = cJSON_GetObjectItem(tracker, "age");
                cJSON* geoIdle = cJSON_GetObjectItem(tracker, "idle");
                cJSON* geoConfidence = cJSON_GetObjectItem(tracker, "confidence");
                cJSON* geoDistance = cJSON_GetObjectItem(tracker, "distance");
                if (geoAge) cJSON_AddNumberToObject(geospaceObject, "age", geoAge->valuedouble);
                if (geoIdle) cJSON_AddNumberToObject(geospaceObject, "idle", geoIdle->valuedouble);
                if (geoConfidence) cJSON_AddNumberToObject(geospaceObject, "confidence", geoConfidence->valuedouble);
                if (geoDistance) cJSON_AddNumberToObject(geospaceObject, "distance", geoDistance->valuedouble);
            }
            snprintf(topic, sizeof(topic), "geospace/%s", ACAP_DEVICE_Prop("serial"));
            MQTT_Publish_JSON(topic, geospaceObject, 0, 0);
            cJSON_Delete(geospaceObject);
        }
    }
    skip_geospace:

    cJSON_Delete(tracker);
}

void Publish_Path( cJSON* path ){
	if( !path ) return;
	// Discard paths with fewer than 3 sampled positions
	cJSON* pathArray = cJSON_GetObjectItem(path, "path");
	if( !pathArray || cJSON_GetArraySize(pathArray) < 3 ) {
		cJSON_Delete(path);
		return;
	}
	// Strip internal 't' timestamps from path points (used for stitching, not needed in published data)
	for (cJSON* pt = pathArray->child; pt != NULL; pt = pt->next)
		cJSON_DeleteItemFromObject(pt, "t");
    char topic[128];
	snprintf(topic, sizeof(topic), "path/%s", ACAP_DEVICE_Prop("serial"));
	MQTT_Publish_JSON(topic, path, 0, 0);

    cJSON* statusPaths = ACAP_STATUS_Object("detections", "paths");
	if (statusPaths) {
		cJSON_AddItemToArray(statusPaths, cJSON_Duplicate(path, 1));
		while (cJSON_GetArraySize(statusPaths) > 10)
			cJSON_DeleteItemFromArray(statusPaths, 0);
	}
	cJSON_Delete(path);
}

/* ── Occupancy helpers ────────────────────────────────────────────── */

/*
 * Standard ray-casting point-in-polygon test.
 * Coordinates are in the 0-1000 canvas space used by the detection pipeline.
 */
static int point_in_polygon(int px, int py, int* xs, int* ys, int n) {
    int inside = 0;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        int xi = xs[i], yi = ys[i];
        int xj = xs[j], yj = ys[j];
        if (((yi > py) != (yj > py)) &&
            (px < (xj - xi) * (py - yi) / (yj - yi) + xi))
            inside = !inside;
    }
    return inside;
}

/*
 * Build a raw per-label counter from the tracker list.
 * Only labels with a non-zero count are present in the result.
 * If poly_n >= 3, only detections whose COG is inside the polygon are counted.
 */
static cJSON* build_raw_counter(cJSON* list, cJSON* labels,
                                int stationary, int moving,
                                double ageThreshold, double idleThreshold,
                                int* poly_x, int* poly_y, int poly_n) {
    cJSON* counter = cJSON_CreateObject();

    int listSize = cJSON_GetArraySize(list);
    for (int i = 0; i < listSize; i++) {
        cJSON* det = cJSON_GetArrayItem(list, i);
        if (!det) continue;
        cJSON* clsItem  = cJSON_GetObjectItem(det, "class");
        cJSON* ageItem  = cJSON_GetObjectItem(det, "age");
        cJSON* idleItem = cJSON_GetObjectItem(det, "idle");
        cJSON* cxItem   = cJSON_GetObjectItem(det, "cx");
        cJSON* cyItem   = cJSON_GetObjectItem(det, "cy");
        if (!clsItem || !clsItem->valuestring || !ageItem || !idleItem) continue;

        /* Only count labels in the allowed set (labels is a JSON array) */
        if (labels) {
            int found = 0;
            for (cJSON* lbl = labels->child; lbl; lbl = lbl->next)
                if (lbl->valuestring && strcmp(lbl->valuestring, clsItem->valuestring) == 0)
                    { found = 1; break; }
            if (!found) continue;
        }

        /* Polygon filter (COG must be inside) */
        if (poly_n >= 3) {
            if (!cxItem || !cyItem) continue;
            if (!point_in_polygon(cxItem->valueint, cyItem->valueint,
                                  poly_x, poly_y, poly_n))
                continue;
        }

        double age  = ageItem->valuedouble;
        double idle = idleItem->valuedouble;
        int is_moving     = (age >= ageThreshold) && (idle < idleThreshold);
        int is_stationary = (age >= ageThreshold) && (idle >= idleThreshold);
        if (!((moving && is_moving) || (stationary && is_stationary)))
            continue;

        cJSON* curr = cJSON_GetObjectItem(counter, clsItem->valuestring);
        if (curr)
            curr->valuedouble += 1.0;
        else
            cJSON_AddNumberToObject(counter, clsItem->valuestring, 1.0);
    }
    return counter;
}

/*
 * Merge ever-seen labels into `raw` (adding them at 0 if absent) and
 * record any newly non-zero labels into `*p_seen`.
 * `*p_seen` is created lazily if NULL.
 */
static void merge_with_seen(cJSON* raw, cJSON** p_seen) {
    if (!*p_seen) *p_seen = cJSON_CreateObject();

    /* Record newly non-zero labels */
    for (cJSON* item = raw->child; item; item = item->next)
        if (item->valuedouble > 0 && !cJSON_GetObjectItem(*p_seen, item->string))
            cJSON_AddTrueToObject(*p_seen, item->string);

    /* Add previously-seen labels at 0 if missing from raw */
    for (cJSON* lbl = (*p_seen)->child; lbl; lbl = lbl->next)
        if (!cJSON_GetObjectItem(raw, lbl->string))
            cJSON_AddNumberToObject(raw, lbl->string, 0.0);
}

/*
 * Hold-down debounce logic shared by areas and the whole-frame fallback.
 *
 * Increases are published immediately.
 * Decreases are held for `holdTime` seconds before publishing.
 * If counts recover within holdTime the pending decrease is cancelled.
 *
 * Returns a pointer to the counter that should be published, or NULL.
 * The returned pointer is `*p_last` (owned by the caller state).
 */
static cJSON* apply_hold_down(cJSON* raw, double now, double holdTime,
                               cJSON** p_last, cJSON** p_pending, double* p_due) {
    int has_increase = 0, has_decrease = 0;

    if (*p_last) {
        for (cJSON* item = raw->child; item; item = item->next) {
            cJSON* prev = cJSON_GetObjectItem(*p_last, item->string);
            double pv = prev ? prev->valuedouble : 0.0;
            if (item->valuedouble > pv) has_increase = 1;
            if (item->valuedouble < pv) has_decrease = 1;
        }
    } else {
        for (cJSON* item = raw->child; item; item = item->next)
            if (item->valuedouble > 0) { has_increase = 1; break; }
    }

    if (has_increase) {
        /* Immediate publish — cancel any pending decrease */
        if (*p_pending) { cJSON_Delete(*p_pending); *p_pending = NULL; }
        if (*p_last) cJSON_Delete(*p_last);
        *p_last = cJSON_Duplicate(raw, 1);
        return *p_last;
    }

    if (!has_decrease) {
        /* Nothing changed vs last_published.
         * If a pending decrease was armed but raw has fully recovered, cancel it. */
        if (*p_pending && *p_last) {
            int recovered = 1;
            for (cJSON* item = raw->child; item; item = item->next) {
                cJSON* prev = cJSON_GetObjectItem(*p_last, item->string);
                if (item->valuedouble < (prev ? prev->valuedouble : 0.0))
                    { recovered = 0; break; }
            }
            if (recovered) { cJSON_Delete(*p_pending); *p_pending = NULL; }
        }
        /* Check if a previously armed pending has matured */
        if (*p_pending && now >= *p_due) {
            if (*p_last) cJSON_Delete(*p_last);
            *p_last = *p_pending;
            *p_pending = NULL;
            return *p_last;
        }
        return NULL;
    }

    /* Pure decrease (or decrease with no increase) — arm / refresh hold-down */
    if (*p_pending) {
        cJSON_Delete(*p_pending);           /* refresh to latest raw */
        *p_pending = cJSON_Duplicate(raw, 1);
        /* intentionally keep original *p_due (don't reset the timer) */
    } else {
        *p_pending = cJSON_Duplicate(raw, 1);
        *p_due = now + holdTime;
    }

    /* Check if the pending decrease has now matured */
    if (now >= *p_due) {
        if (*p_last) cJSON_Delete(*p_last);
        *p_last = *p_pending;
        *p_pending = NULL;
        return *p_last;
    }

    return NULL;
}

/* Forward declaration — needed by periodic_occupancy_cb */
static void publish_occupancy(const char* area_name, cJSON* counter, double now);

/*
 * GLib timer callback: fires every g_occ_interval_sec seconds in periodic mode.
 * Computes per-label average (1 decimal precision) from accumulated samples and publishes.
 */
static gboolean periodic_occupancy_cb(gpointer user_data) {
    (void)user_data;
    double now = ACAP_DEVICE_Timestamp();

    if (g_area_count > 0) {
        for (int i = 0; i < g_area_count; i++) {
            OccupancyArea* a = &g_areas[i];
            if (!a->active || a->acc_count <= 0) continue;

            cJSON* avg  = cJSON_CreateObject();
            cJSON* item = NULL;
            cJSON_ArrayForEach(item, a->acc_sum) {
                double val = round(item->valuedouble / a->acc_count * 10.0) / 10.0;
                cJSON_AddNumberToObject(avg, item->string, val);
            }
            publish_occupancy(a->name, avg, now);

            char status_key[80];
            snprintf(status_key, sizeof(status_key), "area_%d", a->id);
            cJSON* entry = cJSON_CreateObject();
            cJSON_AddStringToObject(entry, "name", a->name);
            cJSON_AddItemToObject(entry, "counter", cJSON_Duplicate(avg, 1));
            ACAP_STATUS_SetObject("occupancy", status_key, entry);
            cJSON_Delete(entry);
            cJSON_Delete(avg);

            cJSON_Delete(a->acc_sum);
            a->acc_sum   = cJSON_CreateObject();
            a->acc_count = 0;
        }
    } else {
        if (g_fallback_acc_count > 0) {
            cJSON* avg  = cJSON_CreateObject();
            cJSON* item = NULL;
            cJSON_ArrayForEach(item, g_fallback_acc_sum) {
                double val = round(item->valuedouble / g_fallback_acc_count * 10.0) / 10.0;
                cJSON_AddNumberToObject(avg, item->string, val);
            }
            publish_occupancy(NULL, avg, now);
            ACAP_STATUS_SetObject("occupancy", "counter", avg);
            cJSON_Delete(avg);

            cJSON_Delete(g_fallback_acc_sum);
            g_fallback_acc_sum   = cJSON_CreateObject();
            g_fallback_acc_count = 0;
        }
    }
    return G_SOURCE_CONTINUE;
}

/*
 * Load / reload area definitions from settings into g_areas[].
 * Called at startup and whenever occupancy settings are saved.
 */
static void Occupancy_Load_Areas(void) {
    /* Free any existing per-area cJSON state */
    for (int i = 0; i < g_area_count; i++) {
        if (g_areas[i].last_published)  { cJSON_Delete(g_areas[i].last_published);  g_areas[i].last_published  = NULL; }
        if (g_areas[i].pending_counter) { cJSON_Delete(g_areas[i].pending_counter); g_areas[i].pending_counter = NULL; }
        if (g_areas[i].seen_labels)     { cJSON_Delete(g_areas[i].seen_labels);     g_areas[i].seen_labels     = NULL; }
        if (g_areas[i].acc_sum)         { cJSON_Delete(g_areas[i].acc_sum);         g_areas[i].acc_sum         = NULL; }
        g_areas[i].acc_count = 0;
    }
    g_area_count = 0;
    /* Reset fallback state */
    if (g_fallback_seen)    { cJSON_Delete(g_fallback_seen);    g_fallback_seen    = NULL; }
    if (g_fallback_acc_sum) { cJSON_Delete(g_fallback_acc_sum); g_fallback_acc_sum = NULL; }
    g_fallback_acc_count = 0;
    /* Cancel any existing periodic timer */
    if (g_periodic_timer_id) { g_source_remove(g_periodic_timer_id); g_periodic_timer_id = 0; }
    g_occ_periodic = 0;

    cJSON* settings = ACAP_Get_Config("settings");
    if (!settings) return;
    cJSON* occupancy = cJSON_GetObjectItem(settings, "occupancy");
    if (!occupancy) return;

    /* Read publish mode before processing areas */
    {
        cJSON* modeItem     = cJSON_GetObjectItem(occupancy, "publishMode");
        cJSON* intervalItem = cJSON_GetObjectItem(occupancy, "periodicInterval");
        const char* mode    = (modeItem && modeItem->valuestring) ? modeItem->valuestring : "on_change";
        int interval_min    = (intervalItem && cJSON_IsNumber(intervalItem)) ? intervalItem->valueint : 5;
        if (interval_min < 1) interval_min = 1;
        g_occ_periodic     = (strcmp(mode, "periodic") == 0) ? 1 : 0;
        g_occ_interval_sec = interval_min * 60;
    }

    cJSON* areas = cJSON_GetObjectItem(occupancy, "areas");
    if (!areas || !cJSON_IsArray(areas)) {
        /* No areas — init fallback accumulator and possibly schedule periodic timer */
        g_fallback_acc_sum = cJSON_CreateObject();
        if (g_occ_periodic)
            g_periodic_timer_id = g_timeout_add_seconds(g_occ_interval_sec, periodic_occupancy_cb, NULL);
        return;
    }

    int n = cJSON_GetArraySize(areas);
    if (n > MAX_AREAS) n = MAX_AREAS;

    for (int i = 0; i < n; i++) {
        cJSON* area = cJSON_GetArrayItem(areas, i);
        if (!area) continue;
        cJSON* activeItem = cJSON_GetObjectItem(area, "active");
        if (activeItem && !cJSON_IsTrue(activeItem)) continue;

        OccupancyArea* a = &g_areas[g_area_count];
        memset(a, 0, sizeof(OccupancyArea));

        cJSON* idItem   = cJSON_GetObjectItem(area, "id");
        cJSON* nameItem = cJSON_GetObjectItem(area, "name");
        a->id     = idItem ? idItem->valueint : (i + 1);
        a->active = 1;
        if (nameItem && nameItem->valuestring)
            snprintf(a->name, sizeof(a->name), "%s", nameItem->valuestring);
        else
            snprintf(a->name, sizeof(a->name), "Area%d", a->id);

        cJSON* polygon = cJSON_GetObjectItem(area, "polygon");
        if (polygon && cJSON_IsArray(polygon)) {
            int pn = cJSON_GetArraySize(polygon);
            if (pn > MAX_POLY_VERTICES) pn = MAX_POLY_VERTICES;
            for (int j = 0; j < pn; j++) {
                cJSON* pt = cJSON_GetArrayItem(polygon, j);
                if (!pt) continue;
                cJSON* xItem = cJSON_GetObjectItem(pt, "x");
                cJSON* yItem = cJSON_GetObjectItem(pt, "y");
                a->poly_x[j] = xItem ? xItem->valueint : 0;
                a->poly_y[j] = yItem ? yItem->valueint : 0;
            }
            a->polygon_count = pn;
        }
        a->acc_sum   = cJSON_CreateObject();
        a->acc_count = 0;
        g_area_count++;
    }

    /* Init fallback accumulator */
    g_fallback_acc_sum   = cJSON_CreateObject();
    g_fallback_acc_count = 0;

    /* Schedule periodic publish timer if requested */
    if (g_occ_periodic)
        g_periodic_timer_id = g_timeout_add_seconds(g_occ_interval_sec, periodic_occupancy_cb, NULL);

    LOG_TRACE("Occupancy: loaded %d active area(s), mode=%s, interval=%ds\n",
              g_area_count, g_occ_periodic ? "periodic" : "on_change", g_occ_interval_sec);
}

/*
 * Publish an occupancy counter for one area (or the whole-frame fallback).
 * area_name == NULL means whole-frame fallback → topic: occupancy/<serial>
 * area_name != NULL                             → topic: occupancy/<serial>/<area>
 */
static void publish_occupancy(const char* area_name, cJSON* counter, double now) {
    char topic[256];
    cJSON* payload = cJSON_CreateObject();
    if (area_name)
        cJSON_AddStringToObject(payload, "area", area_name);
    cJSON_AddItemToObject(payload, "occupancy", cJSON_Duplicate(counter, 1));
    cJSON_AddNumberToObject(payload, "timestamp", now);
    if (area_name)
        snprintf(topic, sizeof(topic), "occupancy/%s/%s",
                 ACAP_DEVICE_Prop("serial"), area_name);
    else
        snprintf(topic, sizeof(topic), "occupancy/%s",
                 ACAP_DEVICE_Prop("serial"));
    MQTT_Publish_JSON(topic, payload, 0, 0);
    cJSON_Delete(payload);
}

/*
 * Main occupancy processing — called each detection cycle.
 * On-change mode: publish immediately when a counter changes (with hold-down debounce).
 * Periodic mode:  accumulate raw samples each cycle; timer callback publishes averages.
 */
static void ProcessOccupancy(cJSON* list) {
    cJSON* settings = ACAP_Get_Config("settings");
    if (!settings) return;
    cJSON* occ = cJSON_GetObjectItem(settings, "occupancy");
    if (!occ) return;

    int    stationary    = cJSON_IsTrue(cJSON_GetObjectItem(occ, "stationary"));
    int    moving        = cJSON_IsTrue(cJSON_GetObjectItem(occ, "moving"));
    double ageThreshold  = cJSON_GetObjectItem(occ, "ageThreshold")
                           ? cJSON_GetObjectItem(occ, "ageThreshold")->valuedouble : 2.0;
    double idleThreshold = cJSON_GetObjectItem(occ, "idleThreshold")
                           ? cJSON_GetObjectItem(occ, "idleThreshold")->valuedouble : 3.0;
    double holdTime      = cJSON_GetObjectItem(occ, "holdTime")
                           ? cJSON_GetObjectItem(occ, "holdTime")->valuedouble : 3.0;

    double now = ACAP_DEVICE_Timestamp();

    cJSON* labels = ObjectDetection_Labels();   /* caller must cJSON_Delete */

    if (g_area_count > 0) {
        /* ── Multi-area mode ── */
        for (int i = 0; i < g_area_count; i++) {
            OccupancyArea* a = &g_areas[i];
            if (!a->active) continue;

            cJSON* raw = build_raw_counter(list, labels,
                                           stationary, moving,
                                           ageThreshold, idleThreshold,
                                           a->poly_x, a->poly_y, a->polygon_count);
            merge_with_seen(raw, &a->seen_labels);

            if (g_occ_periodic) {
                /* Accumulate raw sample for periodic averaging */
                cJSON* item = NULL;
                cJSON_ArrayForEach(item, raw) {
                    cJSON* existing = cJSON_GetObjectItem(a->acc_sum, item->string);
                    if (existing)
                        existing->valuedouble += item->valuedouble;
                    else
                        cJSON_AddNumberToObject(a->acc_sum, item->string, item->valuedouble);
                }
                a->acc_count++;
                /* Keep status up to date with live raw counter for UI display */
                char status_key[80];
                snprintf(status_key, sizeof(status_key), "area_%d", a->id);
                cJSON* entry = cJSON_CreateObject();
                cJSON_AddStringToObject(entry, "name", a->name);
                cJSON_AddItemToObject(entry, "counter", cJSON_Duplicate(raw, 1));
                ACAP_STATUS_SetObject("occupancy", status_key, entry);
                cJSON_Delete(entry);
                cJSON_Delete(raw);
            } else {
                cJSON* to_publish = apply_hold_down(raw, now, holdTime,
                                                    &a->last_published,
                                                    &a->pending_counter,
                                                    &a->decrease_due_at);
                cJSON_Delete(raw);
                if (to_publish) {
                    publish_occupancy(a->name, to_publish, now);
                    char status_key[80];
                    snprintf(status_key, sizeof(status_key), "area_%d", a->id);
                    cJSON* entry = cJSON_CreateObject();
                    cJSON_AddStringToObject(entry, "name", a->name);
                    cJSON_AddItemToObject(entry, "counter", cJSON_Duplicate(to_publish, 1));
                    ACAP_STATUS_SetObject("occupancy", status_key, entry);
                    cJSON_Delete(entry);
                }
            }
        }
    } else {
        /* ── Whole-frame fallback mode ── */
        cJSON* raw = build_raw_counter(list, labels,
                                       stationary, moving,
                                       ageThreshold, idleThreshold,
                                       NULL, NULL, 0);
        merge_with_seen(raw, &g_fallback_seen);

        if (g_occ_periodic) {
            /* Accumulate raw sample for periodic averaging */
            cJSON* item = NULL;
            cJSON_ArrayForEach(item, raw) {
                cJSON* existing = cJSON_GetObjectItem(g_fallback_acc_sum, item->string);
                if (existing)
                    existing->valuedouble += item->valuedouble;
                else
                    cJSON_AddNumberToObject(g_fallback_acc_sum, item->string, item->valuedouble);
            }
            g_fallback_acc_count++;
            /* Keep status up to date with live raw counter for UI display */
            ACAP_STATUS_SetObject("occupancy", "counter", raw);
            cJSON_Delete(raw);
        } else {
            cJSON* to_publish = apply_hold_down(raw, now, holdTime,
                                                &g_fallback_last,
                                                &g_fallback_pending,
                                                &g_fallback_due);
            cJSON_Delete(raw);
            if (to_publish) {
                publish_occupancy(NULL, to_publish, now);
                ACAP_STATUS_SetObject("occupancy", "counter", to_publish);
            }
        }
    }

    if (labels) cJSON_Delete(labels);
}

int lasty_detections_was_empty = 0;

void Detections_Data(cJSON *list) {
    char topic[128];
    if (publishDetections) {
        if (cJSON_GetArraySize(list) > 0 || !lasty_detections_was_empty) {
            snprintf(topic, sizeof(topic), "detections/%s", ACAP_DEVICE_Prop("serial"));
            cJSON* payload = cJSON_CreateObject();
            cJSON_AddItemReferenceToObject(payload, "list", list);
            MQTT_Publish_JSON(topic, payload, 0, 0);
            cJSON_Delete(payload);
        }
    }
    lasty_detections_was_empty = cJSON_GetArraySize(list) == 0;

    if (publishOccupancy)
        ProcessOccupancy(list);

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
        cJSON* enabledItem = cJSON_GetObjectItem(eventFilter, "enabled");
        if (enabledItem && enabledItem->type == cJSON_False) {
            const char* ignoreTopic = cJSON_GetObjectItem(eventFilter, "topic") ? cJSON_GetObjectItem(eventFilter, "topic")->valuestring : 0;
            if (ignoreTopic && strstr(eventTopic->valuestring, ignoreTopic)) {
                cJSON_Delete(eventTopic);
                return;
            }
        }
        eventFilter = eventFilter->next;
    }

    char topic[256];
    snprintf(topic, sizeof(topic), "event/%s/%s", ACAP_DEVICE_Prop("serial"), eventTopic->valuestring);
    cJSON_Delete(eventTopic);
    MQTT_Publish_JSON(topic, event, 0, 0);
}

/* ------------------------------------------------------------------
 * Image capture and MQTT publish
 * Uses VDO to create a temporary JPEG stream, grab one frame,
 * base64-encode it and publish to image/{serial}.
 * Resolution: 640x360 for 16:9, otherwise closest width>=640 from
 * VDO channel resolutions.
 * ------------------------------------------------------------------ */
static void Capture_And_Publish_Image(void) {
    if (!publishImage) return;

    const char *aspect  = ACAP_DEVICE_Prop("aspect");
    const char *serial  = ACAP_DEVICE_Prop("serial");  /* used in MQTT topic */
    if (!serial) serial = "000000000000";
    if (!aspect)  aspect = "16:9";

    /* Determine target resolution and fetch rotation from scene settings */
    unsigned int target_w = 640, target_h = 360;
    int rotation = 0;
    {
        cJSON *s = ACAP_Get_Config("settings");
        cJSON *scene = s ? cJSON_GetObjectItem(s, "scene") : NULL;
        cJSON *rot = scene ? cJSON_GetObjectItem(scene, "rotation") : NULL;
        if (rot) rotation = rot->valueint;
    }

    {
        GError *chErr = NULL;
        VdoChannel *ch = vdo_channel_get(1, &chErr);
        if (ch) {
            if (strcmp(aspect, "16:9") != 0) {
                /* Find smallest supported resolution with width >= 640 */
                VdoResolutionSet *rset = vdo_channel_get_resolutions(ch, NULL, &chErr);
                if (rset) {
                    unsigned int best_area = UINT_MAX;
                    for (size_t i = 0; i < rset->count; i++) {
                        unsigned int w = rset->resolutions[i].width;
                        unsigned int h = rset->resolutions[i].height;
                        if (w >= 640) {
                            unsigned int area = w * h;
                            if (area < best_area) {
                                best_area = area;
                                target_w  = w;
                                target_h  = h;
                            }
                        }
                    }
                    g_free(rset);
                }
            }
            if (chErr) g_clear_error(&chErr);
            g_object_unref(ch);
        } else {
            if (chErr) g_clear_error(&chErr);
        }
    }

    LOG_TRACE("Capturing image %ux%u (aspect %s, rotation %u)\n", target_w, target_h, aspect, rotation);

    /* Create a one-shot JPEG stream */
    GError *error = NULL;
    VdoMap *settings = vdo_map_new();
    if (!settings) { LOG_WARN("Image capture: vdo_map_new failed\n"); return; }

    vdo_map_set_uint32(settings, "channel", 1);
    vdo_map_set_uint32(settings, "format",  VDO_FORMAT_JPEG);
    vdo_map_set_uint32(settings, "width",   target_w);
    vdo_map_set_uint32(settings, "height",  target_h);
    vdo_map_set_uint32(settings, "buffer.count", 2);
    vdo_map_set_string(settings, "image.fit", "scale");

    VdoStream *stream = vdo_stream_new(settings, NULL, &error);
    g_object_unref(settings);
    if (!stream) {
        LOG_WARN("Image capture: failed to create VDO stream: %s\n",
                 error ? error->message : "unknown");
        if (error) g_clear_error(&error);
        return;
    }

    if (!vdo_stream_start(stream, &error)) {
        LOG_WARN("Image capture: failed to start stream: %s\n",
                 error ? error->message : "unknown");
        if (error) g_clear_error(&error);
        g_object_unref(stream);
        return;
    }

    VdoBuffer *buffer = vdo_stream_get_buffer(stream, &error);
    if (!buffer) {
        LOG_WARN("Image capture: failed to get buffer: %s\n",
                 error ? error->message : "unknown");
        if (error) g_clear_error(&error);
        vdo_stream_stop(stream);
        g_object_unref(stream);
        return;
    }

    /* Encode JPEG bytes as base64.
     * VdoBuffer is a typedef for VdoFrame, so frame functions work directly. */
    const guchar *data = (const guchar *)vdo_buffer_get_data(buffer);
    gsize         size = vdo_frame_get_size(buffer);
    gchar        *b64  = g_base64_encode(data, size);

    /* Build payload — serial/name/location are added automatically by MQTT_Publish_JSON */
    cJSON *payload = cJSON_CreateObject();
    cJSON_AddNumberToObject(payload, "rotation",  (double)rotation);
    cJSON_AddStringToObject(payload, "aspect",    aspect);
    cJSON_AddNumberToObject(payload, "timestamp", ACAP_DEVICE_Timestamp());
    cJSON_AddStringToObject(payload, "image",     b64 ? b64 : "");

    char topic[80];
    snprintf(topic, sizeof(topic), "image/%s", serial);
    MQTT_Publish_JSON(topic, payload, 0, 0);

    cJSON_Delete(payload);
    if (b64) g_free(b64);
    g_object_unref(buffer);

    vdo_stream_stop(stream);
    g_object_unref(stream);

    LOG("Image published to %s\n", topic);
}

/* GLib idle callback — safe to call vdo_stream_get_buffer here */
static gboolean Image_Idle_Capture(gpointer user_data) {
    (void)user_data;
    Capture_And_Publish_Image();
    return G_SOURCE_REMOVE;
}

/* Noon timer: fires once per day at 12:00 local time.
 * Rescheduled after each shot for exactly 24 h.          */
static gboolean Noon_Image_Timer(gpointer user_data);

static void Schedule_Noon_Image(void) {
    /* Cancel any existing timer */
    if (image_noon_timer_id) {
        g_source_remove(image_noon_timer_id);
        image_noon_timer_id = 0;
    }
    /* Seconds from now until next 12:00 */
    int sec_since_midnight = ACAP_DEVICE_Seconds_Since_Midnight();
    int noon = 12 * 3600;
    int delay = noon - sec_since_midnight;
    if (delay <= 0) delay += 24 * 3600;   /* already past noon today */
    image_noon_timer_id = g_timeout_add_seconds((guint)delay, Noon_Image_Timer, NULL);
    LOG_TRACE("%s: Next image scheduled in %d min\n", __func__, delay / 60);
}

static gboolean Noon_Image_Timer(gpointer user_data) {
    (void)user_data;
    image_noon_timer_id = 0;
    if (publishImage)
        g_idle_add(Image_Idle_Capture, NULL);
    Schedule_Noon_Image();          /* reschedule for tomorrow */
    return G_SOURCE_REMOVE;
}

static gboolean MQTT_Publish_Device_Status(gpointer user_data) {
    if (!publishStatus)
        return G_SOURCE_CONTINUE;

    cJSON* payload = cJSON_CreateObject();
    cJSON_AddStringToObject(payload, "model", ACAP_DEVICE_Prop("model"));
    cJSON_AddNumberToObject(payload, "Network_Kbps", (int)ACAP_DEVICE_Network_Average());
    cJSON_AddNumberToObject(payload, "CPU_average", (int)(ACAP_DEVICE_CPU_Average() * 100));
    cJSON_AddNumberToObject(payload, "Uptime_Hours", (int)(ACAP_DEVICE_Uptime() / 3600));

    char topic[256];
    snprintf(topic, sizeof(topic), "status/%s", ACAP_DEVICE_Prop("serial"));
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
            mqttConnected = 1;
            snprintf(topic, sizeof(topic), "connect/%s", ACAP_DEVICE_Prop("serial"));
            message = cJSON_CreateObject();
            cJSON_AddTrueToObject(message, "connected");
            cJSON_AddStringToObject(message, "model", ACAP_DEVICE_Prop("model"));
            cJSON_AddStringToObject(message, "address", ACAP_DEVICE_Prop("IPv4"));
            {
                cJSON *labels = ObjectDetection_Labels();
                if (labels) {
                    LOG("MQTT connect: adding %d labels to connect message\n", cJSON_GetArraySize(labels));
                    cJSON_AddItemToObject(message, "labels", labels);
                } else {
                    LOG_WARN("MQTT connect: ObjectDetection_Labels() returned NULL\n");
                }
            }
            MQTT_Publish_JSON(topic, message, 0, 1);
            cJSON_Delete(message);
            MQTT_Publish_Device_Status(0);
            if (publishImage) {
                g_idle_add(Image_Idle_Capture, NULL);
                Schedule_Noon_Image();
            }
            break;
        case MQTT_DISCONNECTING:
            mqttConnected = 0;
            snprintf(topic, sizeof(topic), "connect/%s", ACAP_DEVICE_Prop("serial"));
            message = cJSON_CreateObject();
            cJSON_AddFalseToObject(message, "connected");
            cJSON_AddStringToObject(message, "address", ACAP_DEVICE_Prop("IPv4"));
            MQTT_Publish_JSON(topic, message, 0, 1);
            cJSON_Delete(message);
            break;
        case MQTT_RECONNECTED:
            LOG("%s: Reconnected\n", __func__);
            mqttConnected = 1;
            break;
        case MQTT_DISCONNECTED:
            LOG("%s: Disconnect\n", __func__);
            mqttConnected = 0;
            break;
    }
}

void Main_MQTT_Subscription_Message(const char *topic, const char *payload) {
    LOG_TRACE("Message arrived: %s %s\n", topic, payload);
}

static GMainLoop *main_loop = NULL;

static gboolean signal_handler(gpointer user_data) {
    LOG("Received SIGTERM, initiating shutdown\n");
    if (main_loop && g_main_loop_is_running(main_loop)) {
        g_main_loop_quit(main_loop);
    }
    return G_SOURCE_REMOVE;
}

static void HTTP_ObjectDetections(const ACAP_HTTP_Response response, const ACAP_HTTP_Request request) {
    const char *method = ACAP_HTTP_Get_Method(request);
    if (!method) {
        ACAP_HTTP_Respond_Error(response, 400, "Bad request");
        return;
    }

    if (strcmp(method, "GET") != 0) {
        ACAP_HTTP_Respond_Error(response, 405, "Method Not Allowed - Use GET");
        return;
    }

    cJSON *result = cJSON_CreateObject();
    if (!result) {
        ACAP_HTTP_Respond_Error(response, 500, "Failed to create response");
        return;
    }

    cJSON *object_detection_status = ACAP_STATUS_Group("objectdetection");
    cJSON *detections_status = ACAP_STATUS_Group("detections");
    cJSON *labels = ObjectDetection_Labels();
    cJSON *source_status = ObjectDetection_Source_Status();

    if (object_detection_status)
        cJSON_AddItemToObject(result, "objectdetection", cJSON_Duplicate(object_detection_status, 1));
    if (detections_status)
        cJSON_AddItemToObject(result, "detections", cJSON_Duplicate(detections_status, 1));
    if (labels)
        cJSON_AddItemToObject(result, "labels", labels);
    if (source_status)
        cJSON_AddItemToObject(result, "source", source_status);

    ACAP_HTTP_Respond_JSON(response, result);
    cJSON_Delete(result);
}

void Settings_Updated_Callback(const char* service, cJSON* data) {

    char *json = cJSON_PrintUnformatted(data);
    if( json ) {
        LOG("Settings updated for service '%s': %s\n", service, json);
        free(json);
    } else {
        LOG("Settings updated for service '%s'\n", service);
    }

    if (strcmp(service, "publish") == 0) {
        publishEvents = cJSON_IsTrue(cJSON_GetObjectItem(data, "events"));
        publishDetections = cJSON_IsTrue(cJSON_GetObjectItem(data, "detections"));
        publishTracker = cJSON_IsTrue(cJSON_GetObjectItem(data, "tracker"));
        publishPath = cJSON_IsTrue(cJSON_GetObjectItem(data, "path"));
        publishOccupancy = cJSON_IsTrue(cJSON_GetObjectItem(data, "occupancy"));
        publishStatus = cJSON_IsTrue(cJSON_GetObjectItem(data, "status"));
        publishGeospace = cJSON_IsTrue(cJSON_GetObjectItem(data, "geospace"));
        publishAnomaly = cJSON_IsTrue(cJSON_GetObjectItem(data, "anomaly"));
        int newImage = cJSON_IsTrue(cJSON_GetObjectItem(data, "image"));
        if (newImage && !publishImage && mqttConnected) {
            /* User just enabled image publishing — send one immediately */
            g_idle_add(Image_Idle_Capture, NULL);
            Schedule_Noon_Image();
        } else if (!newImage && publishImage) {
            /* Disabled — cancel noon timer */
            if (image_noon_timer_id) {
                g_source_remove(image_noon_timer_id);
                image_noon_timer_id = 0;
            }
        }
        publishImage = newImage;
    }

    if (strcmp(service, "scene") == 0)
        ObjectDetection_Config(data);

    if (strcmp(service, "matrix") == 0)
        GeoSpace_Matrix(data);

    if (strcmp(service, "stitch") == 0)
        Stitch_Settings(data);

    if (strcmp(service, "occupancy") == 0)
        Occupancy_Load_Areas();
}

void HandleVersionUpdateConfigurations(cJSON* settings) {
    if (!settings) return;
    cJSON* scene = cJSON_GetObjectItem(settings, "scene");
    if (!scene) {
        scene = cJSON_CreateObject();
        cJSON_AddItemToObject(settings, "scene", scene);
    }
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
    if (!cJSON_GetObjectItem(scene, "aoi_polygon"))
        cJSON_AddArrayToObject(scene, "aoi_polygon");
    if (!cJSON_GetObjectItem(scene, "exclusions"))
        cJSON_AddArrayToObject(scene, "exclusions");
    if (!cJSON_GetObjectItem(scene, "ignoreClass"))
        cJSON_AddArrayToObject(scene, "ignoreClass");
    if (!cJSON_GetObjectItem(scene, "cutoff")) {
        cJSON* cutoff = cJSON_CreateObject();
        cJSON_AddFalseToObject(cutoff, "active");
        cJSON_AddNumberToObject(cutoff, "x1", 50);
        cJSON_AddNumberToObject(cutoff, "y1", 50);
        cJSON_AddNumberToObject(cutoff, "x2", 950);
        cJSON_AddNumberToObject(cutoff, "y2", 950);
        cJSON_AddItemToObject(scene, "cutoff", cutoff);
    }

    cJSON* publish = cJSON_GetObjectItem(settings, "publish");
    if (!publish) {
        publish = cJSON_CreateObject();
        cJSON_AddItemToObject(settings, "publish", publish);
    }
    if (!cJSON_GetObjectItem(publish, "geospace"))
        cJSON_AddFalseToObject(publish, "geospace");
    if (!cJSON_GetObjectItem(publish, "image"))
        cJSON_AddFalseToObject(publish, "image");

    if (!cJSON_GetObjectItem(settings, "markers"))
        cJSON_AddArrayToObject(settings, "markers");
    if (!cJSON_GetObjectItem(settings, "matrix"))
        cJSON_AddArrayToObject(settings, "matrix");
}

int main(void) {
    openlog(APP_PACKAGE, LOG_PID | LOG_CONS, LOG_USER);
    LOG("------ Starting ACAP Service ------\n");

    cJSON* settings = ACAP_Init(APP_PACKAGE, Settings_Updated_Callback);
    HandleVersionUpdateConfigurations(settings);

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
        cJSON* initLabels = ObjectDetection_Labels();
        if (initLabels) {
            LOG("Labels from ObjectDetection_Init: %d entries\n", cJSON_GetArraySize(initLabels));
            ACAP_STATUS_SetObject("detections", "labels", initLabels);
            cJSON_Delete(initLabels);
        } else {
            LOG_WARN("ObjectDetection_Labels() returned NULL after init\n");
        }
    } else {
        ACAP_STATUS_SetBool("objectdetection", "connected", 0);
        ACAP_STATUS_SetString("objectdetection", "status", "Object detection is not available");
    }

    GeoSpace_Init();
    Occupancy_Load_Areas();
    g_timeout_add_seconds(15 * 60, MQTT_Publish_Device_Status, NULL);

	Stitch_Init(Publish_Path);
	cJSON* stitchSettings = cJSON_GetObjectItem(settings, "stitch");
	if (stitchSettings)
		Stitch_Settings(stitchSettings);



	ACAP_EVENTS_Add_Event("anomaly", "DataQ: Anomaly", 1);
    ACAP_HTTP_Node("objectdetections", HTTP_ObjectDetections);
    main_loop = g_main_loop_new(NULL, FALSE);
    GSource *signal_source = g_unix_signal_source_new(SIGTERM);
    if (signal_source) {
        g_source_set_callback(signal_source, signal_handler, NULL, NULL);
        g_source_attach(signal_source, NULL);
    } else {
        LOG_WARN("Signal detection failed");
    }

    MQTT_Init(Main_MQTT_Status, Main_MQTT_Subscription_Message);
    ACAP_Set_Config("mqtt", MQTT_Settings());

    g_main_loop_run(main_loop);

    LOG("Terminating and cleaning up %s\n", APP_PACKAGE);
    Main_MQTT_Status(MQTT_DISCONNECTING);
    ObjectDetection_Shutdown();
    LOG("Object detection cleanup complete\n");
    MQTT_Cleanup();
    LOG("MQTT cleanup complete\n");
    closelog();
    return 0;
}
