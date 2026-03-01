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
#include <stdint.h>
#include <string.h>
#include <syslog.h>
#include <glib.h>
#include <time.h>
#include <glib-unix.h>
#include <signal.h>
#include <math.h>

#include <curl/curl.h>

#include "cJSON.h"
#include "ACAP.h"
#include "MQTT.h"
#include "RadarDetection.h"

#define APP_PACKAGE "DataQ"

#define LOG(fmt, args...)      { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_WARN(fmt, args...) { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args); }
//#define LOG_TRACE(fmt, args...) { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args); }
#define LOG_TRACE(fmt, args...) {}

/* VAPIX curl handle + credentials — owned by ACAP.c */
extern char  *VAPIX_Credentials;
extern CURL  *VAPIX_CURL;

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
int publishTracker = 1;
int publishPath = 1;
int publishOccupancy = 0;
int publishStatus = 1;
int publishGeospace = 0;
int publishImage = 0;
int shouldReset = 0;

#define MAX_OCCUPANCY_HISTORY 64  // Adjust if your max burst rate is super high

typedef struct {
    double timestamp_ms; ///< in seconds (use e.g. gettimeofday or monotonic)
    cJSON* counter;   ///< cJSON object: { "Human":3, "Car":5, ... }
} OccupancySample;

OccupancySample occupancy_history[MAX_OCCUPANCY_HISTORY];
int occupancy_history_start = 0;
int occupancy_history_count = 0;

/* Forward declaration — defined later in this file */
void Publish_Path(cJSON* path);
void Publish_Geospace(cJSON *tracker);

cJSON* ProcessPaths(cJSON* tracker) {
    LOG_TRACE("%s: Entry", __func__);
    if (!PathCache)
        PathCache = cJSON_CreateObject();
	
    const char* id = cJSON_GetObjectItem(tracker, "id") ? cJSON_GetObjectItem(tracker, "id")->valuestring : 0;
    if (!id) { LOG("ProcessPaths: no id\n"); return 0; }

    const char* class = cJSON_GetObjectItem(tracker, "class") ? cJSON_GetObjectItem(tracker, "class")->valuestring : 0;
    if (!class) { LOG("ProcessPaths %s: no class\n",id); return 0; }

    int active = cJSON_GetObjectItem(tracker, "active") ? cJSON_GetObjectItem(tracker, "active")->type == cJSON_True : 0;

    int confidence = cJSON_GetObjectItem(tracker, "confidence") ? cJSON_GetObjectItem(tracker, "confidence")->valueint : 0;
    if (!confidence) { LOG("ProcessPaths %s: no confidence\n",id); return 0; }

    double age = cJSON_GetObjectItem(tracker, "age") ? cJSON_GetObjectItem(tracker, "age")->valuedouble : 0;
    if (!age) { LOG("ProcessPaths %s: age=0, skip\n",id); return 0; }

    double distance = cJSON_GetObjectItem(tracker, "distance") ? cJSON_GetObjectItem(tracker, "distance")->valuedouble : 0;
    /* Note: do NOT gate on distance==0 here — the goodbye call fires with distance=0
     * when an object disappears without having crossed the movement threshold. */

    LOG("ProcessPaths id=%s active=%d age=%.1f dist=%.0f\n", id, active, age, distance);

    cJSON* path = cJSON_GetObjectItem(PathCache, id);
    if (!path && active) {
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
        cJSON_AddNumberToObject(path, "timestamp", cJSON_GetObjectItem(tracker, "birth")->valuedouble);
        cJSON_AddNumberToObject(path, "dwell", 0);
        cJSON_AddStringToObject(path, "id", id);

        cJSON* face = cJSON_GetObjectItem(tracker, "face");
        if (face) cJSON_AddBoolToObject(path, "face", cJSON_IsTrue(face));
        cJSON* hat = cJSON_GetObjectItem(tracker, "hat");
        if (hat) cJSON_AddStringToObject(path, "hat", hat->valuestring);

        cJSON* pathArr = cJSON_CreateArray();
        cJSON* pos1 = cJSON_CreateObject();
        cJSON_AddNumberToObject(pos1, "x", cJSON_GetObjectItem(tracker, "bx")->valuedouble);
        cJSON_AddNumberToObject(pos1, "y", cJSON_GetObjectItem(tracker, "by")->valuedouble);
        cJSON_AddNumberToObject(pos1, "d", cJSON_GetObjectItem(tracker, "idle")->valuedouble);
        {
            double birthTime = cJSON_GetObjectItem(tracker, "birth") ?
                              cJSON_GetObjectItem(tracker, "birth")->valuedouble : 0;
            if (birthTime > 0)
                cJSON_AddNumberToObject(pos1, "t", birthTime / 1000.0);
        }
        cJSON_AddItemToArray(pathArr, pos1);

        cJSON* pos2 = cJSON_CreateObject();
        cJSON_AddNumberToObject(pos2, "x", cJSON_GetObjectItem(tracker, "cx")->valuedouble);
        cJSON_AddNumberToObject(pos2, "y", cJSON_GetObjectItem(tracker, "cy")->valuedouble);
        cJSON_AddNumberToObject(pos2, "d", 0);
        {
            double ts2 = cJSON_GetObjectItem(tracker, "timestamp") ?
                        cJSON_GetObjectItem(tracker, "timestamp")->valuedouble : 0;
            if (ts2 > 0) cJSON_AddNumberToObject(pos2, "t", ts2 / 1000.0);
            double lat2 = cJSON_GetObjectItem(tracker, "lat") ? cJSON_GetObjectItem(tracker, "lat")->valuedouble : 0;
            double lon2 = cJSON_GetObjectItem(tracker, "lon") ? cJSON_GetObjectItem(tracker, "lon")->valuedouble : 0;
            if (lat2 && lon2) {
                cJSON_AddNumberToObject(pos2, "lat", lat2);
                cJSON_AddNumberToObject(pos2, "lon", lon2);
            }
        }
        cJSON_AddItemToArray(pathArr, pos2);

        cJSON_AddItemToObject(path, "path", pathArr);
        cJSON_AddItemToObject(PathCache, id, path);
        return 0;
    }
    if (path && active) {
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

        double idle = cJSON_GetObjectItem(tracker, "idle")->valuedouble;
        cJSON* dwell_item = cJSON_GetObjectItem(path, "dwell");
        if (idle > dwell_item->valuedouble)
            cJSON_SetNumberValue(dwell_item, idle);

        cJSON* pathArr = cJSON_GetObjectItem(path, "path");
        int pathLen = cJSON_GetArraySize(pathArr);
        if (pathLen > 0) {
            cJSON* last = cJSON_GetArrayItem(pathArr, pathLen - 1);
            cJSON_ReplaceItemInObject(last, "d", cJSON_Duplicate(cJSON_GetObjectItem(tracker, "idle"), 1));
        }

        cJSON* pos = cJSON_CreateObject();
        cJSON_AddNumberToObject(pos, "x", cJSON_GetObjectItem(tracker, "cx")->valuedouble);
        cJSON_AddNumberToObject(pos, "y", cJSON_GetObjectItem(tracker, "cy")->valuedouble);
        cJSON_AddNumberToObject(pos, "d", 0);
        {
            double ts = cJSON_GetObjectItem(tracker, "timestamp") ?
                       cJSON_GetObjectItem(tracker, "timestamp")->valuedouble : 0;
            if (ts > 0) cJSON_AddNumberToObject(pos, "t", ts / 1000.0);
        }
        double lat = cJSON_GetObjectItem(tracker, "lat") ? cJSON_GetObjectItem(tracker, "lat")->valuedouble : 0;
        double lon = cJSON_GetObjectItem(tracker, "lon") ? cJSON_GetObjectItem(tracker, "lon")->valuedouble : 0;
        if (lat && lon) {
            cJSON_AddNumberToObject(pos, "lat", lat);
            cJSON_AddNumberToObject(pos, "lon", lon);
        }
        cJSON_AddItemToArray(pathArr, pos);
        return 0;
    }
    if (path && !active) {
        cJSON* pathArr = cJSON_GetObjectItem(path, "path");
        int pathLen = cJSON_GetArraySize(pathArr);
        if (pathLen > 1) {
            cJSON_DetachItemFromObject(PathCache, id);
			double distance = cJSON_GetObjectItem(path,"distance")->valuedouble;
			double age = cJSON_GetObjectItem(path,"age")->valuedouble;
			cJSON* dir_item = cJSON_GetObjectItem(tracker, "directions");
			cJSON_AddNumberToObject(path, "directions", dir_item ? dir_item->valueint : 0);
			double maxSpeedVal = cJSON_GetObjectItem(tracker,"maxSpeed") ? cJSON_GetObjectItem(tracker,"maxSpeed")->valuedouble : 0;
			cJSON *radarPath = cJSON_CreateObject();
			cJSON_AddNumberToObject(radarPath, "maxSpeed", maxSpeedVal);
			cJSON_AddItemToObject(path, "radar", radarPath);
			LOG("Path complete: id=%s age=%.1f dist=%.0f points=%d\n", id, age, distance, pathLen);
            return path;
        }
        cJSON_DetachItemFromObject(PathCache, id);
        cJSON_Delete(path);
        return 0;
    }
    LOG_TRACE("%s: Exit", __func__);
    return 0;
}



/* Compute the object's geographic position from radar polar coords and
 * device location/heading.  Writes "lat" and "lon" into *tracker* and
 * returns 1 if the device is calibrated (lat/lon != 0), 0 otherwise.
 * Safe to call with any tracker — returns 0 silently if data is missing. */
static int
Geospace_Enrich_Tracker(cJSON *tracker) {
    cJSON *radarObj = cJSON_GetObjectItem(tracker, "radar");
    if (!radarObj) return 0;

    double dist_m    = cJSON_GetObjectItem(radarObj, "distance") ? cJSON_GetObjectItem(radarObj, "distance")->valuedouble : 0.0;
    double angle_deg = cJSON_GetObjectItem(radarObj, "angle")    ? cJSON_GetObjectItem(radarObj, "angle")->valuedouble    : 0.0;
    if (dist_m <= 0.0) return 0;

    cJSON *settings = ACAP_Get_Config("settings");
    cJSON *locCfg   = settings ? cJSON_GetObjectItem(settings, "location") : NULL;
    if (!locCfg) return 0;

    double device_lat = cJSON_GetObjectItem(locCfg, "lat")     ? cJSON_GetObjectItem(locCfg, "lat")->valuedouble     : 0.0;
    double device_lon = cJSON_GetObjectItem(locCfg, "lon")     ? cJSON_GetObjectItem(locCfg, "lon")->valuedouble     : 0.0;
    double heading    = cJSON_GetObjectItem(locCfg, "heading") ? cJSON_GetObjectItem(locCfg, "heading")->valuedouble : 0.0;
    if (device_lat == 0.0 && device_lon == 0.0) return 0;

    double bearing_rad = fmod(heading + angle_deg + 360.0, 360.0) * M_PI / 180.0;
    double lat_rad     = device_lat * M_PI / 180.0;
    double obj_lat = device_lat + dist_m * cos(bearing_rad) / 111320.0;
    double obj_lon = device_lon + dist_m * sin(bearing_rad) / (111320.0 * cos(lat_rad));

    /* Write back into tracker so ProcessPaths and Publish_Geospace can read them */
    cJSON_DeleteItemFromObject(tracker, "lat");
    cJSON_DeleteItemFromObject(tracker, "lon");
    cJSON_AddNumberToObject(tracker, "lat", obj_lat);
    cJSON_AddNumberToObject(tracker, "lon", obj_lon);
    return 1;
}

void Tracker_Data(cJSON *tracker, int timer) {
    char topic[128];
    LOG("Tracker_Data called timer=%d\n", timer);

    /* Enrich tracker with geographic coords whenever device is calibrated.
     * This must happen before publishTracker MQTT, ProcessPaths, and
     * Publish_Geospace so all three see consistent lat/lon values. */
    Geospace_Enrich_Tracker(tracker);

    if (publishTracker) {
        snprintf(topic, sizeof(topic), "tracker/%s", ACAP_DEVICE_Prop("serial"));
        MQTT_Publish_JSON(topic, tracker, 0, 0);
    }

    if (publishGeospace)
        Publish_Geospace(tracker);

    if (publishPath && !timer && tracker) {
		cJSON* path = ProcessPaths(tracker);
		if (path) Publish_Path(path);
	}

    cJSON_Delete(tracker);
}

/* -------------------------------------------------------
 * Deferred VAPIX location push — runs on the GLib main
 * loop so it never blocks the FastCGI HTTP thread.
 * ------------------------------------------------------- */
static double pending_vapix_lat     = 0.0;
static double pending_vapix_lon     = 0.0;
static double pending_vapix_heading = 0.0;

static gboolean Store_Location_VAPIX(gpointer userdata) {
    (void)userdata;
    ACAP_DEVICE_Set_Location(pending_vapix_lat, pending_vapix_lon, pending_vapix_heading);
    return G_SOURCE_REMOVE;
}

void Publish_Geospace(cJSON *tracker) {
    /* lat/lon already computed by Geospace_Enrich_Tracker() in Tracker_Data */
    double obj_lat = cJSON_GetObjectItem(tracker, "lat") ? cJSON_GetObjectItem(tracker, "lat")->valuedouble : 0.0;
    double obj_lon = cJSON_GetObjectItem(tracker, "lon") ? cJSON_GetObjectItem(tracker, "lon")->valuedouble : 0.0;
    if (obj_lat == 0.0 && obj_lon == 0.0) return;

    cJSON *radarObj = cJSON_GetObjectItem(tracker, "radar");
    int active = cJSON_GetObjectItem(tracker, "active")
                 && cJSON_GetObjectItem(tracker, "active")->type == cJSON_True;

    cJSON *geo = cJSON_CreateObject();
    cJSON_AddStringToObject(geo, "id",         cJSON_GetObjectItem(tracker, "id")    ? cJSON_GetObjectItem(tracker, "id")->valuestring    : "?");
    cJSON_AddStringToObject(geo, "class",      cJSON_GetObjectItem(tracker, "class") ? cJSON_GetObjectItem(tracker, "class")->valuestring : "Unknown");
    cJSON_AddNumberToObject(geo, "lat",        obj_lat);
    cJSON_AddNumberToObject(geo, "lon",        obj_lon);
    cJSON_AddBoolToObject  (geo, "active",     active);
    cJSON_AddNumberToObject(geo, "confidence", cJSON_GetObjectItem(tracker, "confidence") ? cJSON_GetObjectItem(tracker, "confidence")->valueint    : 0);
    cJSON_AddNumberToObject(geo, "age",        cJSON_GetObjectItem(tracker, "age")        ? cJSON_GetObjectItem(tracker, "age")->valuedouble        : 0.0);
    if (radarObj) cJSON_AddItemToObject(geo, "radar", cJSON_Duplicate(radarObj, 1));

    char topic[128];
    snprintf(topic, sizeof(topic), "geospace/%s", ACAP_DEVICE_Prop("serial"));
    MQTT_Publish_JSON(topic, geo, 0, 0);
    cJSON_Delete(geo);
}

void Publish_Path( cJSON* path ){
	if( !path ) return;
	/* Discard paths with fewer than 3 sampled positions */
	cJSON* pathArray = cJSON_GetObjectItem(path, "path");
	if( !pathArray || cJSON_GetArraySize(pathArray) < 3 ) {
		cJSON_Delete(path);
		return;
	}
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

    /* Radar only reports moving objects — when an object stops it
     * disappears from tracking entirely.  We count every tracked
     * object that has been present long enough to be reliable. */
    double ageThreshold = cJSON_GetObjectItem(occupancy, "ageThreshold")
                          ? cJSON_GetObjectItem(occupancy, "ageThreshold")->valuedouble : 2.0;
    int listSize = cJSON_GetArraySize(list);

    if (listSize == 0) {
        cJSON* empty = cJSON_CreateObject();
        int changed = 1;
        if (last_occupancy && Check_Counters_Equal(last_occupancy, empty))
            changed = 0;
        if (!changed) {
            cJSON_Delete(empty);
            return 0;
        }
        double now = ACAP_DEVICE_Timestamp();
        push_occupancy_zero_sample(now);
        if (last_occupancy) cJSON_Delete(last_occupancy);
        last_occupancy = cJSON_Duplicate(empty, 1);
        return empty;
    }

    cJSON* counters = cJSON_CreateObject();
    for (int i = 0; i < listSize; ++i) {
        cJSON* det = cJSON_GetArrayItem(list, i);
        cJSON* cls_item = cJSON_GetObjectItem(det, "class");
        cJSON* age_item = cJSON_GetObjectItem(det, "age");
        if (!cls_item || !age_item) continue;
        double age = age_item->valuedouble;
        if (age < ageThreshold) continue;
        const char* cls = cls_item->valuestring;
        cJSON* curr = cJSON_GetObjectItem(counters, cls);
        if (curr) curr->valuedouble += 1.0;
        else cJSON_AddNumberToObject(counters, cls, 1.0);
    }

    /* Round counts */
    cJSON* item = counters->child;
    while (item) {
        item->valueint = (int)(item->valuedouble + 0.5);
        item->valuedouble = (double)item->valueint;
        item = item->next;
    }
    /* Drop zero-count classes */
    item = counters->child;
    while (item) {
        cJSON* next = item->next;
        if (item->valueint == 0)
            cJSON_DeleteItemFromObjectCaseSensitive(counters, item->string);
        item = next;
    }

    if (last_occupancy && Check_Counters_Equal(last_occupancy, counters)) {
        cJSON_Delete(counters);
        return 0;
    }

    double now = ACAP_DEVICE_Timestamp();
    push_occupancy_sample(now, counters);
    if (last_occupancy) cJSON_Delete(last_occupancy);
    last_occupancy = cJSON_Duplicate(counters, 1);
    ACAP_STATUS_SetObject("occupancy", "counter", counters);
    return counters;
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
				snprintf(topic, sizeof(topic), "occupancy/%s", ACAP_DEVICE_Prop("serial"));
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

/* -------------------------------------------------------
 * Image capture + MQTT publish
 * ------------------------------------------------------- */
typedef struct {
    unsigned char *data;
    size_t         size;
} BinaryBuffer;

static size_t binary_write_cb(void *ptr, size_t size, size_t nmemb, void *userdata) {
    BinaryBuffer *buf = (BinaryBuffer *)userdata;
    size_t bytes = size * nmemb;
    unsigned char *tmp = realloc(buf->data, buf->size + bytes);
    if (!tmp) return 0;
    memcpy(tmp + buf->size, ptr, bytes);
    buf->data = tmp;
    buf->size += bytes;
    return bytes;
}

static const char b64_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static char *base64_encode(const unsigned char *in, size_t len, size_t *out_len) {
    size_t olen = 4 * ((len + 2) / 3);
    char  *out  = malloc(olen + 1);
    if (!out) return NULL;
    size_t i, j;
    for (i = 0, j = 0; i < len;) {
        uint32_t a = i < len ? in[i++] : 0;
        uint32_t b = i < len ? in[i++] : 0;
        uint32_t c = i < len ? in[i++] : 0;
        uint32_t triple = (a << 16) | (b << 8) | c;
        out[j++] = b64_table[(triple >> 18) & 0x3F];
        out[j++] = b64_table[(triple >> 12) & 0x3F];
        out[j++] = (i > len + 1) ? '=' : b64_table[(triple >> 6) & 0x3F];
        out[j++] = (i > len)     ? '=' : b64_table[triple & 0x3F];
    }
    out[j] = '\0';
    if (out_len) *out_len = j;
    return out;
}

static gboolean Publish_Image(gpointer user_data) {
    if (!publishImage || !VAPIX_Credentials || !VAPIX_CURL) return G_SOURCE_REMOVE;

    /* Pick a resolution: smallest with width >= 640 for the device aspect */
    const char *aspect = ACAP_DEVICE_Prop("aspect");
    if (!aspect) aspect = "16:9";
    const char *resolution = "640x360";  /* default for 16:9 */
    cJSON *resolutions = ACAP_DEVICE_JSON("resolutions");
    if (resolutions) {
        cJSON *list = cJSON_GetObjectItem(resolutions, aspect);
        if (list) {
            cJSON *item = list->child;
            while (item) {
                /* Pick smallest with width >= 640 */
                int w = 0;
                sscanf(item->valuestring, "%d", &w);
                if (w >= 640) { resolution = item->valuestring; break; }
                item = item->next;
            }
        }
    }

    char url[256];
    snprintf(url, sizeof(url),
             "http://127.0.0.12/axis-cgi/jpg/image.cgi?resolution=%s", resolution);

    BinaryBuffer buf = { NULL, 0 };
    curl_easy_setopt(VAPIX_CURL, CURLOPT_URL, url);
    curl_easy_setopt(VAPIX_CURL, CURLOPT_USERPWD, VAPIX_Credentials);
    curl_easy_setopt(VAPIX_CURL, CURLOPT_HTTPAUTH, CURLAUTH_BASIC);
    curl_easy_setopt(VAPIX_CURL, CURLOPT_HTTPGET, 1L);
    curl_easy_setopt(VAPIX_CURL, CURLOPT_WRITEFUNCTION, binary_write_cb);
    curl_easy_setopt(VAPIX_CURL, CURLOPT_WRITEDATA, &buf);

    CURLcode res = curl_easy_perform(VAPIX_CURL);
    if (res != CURLE_OK || !buf.data || buf.size == 0) {
        LOG_WARN("Image capture failed: %s\n", curl_easy_strerror(res));
        free(buf.data);
        return G_SOURCE_REMOVE;
    }
    long http_code = 0;
    curl_easy_getinfo(VAPIX_CURL, CURLINFO_RESPONSE_CODE, &http_code);
    if (http_code >= 300) {
        LOG_WARN("Image capture HTTP %ld\n", http_code);
        free(buf.data);
        return G_SOURCE_REMOVE;
    }

    size_t b64len = 0;
    char *b64 = base64_encode(buf.data, buf.size, &b64len);
    free(buf.data);
    if (!b64) { LOG_WARN("Image base64 encode failed\n"); return G_SOURCE_REMOVE; }

    cJSON *payload = cJSON_CreateObject();
    cJSON_AddNumberToObject(payload, "rotation", ACAP_DEVICE_Prop_Int("rotation"));
    cJSON_AddStringToObject(payload, "aspect", aspect);
    cJSON_AddNumberToObject(payload, "timestamp", ACAP_DEVICE_Timestamp());
    cJSON_AddStringToObject(payload, "image", b64);
    free(b64);

    char topic[128];
    snprintf(topic, sizeof(topic), "image/%s", ACAP_DEVICE_Prop("serial"));
    MQTT_Publish_JSON(topic, payload, 0, 0);
    cJSON_Delete(payload);
    LOG("Published image (%zu bytes, %s)\n", buf.size, resolution);
    return G_SOURCE_REMOVE;
}

/* Publish image once immediately, then daily at noon */
static gboolean Image_Daily_Check(gpointer user_data) {
    if (!publishImage) return G_SOURCE_CONTINUE;
    int secs = ACAP_DEVICE_Seconds_Since_Midnight();
    /* Fire within 60-second window around noon (43200s) */
    if (secs >= 43200 && secs < 43260) {
        g_idle_add(Publish_Image, NULL);
    }
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
            snprintf(topic, sizeof(topic), "connect/%s", ACAP_DEVICE_Prop("serial"));
            message = cJSON_CreateObject();
            cJSON_AddTrueToObject(message, "connected");
            cJSON_AddStringToObject(message, "model", ACAP_DEVICE_Prop("model"));
            cJSON_AddStringToObject(message, "address", ACAP_DEVICE_Prop("IPv4"));
            {
                /* Build labels array from radar classification table */
                cJSON* labels = cJSON_CreateArray();
                const char* radar_classes[] = {"Human", "Vehicle", "Unknown"};
                int nclasses = 3;
                cJSON* settings_lbl = ACAP_Get_Config("settings");
                cJSON* scene_lbl = settings_lbl ? cJSON_GetObjectItem(settings_lbl, "scene") : NULL;
                cJSON* ignoreList = scene_lbl ? cJSON_GetObjectItem(scene_lbl, "ignoreClass") : NULL;
                for (int li = 0; li < nclasses; li++) {
                    cJSON* lbl = cJSON_CreateObject();
                    cJSON_AddStringToObject(lbl, "id", radar_classes[li]);
                    cJSON_AddStringToObject(lbl, "name", radar_classes[li]);
                    int enabled = 1;
                    if (ignoreList) {
                        cJSON* ig = ignoreList->child;
                        while (ig) {
                            if (ig->valuestring && strcmp(ig->valuestring, radar_classes[li]) == 0) {
                                enabled = 0;
                                break;
                            }
                            ig = ig->next;
                        }
                    }
                    cJSON_AddBoolToObject(lbl, "enabled", enabled);
                    cJSON_AddItemToArray(labels, lbl);
                }
                cJSON_AddItemToObject(message, "labels", labels);
            }
            MQTT_Publish_JSON(topic, message, 0, 1);
            cJSON_Delete(message);
			MQTT_Publish_Device_Status(0);
            break;
        case MQTT_DISCONNECTING:
            snprintf(topic, sizeof(topic), "connect/%s", ACAP_DEVICE_Prop("serial"));
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

    char *json = cJSON_PrintUnformatted(data);
    if( json ) {
        LOG("Settings updated for service '%s': %s\n", service, json);
        free(json);
    } else {
        LOG("Settings updated for service '%s'\n", service);
    }

    if (strcmp(service, "location") == 0) {
        /* Location is persisted by ACAP.c's settings endpoint.  Also push
         * lat/lon/heading to the official VAPIX geolocation/sphere API,
         * deferred to the GLib main loop so the HTTP response is not
         * delayed by the synchronous VAPIX call. */
        pending_vapix_lat     = cJSON_GetObjectItem(data, "lat")     ? cJSON_GetObjectItem(data, "lat")->valuedouble     : 0.0;
        pending_vapix_lon     = cJSON_GetObjectItem(data, "lon")     ? cJSON_GetObjectItem(data, "lon")->valuedouble     : 0.0;
        pending_vapix_heading = cJSON_GetObjectItem(data, "heading") ? cJSON_GetObjectItem(data, "heading")->valuedouble : 0.0;
        g_idle_add(Store_Location_VAPIX, NULL);
    }

    if (strcmp(service, "publish") == 0) {
        publishEvents = cJSON_IsTrue(cJSON_GetObjectItem(data, "events"));
        publishDetections = cJSON_IsTrue(cJSON_GetObjectItem(data, "detections"));
        publishTracker = cJSON_IsTrue(cJSON_GetObjectItem(data, "tracker"));
        publishPath = cJSON_IsTrue(cJSON_GetObjectItem(data, "path"));
        publishOccupancy = cJSON_IsTrue(cJSON_GetObjectItem(data, "occupancy"));
        publishStatus    = cJSON_IsTrue(cJSON_GetObjectItem(data, "status"));
        publishGeospace  = cJSON_IsTrue(cJSON_GetObjectItem(data, "geospace"));
        int wasImage = publishImage;
        publishImage = cJSON_IsTrue(cJSON_GetObjectItem(data, "image"));
        if (publishImage && !wasImage)
            g_idle_add(Publish_Image, NULL);
    }

    if (strcmp(service, "scene") == 0)
        RadarDetection_Config(data);

}

void HandleVersionUpdateConfigurations(cJSON* settings) {
    cJSON* scene = cJSON_GetObjectItem(settings, "scene");
    if (!cJSON_GetObjectItem(scene, "maxIdle"))
        cJSON_AddNumberToObject(scene, "maxIdle", 0);
    if (!cJSON_GetObjectItem(scene, "tracker_confidence"))
        cJSON_AddTrueToObject(scene, "tracker_confidence");
    if (!cJSON_GetObjectItem(scene, "hanging_objects"))
        cJSON_AddNumberToObject(scene, "hanging_objects", 5);

    /* AOI: always enforce full 0-1000 range for radar.
     * Old versions used 50/950 which clipped detections at the far end
     * of the indoor coverage area. Force-update even if aoi already exists. */
    {
        cJSON* aoi = cJSON_GetObjectItem(scene, "aoi");
        if (!aoi) {
            aoi = cJSON_CreateObject();
            cJSON_AddItemToObject(scene, "aoi", aoi);
        }
        /* Replace or add each boundary to ensure full-range coverage */
        if (cJSON_GetObjectItem(aoi, "x1")) cJSON_ReplaceItemInObject(aoi, "x1", cJSON_CreateNumber(0));
        else cJSON_AddNumberToObject(aoi, "x1", 0);
        if (cJSON_GetObjectItem(aoi, "x2")) cJSON_ReplaceItemInObject(aoi, "x2", cJSON_CreateNumber(1000));
        else cJSON_AddNumberToObject(aoi, "x2", 1000);
        if (cJSON_GetObjectItem(aoi, "y1")) cJSON_ReplaceItemInObject(aoi, "y1", cJSON_CreateNumber(0));
        else cJSON_AddNumberToObject(aoi, "y1", 0);
        if (cJSON_GetObjectItem(aoi, "y2")) cJSON_ReplaceItemInObject(aoi, "y2", cJSON_CreateNumber(1000));
        else cJSON_AddNumberToObject(aoi, "y2", 1000);
    }
    if (!cJSON_GetObjectItem(scene, "ignoreClass"))
        cJSON_AddArrayToObject(scene, "ignoreClass");

    cJSON* publish = cJSON_GetObjectItem(settings, "publish");
    if (!publish) {
        publish = cJSON_CreateObject();
        cJSON_AddItemToObject(settings, "publish", publish);
    }
    if (!cJSON_GetObjectItem(publish, "geospace"))
        cJSON_AddFalseToObject(publish, "geospace");
    if (!cJSON_GetObjectItem(publish, "image"))
        cJSON_AddFalseToObject(publish, "image");
}

int main(void) {
    openlog(APP_PACKAGE, LOG_PID | LOG_CONS, LOG_USER);
    LOG("------ Starting ACAP Service ------\n");

    cJSON* settings = ACAP(APP_PACKAGE, Settings_Updated_Callback);
    HandleVersionUpdateConfigurations(settings);
    /* Re-apply scene config after version fix-up so runtime AOI reflects
     * the corrected (full 0-1000) values, not the old 50/950 from storage. */
    RadarDetection_Config(cJSON_GetObjectItem(settings, "scene"));

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

    if (RadarDetection_Init(Detections_Data, Tracker_Data)) {
        ACAP_STATUS_SetBool("objectdetection", "connected", 1);
        ACAP_STATUS_SetString("objectdetection", "status", "OK");
    } else {
        ACAP_STATUS_SetBool("objectdetection", "connected", 0);
        ACAP_STATUS_SetString("objectdetection", "status", "Object detection is not available");
    }

    g_timeout_add_seconds(15 * 60, MQTT_Publish_Device_Status, NULL);

    /* Image publishing: load initial state, schedule daily check every 60s */
    cJSON* pub = cJSON_GetObjectItem(settings, "publish");
    publishImage = pub && cJSON_IsTrue(cJSON_GetObjectItem(pub, "image"));
    g_timeout_add_seconds(60, Image_Daily_Check, NULL);

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
    RadarDetection_Cleanup();
    Main_MQTT_Status(MQTT_DISCONNECTING);
    MQTT_Cleanup();
    ACAP_Cleanup();
    closelog();
    return 0;
}
