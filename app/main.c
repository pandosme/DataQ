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
	
    const char* id = cJSON_GetObjectItem(tracker, "id") ? cJSON_GetObjectItem(tracker, "id")->valuestring : 0;
    if (!id) return 0;

    const char* class = cJSON_GetObjectItem(tracker, "class") ? cJSON_GetObjectItem(tracker, "class")->valuestring : 0;
    if (!class) return 0;

    int active = cJSON_GetObjectItem(tracker, "active") ? cJSON_GetObjectItem(tracker, "active")->type == cJSON_True : 0;

    int confidence = cJSON_GetObjectItem(tracker, "confidence") ? cJSON_GetObjectItem(tracker, "confidence")->valueint : 0;
    if (!confidence) return 0;

    double age = cJSON_GetObjectItem(tracker, "age") ? cJSON_GetObjectItem(tracker, "age")->valuedouble : 0;
    if (!age) return 0;

    double distance = cJSON_GetObjectItem(tracker, "distance") ? cJSON_GetObjectItem(tracker, "distance")->valuedouble : 0;
    if (!distance) return 0;

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

        double lat = cJSON_GetObjectItem(tracker, "lat") ? cJSON_GetObjectItem(tracker, "lat")->valuedouble : 0;
        double lon = cJSON_GetObjectItem(tracker, "lon") ? cJSON_GetObjectItem(tracker, "lon")->valuedouble : 0;
        double blat = 0, blon = 0;
        if (lat && lon)
            GeoSpace_transform(cJSON_GetObjectItem(tracker, "bx")->valueint, cJSON_GetObjectItem(tracker, "by")->valueint, &blat, &blon);

        cJSON* pathArr = cJSON_CreateArray();
        cJSON* pos1 = cJSON_CreateObject();
        cJSON_AddNumberToObject(pos1, "x", cJSON_GetObjectItem(tracker, "bx")->valuedouble);
        cJSON_AddNumberToObject(pos1, "y", cJSON_GetObjectItem(tracker, "by")->valuedouble);
        cJSON_AddNumberToObject(pos1, "d", cJSON_GetObjectItem(tracker, "idle")->valuedouble);
        if (blat && blon) {
            cJSON_AddNumberToObject(pos1, "lat", blat);
            cJSON_AddNumberToObject(pos1, "lon", blon);
        }
        cJSON_AddItemToArray(pathArr, pos1);

        cJSON* pos2 = cJSON_CreateObject();
        cJSON_AddNumberToObject(pos2, "x", cJSON_GetObjectItem(tracker, "cx")->valuedouble);
        cJSON_AddNumberToObject(pos2, "y", cJSON_GetObjectItem(tracker, "cy")->valuedouble);
        if (lat && lon) {
            cJSON_AddNumberToObject(pos2, "lat", lat);
            cJSON_AddNumberToObject(pos2, "lon", lon);
        }
        cJSON_AddNumberToObject(pos2, "d", 0);
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
        if (pathLen > 2) {
            cJSON_DetachItemFromObject(PathCache, id);
            return path;
        }
        cJSON_DetachItemFromObject(PathCache, id);
        cJSON_Delete(path);
        return 0;
    }
    return 0;
}

void Tracker_Data(cJSON *tracker) {
    char topic[128];
    int published_by_timer = 0;
    if (cJSON_GetObjectItem(tracker, "timer")) {
        published_by_timer = 1;
        cJSON_DeleteItemFromObject(tracker, "timer");
    }

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
            cJSON_AddStringToObject(geospaceObject, "id", cJSON_GetObjectItem(tracker, "id")->valuestring);
			cJSON_AddItemToObject(geospaceObject, "active", cJSON_Duplicate(cJSON_GetObjectItem(tracker,"active"), 1));
            sprintf(topic, "geospace/%s", ACAP_DEVICE_Prop("serial"));
            MQTT_Publish_JSON(topic, geospaceObject, 0, 0);
            cJSON_Delete(geospaceObject);
        }
    }

    cJSON* statusPaths = ACAP_STATUS_Object("detections", "paths");
    if (publishPath && !published_by_timer) {
        cJSON* path = ProcessPaths(tracker);
        if (path) {
            sprintf(topic, "path/%s", ACAP_DEVICE_Prop("serial"));
            MQTT_Publish_JSON(topic, path, 0, 0);
            cJSON_AddItemToArray(statusPaths, cJSON_Duplicate(path, 1));
            cJSON_Delete(path);
        }
    }
    while (cJSON_GetArraySize(statusPaths) > 10)
        cJSON_DeleteItemFromArray(statusPaths, 0);
    cJSON_Delete(tracker);
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
    
    if (listSize == 0) {
        // Empty array: push an empty ("zero") sample!
        double now = ACAP_DEVICE_Timestamp();
        push_occupancy_zero_sample(now);
        cJSON* empty = cJSON_CreateObject();
        int changed = 1;
        if (last_occupancy && cJSON_Compare(last_occupancy, empty, 1))
            changed = 0;
        if (!changed) {
            cJSON_Delete(empty);
            return 0;
        }
        if (last_occupancy)
            cJSON_Delete(last_occupancy);
        last_occupancy = cJSON_Duplicate(empty, 1);
        return empty;
    }
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
                curr->valueint = (int)curr->valuedouble;
            } else {
                cJSON_AddNumberToObject(counters, cls, 1);
            }
        }
    }
    cJSON* item = counters->child;
    while (item) {
        cJSON* next = item->next;
        if (item->valueint == 0)
            cJSON_DeleteItemFromObjectCaseSensitive(counters, item->string);
        item = next;
    }
    // Push sample to buffer
    double now = ACAP_DEVICE_Timestamp();
    push_occupancy_sample(now, counters);

    if (Check_Counters_Equal(last_occupancy, counters) == 1)
        return 0;
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
        if (counter)
            cJSON_Delete(counter); // Unstabilized, only for transition/compat use
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

static GMainLoop *main_loop = NULL;

static gboolean signal_handler(gpointer user_data) {
    LOG("Received SIGTERM, initiating shutdown\n");
    if (main_loop && g_main_loop_is_running(main_loop)) {
        g_main_loop_quit(main_loop);
    }
    return G_SOURCE_REMOVE;
}

void Settings_Updated_Callback(const char* service, cJSON* data) {
    char* json = cJSON_PrintUnformatted(data);
    if (json) {
        LOG_TRACE("%s: %s=%s\n", __func__, service, json);
        free(json);
    } else {
        LOG_WARN("%s: JSON Parse error\n", __func__);
    }

    if (strcmp(service, "publish") == 0) {
        publishEvents = cJSON_IsTrue(cJSON_GetObjectItem(data, "events"));
        publishDetections = cJSON_IsTrue(cJSON_GetObjectItem(data, "detections"));
        publishTracker = cJSON_IsTrue(cJSON_GetObjectItem(data, "tracker"));
        publishPath = cJSON_IsTrue(cJSON_GetObjectItem(data, "path"));
        publishOccupancy = cJSON_IsTrue(cJSON_GetObjectItem(data, "occupancy"));
        publishStatus = cJSON_IsTrue(cJSON_GetObjectItem(data, "status"));
        publishGeospace = cJSON_IsTrue(cJSON_GetObjectItem(data, "geospace"));
    }

    if (strcmp(service, "scene") == 0)
        ObjectDetection_Config(data);

    if (strcmp(service, "matrix") == 0)
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
