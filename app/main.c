#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <glib.h>
#include <glib-unix.h>
#include <signal.h>
#include <math.h>

#include "cJSON.h"
#include "ACAP.h"
#include "MQTT.h"
#include "ObjectDetection.h"
#include "GeoSpace.h"


#define APP_PACKAGE	"DataQ"

#define LOG(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args);}
#define LOG_WARN(fmt, args...)    { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args);}
//#define LOG_TRACE(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_TRACE(fmt, args...)    {}

#define JGET(obj, key) cJSON_GetObjectItem(obj, key)
#define JSTR(obj, key) (JGET(obj, key) ? JGET(obj, key)->valuestring : "")
#define JNUM(obj, key) (JGET(obj, key) ? JGET(obj, key)->valuedouble : 0)
#define JBOOL(obj, key) (JGET(obj, key) && JGET(obj, key)->type == cJSON_True)

cJSON* activeTrackers = 0;
cJSON* PreviousPosition = 0;
cJSON* lastPublishedTracker = 0;
cJSON* pathsCache = 0;
cJSON* occupancyDetectionCounter = 0;
cJSON* classCounterArrays = 0;
cJSON* previousOccupancy = 0;
cJSON* blacklist = 0;

int lastDetectionListWasEmpty = 0;
int publishEvents = 1;
int publishDetections = 1;
int publishTracker = 1;
int publishPath = 1;
int publishOccupancy = 0;
int publishStatus = 1;
int publishGeospace = 0;
int shouldReset = 0;

int
Label_Blacklisted(cJSON* detection) {
    if (!blacklist || !detection) {
        return 0;
    }
	const char* label = cJSON_GetObjectItem(detection,"class")->valuestring;
    
    cJSON* item = blacklist->child;
    while (item) {
        if (cJSON_IsString(item) && strcmp(label, item->valuestring) == 0) {
            return 1;
        }
        item = item->next;
    }
    return 0;
}

cJSON* Process_Paths(cJSON* trackers) {
    LOG_TRACE("%s: Entry\n", __func__);

    if (!pathsCache)
        pathsCache = cJSON_CreateObject();
    cJSON* response = cJSON_CreateArray();

    cJSON* settings = ACAP_Get_Config("settings");
    if (!settings) return response;
    cJSON* pathFilter = JGET(settings, "pathFilter");
    if (!pathFilter) return response;
    cJSON* aoi = JGET(pathFilter, "aoi");
    if (!aoi) return response;

    cJSON* tracker = trackers->child;
    while (tracker) {
        const char* id = JSTR(tracker, "id");
        cJSON* path = JGET(pathsCache, id);

        if (!path) {
            if (JBOOL(tracker, "active") && !JBOOL(tracker, "ignore")) {
                path = cJSON_CreateObject();
                cJSON_AddStringToObject(path, "id", id);
                cJSON_AddStringToObject(path, "class", JSTR(tracker, "class"));
                cJSON_AddNumberToObject(path, "confidence", JNUM(tracker, "confidence"));
                cJSON_AddNumberToObject(path, "timestamp", JNUM(tracker, "birth"));
                cJSON_AddNumberToObject(path, "age", JNUM(tracker, "age"));
                cJSON_AddNumberToObject(path, "dx", JNUM(tracker, "dx"));
                cJSON_AddNumberToObject(path, "dy", JNUM(tracker, "dy"));
                cJSON_AddNumberToObject(path, "distance", JNUM(tracker, "distance"));
                cJSON_AddNumberToObject(path, "dwell", 0);
                cJSON_AddStringToObject(path, "color", JSTR(tracker, "color"));
                cJSON_AddStringToObject(path, "color2", JSTR(tracker, "color2"));

                cJSON* position = cJSON_CreateObject();
                cJSON_AddNumberToObject(position, "x", JNUM(tracker, "cx"));
                cJSON_AddNumberToObject(position, "y", JNUM(tracker, "cy"));
                if (JGET(tracker, "lat"))
                    cJSON_AddNumberToObject(position, "lat", JNUM(tracker, "lat"));
                if (JGET(tracker, "lon"))
                    cJSON_AddNumberToObject(position, "lon", JNUM(tracker, "lon"));
                cJSON_AddNumberToObject(position, "d", 0);

                cJSON* pathList = cJSON_CreateArray();
                cJSON_AddItemToArray(pathList, position);
                cJSON_AddItemToObject(path, "path", pathList);
                cJSON_AddItemToObject(pathsCache, id, path);
            }
        } else {
            // Update path meta
            cJSON_ReplaceItemInObject(path, "confidence", cJSON_CreateNumber(JNUM(tracker, "confidence")));
            cJSON_ReplaceItemInObject(path, "class", cJSON_CreateString(JSTR(tracker, "class")));
            cJSON_ReplaceItemInObject(path, "age", cJSON_CreateNumber(JNUM(tracker, "age")));
            cJSON_ReplaceItemInObject(path, "dx", cJSON_CreateNumber(JNUM(tracker, "dx")));
            cJSON_ReplaceItemInObject(path, "dy", cJSON_CreateNumber(JNUM(tracker, "dy")));
            cJSON_ReplaceItemInObject(path, "distance", cJSON_CreateNumber(JNUM(tracker, "distance")));
            cJSON_ReplaceItemInObject(path, "topVelocity", cJSON_CreateNumber(JNUM(tracker, "topVelocity")));
            cJSON_ReplaceItemInObject(path, "color", cJSON_CreateString(JSTR(tracker, "color")));
            cJSON_ReplaceItemInObject(path, "color2", cJSON_CreateString(JSTR(tracker, "color2")));

            // Dwell time calculation
            double duration = JNUM(tracker, "timestamp") - JNUM(path, "sampletime");
            duration /= 1000.0;
            if (duration > JNUM(path, "dwell"))
                cJSON_ReplaceItemInObject(path, "dwell", cJSON_CreateNumber(duration));
            cJSON_ReplaceItemInObject(path, "sampletime", cJSON_CreateNumber(JNUM(tracker, "timestamp")));

            // Update last position's duration
            cJSON* pathList = JGET(path, "path");
            if (pathList && cJSON_GetArraySize(pathList) > 0) {
                cJSON* lastPosition = cJSON_GetArrayItem(pathList, cJSON_GetArraySize(pathList) - 1);
                cJSON_ReplaceItemInObject(lastPosition, "d", cJSON_CreateNumber(duration));
            }

            // Add new position sample
            cJSON* position = cJSON_CreateObject();
            cJSON_AddNumberToObject(position, "x", JNUM(tracker, "cx"));
            cJSON_AddNumberToObject(position, "y", JNUM(tracker, "cy"));
            cJSON_AddNumberToObject(position, "d", 0);
            if (JGET(tracker, "lat"))
                cJSON_AddNumberToObject(position, "lat", JNUM(tracker, "lat"));
            if (JGET(tracker, "lon"))
                cJSON_AddNumberToObject(position, "lon", JNUM(tracker, "lon"));
            cJSON_AddItemToArray(pathList, position);
        }

        // Finalize and publish if object is dead or ignored
        if (path && (!JBOOL(tracker, "active") || JBOOL(tracker, "ignore"))) {
            cJSON* complete = cJSON_DetachItemFromObject(pathsCache, id);
            int accept = 1;
            double minAge = JNUM(pathFilter, "age");
            double minDistance = JNUM(pathFilter, "distance");

            if (JNUM(complete, "distance") < minDistance) accept = 0;
            if (accept && JNUM(complete, "age") < minAge) accept = 0;

            // AOI check
            if (accept) {
                int x1 = JNUM(aoi, "x1"), y1 = JNUM(aoi, "y1");
                int x2 = JNUM(aoi, "x2"), y2 = JNUM(aoi, "y2");
                accept = 0;
                cJSON* pathList = JGET(complete, "path");
                for (int i = 0; pathList && i < cJSON_GetArraySize(pathList); ++i) {
                    cJSON* pos = cJSON_GetArrayItem(pathList, i);
                    int px = JNUM(pos, "x"), py = JNUM(pos, "y");
                    if (px > x1 && px < x2 && py > y1 && py < y2) { accept = 1; break; }
                }
            }
            if (accept && (cJSON_GetArraySize(JGET(complete, "path")) < 2 || JNUM(complete, "age") < 0.5)) accept = 0;

            if (accept) {
                if (strcmp(JSTR(tracker, "class"), "Head") == 0) {
                    if (JGET(tracker, "hat"))
                        cJSON_AddStringToObject(complete, "hat", JSTR(tracker, "hat"));
                    if (JGET(tracker, "face"))
                        cJSON_AddBoolToObject(complete, "face", JBOOL(tracker, "face"));
                }
                cJSON_DeleteItemFromObject(complete, "sampleTime");
                cJSON_AddItemToArray(response, complete);
            } else {
                cJSON_Delete(complete);
            }
        }
        tracker = tracker->next;
    }
    LOG_TRACE("%s: Exit\n", __func__);
    return response;
}



cJSON*
Process_Occupancy( cJSON* detections ) {
	//Will return 0 if there is no occupancy change;
	//Do NOT free return JSON

	LOG_TRACE("%s: Entry\n",__func__);
	
	cJSON* item;
	cJSON* settings = ACAP_Get_Config("settings");
	if(!settings)
		return 0;

	int size = 20;
	cJSON* occupancyFilter = cJSON_GetObjectItem(settings,"occupancyFilter");
	if( occupancyFilter )
		size = cJSON_GetObjectItem(occupancyFilter,"integration")?cJSON_GetObjectItem(occupancyFilter,"integration")->valueint:20;
	
	if(!occupancyDetectionCounter)
		occupancyDetectionCounter = cJSON_CreateObject();
	//Count the number of detections
	item = occupancyDetectionCounter->child;
	while(item) {
		item->valueint = 0;		
		item = item->next;
	}

	item = detections->child;
	while(item) {
		if( cJSON_GetObjectItem(item,"active")->type == cJSON_True && cJSON_GetObjectItem(item,"ignore")->type == cJSON_False ) {
			const char* class = cJSON_GetObjectItem(item,"class")->valuestring;
			if( cJSON_GetObjectItem(occupancyDetectionCounter,class)) {
				cJSON_GetObjectItem(occupancyDetectionCounter,class)->valueint++;
			} else {
				cJSON_AddNumberToObject(occupancyDetectionCounter,class,1);
			}
		}
		item = item->next;
	}
	if( !classCounterArrays )
		classCounterArrays = cJSON_CreateObject();
	item = occupancyDetectionCounter->child;
	while( item ) {
		const char* class = item->string;
		cJSON* classCounterArray = cJSON_GetObjectItem(classCounterArrays,class);
		if( !classCounterArray ) {
			classCounterArray = cJSON_CreateArray();
			cJSON_AddItemToObject(classCounterArrays,class,classCounterArray);
		}
		cJSON_AddItemToArray(classCounterArray,cJSON_CreateNumber(item->valueint));
		while( cJSON_GetArraySize(classCounterArray) > size )
			cJSON_DeleteItemFromArray(classCounterArray,0);
		item = item->next;
	}

	// Calculate integration value and see if there is a change since last iteration
	if( !previousOccupancy )
		previousOccupancy = cJSON_CreateObject();

	cJSON* response = cJSON_CreateObject();
	int change = 0;

	item = classCounterArrays->child;
	while(item) {
		double value = 0;
		cJSON* indexValue = item->child;
		while(indexValue) {
			value += indexValue->valueint;
			indexValue = indexValue->next;
		}
		value = round( value / (double)cJSON_GetArraySize(item) );
		if(!cJSON_GetObjectItem(previousOccupancy,item->string) ) {
			change = 1;
		} else {
			if( cJSON_GetObjectItem(previousOccupancy,item->string)->valuedouble != value )
				change = 1;
		}
		cJSON_AddNumberToObject( response, item->string, value);
		item = item->next;
	}

	if( previousOccupancy )
		cJSON_Delete( previousOccupancy );
	previousOccupancy = response;
	if( !change )
		return 0;
	ACAP_STATUS_SetObject("occupancy", "status", previousOccupancy);
	LOG_TRACE("%s: Exit\n",__func__);	
	return response;
}

void
Process_VOD_Data(cJSON *list) {
    char topic[128];
    LOG_TRACE("%s:\n", __func__);

	if( cJSON_GetArraySize(list) ) {
		sprintf(topic, "raw/%s", ACAP_DEVICE_Prop("serial"));
		cJSON* payload = cJSON_CreateObject();
		cJSON_AddItemReferenceToObject(payload,"list",list);
		MQTT_Publish_JSON(topic, payload, 0, 0);
		cJSON_Delete(payload);
	}
	
	return;

	return;
    // Detections (for visualization)
    if (publishDetections) {
        sprintf(topic, "detections/%s", ACAP_DEVICE_Prop("serial"));
        cJSON* payload = cJSON_CreateObject();
        cJSON* shortList = cJSON_CreateArray();
        cJSON* item = list->child;
//		LOG("Detections: %d\n", cJSON_GetArraySize(list) );
        while (item) {
            if (!Label_Blacklisted(item)) {
                cJSON* c = cJSON_CreateObject();
                cJSON_AddStringToObject(c, "class", JSTR(item, "class"));
                cJSON_AddNumberToObject(c, "confidence", JNUM(item, "confidence"));
                cJSON_AddNumberToObject(c, "age", JNUM(item, "age"));
                cJSON_AddNumberToObject(c, "distance", JNUM(item, "distance"));
                cJSON_AddNumberToObject(c, "cx", JNUM(item, "cx"));
                cJSON_AddNumberToObject(c, "cy", JNUM(item, "cy"));
                cJSON_AddNumberToObject(c, "bx", JNUM(item, "bx"));
                cJSON_AddNumberToObject(c, "by", JNUM(item, "by"));
                cJSON_AddNumberToObject(c, "x", JNUM(item, "x"));
                cJSON_AddNumberToObject(c, "y", JNUM(item, "y"));
                cJSON_AddNumberToObject(c, "w", JNUM(item, "w"));
                cJSON_AddNumberToObject(c, "h", JNUM(item, "h"));
                cJSON_AddStringToObject(c, "color", JSTR(item, "color"));
                cJSON_AddStringToObject(c, "color2", JSTR(item, "color2"));
                if (JGET(item, "hat"))
                    cJSON_AddStringToObject(c, "hat", JSTR(item, "hat"));
                if (JGET(item, "face"))
                    cJSON_AddBoolToObject(c, "face", JBOOL(item, "face"));
                cJSON_AddNumberToObject(c, "timestamp", JNUM(item, "timestamp"));
                cJSON_AddStringToObject(c, "id", JSTR(item, "id"));
                cJSON_AddBoolToObject(c, "active", JBOOL(item, "active"));
				cJSON_AddItemToObject(c,"attributes", cJSON_Duplicate(cJSON_GetObjectItem(item,"attributes"),1));
                cJSON_AddItemToArray(shortList, c);
            }
            item = item->next;
        }

		if( cJSON_GetArraySize(shortList) != 0 || lastDetectionListWasEmpty == 0 ) {
			cJSON_AddItemToObject(payload, "list", shortList);
			MQTT_Publish_JSON(topic, payload, 0, 0);
			cJSON_Delete(payload);
		}
		lastDetectionListWasEmpty = cJSON_GetArraySize(shortList) == 0;
    }
	return;
    // Trackers (for automations)
    cJSON* trackers = cJSON_CreateArray();
    cJSON* item = NULL;
    cJSON_ArrayForEach(item, list) {
        if (JBOOL(item, "significantMovement") && !Label_Blacklisted(item)) {
            cJSON* c = cJSON_CreateObject();
            cJSON_AddStringToObject(c, "class", JSTR(item, "class"));
            cJSON_AddNumberToObject(c, "confidence", JNUM(item, "confidence"));
            cJSON_AddNumberToObject(c, "age", JNUM(item, "age"));
            cJSON_AddNumberToObject(c, "distance", JNUM(item, "distance"));
            cJSON_AddStringToObject(c, "color", JSTR(item, "color"));
            cJSON_AddStringToObject(c, "color2", JSTR(item, "color2"));
            cJSON_AddNumberToObject(c, "x", JNUM(item, "x"));
            cJSON_AddNumberToObject(c, "y", JNUM(item, "y"));
            cJSON_AddNumberToObject(c, "w", JNUM(item, "w"));
            cJSON_AddNumberToObject(c, "h", JNUM(item, "h"));
            cJSON_AddNumberToObject(c, "cx", JNUM(item, "cx"));
            cJSON_AddNumberToObject(c, "cy", JNUM(item, "cy"));
            cJSON_AddNumberToObject(c, "bx", JNUM(item, "bx"));
            cJSON_AddNumberToObject(c, "by", JNUM(item, "by"));
            cJSON_AddNumberToObject(c, "dx", JNUM(item, "dx"));
            cJSON_AddNumberToObject(c, "dy", JNUM(item, "dy"));
            if (JGET(item, "hat"))
                cJSON_AddStringToObject(c, "hat", JSTR(item, "hat"));
            if (JGET(item, "face"))
                cJSON_AddBoolToObject(c, "face", JBOOL(item, "face"));
            cJSON_AddNumberToObject(c, "timestamp", JNUM(item, "timestamp"));
            cJSON_AddNumberToObject(c, "birth", JNUM(item, "birth"));
            cJSON_AddStringToObject(c, "id", JSTR(item, "id"));
            cJSON_AddBoolToObject(c, "active", JBOOL(item, "active"));
            cJSON_AddItemToArray(trackers, c);
        }
    }

    if (publishTracker) {
        cJSON_ArrayForEach(item, trackers) {
            if (JNUM(item, "distance") >= 5 && !Label_Blacklisted(item)) {
                sprintf(topic, "tracker/%s", ACAP_DEVICE_Prop("serial"));
                MQTT_Publish_JSON(topic, item, 0, 0);
            }
        }
    }

    // Paths (for forensic/post-processing)
    cJSON* paths = Process_Paths(trackers);
    cJSON* statusPaths = ACAP_STATUS_Object("detections", "paths");

    if (publishPath) {
        item = paths->child;
        while (item) {
            cJSON_AddItemToArray(statusPaths, cJSON_Duplicate(item, 1));
            sprintf(topic, "path/%s", ACAP_DEVICE_Prop("serial"));
            MQTT_Publish_JSON(topic, item, 0, 0);
            item = item->next;
        }
    }

    while (cJSON_GetArraySize(statusPaths) > 10)
        cJSON_DeleteItemFromArray(statusPaths, 0);

    cJSON_Delete(trackers);
    cJSON_Delete(paths);
    LOG_TRACE("%s: Exit\n", __func__);
}

void
Event_Callback(cJSON *event, void* userdata) {
	if(!event)
		return;

	cJSON* settings = ACAP_Get_Config("settings");
	if( !settings )
		return;
	cJSON* publish = cJSON_GetObjectItem(settings,"publish");
	if(!publish)
		return;
	
	if( !cJSON_GetObjectItem(publish,"events") || cJSON_GetObjectItem(publish,"events")->type != cJSON_True )
		return;

	cJSON* eventTopic = cJSON_DetachItemFromObject(event, "event");
	if(!eventTopic)
		return;

	int ignore = 0;
	if( !ignore && strstr(eventTopic->valuestring,"HardwareFailure") ) ignore = 1;
	if( !ignore && strstr(eventTopic->valuestring,"SystemReady") ) ignore = 1;
	if( !ignore && strstr(eventTopic->valuestring,"ClientStatus") ) ignore = 1;
	if( !ignore && strstr(eventTopic->valuestring,"RingPowerLimitExceeded") ) ignore = 1;
	if( !ignore && strstr(eventTopic->valuestring,"PTZPowerFailure") ) ignore = 1;
	if( !ignore && strstr(eventTopic->valuestring,"SystemInitializing") ) ignore = 1;
	if( !ignore && strstr(eventTopic->valuestring,"Network") ) ignore = 1;
	if( !ignore && strstr(eventTopic->valuestring,"xinternal_data") ) ignore = 1;
	if( ignore ) {
		cJSON_Delete( eventTopic );
		return;
	}
		
	//Filter unwanted topics
	cJSON* eventFilter = cJSON_GetObjectItem(settings,"eventTopics")?cJSON_GetObjectItem(settings,"eventTopics")->child:0;
	while( eventFilter ) { 
		if( cJSON_GetObjectItem(eventFilter,"enabled")->type == cJSON_False ) {
			const char* ignoreTopic = cJSON_GetObjectItem(eventFilter, "topic")?cJSON_GetObjectItem(eventFilter, "topic")->valuestring:0;
			if( ignoreTopic && strstr(eventTopic->valuestring, ignoreTopic) ) {
				cJSON_Delete( eventTopic );
				return;
			}
		}
		eventFilter = eventFilter->next;
	}
		
	char topic[256];
	sprintf(topic,"event/%s/%s",ACAP_DEVICE_Prop("serial"),eventTopic->valuestring);
	cJSON_Delete( eventTopic );
	MQTT_Publish_JSON( topic, event, 0, 0 );
}

void 
Main_MQTT_Status (int state) {
	char topic[64];
	cJSON* connection = 0;

	switch( state ) {
		case MQTT_INITIALIZING:
			LOG("%s: Initializing\n",__func__);
			ACAP_STATUS_SetString("mqtt","status","Initializing");
			ACAP_STATUS_SetBool("mqtt","connected",0);
			break;
		case MQTT_CONNECTING:
			ACAP_STATUS_SetString("mqtt","status","Connecting");
			ACAP_STATUS_SetBool("mqtt","connected",0);
			LOG("%s: Connecting\n",__func__);
			break;
		case MQTT_CONNECTED:
			ACAP_STATUS_SetString("mqtt","status","Connected");
			ACAP_STATUS_SetBool("mqtt","connected",1);
			LOG("%s: Connected\n",__func__);
			sprintf(topic,"connect/%s",ACAP_DEVICE_Prop("serial"));
			connection = cJSON_CreateObject();
			cJSON_AddTrueToObject(connection,"connected");
			cJSON_AddStringToObject(connection,"address",ACAP_DEVICE_Prop("IPv4"));
			if(!MQTT_Publish_JSON(topic,connection,0,1) )
				LOG_WARN("%s: Announce messaged failed\n",__func__);
			cJSON_Delete(connection);
			break;
		case MQTT_DISCONNECTING:
			sprintf(topic,"connect/%s",ACAP_DEVICE_Prop("serial"));
			connection = cJSON_CreateObject();
			cJSON_AddFalseToObject(connection,"connected");
			cJSON_AddStringToObject(connection,"address",ACAP_DEVICE_Prop("IPv4"));
			if(!MQTT_Publish_JSON(topic,connection,0,1) )
				LOG_WARN("%s: Disconnect messaged failed\n",__func__);
			cJSON_Delete(connection);
			break;
		
		case MQTT_RECONNECTING:
			ACAP_STATUS_SetString("mqtt","status","Reconnecting");
			ACAP_STATUS_SetBool("mqtt","connected",0);
			LOG("%s: Reconnecting\n",__func__);
			break;
		case MQTT_DISCONNECTED:
			ACAP_STATUS_SetString("mqtt","status","Disconnected");
			ACAP_STATUS_SetBool("mqtt","connected",0);
			LOG("%s: Disconnect\n",__func__);
			break;
	}
}

void
Main_MQTT_Subscription_Message(const char *topic, const char *payload) {
	LOG("Message arrived: %s %s\n",topic,payload);
}

static gboolean
MQTT_Publish_Device_Status(gpointer user_data) {


	if(!publishStatus)
		return G_SOURCE_CONTINUE;
	
	cJSON* payload = cJSON_CreateObject();
	cJSON_AddNumberToObject(payload,"Network_Kbps",(int)ACAP_DEVICE_Network_Average());
	cJSON_AddNumberToObject(payload,"CPU_average",(int)(ACAP_DEVICE_CPU_Average()*100));
	cJSON_AddNumberToObject(payload,"Uptime_Hours",(int)(ACAP_DEVICE_Uptime()/3600));

	char topic[256];
	sprintf(topic,"status/%s",ACAP_DEVICE_Prop("serial"));
	MQTT_Publish_JSON( topic, payload, 0, 0 );
	cJSON_Delete( payload );
	
    return G_SOURCE_CONTINUE;  // Return TRUE to continue the timer
}

static GMainLoop *main_loop = NULL;

static gboolean
signal_handler(gpointer user_data) {
    LOG("Received SIGTERM, initiating shutdown\n");
    if (main_loop && g_main_loop_is_running(main_loop)) {
        g_main_loop_quit(main_loop);
    }
    return G_SOURCE_REMOVE;
}

void
Settings_Updated_Callback( const char* service, cJSON* data) {
	char* json = cJSON_PrintUnformatted(data);
	if( json ) {
		LOG_TRACE("%s: %s=%s\n",__func__, service, json);
		free(json);
	} else {
		LOG_WARN("%s: JSON Parse error\n",__func__);
	}

	if( strcmp( service, "publish" ) == 0 ) {
		publishEvents = cJSON_IsTrue( cJSON_GetObjectItem(data,"events"));
		publishDetections = cJSON_IsTrue(cJSON_GetObjectItem(data,"detections"));
		publishTracker = cJSON_IsTrue(cJSON_GetObjectItem(data,"tracker"));
		publishPath = cJSON_IsTrue(cJSON_GetObjectItem(data,"path"));
		publishOccupancy = cJSON_IsTrue(cJSON_GetObjectItem(data,"occupancy"));
		publishStatus = cJSON_IsTrue(cJSON_GetObjectItem(data,"status"));
		publishGeospace = cJSON_IsTrue(cJSON_GetObjectItem(data,"geospace"));
	}

	if( strcmp( service,"scene" ) == 0 ) {
		ObjectDetection_Config( data );
	    blacklist = cJSON_GetObjectItem(data, "ignoreClass");
	}

	if( strcmp( service,"matrix" ) == 0 )
		GeoSpace_Matrix( data );
	
}

static gboolean
MemoryMonitor(gpointer user_data) {
	int c2 = cJSON_GetArraySize(activeTrackers);
	int c3 = cJSON_GetArraySize(PreviousPosition);
	int c4 = cJSON_GetArraySize(lastPublishedTracker);
	int c5 = cJSON_GetArraySize(pathsCache);
	int c6 = cJSON_GetArraySize(occupancyDetectionCounter);
	int c7 = cJSON_GetArraySize(classCounterArrays);
	int c8 = cJSON_GetArraySize(previousOccupancy);
	int c9 = ObjectDetection_CacheSize();
	LOG("%d %d %d %d %d %d %d %d\n",c2,c3,c4,c5,c6,c7,c8,c9);

    return G_SOURCE_CONTINUE;  // Return TRUE to continue the timer
}

void
HandleVersionUpdateConfigurations(cJSON* settings ) {
	cJSON* scene = cJSON_GetObjectItem(settings,"scene");
	if(!cJSON_GetObjectItem(scene,"maxIdle"))
		cJSON_AddNumberToObject(scene,"maxIdle",0);
	if(!cJSON_GetObjectItem(scene,"tracker_confidence"))
		cJSON_AddTrueToObject(scene,"tracker_confidence");
	if(!cJSON_GetObjectItem(scene,"hanging_objects"))
		cJSON_AddNumberToObject(scene,"hanging_objects", 5);

	if(!cJSON_GetObjectItem(scene,"minWidth"))
		cJSON_AddNumberToObject(scene,"minWidth",10);
	if(!cJSON_GetObjectItem(scene,"minHeight"))
		cJSON_AddNumberToObject(scene,"minHeight",10);
	if(!cJSON_GetObjectItem(scene,"maxWidth"))
		cJSON_AddNumberToObject(scene,"maxWidth",800);
	if(!cJSON_GetObjectItem(scene,"maxHeight"))
		cJSON_AddNumberToObject(scene,"maxHeight",10);
	if(!cJSON_GetObjectItem(scene,"aoi")) {
		cJSON* aoi = cJSON_CreateObject();
		cJSON_AddNumberToObject(aoi,"x1",50);
		cJSON_AddNumberToObject(aoi,"x2",950);
		cJSON_AddNumberToObject(aoi,"y1",50);
		cJSON_AddNumberToObject(aoi,"y2",950);
		cJSON_AddItemToObject(scene,"aoi",aoi);
	}
	if(!cJSON_GetObjectItem(scene,"ignoreClass"))
		cJSON_AddArrayToObject(scene,"ignoreClass");

	cJSON* publish = cJSON_GetObjectItem(settings,"publish");
	if(!cJSON_GetObjectItem(publish,"geospace"))
		cJSON_AddFalseToObject(publish,"geospace");
}

int
main(void) {
	openlog(APP_PACKAGE, LOG_PID|LOG_CONS, LOG_USER);
	LOG("------ Starting ACAP Service ------\n");

	cJSON* settings = ACAP( APP_PACKAGE, Settings_Updated_Callback );
	HandleVersionUpdateConfigurations(settings);

	//MQTT
	ACAP_STATUS_SetString("mqtt","status","No initialized");
	ACAP_STATUS_SetBool("mqtt","connected",0);
	MQTT_Init( Main_MQTT_Status, Main_MQTT_Subscription_Message );
	ACAP_Set_Config("mqtt", MQTT_Settings());
	ACAP_STATUS_SetObject("detections", "paths", cJSON_CreateArray());
	//Events
	ACAP_EVENTS_SetCallback( Event_Callback );
	cJSON* eventSubscriptions = ACAP_FILE_Read( "settings/subscriptions.json" );
	cJSON* subscription = eventSubscriptions?eventSubscriptions->child:0;
	while(subscription){
		ACAP_EVENTS_Subscribe( subscription, NULL );
		subscription = subscription->next;
	}
	//Object detection
	
	if( ObjectDetection_Init( Process_VOD_Data ) ) {
		ACAP_STATUS_SetBool("objectdetection","connected",1);
		ACAP_STATUS_SetString("objectdetection", "status", "OK");
	} else {
		ACAP_STATUS_SetBool("objectdetection","connected",0);
		ACAP_STATUS_SetString("objectdetection", "status", "Object detection is not avaialble");
	}
		
	GeoSpace_Init();
	g_timeout_add_seconds(15 * 60, MQTT_Publish_Device_Status, NULL);
//	g_timeout_add_seconds(120, MemoryMonitor, NULL);
	
	main_loop = g_main_loop_new(NULL, FALSE);
    GSource *signal_source = g_unix_signal_source_new(SIGTERM);
    if (signal_source) {
		g_source_set_callback(signal_source, signal_handler, NULL, NULL);
		g_source_attach(signal_source, NULL);
	} else {
		LOG_WARN("Signal detection failed");
	}

	g_main_loop_run(main_loop);

	LOG("Terminating and cleaning up %s\n",APP_PACKAGE);
	Main_MQTT_Status(MQTT_DISCONNECTING); //Send graceful disconnect message

	MQTT_Cleanup();
	ACAP_Cleanup();
	
    return 0;
}
