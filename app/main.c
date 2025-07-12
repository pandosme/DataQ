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

#define LOG(fmt, args...) { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_WARN(fmt, args...) { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args); }
//#define LOG_TRACE(fmt, args...) { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_TRACE(fmt, args...) {}
#define LOG_DEEP(fmt, args...) { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
//#define LOG_DEEP(fmt, args...) {}


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


cJSON* ProcessPaths(cJSON* tracker) {
    if (!PathCache)
        PathCache = cJSON_CreateObject();
	LOG_DEEP("<main:%s",__func__);
    const char* id = cJSON_GetObjectItem(tracker, "id")?cJSON_GetObjectItem(tracker, "id")->valuestring:0;
    if (!id){
		LOG_DEEP("main:%s:invlaid id:>",__func__);
		return 0;
	}
    const char* class = cJSON_GetObjectItem(tracker, "class")?cJSON_GetObjectItem(tracker, "class")->valuestring:0;
    if (!class){
		LOG_WARN("main:%s:invlaid class:>",__func__);
		return 0;
	}

    int active = cJSON_GetObjectItem(tracker, "active")?cJSON_GetObjectItem(tracker, "active")->type == cJSON_True:0;

    int confidence = cJSON_GetObjectItem(tracker, "confidence")?cJSON_GetObjectItem(tracker, "confidence")->valueint:0;
    if (!confidence){
		LOG_WARN("main:%s:invlaid confidence",__func__);
		return 0;
	}

    double age = cJSON_GetObjectItem(tracker, "age")?cJSON_GetObjectItem(tracker, "age")->valuedouble:0;
    if (!age){
		LOG_WARN("main:%s:invlaid age",__func__);
		return 0;
	}

    double distance = cJSON_GetObjectItem(tracker, "distance")?cJSON_GetObjectItem(tracker, "distance")->valuedouble:0;
    if (!distance){
		LOG_WARN("main:%s:invlaid distance",__func__);
		return 0;
	}

    // Try to get existing path
    cJSON* path = cJSON_GetObjectItem(PathCache, id);
    if (!path) {
		LOG_DEEP("main:%s:createNew: New ID %s",__func__,id);
	}

    if (!path && !active) {
        // Create new path
		LOG_DEEP("<main:%s:createNew",__func__);
        path = cJSON_CreateObject();
        cJSON_AddStringToObject(path, "class", class);
        cJSON_AddNumberToObject(path, "confidence", confidence);
        cJSON_AddNumberToObject(path, "age", age);
        cJSON_AddNumberToObject(path, "distance", distance);
		LOG_DEEP("<main:%s:createNew 1",__func__);

        cJSON_AddNumberToObject(path, "cx", cJSON_GetObjectItem(tracker, "cx")->valuedouble);
        cJSON_AddNumberToObject(path, "cy", cJSON_GetObjectItem(tracker, "cy")->valuedouble);
        cJSON_AddNumberToObject(path, "dx", cJSON_GetObjectItem(tracker, "dx")->valuedouble);
        cJSON_AddNumberToObject(path, "dy", cJSON_GetObjectItem(tracker, "dy")->valuedouble);
        cJSON_AddNumberToObject(path, "bx", cJSON_GetObjectItem(tracker, "bx")->valuedouble);
        cJSON_AddNumberToObject(path, "by", cJSON_GetObjectItem(tracker, "by")->valuedouble);
        cJSON_AddNumberToObject(path, "timestamp", cJSON_GetObjectItem(tracker, "birthTime")->valuedouble);
        cJSON_AddNumberToObject(path, "dwell", 0);
        cJSON_AddStringToObject(path, "id", id);
		LOG_DEEP("<main:%s:createNew 1",__func__);
        // Optional fields
        cJSON* face = cJSON_GetObjectItem(tracker, "face");
        if (face) cJSON_AddBoolToObject(path, "face", cJSON_IsTrue(face));
        cJSON* hat = cJSON_GetObjectItem(tracker, "hat");
        if (hat) cJSON_AddStringToObject(path, "hat", hat->valuestring);

        // Path array
        cJSON* pathArr = cJSON_CreateArray();
        cJSON* pos1 = cJSON_CreateObject();
        cJSON_AddNumberToObject(pos1, "x", cJSON_GetObjectItem(tracker, "bx")->valuedouble);
        cJSON_AddNumberToObject(pos1, "y", cJSON_GetObjectItem(tracker, "by")->valuedouble);
        cJSON_AddNumberToObject(pos1, "d", cJSON_GetObjectItem(tracker, "idle")->valuedouble);
        cJSON_AddItemToArray(pathArr, pos1);

        cJSON* pos2 = cJSON_CreateObject();
        cJSON_AddNumberToObject(pos2, "x", cJSON_GetObjectItem(tracker, "cx")->valuedouble);
        cJSON_AddNumberToObject(pos2, "y", cJSON_GetObjectItem(tracker, "cy")->valuedouble);
		double lat = cJSON_GetObjectItem(tracker,"lat")?cJSON_GetObjectItem(tracker,"lat")->valuedouble:0;
		double lon = cJSON_GetObjectItem(tracker,"lon")?cJSON_GetObjectItem(tracker,"lon")->valuedouble:0;
		if( lat && lon ) {
			cJSON_AddNumberToObject(pos2, "lat", lat);
			cJSON_AddNumberToObject(pos2, "lon", lon);
		}
        cJSON_AddNumberToObject(pos2, "d", 0);
        cJSON_AddItemToArray(pathArr, pos2);

        cJSON_AddItemToObject(path, "path", pathArr);

        cJSON_AddItemToObject(PathCache, id, path);
		LOG_DEEP("main:%s:createNew>",__func__);
        return 0;
    } else if (path && active) {
		LOG_DEEP("<main:%s:new object",__func__);
        // Update existing path
        cJSON_ReplaceItemInObject(path, "class", cJSON_Duplicate(cJSON_GetObjectItem(tracker, "class"), 1));
        cJSON_ReplaceItemInObject(path, "confidence", cJSON_Duplicate(cJSON_GetObjectItem(tracker, "confidence"), 1));
        cJSON_ReplaceItemInObject(path, "age", cJSON_Duplicate(cJSON_GetObjectItem(tracker, "age"), 1));
        cJSON_ReplaceItemInObject(path, "distance", cJSON_Duplicate(cJSON_GetObjectItem(tracker, "distance"), 1));
        cJSON_ReplaceItemInObject(path, "cx", cJSON_Duplicate(cJSON_GetObjectItem(tracker, "cx"), 1));
        cJSON_ReplaceItemInObject(path, "cy", cJSON_Duplicate(cJSON_GetObjectItem(tracker, "cy"), 1));
        cJSON_ReplaceItemInObject(path, "dx", cJSON_Duplicate(cJSON_GetObjectItem(tracker, "dx"), 1));
        cJSON_ReplaceItemInObject(path, "dy", cJSON_Duplicate(cJSON_GetObjectItem(tracker, "dy"), 1));

        // Optional fields
        cJSON* face = cJSON_GetObjectItem(tracker, "face");
        if (face) cJSON_ReplaceItemInObject(path, "face", cJSON_Duplicate(face, 1));
        cJSON* hat = cJSON_GetObjectItem(tracker, "hat");
        if (hat) cJSON_ReplaceItemInObject(path, "hat", cJSON_Duplicate(hat, 1));

        // Update dwell
        double idle = cJSON_GetObjectItem(tracker, "idle")->valuedouble;
        cJSON* dwell_item = cJSON_GetObjectItem(path, "dwell");
        if (idle > dwell_item->valuedouble)
            cJSON_SetNumberValue(dwell_item, idle);

        // Update last path entry's d
        cJSON* pathArr = cJSON_GetObjectItem(path, "path");
        int pathLen = cJSON_GetArraySize(pathArr);
        if (pathLen > 0) {
            cJSON* last = cJSON_GetArrayItem(pathArr, pathLen - 1);
            cJSON_ReplaceItemInObject(last, "d", cJSON_Duplicate(cJSON_GetObjectItem(tracker, "idle"), 1));
        }

        // Append new position
        cJSON* pos = cJSON_CreateObject();
        cJSON_AddNumberToObject(pos, "x", cJSON_GetObjectItem(tracker, "cx")->valuedouble);
        cJSON_AddNumberToObject(pos, "y", cJSON_GetObjectItem(tracker, "cy")->valuedouble);
        cJSON_AddNumberToObject(pos, "d", 0);
		double lat = cJSON_GetObjectItem(tracker,"lat")?cJSON_GetObjectItem(tracker,"lat")->valuedouble:0;
		double lon = cJSON_GetObjectItem(tracker,"lon")?cJSON_GetObjectItem(tracker,"lon")->valuedouble:0;
		if( lat && lon ) {
			cJSON_AddNumberToObject(pos, "lat", lat);
			cJSON_AddNumberToObject(pos, "lon", lon);
		}
        cJSON_AddItemToArray(pathArr, pos);
		LOG_DEEP("main: %s: active>",__func__);
        return 0;
    } else if (path && !active) {
        // Only send if path length > 2 and age > 3
        cJSON* pathArr = cJSON_GetObjectItem(path, "path");
        int pathLen = cJSON_GetArraySize(pathArr);
        double age = cJSON_GetObjectItem(path, "age")->valuedouble;
        if (pathLen > 2 && age > 2) {
            // Remove from cache and return a copy for publishing
            cJSON_DetachItemFromObject(PathCache, id);
			LOG_DEEP("ignore main:%s>",__func__);		
            return path;
        } else {
            cJSON_DetachItemFromObject(PathCache, id);
            cJSON_Delete(path);
			LOG_DEEP("remove object:%s>",__func__);
            return 0;
        }
    }
	LOG_DEEP("no active path:%s>",__func__);
    return 0;
}

void
Tracker_Data(cJSON *tracker ) {
	LOG_DEEP("<main:%s",__func__);
	char topic[128];
	if( publishTracker) {
		sprintf(topic,"tracker/%s", ACAP_DEVICE_Prop("serial") );
		MQTT_Publish_JSON(topic,tracker,0,0);
	}

	LOG_DEEP("main:%s Tracker published>: ",__func__);
	
	if( publishGeospace && ACAP_STATUS_Bool("geospace", "active")) {
		LOG_DEEP("<%s: main: geospace start",__func__);
		double lat = 0;
		double lon = 0;
		GeoSpace_transform(cJSON_GetObjectItem(tracker,"cx")->valueint, cJSON_GetObjectItem(tracker,"cy")->valueint,&lat,&lon);
		if( lat && lon ) {
			cJSON_AddNumberToObject(tracker,"lat",lat);
			cJSON_AddNumberToObject(tracker,"lon",lon);
			cJSON* geopsaceObject = cJSON_CreateObject();

			cJSON_AddStringToObject(geopsaceObject,"class",cJSON_GetObjectItem(tracker,"class")->valuestring);
			cJSON_AddNumberToObject(geopsaceObject,"lat",lat);
			cJSON_AddNumberToObject(geopsaceObject,"lon",lon);
			cJSON_AddNumberToObject(geopsaceObject,"age",cJSON_GetObjectItem(tracker,"age")->valuedouble);			
		
			sprintf(topic,"geospace/%s", ACAP_DEVICE_Prop("serial") );
			MQTT_Publish_JSON(topic,geopsaceObject,0,0);
			cJSON_Delete(geopsaceObject);
			
		} else {
			LOG_DEEP("%s main>:  Invalid geoposions>",__func__);
		}
		LOG_DEEP("main: %s: geospace exit>",__func__);
	}
	
	if( publishPath ) {
		cJSON* path = ProcessPaths( tracker );
		if( path ) {
			sprintf(topic,"path/%s", ACAP_DEVICE_Prop("serial") );
			MQTT_Publish_JSON(topic,path,0,0);
			cJSON_Delete( path );
		}
	}
	cJSON_Delete(tracker);
	LOG_DEEP("%s>",__func__);
}

void
Detections_Data (cJSON *list ) {
	char topic[128];
//	LOG_DEEP("<main:%s",__func__);

	if( publishDetections) {
		sprintf(topic,"detections/%s", ACAP_DEVICE_Prop("serial") );
		cJSON* payload = cJSON_CreateObject();
		cJSON_AddItemToObject( payload, "list", list );
		MQTT_Publish_JSON(topic,payload,0,0);
		cJSON_Delete( payload );
//		LOG_DEEP("main:%s>",__func__);
		return;
	}
	cJSON_Delete(list);
//	LOG_DEEP("man:%s>",__func__);
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
	cJSON* message = 0;

	switch( state ) {
		case MQTT_INITIALIZING:
			LOG("%s: Initializing\n",__func__);
			break;
		case MQTT_CONNECTING:
			LOG("%s: Connecting\n",__func__);
			break;
        case MQTT_CONNECTED:
            LOG("%s: Connected\n",__func__);
			sprintf(topic,"connect/%s",ACAP_DEVICE_Prop("serial"));
			message = cJSON_CreateObject();
			cJSON_AddTrueToObject(message,"connected");
			cJSON_AddStringToObject(message,"address",ACAP_DEVICE_Prop("IPv4"));
			MQTT_Publish_JSON(topic,message,0,1);
			cJSON_Delete(message);			
            break;
		case MQTT_DISCONNECTING:
			sprintf(topic,"connect/%s",ACAP_DEVICE_Prop("serial"));
			message = cJSON_CreateObject();
			cJSON_AddFalseToObject(message,"connected");
			cJSON_AddStringToObject(message,"address",ACAP_DEVICE_Prop("IPv4"));
			MQTT_Publish_JSON(topic,message,0,1);
			cJSON_Delete(message);			
			break;
		
			break;
		case MQTT_RECONNECTED:
			// Make all your MQTT subscriptions here
			LOG("%s: Reconnected\n",__func__);
			break;
		case MQTT_DISCONNECTED:
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

	if( strcmp( service,"scene" ) == 0 )
		ObjectDetection_Config( data );

	if( strcmp( service,"matrix" ) == 0 )
		GeoSpace_Matrix( data );
	
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
	
	if( ObjectDetection_Init( Detections_Data, Tracker_Data ) ) {
		ACAP_STATUS_SetBool("objectdetection","connected",1);
		ACAP_STATUS_SetString("objectdetection", "status", "OK");
	} else {
		ACAP_STATUS_SetBool("objectdetection","connected",0);
		ACAP_STATUS_SetString("objectdetection", "status", "Object detection is not avaialble");
	}
		
	GeoSpace_Init();
	g_timeout_add_seconds(15 * 60, MQTT_Publish_Device_Status, NULL);
//	g_timeout_add_seconds(1, occupancyTimer, NULL);

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
    closelog();
    return 0;
}
