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


cJSON* activeTrackers = 0;
cJSON* PreviousPosition = 0;
cJSON* lastPublishedTracker = 0;
cJSON* pathsCache = 0;
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

void
Tracker_Data(cJSON *tracker ) {
	char topic[128];
	if( publishTracker) {
		sprintf(topic,"tracker/%s", ACAP_DEVICE_Prop("serial") );
		MQTT_Publish_JSON(topic,tracker,0,0);
	}
	cJSON_Delete(tracker);
}

void
Detections_Data (cJSON *list ) {
	char topic[128];

	if( publishDetections) {
		sprintf(topic,"detections/%s", ACAP_DEVICE_Prop("serial") );
		cJSON* payload = cJSON_CreateObject();
		cJSON_AddItemToObject( payload, "list", list );
		MQTT_Publish_JSON(topic,payload,0,0);
		cJSON_Delete( payload );
	} else {
		cJSON_Delete(list);
	}

/*
	//Trackers
	cJSON* trackers = cJSON_CreateArray();
	cJSON_ArrayForEach(item, list) {
		int valid = 0;
		if( cJSON_GetObjectItem(item,"idle")->valueint == 0 )  //The object is born or passed 5% ditance threshold
			valid=1;
		if( cJSON_GetObjectItem(item,"active")->type == cJSON_False )
			valid=1;
	
		if( valid ) {
			cJSON* c = cJSON_CreateObject();
			cJSON_AddStringToObject( c,"class", cJSON_GetObjectItem(item,"class")->valuestring);
			cJSON_AddNumberToObject( c,"confidence", cJSON_GetObjectItem(item,"confidence")->valueint);
			cJSON_AddNumberToObject( c,"age", cJSON_GetObjectItem(item,"age")->valuedouble);
			cJSON_AddNumberToObject( c,"distance", cJSON_GetObjectItem(item,"distance")->valueint);
			cJSON_AddStringToObject( c,"color", cJSON_GetObjectItem(item,"color")->valuestring);
			cJSON_AddStringToObject( c,"color2", cJSON_GetObjectItem(item,"color2")->valuestring);
			cJSON_AddNumberToObject( c,"x", cJSON_GetObjectItem(item,"x")->valueint);
			cJSON_AddNumberToObject( c,"y", cJSON_GetObjectItem(item,"y")->valueint);
			cJSON_AddNumberToObject( c,"w", cJSON_GetObjectItem(item,"w")->valueint);
			cJSON_AddNumberToObject( c,"h", cJSON_GetObjectItem(item,"h")->valueint);
			cJSON_AddNumberToObject( c,"cx", cJSON_GetObjectItem(item,"cx")->valueint);
			cJSON_AddNumberToObject( c,"cy", cJSON_GetObjectItem(item,"cy")->valueint);
			cJSON_AddNumberToObject( c,"bx", cJSON_GetObjectItem(item,"bx")->valueint);
			cJSON_AddNumberToObject( c,"by", cJSON_GetObjectItem(item,"by")->valueint);
			cJSON_AddNumberToObject( c,"dx", cJSON_GetObjectItem(item,"dx")->valueint);
			cJSON_AddNumberToObject( c,"dy", cJSON_GetObjectItem(item,"dy")->valueint);
			cJSON_AddNumberToObject( c,"timestamp", cJSON_GetObjectItem(item,"timestamp")->valuedouble);
			cJSON_AddNumberToObject( c,"birth", cJSON_GetObjectItem(item,"birth")->valuedouble);
//			cJSON_AddNumberToObject( c,"topVelocity", cJSON_GetObjectItem(item,"topVelocity")->valuedouble);
			cJSON_AddStringToObject( c,"id", cJSON_GetObjectItem(item,"id")->valuestring);
			if( cJSON_GetObjectItem(item,"active")->type == cJSON_True )
				cJSON_AddTrueToObject(c,"active");
			else
				cJSON_AddFalseToObject(c,"active");
			cJSON_AddItemToArray(trackers,c);
		}
	}
	
	if( publishTracker ) {
		cJSON_ArrayForEach(item, trackers) {
			if( cJSON_GetObjectItem(item,"distance")->valueint >= 5 && !Label_Blacklisted(item)) {
				sprintf(topic, "tracker/%s", ACAP_DEVICE_Prop("serial"));
				MQTT_Publish_JSON(topic, item, 0, 0);
			}
		}
	}

	//GeoSpace locations
	if( publishGeospace ) {
		item = trackers->child;
		while( item ) {
			if( !Label_Blacklisted(item) ) {
				sprintf(topic,"geospace/%s", ACAP_DEVICE_Prop("serial") );
				cJSON* payload = cJSON_CreateObject();
				cJSON_AddStringToObject(payload,"id",cJSON_GetObjectItem(item,"id")->valuestring);
				cJSON_AddTrueToObject(payload,"active");
				if( cJSON_GetObjectItem(item,"active")->type == cJSON_False )
					cJSON_AddFalseToObject(payload,"active")->type = cJSON_False;
				double lat = 0;
				double lon = 0;
				GeoSpace_transform(cJSON_GetObjectItem(item,"cx")->valueint,
								   cJSON_GetObjectItem(item,"cy")->valueint
								   ,&lat,
								   &lon);
				if( cJSON_GetObjectItem( item,"lat" ) )
					cJSON_ReplaceItemInObject(item,"lat",cJSON_CreateNumber(lat));
				else
					cJSON_AddNumberToObject(item,"lat",lat);
				if( cJSON_GetObjectItem( item,"lon" ) )
					cJSON_ReplaceItemInObject(item,"lon",cJSON_CreateNumber(lon));
				else
					cJSON_AddNumberToObject(item,"lon",lon);
				cJSON_AddNumberToObject(payload,"lat",lat);
				cJSON_AddNumberToObject(payload,"lon",lon);
				cJSON_AddStringToObject(payload,"class",cJSON_GetObjectItem(item,"class")->valuestring);
				cJSON_AddNumberToObject(payload,"age",cJSON_GetObjectItem(item,"age")->valuedouble);
				MQTT_Publish_JSON(topic,payload,0,0);
				cJSON_Delete(payload);
			}
			item = item->next;
		}
	}


	//Paths
	cJSON* paths = Process_Paths( trackers );
	//Paths displayed in Path user interface
	cJSON* statusPaths = ACAP_STATUS_Object("detections", "paths");
	statusPaths = ACAP_STATUS_Object("detections", "paths");

	if( publishPath ) {
		item = paths->child;
		while( item ) {
			cJSON_AddItemToArray(statusPaths,cJSON_Duplicate(item, 1));
			sprintf(topic,"path/%s", ACAP_DEVICE_Prop("serial") );
			MQTT_Publish_JSON(topic,item,0,0);
			item = item->next;
		}
	}

	while( cJSON_GetArraySize(statusPaths) > 10 )
		cJSON_DeleteItemFromArray(statusPaths,0);
	cJSON_Delete(paths);
	cJSON_Delete(trackers);
*/	
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

	if( strcmp( service,"scene" ) == 0 ) {
		ObjectDetection_Config( data );
	}

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

	Main_MQTT_Status(MQTT_DISCONNECTING); //Send graceful disconnect message
	MQTT_Cleanup();
	ACAP_Cleanup();
    closelog();
    return 0;
}
