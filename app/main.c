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

#define APP_PACKAGE	"DataQ"

#define LOG(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args);}
#define LOG_WARN(fmt, args...)    { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args);}
#define LOG_TRACE(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
//#define LOG_TRACE(fmt, args...)    {}

cJSON* detectionsList = 0;
cJSON* trackerList = 0;
cJSON* pathList = 0;
cJSON* detectionsPassed = 0;
cJSON* PreviousPosition = 0;
cJSON* PreviousTimestamp = 0;
cJSON* lastPublishedTracker = 0;
cJSON* publish = 0;
cJSON* attributes = 0;

void
Settings_Updated_Callback( const char* service, cJSON* data) {
	char* json = cJSON_PrintUnformatted(data);
	LOG_TRACE("%s: %s=%s\n",__func__, service, json);
	free(json);
	if( strcmp( service,"scene" ) == 0 ) {
		int confidence = cJSON_GetObjectItem( data,"confidence")?cJSON_GetObjectItem( data,"confidence")->valueint:30;
		int rotation = cJSON_GetObjectItem( data,"rotation")?cJSON_GetObjectItem( data,"rotation")->valueint:0;
		int cog = cJSON_GetObjectItem( data,"cog")?cJSON_GetObjectItem( data,"cog")->valueint:1;
		ObjectDetection_Set( confidence, rotation, cog );
	}
}

cJSON* paths = 0;

void
Trackers( cJSON* tracker ) {
	char topic[128];

	const char* id = cJSON_GetObjectItem(tracker,"id")->valuestring;

	if( !lastPublishedTracker )
		lastPublishedTracker = cJSON_CreateObject();
	if( cJSON_GetObjectItem(tracker,"active")->type == cJSON_False ) {
		cJSON_DeleteItemFromObject( lastPublishedTracker,id);
	} else {
		if( !cJSON_GetObjectItem( lastPublishedTracker, id ) ) {
			cJSON_AddItemToObject( lastPublishedTracker,id, cJSON_CreateNumber( cJSON_GetObjectItem(tracker,"timestamp")->valuedouble ) );
		} else {
			cJSON_ReplaceItemInObject( lastPublishedTracker,id, cJSON_CreateNumber( cJSON_GetObjectItem(tracker,"timestamp")->valuedouble ) );
		}
	}

	if( cJSON_GetObjectItem(publish,"tracker") && cJSON_GetObjectItem(publish,"tracker")->type == cJSON_True ) {
		sprintf(topic,"tracker/%s", ACAP_DEVICE_Prop("serial") );
		MQTT_Publish_JSON(topic,tracker,0,0);
	}

	if( !paths )
		paths = cJSON_CreateObject();
	cJSON* path = cJSON_GetObjectItem(paths,id);
	if( !path ) {
		path = cJSON_CreateObject();
		cJSON_AddStringToObject(path,"id",id);
		cJSON_AddStringToObject(path,"class",cJSON_GetObjectItem(tracker,"class")->valuestring);
		cJSON_AddNumberToObject(path,"confidence",cJSON_GetObjectItem(tracker,"confidence")->valuedouble);
		cJSON_AddNumberToObject(path,"timestamp",cJSON_GetObjectItem(tracker,"birth")->valuedouble);
		cJSON_AddNumberToObject(path,"age",cJSON_GetObjectItem(tracker,"age")->valuedouble);
		cJSON_AddNumberToObject(path,"dx",cJSON_GetObjectItem(tracker,"dx")->valuedouble);
		cJSON_AddNumberToObject(path,"dy",cJSON_GetObjectItem(tracker,"dy")->valuedouble);
		cJSON_AddNumberToObject(path,"distance",cJSON_GetObjectItem(tracker,"distance")->valuedouble);
		cJSON_AddNumberToObject(path,"topVelocity",cJSON_GetObjectItem(tracker,"topVelocity")->valuedouble);
		cJSON_AddNumberToObject(path,"sampletime",ACAP_DEVICE_Timestamp());
		cJSON_AddNumberToObject(path,"dwell",0);
		cJSON_AddStringToObject(path,"color",cJSON_GetObjectItem(tracker,"color")->valuestring);
		cJSON_AddStringToObject(path,"color2",cJSON_GetObjectItem(tracker,"color2")->valuestring);
		cJSON_AddFalseToObject(path,"hat");
		cJSON_GetObjectItem(path,"hat")->type = cJSON_GetObjectItem(tracker,"hat")->type;
		cJSON_AddFalseToObject(path,"face");
		cJSON_GetObjectItem(path,"face")->type = cJSON_GetObjectItem(tracker,"face")->type;
		cJSON* position = cJSON_CreateObject();
		cJSON_AddNumberToObject(position,"x",cJSON_GetObjectItem(tracker,"cx")->valuedouble);
		cJSON_AddNumberToObject(position,"y",cJSON_GetObjectItem(tracker,"cy")->valuedouble);
		cJSON_AddNumberToObject(position,"d",0);
		cJSON* pathList = cJSON_CreateArray();
		cJSON_AddItemToArray(pathList,position);
		cJSON_AddItemToObject(path,"path",pathList);
		cJSON_AddItemToObject(paths,id,path);
	} else {
		cJSON_ReplaceItemInObject(path,"confidence",cJSON_CreateNumber(cJSON_GetObjectItem(tracker,"confidence")->valuedouble));
		cJSON_ReplaceItemInObject(path,"class",cJSON_CreateString(cJSON_GetObjectItem(tracker,"class")->valuestring));
		cJSON_ReplaceItemInObject(path,"age",cJSON_CreateNumber(cJSON_GetObjectItem(tracker,"age")->valuedouble));
		cJSON_ReplaceItemInObject(path,"dx",cJSON_CreateNumber(cJSON_GetObjectItem(tracker,"dx")->valuedouble));
		cJSON_ReplaceItemInObject(path,"dy",cJSON_CreateNumber(cJSON_GetObjectItem(tracker,"dy")->valuedouble));
		cJSON_ReplaceItemInObject(path,"distance",cJSON_CreateNumber(cJSON_GetObjectItem(tracker,"distance")->valuedouble));
		cJSON_ReplaceItemInObject(path,"topVelocity",cJSON_CreateNumber(cJSON_GetObjectItem(tracker,"topVelocity")->valuedouble));
		cJSON_ReplaceItemInObject(path,"color",cJSON_CreateString(cJSON_GetObjectItem(tracker,"color")->valuestring));
		cJSON_ReplaceItemInObject(path,"color2",cJSON_CreateString(cJSON_GetObjectItem(tracker,"color2")->valuestring));
		cJSON_GetObjectItem(path,"hat")->type = cJSON_GetObjectItem(tracker,"hat")->type;
		cJSON_GetObjectItem(path,"face")->type = cJSON_GetObjectItem(tracker,"face")->type;
		double duration = cJSON_GetObjectItem(tracker,"timestamp")->valuedouble;
		duration -= cJSON_GetObjectItem(path,"sampletime")->valuedouble;
		duration /= 1000;
		if( duration > cJSON_GetObjectItem(path,"dwell")->valuedouble )
			cJSON_ReplaceItemInObject(path,"dwell",cJSON_CreateNumber(duration));
		cJSON_ReplaceItemInObject(path,"sampletime",cJSON_CreateNumber(cJSON_GetObjectItem(tracker,"timestamp")->valuedouble));
		cJSON* pathList = cJSON_GetObjectItem(path,"path");
		cJSON* lastPosition = cJSON_GetArrayItem(pathList,cJSON_GetArraySize(pathList)-1);
		cJSON_ReplaceItemInObject(lastPosition,"d",cJSON_CreateNumber(duration));
		cJSON* position = cJSON_CreateObject();
		cJSON_AddNumberToObject(position,"x",cJSON_GetObjectItem(tracker,"cx")->valuedouble);
		cJSON_AddNumberToObject(position,"y",cJSON_GetObjectItem(tracker,"cy")->valuedouble);
		cJSON_AddNumberToObject(position,"d",0);
		cJSON_AddItemToArray(pathList,position);
	}

	if( cJSON_GetObjectItem(tracker,"active")->type != cJSON_False )
		return;

	if( cJSON_GetObjectItem(publish,"path") && cJSON_GetObjectItem(publish,"path")->type == cJSON_True ) {
		if( cJSON_GetObjectItem(path,"age")->valuedouble > 1.0 && cJSON_GetArraySize(cJSON_GetObjectItem(path,"path")) > 1 && cJSON_GetObjectItem(path,"distance")->valuedouble  > 10 ) {
			sprintf(topic,"path/%s", ACAP_DEVICE_Prop("serial") );
			MQTT_Publish_JSON(topic,path,0,0);
		}
	}
	cJSON_DeleteItemFromObject(paths,id);
}

void
MAIN_Detections_Callback(cJSON *list ) {
	char topic[128];

	if( !attributes )
		return;
//	LOG_TRACE("%s: Process Detection\n",__func__);
	
	//Process Detections
	cJSON* settings = ACAP_Get_Config("settings");
	if( !settings ) {return;};
	
	cJSON* detectionsFilter = cJSON_GetObjectItem(settings,"detectionsFilter");
	cJSON* scene = cJSON_GetObjectItem(settings,"scene");
	double confidence = cJSON_GetObjectItem(scene,"confidence")->valueint;
	
	
	cJSON* humanColors = cJSON_GetObjectItem(attributes,"humanColors");
	if( !humanColors ) {
		LOG_TRACE("%s:Invalid humanColors",__func__);
		return;
	}

	cJSON* vehicleColors = cJSON_GetObjectItem(attributes,"vehicleColors");
	cJSON* vehicles = cJSON_GetObjectItem(attributes,"vehicles");
	cJSON* bags = cJSON_GetObjectItem(attributes,"bags");
	cJSON* hats = cJSON_GetObjectItem(attributes,"hats");
	cJSON* names = cJSON_GetObjectItem(attributes,"names");
	if( !detectionsFilter ) {LOG_TRACE("%s:Invalid detectionsFilter",__func__);return;};

	//Process detections
	cJSON* item = list->child;
	cJSON* lookup = 0;

	while(item) {
		cJSON* attribute = cJSON_GetObjectItem(item,"attributes")?cJSON_GetObjectItem(item,"attributes")->child:0;
		if( !cJSON_GetObjectItem(item,"color") )
			cJSON_AddStringToObject(item,"color","");
		if( !cJSON_GetObjectItem(item,"color") )
			cJSON_AddStringToObject(item,"color2","");
		if( !cJSON_GetObjectItem(item,"hat") )
			cJSON_AddFalseToObject(item,"hat");
		if( !cJSON_GetObjectItem(item,"face") )
			cJSON_AddFalseToObject(item,"face");
		while( attribute ) {
			int type = cJSON_GetObjectItem(attribute,"type")?cJSON_GetObjectItem(attribute,"type")->valueint: 0;
			int value = cJSON_GetObjectItem(attribute,"value")?cJSON_GetObjectItem(attribute,"value")->valueint: 0;
			int score = cJSON_GetObjectItem(attribute,"value")?cJSON_GetObjectItem(attribute,"value")->valueint: 0;
			switch( type ) {
				case 0:  //Undefined type
					break;
				case 1:  //Undefined type
					break;
				case 2: //Vehicle color
					lookup = cJSON_GetArrayItem(vehicleColors,value);
					if( lookup ) {
						if( cJSON_GetObjectItem(item,"color") ) {
							cJSON_ReplaceItemInObject(item,"color",cJSON_CreateString( lookup->valuestring) );
						} else {
							cJSON_AddStringToObject(item,"color",lookup->valuestring);
						}
					}
					break;
				case 3: //Upper color;
					lookup = cJSON_GetArrayItem(humanColors,value);
					if( lookup ) {
						if( cJSON_GetObjectItem(item,"color") ) {
							cJSON_ReplaceItemInObject(item,"color",cJSON_CreateString( lookup->valuestring) );
						} else {
							cJSON_AddStringToObject(item,"color",lookup->valuestring);
						}
					}
					break;
				case 4: //Lower color;
					lookup = cJSON_GetArrayItem(humanColors,value);
					if( lookup ) {
						if( cJSON_GetObjectItem(item,"color") ) {
							cJSON_ReplaceItemInObject(item,"color",cJSON_CreateString( lookup->valuestring) );
						} else {
							cJSON_AddStringToObject(item,"color",lookup->valuestring);
						}
					}
					break;
				case 5: //Bag type
					lookup = cJSON_GetArrayItem(bags,value);
					if( lookup )
						cJSON_ReplaceItemInObject( item,"class", cJSON_CreateString(lookup->valuestring));
					break;
				case 6: //Hat type 
					lookup = cJSON_GetArrayItem(hats,value);
					cJSON* hat = cJSON_GetObjectItem(item,"hat");
					if( lookup && hat )
						cJSON_ReplaceItemInObject( item, "hat", cJSON_CreateString(lookup->valuestring));
					if( lookup && !hat )
						cJSON_AddStringToObject( item, "hat", lookup->valuestring);
					break;
					case 7: //human_face_visibility
					if( cJSON_GetObjectItem(item,"face") ) {
						if( value = 0 )
							cJSON_GetObjectItem(item,"face")->type = cJSON_False;
						else
							cJSON_GetObjectItem(item,"face")->type = cJSON_True;
					} else {
						if( value = 0 )
							cJSON_AddFalseToObject(item,"face");
						else
							cJSON_AddTrueToObject(item,"face");
					}
					break;
				case 8:
					lookup = cJSON_GetArrayItem(vehicles,value);
					if( lookup )
						cJSON_ReplaceItemInObject( item,"class", cJSON_CreateString(lookup->valuestring));
					break;
				case 9:
					break;
				case 10:
					break;
					
			}
			attribute = attribute->next;
		}
		lookup = cJSON_GetObjectItem(names, cJSON_GetObjectItem(item,"class")->valuestring);
		if( lookup )
			cJSON_ReplaceItemInObject( item,"class", cJSON_CreateString(lookup->valuestring));
		item = item->next;
	}

	//Filter Detections
	if( !detectionsPassed )
		detectionsPassed = cJSON_CreateObject();
	cJSON* filteredDetections = cJSON_CreateArray();
	item = list->child;
	int accept = 1;
	while(item) {
		const char* id = cJSON_GetObjectItem(item,"id")->valuestring;
		if( cJSON_GetObjectItem( detectionsPassed, id ) ) {
			if( cJSON_GetObjectItem(item,"active")->type == cJSON_False ) {
				cJSON_DetachItemFromObject( filteredDetections, id );
				cJSON_AddItemReferenceToArray( filteredDetections, item );	
			} else {
				accept = 1;
				if( accept && cJSON_GetObjectItem(item,"confidence")->valueint < confidence ) accept = 0;
				cJSON* ignore = cJSON_GetObjectItem(detectionsFilter,"ignoreClass")?cJSON_GetObjectItem(detectionsFilter,"ignoreClass")->child:0;
				while( ignore && accept ) {
					if( strcmp( ignore->valuestring, cJSON_GetObjectItem(item,"class")->valuestring) == 0 )
						accept = 0;
					ignore = ignore->next;
				}
				if( accept )
					cJSON_AddItemReferenceToArray( filteredDetections, item );
			}
		} else {
			cJSON* aoi = cJSON_GetObjectItem(detectionsFilter,"aoi");
			int minWidth = cJSON_GetObjectItem(detectionsFilter,"minWidth")->valueint;
			int minHeight = cJSON_GetObjectItem(detectionsFilter,"minHeight")->valueint;
			int maxWidth = cJSON_GetObjectItem(detectionsFilter,"maxWidth")->valueint;
			int maxHeight = cJSON_GetObjectItem(detectionsFilter,"maxHeight")->valueint;	
			int x1 = cJSON_GetObjectItem(aoi,"x1")->valueint;
			int x2 = cJSON_GetObjectItem(aoi,"x2")->valueint;
			int y1 = cJSON_GetObjectItem(aoi,"y1")->valueint;
			int y2 = cJSON_GetObjectItem(aoi,"y2")->valueint;
			
			accept = 1;
			if( cJSON_GetObjectItem(item,"confidence")->type != cJSON_Number ) accept = 0;
			if( accept && cJSON_GetObjectItem(item,"confidence")->valueint < confidence ) accept = 0;
			if( accept && (cJSON_GetObjectItem(item,"cx")->valueint < x1 || cJSON_GetObjectItem(item,"cx")->valueint > x2 ) ) accept = 0;
			if( accept && (cJSON_GetObjectItem(item,"cy")->valueint < y1 || cJSON_GetObjectItem(item,"cy")->valueint > y2) ) accept = 0;
			if( accept && (cJSON_GetObjectItem(item,"w")->valueint < minWidth || cJSON_GetObjectItem(item,"w")->valueint > maxWidth )) accept = 0;
			if( accept && (cJSON_GetObjectItem(item,"h")->valueint < minHeight || cJSON_GetObjectItem(item,"w")->valueint > maxHeight )) accept = 0;
			cJSON* ignore = cJSON_GetObjectItem(detectionsFilter,"ignoreClass")?cJSON_GetObjectItem(detectionsFilter,"ignoreClass")->child:0;
			while( ignore && accept ) {
				if( strcmp( ignore->valuestring, cJSON_GetObjectItem(item,"class")->valuestring) == 0 )
					accept = 0;
				ignore = ignore->next;
			}
			if( accept ) {
				cJSON_AddTrueToObject(detectionsPassed, id );
				cJSON_AddItemReferenceToArray( filteredDetections, item );	
			}
		}
		item = item->next;
	}


	//Publish Detections
	if( cJSON_GetObjectItem(publish,"detections") && cJSON_GetObjectItem(publish,"detections")->type == cJSON_True ) {
		if( cJSON_GetArraySize(filteredDetections) > 0 ) {
			sprintf(topic,"detections/%s", ACAP_DEVICE_Prop("serial") );
			cJSON* detections = cJSON_CreateObject();
			cJSON_AddItemReferenceToObject( detections, "list", filteredDetections );
			MQTT_Publish_JSON(topic,detections,0,0);
			cJSON_Delete( detections );
		}
	}

	//Process Tracker
	if(!PreviousPosition)
		PreviousPosition = cJSON_CreateObject();
	if(!PreviousTimestamp)
		PreviousTimestamp = cJSON_CreateObject();

	item = filteredDetections->child;
	while( item ) {
		const char* id = cJSON_GetObjectItem(item,"id")->valuestring;
		
		if( cJSON_GetObjectItem(item,"active")->type == cJSON_False && cJSON_GetObjectItem(PreviousPosition,id) ) {
			cJSON_DeleteItemFromObject(PreviousPosition,id);
			Trackers( item );
		} else {
			cJSON* position = cJSON_GetObjectItem(PreviousPosition,id);
			if( position ) {
				double cx = cJSON_GetObjectItem(item,"cx")->valuedouble;
				double cy = cJSON_GetObjectItem(item,"cy")->valuedouble;
				double pcx = cJSON_GetObjectItem(position,"cx")->valuedouble;
				double pcy = cJSON_GetObjectItem(position,"cy")->valuedouble;
				double dx = cx - pcx;
				double dy = cy - pcy;
				int distance = sqrt( (dx*dx) + (dy*dy) ) / 10.0;
				if( distance > 5 ) {
					double topVelocity = cJSON_GetObjectItem(item,"topVelocity")->valuedouble;
					double seconds = (cJSON_GetObjectItem(item,"timestamp")->valuedouble - cJSON_GetObjectItem(position,"timestamp")->valuedouble)/1000.0;
					double velocity = distance / seconds;
					if( velocity > topVelocity )
						cJSON_ReplaceItemInObject(item,"topVelocity",cJSON_CreateNumber(velocity));
					cJSON_ReplaceItemInObject(position,"cx",cJSON_CreateNumber(cx));
					cJSON_ReplaceItemInObject(position,"cy",cJSON_CreateNumber(cy));
					cJSON_ReplaceItemInObject(position,"timestamp",cJSON_CreateNumber(cJSON_GetObjectItem(item,"timestamp")->valuedouble));
					int previousDistance = cJSON_GetObjectItem(item,"distance")->valueint;
					cJSON_ReplaceItemInObject(item,"distance",cJSON_CreateNumber(previousDistance+distance));
					Trackers( item );
				}
			
				if( lastPublishedTracker && cJSON_GetObjectItem(publish,"tracker") && cJSON_GetObjectItem(publish,"tracker")->type == cJSON_True ) {
					double duration = (ACAP_DEVICE_Timestamp() - cJSON_GetObjectItem(lastPublishedTracker,id)->valuedouble) / 1000.0;
					if( duration > 2 ) {
						sprintf(topic,"tracker/%s", ACAP_DEVICE_Prop("serial") );
						MQTT_Publish_JSON(topic,item,0,0);
						cJSON_ReplaceItemInObject( lastPublishedTracker,id,cJSON_CreateNumber(ACAP_DEVICE_Timestamp()));
					}
				}
			} else {
				cJSON* position = cJSON_CreateObject();
				cJSON_AddNumberToObject(position,"cx",cJSON_GetObjectItem(item,"cx")->valuedouble);
				cJSON_AddNumberToObject(position,"cy",cJSON_GetObjectItem(item,"cy")->valuedouble);
				cJSON_AddNumberToObject(position,"timestamp",cJSON_GetObjectItem(item,"timestamp")->valuedouble);
				
				cJSON_AddItemToObject(PreviousPosition,id,position);
				Trackers( item );
			}
		}
		item = item->next;
	}
	
	cJSON_Delete(filteredDetections);
}


void
MAIN_Event_Callback(cJSON *event, void* userdata) {
	if(!event)
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
	if( ignore ) {
		cJSON_Delete( eventTopic );
		return;
	}
		

	//Filter unwanted topics
	cJSON* settings = ACAP_Get_Config("settings");
	cJSON* eventFilter = cJSON_GetObjectItem(settings,"eventTopics")?cJSON_GetObjectItem(settings,"eventTopics")->child:0;
	while( eventFilter ) { 
		if( cJSON_GetObjectItem(eventFilter,"enabled")->type == cJSON_False ) {
			const char* ignoreTopic = cJSON_GetObjectItem(eventFilter, "topic")?cJSON_GetObjectItem(eventFilter, "topic")->valuestring:0;
			if( ignoreTopic && strstr(eventTopic->valuestring, ignoreTopic) ) {
LOG("%s\n",eventTopic->valuestring);
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
Connection_Status (int state) {
	switch( state ) {
		case MQTT_CONNECT:
			LOG_TRACE("%s: Connect\n",__func__);
			break;
		case MQTT_RECONNECT:
			LOG("%s: Reconnect\n",__func__);
			break;
		case MQTT_DISCONNECT:
			LOG("%s: Disconnect\n",__func__);
			break;
	}
}

void
MAIN_MQTT_Subscription(const char *topic, const char *payload) {
	LOG("Subscription: %s %s\n",topic,payload);
}

static gboolean
Main_Status_Update(gpointer user_data) {
	cJSON* payload = cJSON_CreateObject();
	cJSON_AddNumberToObject(payload,"Network_Kbps",(int)ACAP_DEVICE_Network_Average());
	cJSON_AddNumberToObject(payload,"CPU_average",(int)(ACAP_DEVICE_CPU_Average()*100));
	cJSON_AddNumberToObject(payload,"Uptime_Hours",(int)(ACAP_DEVICE_Uptime()/3600));
	
	if( !cJSON_GetObjectItem(publish,"status") || cJSON_GetObjectItem(publish,"status")->type != cJSON_True ) {
		cJSON_Delete(payload);
		return G_SOURCE_CONTINUE;
	}

	char topic[256];
	sprintf(topic,"status/%s",ACAP_DEVICE_Prop("serial"));
	MQTT_Publish_JSON( topic, payload, 0, 0 );
	
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

int
main(void) {
	openlog(APP_PACKAGE, LOG_PID|LOG_CONS, LOG_USER);
	LOG("------ Starting ACAP Service ------\n");
	
	ACAP( APP_PACKAGE, Settings_Updated_Callback );
	MQTT_Init( APP_PACKAGE, Connection_Status );
	ACAP_Set_Config("mqtt", MQTT_Settings());
	MQTT_Subscribe( "mqtt", MAIN_MQTT_Subscription );


	ACAP_EVENTS_SetCallback( MAIN_Event_Callback );
	cJSON* eventSubscriptions = ACAP_FILE_Read( "settings/subscriptions.json" );
	cJSON* subscription = eventSubscriptions?eventSubscriptions->child:0;
	while(subscription){
		ACAP_EVENTS_Subscribe( subscription, NULL );
		subscription = subscription->next;
	}

	char* json = cJSON_PrintUnformatted( ACAP_Get_Config("settings") );
	if( json ) {
		LOG_TRACE("%s: %s\n",__func__,json);
		free(json);
	}

	attributes = ACAP_FILE_Read( "settings/attributes.json" );
	if( !attributes ) {
		LOG_WARN("Missing attributed\n");
	} else {
		ACAP_Set_Config("attributes", attributes);
		json = cJSON_PrintUnformatted(attributes);
		if(json) {
			LOG_TRACE("%s: %s\n",__func__, json );
			free(json);
		}
	}
	publish = cJSON_GetObjectItem(ACAP_Get_Config("settings"),"publish");

	ObjectDetection_Init( MAIN_Detections_Callback );
	g_timeout_add_seconds(15 * 60, Main_Status_Update, NULL);	
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
	ACAP_Cleanup();
	MQTT_Cleanup();
	
    return 0;
}
