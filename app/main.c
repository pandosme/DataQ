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


cJSON*
Process_Detections( cJSON* detections ) {
	cJSON* response = cJSON_CreateArray();
	int accept;
	LOG_TRACE("%s: Entry\n",__func__);

	cJSON* item = detections->child;
	while(item) {
		if( cJSON_GetObjectItem(item,"ignore")->type == cJSON_False )
			cJSON_AddItemReferenceToArray( response, item );
		if( cJSON_GetObjectItem(item,"active")->type == cJSON_False )
			cJSON_AddItemReferenceToArray( response, item );
		item = item->next;
	}
	
	
	if( cJSON_GetArraySize( response ) > 0 ) {
		lastDetectionListWasEmpty = 0;
		return response;
	}
	
	if( lastDetectionListWasEmpty == 1 )
		return 0;
	lastDetectionListWasEmpty = 1;
	
	LOG_TRACE("%s: Exit\n",__func__);
	return response;
}

cJSON*
Process_Trackers( cJSON* detections ) {
	const char* id = 0;

	LOG_TRACE("%s: Entry\n",__func__);
	
	cJSON* response = cJSON_CreateArray();
	cJSON* settings = ACAP_Get_Config("settings");
	if(!settings)
		return response;
	
	cJSON* trackerFilter = cJSON_GetObjectItem(settings,"trackerFilter");
	if( !trackerFilter )
		return response;

	cJSON* sceneFilter = cJSON_GetObjectItem(settings,"scene");
	if( !sceneFilter )
		return response;
		
	double minAge = cJSON_GetObjectItem(trackerFilter,"age")->valuedouble;
	int minDistance = cJSON_GetObjectItem(trackerFilter,"distance")->valuedouble;
	double maxIdle = cJSON_GetObjectItem(sceneFilter,"maxIdle")->valuedouble;
	
	if(!PreviousPosition)
		PreviousPosition = cJSON_CreateObject();

	if(!activeTrackers)
		activeTrackers = cJSON_CreateObject();

	cJSON* item = detections->child;
	while( item ) {
		id = cJSON_GetObjectItem(item,"id")->valuestring;
		cJSON* position = cJSON_GetObjectItem(PreviousPosition,id);
		if( cJSON_GetObjectItem(item,"active")->type == cJSON_False ) {
			cJSON_DeleteItemFromObject(PreviousPosition,id);
			cJSON_DeleteItemFromObject(activeTrackers,id);
			if( position )
				cJSON_AddItemReferenceToArray(response,item);
		} else {
			if( position ) {
				double cx = cJSON_GetObjectItem(item,"cx")->valuedouble;
				double cy = cJSON_GetObjectItem(item,"cy")->valuedouble;
				double pcx = cJSON_GetObjectItem(position,"cx")->valuedouble;
				double pcy = cJSON_GetObjectItem(position,"cy")->valuedouble;
				double dx = cx - pcx;
				double dy = cy - pcy;
				double distance = sqrt( (dx*dx) + (dy*dy) ) / 10.0;
				double lat = 0;
				double lng = 0;
				if( distance >= 5.0 ) {
					double topVelocity = cJSON_GetObjectItem(item,"topVelocity")->valuedouble;
					double seconds = (cJSON_GetObjectItem(item,"timestamp")->valuedouble - cJSON_GetObjectItem(position,"timestamp")->valuedouble)/1000.0;
					double velocity = distance / seconds;
					if( velocity > topVelocity )
						cJSON_ReplaceItemInObject(item,"topVelocity",cJSON_CreateNumber(velocity));
					cJSON_ReplaceItemInObject(position,"cx",cJSON_CreateNumber(cx));
					cJSON_ReplaceItemInObject(position,"cy",cJSON_CreateNumber(cy));
					if( cJSON_GetObjectItem(item,"ignore")->type == cJSON_True ) {
						cJSON_GetObjectItem(item,"ignore")->type = cJSON_False;
						cJSON_ReplaceItemInObject(position,"birth",cJSON_CreateNumber(cJSON_GetObjectItem(item,"timestamp")->valuedouble));
					}
					cJSON_ReplaceItemInObject(position,"timestamp",cJSON_CreateNumber(cJSON_GetObjectItem(item,"timestamp")->valuedouble));
					cJSON_ReplaceItemInObject(item,"idle",cJSON_CreateNumber(0));
					
					int previousDistance = cJSON_GetObjectItem(item,"distance")->valueint;
					cJSON_ReplaceItemInObject(item,"distance",cJSON_CreateNumber(previousDistance+(int)distance));
					cJSON_AddItemReferenceToArray(response,item);
				} else {
					double currentTime = cJSON_GetObjectItem(item,"timestamp")->valuedouble;
					double lastMovementTime = cJSON_GetObjectItem(position,"timestamp")->valuedouble;
					double idle = (currentTime - lastMovementTime)/1000;
					cJSON_ReplaceItemInObject(item,"idle",cJSON_CreateNumber(idle));
					if( maxIdle > 0 && idle > maxIdle && cJSON_GetObjectItem(item,"ignore")->type == cJSON_False ) {
						cJSON_GetObjectItem(item,"ignore")->type = cJSON_True;
						cJSON_AddItemReferenceToArray(response,item);
					}
				}
			} else {
				int x = cJSON_GetObjectItem(item,"cx")->valueint;
				int y = cJSON_GetObjectItem(item,"cy")->valueint;
				double bx = cJSON_GetObjectItem(item,"bx")->valuedouble;
				double by = cJSON_GetObjectItem(item,"by")->valuedouble;
				double dx = x - bx;
				double dy = y - by;
				int distance = sqrt( (dx*dx) + (dy*dy) ) / 10.0;
				int accept = 1;
				if( accept && cJSON_GetObjectItem(item,"ignore")->type == cJSON_True ) accept = 0;
				if( accept && cJSON_GetObjectItem(item,"age")->valuedouble < minAge ) accept = 0;
				if( accept && distance < minDistance ) accept = 0;
				if( accept ) {
					cJSON* position = cJSON_CreateObject();
					cJSON_AddNumberToObject(position,"cx",x);
					cJSON_AddNumberToObject(position,"cy",y);
					cJSON_AddNumberToObject(position,"timestamp",cJSON_GetObjectItem(item,"timestamp")->valuedouble);
					cJSON_ReplaceItemInObject(item,"idle",cJSON_CreateNumber(0));
					cJSON_AddItemReferenceToObject(activeTrackers,id,item);
					cJSON_AddItemToObject(PreviousPosition,id,position);
					cJSON_AddItemReferenceToArray(response,item);
				}
				//Handle case when exisitng objects does not pass the minimum distance
				if( maxIdle > 0 && cJSON_GetObjectItem(item,"age")->valuedouble > maxIdle && cJSON_GetObjectItem(item,"ignore")->type == cJSON_False )
					cJSON_GetObjectItem(item,"ignore")->type = cJSON_True;
			}
		}
		item = item->next;
	}
	LOG_TRACE("%s: Exit\n",__func__);
	return response;
}

cJSON*
Process_Paths( cJSON* trackers ) {
	//Important to free the return JSON

	LOG_TRACE("%s: Entry\n",__func__);

	if( !pathsCache )
		pathsCache = cJSON_CreateObject();
	cJSON* response = cJSON_CreateArray();
	
	cJSON* settings = ACAP_Get_Config("settings");
	if(!settings)
		return response;
	
	cJSON* pathFilter = cJSON_GetObjectItem(settings,"pathFilter");
	if( !pathFilter )
		return response;
	cJSON* aoi = cJSON_GetObjectItem(pathFilter,"aoi");
	if( !aoi )
		return response;

	
	cJSON* tracker = trackers->child;
	while( tracker ) {
		const char* id = cJSON_GetObjectItem(tracker,"id")->valuestring;
		cJSON* path = cJSON_GetObjectItem(pathsCache,id);
		if( !path) {
			if( cJSON_GetObjectItem(tracker,"active")->type == cJSON_True && cJSON_GetObjectItem(tracker,"ignore")->type == cJSON_False ) {
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
				cJSON* position = cJSON_CreateObject();
				cJSON_AddNumberToObject(position,"x",cJSON_GetObjectItem(tracker,"cx")->valuedouble);
				cJSON_AddNumberToObject(position,"y",cJSON_GetObjectItem(tracker,"cy")->valuedouble);
				if( cJSON_GetObjectItem(tracker,"lat") )
					cJSON_AddNumberToObject(position,"lat", cJSON_GetObjectItem(tracker,"lat")->valuedouble );
				if( cJSON_GetObjectItem(tracker,"lon") )
					cJSON_AddNumberToObject(position,"lon", cJSON_GetObjectItem(tracker,"lon")->valuedouble );
				cJSON_AddNumberToObject(position,"d",0);
				cJSON* pathList = cJSON_CreateArray();
				cJSON_AddItemToArray(pathList,position);
				cJSON_AddItemToObject(path,"path",pathList);
				cJSON_AddItemToObject(pathsCache,id,path);
			}
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
			if( cJSON_GetObjectItem(tracker,"lat") )
				cJSON_AddNumberToObject(position,"lat", cJSON_GetObjectItem(tracker,"lat")->valuedouble );
			if( cJSON_GetObjectItem(tracker,"lon") )
				cJSON_AddNumberToObject(position,"lon", cJSON_GetObjectItem(tracker,"lon")->valuedouble );
			cJSON_AddItemToArray(pathList,position);
		}

		if( path && ( cJSON_GetObjectItem(tracker,"active")->type == cJSON_False ||  cJSON_GetObjectItem(tracker,"ignore")->type == cJSON_True ) ) {
			cJSON* complete = cJSON_DetachItemFromObject( pathsCache, id );
			int accept = 1;
			double minAge = cJSON_GetObjectItem(pathFilter,"age")->valuedouble;
			double minDistance = cJSON_GetObjectItem(pathFilter,"distance")->valueint;
			
			if( cJSON_GetObjectItem(complete,"distance")->valuedouble < minDistance )  accept = 0;
			if( accept && cJSON_GetObjectItem(complete,"age")->valuedouble < minAge ) accept = 0;
			if( accept ) {
				int x1 = cJSON_GetObjectItem(aoi,"x1")->valueint;
				int y1 = cJSON_GetObjectItem(aoi,"y1")->valueint;
				int x2 = cJSON_GetObjectItem(aoi,"x2")->valueint;
				int y2 = cJSON_GetObjectItem(aoi,"y2")->valueint;
				accept = 0;
				cJSON* position = cJSON_GetObjectItem(path,"path")->child;
				if( position && cJSON_GetArraySize(position) > 2 ) {
					while(position && !accept ) {
						if( cJSON_GetObjectItem(position,"x")->valueint > x1 && 
							cJSON_GetObjectItem(position,"x")->valueint < x2 &&
							cJSON_GetObjectItem(position,"y")->valueint > y1 &&
							cJSON_GetObjectItem(position,"y")->valueint < y2
						) accept = 1;
						position = position->next;
					}
				}
			}
			if( accept && (cJSON_GetArraySize(cJSON_GetObjectItem(complete,"path")) < 2 || cJSON_GetObjectItem(complete,"age")->valuedouble < 0.5 ) ) accept = 0;
			if( accept ) {
				if( strcmp( cJSON_GetObjectItem(tracker,"class")->valuestring,"Head") == 0 ) {
					cJSON_AddStringToObject( complete,"hat", cJSON_GetObjectItem(tracker,"hat")->valuestring);
					if( cJSON_GetObjectItem(tracker,"face")->type == cJSON_False )
						cJSON_AddFalseToObject(complete,"face");
					else
						cJSON_AddTrueToObject(complete,"face");
				}
				cJSON_DeleteItemFromObject(complete,"sampleTime");
				cJSON_AddItemToArray( response, complete );
			} else {
				cJSON_Delete(complete);
			}
		}
		tracker = tracker->next;
	}
	LOG_TRACE("%s: Exit\n",__func__);	
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
Process_VOD_Data(cJSON *list ) {
	char topic[128];
	cJSON* item = 0;
	LOG_TRACE("%s:\n",__func__);

	if( !lastPublishedTracker )
		lastPublishedTracker = cJSON_CreateObject();

	//Detections
	cJSON* detections = Process_Detections( list );
	if( publishDetections && detections ) {
		sprintf(topic,"detections/%s", ACAP_DEVICE_Prop("serial") );
		cJSON* payload = cJSON_CreateObject();
		cJSON_AddItemReferenceToObject( payload, "list", detections );
		MQTT_Publish_JSON(topic,payload,0,0);
		cJSON_Delete( payload );
	}
	if( detections )
		cJSON_Delete(detections);
	
	//Occupancy
	cJSON* occupancy = Process_Occupancy( list );
	if( publishOccupancy && occupancy ) {
		cJSON* payload = cJSON_CreateObject();
		cJSON_AddItemReferenceToObject(payload,"occupancy",occupancy);
		sprintf(topic,"occupancy/%s", ACAP_DEVICE_Prop("serial") );
		MQTT_Publish_JSON(topic,payload,0,0);
		cJSON_Delete(payload);
	}	

	//Trackers
	cJSON* trackers = Process_Trackers( list );
	double now = ACAP_DEVICE_Timestamp();
	if( publishTracker ) {
		item = trackers->child;
		while( item ) {
			sprintf(topic,"tracker/%s", ACAP_DEVICE_Prop("serial") );
			MQTT_Publish_JSON(topic,item,0,0);
			char* id = cJSON_GetObjectItem(item,"id")->valuestring;
			cJSON* lastPublished = cJSON_GetObjectItem(lastPublishedTracker,id);
			if( lastPublished ) {
				lastPublished->valueint = (int)now;
				lastPublished->valuedouble = now;
			} else {
				cJSON_AddNumberToObject(lastPublishedTracker,id,now);
			}
			if( cJSON_GetObjectItem(item,"active")->type == cJSON_False )
				cJSON_DeleteItemFromObject(lastPublishedTracker,id);
			item = item->next;
		}
		//Publish tracker updates while idle
		item = activeTrackers?activeTrackers->child:0;
		while( item ) {
			char* id = cJSON_GetObjectItem(item,"id")->valuestring;
			//Check if reach max idle time.  If so, do not publish
			if( cJSON_GetObjectItem( item,"ignore")->type == cJSON_False ) {
				cJSON* lastPublished = cJSON_GetObjectItem(lastPublishedTracker,id);
				if( lastPublished ) {
					if( now - lastPublished->valuedouble > 2000 ) {
						sprintf(topic,"tracker/%s", ACAP_DEVICE_Prop("serial") );
						MQTT_Publish_JSON(topic,item,0,0);
						lastPublished->valueint = (int)now;
						lastPublished->valuedouble = now;
					}
				}
			}
			item = item->next;
		}
	}

	//GeoSpace locations
	if( publishGeospace ) {
		item = trackers->child;
		while( item ) {
			sprintf(topic,"geospace/%s", ACAP_DEVICE_Prop("serial") );
			cJSON* payload = cJSON_CreateObject();
			cJSON_AddStringToObject(payload,"id",cJSON_GetObjectItem(item,"id")->valuestring);
			cJSON_AddTrueToObject(payload,"active");
			if( cJSON_GetObjectItem(item,"active")->type == cJSON_False || cJSON_GetObjectItem(item,"ignore")->type == cJSON_True)
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

	cJSON_Delete(trackers);
	cJSON_Delete(paths);
	LOG_TRACE("%s: Exit\n",__func__);
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
MQTT_Status_Callback (int state) {
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
MQTT_Subscription(const char *topic, const char *payload) {
	LOG("Subscription: %s %s\n",topic,payload);
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
	MQTT_Init( MQTT_Status_Callback, MQTT_Subscription );
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
	MQTT_Status_Callback(MQTT_DISCONNECTING); //Send graceful disconnect message

	MQTT_Cleanup();
	ACAP_Cleanup();
	
    return 0;
}
