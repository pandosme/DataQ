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
//#define LOG_TRACE(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_TRACE(fmt, args...)    {}

cJSON* attributes = 0;
cJSON* detectionsPassed = 0;
cJSON* lastPublishedTracker = 0;
cJSON* pathsCache = 0;
cJSON* PreviousPosition = 0;
cJSON* occupancyDetectionCounter = 0;
cJSON* classCounterArrays = 0;
cJSON* previousOccupancy = 0;

cJSON*
ProcessDetections( cJSON* list ) {
	if( !attributes )
		return list;
	
	cJSON* settings = ACAP_Get_Config("settings");
	if( !settings )	{
		return list;
	};
	
	cJSON* humanColors = cJSON_GetObjectItem(attributes,"humanColors");
	if( !humanColors ) {
		LOG_TRACE("%s:Invalid humanColors",__func__);
		return list;
	}

	cJSON* vehicleColors = cJSON_GetObjectItem(attributes,"vehicleColors");
	cJSON* vehicles = cJSON_GetObjectItem(attributes,"vehicles");
	cJSON* bags = cJSON_GetObjectItem(attributes,"bags");
	cJSON* hats = cJSON_GetObjectItem(attributes,"hats");
	cJSON* names = cJSON_GetObjectItem(attributes,"names");

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
					if( cJSON_GetObjectItem(settings,"vehicleAttributeType")->valueint == 0 ) {
						lookup = cJSON_GetArrayItem(vehicles,value);
						if( lookup )
							cJSON_ReplaceItemInObject( item,"class", cJSON_CreateString(lookup->valuestring));
					}
					break;
				case 1:  //Undefined type
					if( cJSON_GetObjectItem(settings,"vehicleAttributeType")->valueint == 1 ) {
						lookup = cJSON_GetArrayItem(vehicles,value);
						if( lookup )
							cJSON_ReplaceItemInObject( item,"class", cJSON_CreateString(lookup->valuestring));
					}
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
					if( cJSON_GetObjectItem(settings,"vehicleAttributeType")->valueint == 8 ) {
						lookup = cJSON_GetArrayItem(vehicles,value);
						if( lookup )
							cJSON_ReplaceItemInObject( item,"class", cJSON_CreateString(lookup->valuestring));
					}
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
	return list;
}

cJSON*
FilterDetections( cJSON* list ) {
	//Filter Detections
	if( !detectionsPassed )
		detectionsPassed = cJSON_CreateObject();

	cJSON* settings = ACAP_Get_Config("settings");
	if(!settings)
		return list;
	cJSON* detectionsFilter = cJSON_GetObjectItem(settings,"detectionsFilter");
	if( !detectionsFilter )
		return list;

	cJSON* response = cJSON_CreateArray();
	cJSON* item = list->child;
	int accept = 1;
	cJSON* aoi = cJSON_GetObjectItem(detectionsFilter,"aoi");	
	int x1 = cJSON_GetObjectItem(aoi,"x1")->valueint;
	int x2 = cJSON_GetObjectItem(aoi,"x2")->valueint;
	int y1 = cJSON_GetObjectItem(aoi,"y1")->valueint;
	int y2 = cJSON_GetObjectItem(aoi,"y2")->valueint;
	int minWidth = cJSON_GetObjectItem(detectionsFilter,"minWidth")->valueint;
	int minHeight = cJSON_GetObjectItem(detectionsFilter,"minHeight")->valueint;
	int maxWidth = cJSON_GetObjectItem(detectionsFilter,"maxWidth")->valueint;
	int maxHeight = cJSON_GetObjectItem(detectionsFilter,"maxHeight")->valueint;	
	int trackOutside = (cJSON_GetObjectItem(detectionsFilter,"maxHeight")->type == cJSON_True)?1:0;	
	
	while(item) {
		const char* id = cJSON_GetObjectItem(item,"id")->valuestring;
		if( cJSON_GetObjectItem( detectionsPassed, id ) ) {
			if( cJSON_GetObjectItem(item,"active")->type != cJSON_True ) {
				cJSON_DetachItemFromObject( detectionsPassed, id );
				cJSON_AddItemReferenceToArray( response, item );	
			} else {
/*				
				accept = 1;
				if( !trackOutside ) {
					if( accept && (cJSON_GetObjectItem(item,"cx")->valueint < x1 || cJSON_GetObjectItem(item,"cx")->valueint > x2 ) ) accept = 0;
					if( accept && (cJSON_GetObjectItem(item,"cy")->valueint < y1 || cJSON_GetObjectItem(item,"cy")->valueint > y2) ) accept = 0;
					if( accept && (cJSON_GetObjectItem(item,"w")->valueint < minWidth || cJSON_GetObjectItem(item,"w")->valueint > maxWidth )) accept = 0;
					if( accept && (cJSON_GetObjectItem(item,"h")->valueint < minHeight || cJSON_GetObjectItem(item,"w")->valueint > maxHeight )) accept = 0;
				}

				if( !accept )
					cJSON_GetObjectItem(item,"ignore")->type = cJSON_NULL;
*/				
				cJSON* ignore = cJSON_GetObjectItem(detectionsFilter,"ignoreClass")?cJSON_GetObjectItem(detectionsFilter,"ignoreClass")->child:0;
				while( ignore && accept ) {
					if( strcmp( ignore->valuestring, cJSON_GetObjectItem(item,"class")->valuestring) == 0 )
						accept = 0;
					ignore = ignore->next;
				}
				if( accept )
					cJSON_AddItemReferenceToArray( response, item );
			}
		} else {
			accept = 1;
			if( cJSON_GetObjectItem(item,"active")->type == cJSON_False ) accept = 0;
			if( cJSON_GetObjectItem(item,"ignore")->type == cJSON_True ) accept = 0;
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
				cJSON_AddItemReferenceToArray( response, item );	
			}
		}
		item = item->next;
	}
	return response;
}

cJSON* 
UpdateTrackerPublish( cJSON* tracker ) {

	const char* id = cJSON_GetObjectItem(tracker,"id")->valuestring;

	if( !lastPublishedTracker )
		lastPublishedTracker = cJSON_CreateObject();
	if( cJSON_GetObjectItem(tracker,"active")->type == cJSON_False ) {
		cJSON_DeleteItemFromObject( lastPublishedTracker,id);
	} else {
		if( !cJSON_GetObjectItem( lastPublishedTracker, id ) ) {
			cJSON_AddItemToObject( lastPublishedTracker,id, cJSON_CreateNumber( ACAP_DEVICE_Timestamp() ) );
		} else {
			cJSON_ReplaceItemInObject( lastPublishedTracker,id, cJSON_CreateNumber( ACAP_DEVICE_Timestamp() ) );
		}
	}	
}

cJSON*
ProcessTrackers( cJSON* detections ) {
	const char* id = 0;
	cJSON* response = cJSON_CreateArray();
	
	if(!PreviousPosition)
		PreviousPosition = cJSON_CreateObject();

	cJSON* item = detections->child;
	while( item ) {
		id = cJSON_GetObjectItem(item,"id")->valuestring;
		if( cJSON_GetObjectItem(item,"active")->type == cJSON_False && cJSON_GetObjectItem(PreviousPosition,id) ) {
			cJSON_DeleteItemFromObject(PreviousPosition,id);
			cJSON_AddItemReferenceToArray(response,item);
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
					cJSON_AddItemReferenceToArray(response,item);
				}
			} else {
				if( cJSON_GetObjectItem(item,"active")->type == cJSON_True && cJSON_GetObjectItem(item,"ignore")->type == cJSON_False ) {
					cJSON* position = cJSON_CreateObject();
					cJSON_AddNumberToObject(position,"cx",cJSON_GetObjectItem(item,"cx")->valuedouble);
					cJSON_AddNumberToObject(position,"cy",cJSON_GetObjectItem(item,"cy")->valuedouble);
					cJSON_AddNumberToObject(position,"timestamp",cJSON_GetObjectItem(item,"timestamp")->valuedouble);
					cJSON_AddItemToObject(PreviousPosition,id,position);
					cJSON_AddItemReferenceToArray(response,item);
				}
			}
		}
		item = item->next;
	}
	return response;
}

cJSON*
ProcessPaths( cJSON* trackers ) {
	//Important to free the return JSON
	if( !pathsCache )
		pathsCache = cJSON_CreateObject();
	cJSON* response = cJSON_CreateArray();
	
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

		if( path && cJSON_GetObjectItem(tracker,"active")->type == cJSON_False ) {
			cJSON* complete = cJSON_DetachItemFromObject( pathsCache, id );
			int accept = 1;
			if( accept && cJSON_GetObjectItem(complete,"distance")->valuedouble < 10 ) accept = 0;
			if( accept && cJSON_GetObjectItem(complete,"age")->valuedouble < 1.0 ) accept = 0;
			if( accept )
				cJSON_AddItemToArray( response, complete );
			else 
				cJSON_Delete(complete);
		}
		tracker = tracker->next;
	}
	return response;
}

cJSON*
ProcessOccupancy( cJSON* detections ) {
	//Will reuturn 0 if there is no occupancy change;
	//Do NOT free return JSON
	cJSON* item;
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
	
	//Integrate counters
	int size = cJSON_GetObjectItem(ACAP_Get_Config("settings"),"occupancyIntegration")->valueint;

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
	return response;
}

void
MAIN_Detections_Callback(cJSON *list ) {
	char topic[128];
	cJSON* item = 0;
	cJSON* settings = ACAP_Get_Config("settings");
	if( !settings )
		return;
	cJSON* publish = cJSON_GetObjectItem(settings,"publish");
	list = ProcessDetections( list );
	//Publish Detections
	cJSON* detections = FilterDetections( list );
	if( cJSON_GetObjectItem(publish,"detections") && cJSON_GetObjectItem(publish,"detections")->type == cJSON_True ) {
		sprintf(topic,"detections/%s", ACAP_DEVICE_Prop("serial") );
		cJSON* payload = cJSON_CreateObject();
		cJSON_AddItemReferenceToObject( payload, "list", detections );
		MQTT_Publish_JSON(topic,payload,0,0);
		cJSON_Delete( payload );
	}

	//Publish Occupancy
	cJSON* occupancy = ProcessOccupancy( detections );
	
	if( occupancy && cJSON_GetObjectItem(publish,"occupancy") && cJSON_GetObjectItem(publish,"occupancy")->type == cJSON_True ) {
		cJSON* payload = cJSON_CreateObject();
		cJSON_AddItemReferenceToObject(payload,"occupancy",occupancy);
		sprintf(topic,"occupancy/%s", ACAP_DEVICE_Prop("serial") );
		MQTT_Publish_JSON(topic,payload,0,0);
		cJSON_Delete(payload);
	}	
	
	//Publish trackers
	cJSON* trackers = ProcessTrackers( detections );
	if( cJSON_GetObjectItem(publish,"tracker") && cJSON_GetObjectItem(publish,"tracker")->type == cJSON_True ) {
		item = trackers->child;
		while( item ) {
			UpdateTrackerPublish(item);
			sprintf(topic,"tracker/%s", ACAP_DEVICE_Prop("serial") );
			MQTT_Publish_JSON(topic,item,0,0);
			item = item->next;
		}
		//Check and publish idle trackers
		if( !lastPublishedTracker )
			lastPublishedTracker = cJSON_CreateObject();
		double now = ACAP_DEVICE_Timestamp();
		cJSON* activeTracker = lastPublishedTracker->child;
		while( activeTracker ) {
			double idle = (now - activeTracker->valuedouble) / 1000;
			idle = round(idle*10)/10;
			if( idle > 2 ) {
				const char* id = activeTracker->string;
				cJSON* detection = detections->child;
				cJSON* found = 0;
				while( detection && !found ) {
					if( strcmp( id, cJSON_GetObjectItem(detection,"id")->valuestring ) == 0 )
						found = detection;
					detection = detection->next;
				}
				if( found ) {
					UpdateTrackerPublish(found);
					double age = cJSON_GetObjectItem(found,"age")->valuedouble;
					cJSON_ReplaceItemInObject(found,"age",cJSON_CreateNumber(age+idle));
					sprintf(topic,"tracker/%s", ACAP_DEVICE_Prop("serial") );
					MQTT_Publish_JSON(topic,found,0,0);
				}
			}
			activeTracker = activeTracker->next;
		}
	}

	//Publish paths
	cJSON* paths = ProcessPaths( trackers );

	if( cJSON_GetObjectItem(publish,"path") && cJSON_GetObjectItem(publish,"path")->type == cJSON_True ) {
		item = paths->child;
		while( item ) {
			sprintf(topic,"path/%s", ACAP_DEVICE_Prop("serial") );
			MQTT_Publish_JSON(topic,item,0,0);
			item = item->next;
		}
	}

	cJSON_Delete(trackers);
	cJSON_Delete(detections);
	cJSON_Delete(paths);
}


void
MAIN_Event_Callback(cJSON *event, void* userdata) {
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
//LOG("%s\n",eventTopic->valuestring);
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
	
	cJSON* settings = ACAP_Get_Config("settings");
	if( !settings )
		return G_SOURCE_CONTINUE;
	cJSON* publish = cJSON_GetObjectItem(settings,"publish");
	if(!publish)
		return G_SOURCE_CONTINUE;
	
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

void
Settings_Updated_Callback( const char* service, cJSON* data) {
	char* json = cJSON_PrintUnformatted(data);
	if( json ) {
		LOG_TRACE("%s: %s=%s\n",__func__, service, json);
		free(json);
	} else {
		LOG_WARN("%s: JSON Parse error\n",__func__);
	}
	if( strcmp( service,"scene" ) == 0 ) {
		int confidence = cJSON_GetObjectItem( data,"confidence")?cJSON_GetObjectItem( data,"confidence")->valueint:30;
		int rotation = cJSON_GetObjectItem( data,"rotation")?cJSON_GetObjectItem( data,"rotation")->valueint:0;
		int cog = cJSON_GetObjectItem( data,"cog")?cJSON_GetObjectItem( data,"cog")->valueint:1;
		int maxAge = cJSON_GetObjectItem( data,"maxAge")?cJSON_GetObjectItem( data,"maxAge")->valueint:86400;
		ObjectDetection_Set( confidence, rotation, cog, maxAge );
	}
}

static gboolean
CacheMonitor(gpointer user_data) {
	int paths = pathsCache? cJSON_GetArraySize(pathsCache):0;
	int previosPos = PreviousPosition?cJSON_GetArraySize(PreviousPosition):0;
	LOG("Paths: %d Positions: %d\n", paths, previosPos)
	return G_SOURCE_CONTINUE;
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

	attributes = ACAP_FILE_Read( "settings/attributes.json" );
	if( !attributes ) {
		LOG_WARN("Missing attributed\n");
	} else {
		ACAP_Set_Config("attributes", attributes);
	}

	ObjectDetection_Init( MAIN_Detections_Callback );
	g_timeout_add_seconds(15 * 60, Main_Status_Update, NULL);	
	g_timeout_add_seconds(5 * 60, CacheMonitor, NULL);	
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
