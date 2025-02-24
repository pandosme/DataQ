/*------------------------------------------------------------------
 *  Fred Juhlin (2023)
 *  
 *------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <syslog.h>
#include <assert.h>
#include <glib.h>
#include "TIME.h"
#include "ObjectDetection.h"
#include "cJSON.h"
#include "ACAP.h"
#include "video_object_detection_subscriber.h"
#include "video_object_detection.pb-c.h"

#define LOG(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args);}
#define LOG_WARN(fmt, args...)    { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args);}
//#define LOG_TRACE(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_TRACE(fmt, args...)    {}

static video_object_detection_subscriber_t *ObjectDetection_Handler = NULL;
ObjectDetection_Callback ObjectDetection_UserCallback = NULL;
video_object_detection_subscriber_class_t **classifications;


cJSON* activeDetections = 0;
cJSON* detectionsFilter = 0;

int minimumConfidence = 30;
int low_tracker_confidence = 0;
int centerOfGravity = 0;
int dataRotation = 0;
int maxAge = 86400;
int lastDetectionWasEmpty = 0;
cJSON* attributes = 0;

int
ObjectDetection_CacheSize() {
	return cJSON_GetArraySize(activeDetections);
}

void
ObjectDetection_Ignore(const char* id) {
	cJSON* detection = cJSON_GetObjectItem(activeDetections,id);
	if( detection )
		cJSON_GetObjectItem(detection,"ignore")->type = cJSON_True;
}

void
ObjectDetection_Reset(const char* id) {
	cJSON* detection = cJSON_GetObjectItem(activeDetections,id);
	if( detection )
		cJSON_GetObjectItem(detection,"active")->type = cJSON_False;
}

cJSON*
ProcessAttributes( cJSON* item ) {
	if( !attributes || !item)
		return item;
	
	
	cJSON* humanColors = cJSON_GetObjectItem(attributes,"humanColors");
	if( !humanColors ) {
		LOG_TRACE("%s:Invalid humanColors",__func__);
		return item;
	}

	cJSON* vehicleColors = cJSON_GetObjectItem(attributes,"vehicleColors");
	cJSON* vehicles = cJSON_GetObjectItem(attributes,"vehicles");
	cJSON* bags = cJSON_GetObjectItem(attributes,"bags");
	cJSON* hats = cJSON_GetObjectItem(attributes,"hats");
	cJSON* names = cJSON_GetObjectItem(attributes,"names");

	//Process detections
	cJSON* lookup = 0;
	cJSON* attribute = cJSON_GetObjectItem(item,"attributes")?cJSON_GetObjectItem(item,"attributes")->child:0;
	while( attribute ) {
		int type = cJSON_GetObjectItem(attribute,"type")?cJSON_GetObjectItem(attribute,"type")->valueint: 0;
		int value = cJSON_GetObjectItem(attribute,"value")?cJSON_GetObjectItem(attribute,"value")->valueint: 0;
		int score = cJSON_GetObjectItem(attribute,"value")?cJSON_GetObjectItem(attribute,"value")->valueint: 0;
		switch( type ) {
			case 0:  //Undefined type
				lookup = cJSON_GetArrayItem(vehicles,value);
				if( lookup )
					cJSON_ReplaceItemInObject( item,"class", cJSON_CreateString(lookup->valuestring));
				break;
			case 1:  //Undefined type
				lookup = cJSON_GetArrayItem(vehicles,value);
				if( lookup )
					cJSON_ReplaceItemInObject( item,"class", cJSON_CreateString(lookup->valuestring));
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
					if( value == 0 )
						cJSON_GetObjectItem(item,"face")->type = cJSON_False;
					else
						cJSON_GetObjectItem(item,"face")->type = cJSON_True;
				} else {
					if( value == 0 )
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
	return item;
}


static void
ObjectDetection_Scene_Callback(const uint8_t *detection_data, size_t data_size, void *user_data) {
	cJSON*	detection;
    unsigned int i,objectID;
	int ignore;
	char idString[16];
	double x1,y1,x2,y2;

	if(!activeDetections)
		activeDetections = cJSON_CreateObject();

//	LOG_TRACE("Scene:\n");

	if( !data_size || !detection_data )
		return;

	double timestamp = TIME_Timestamp();
	
	VOD__Scene *recv_scene;
	recv_scene = vod__scene__unpack(NULL, data_size, detection_data);
	if (recv_scene == NULL)
		return;

	detection = activeDetections->child;
	while( detection ) {
		cJSON_GetObjectItem(detection,"visible")->valueint--;
		detection = detection->next;
	}

//	LOG_TRACE("Detection\n");

    for (i = 0; i < recv_scene->n_detections; i++) {
		LOG_TRACE("Next");

        VOD__Detection *recv_det = recv_scene->detections[i];

		if( low_tracker_confidence && recv_det->detection_status != VOD__DETECTION__DETECTION_STATUS__TRACKED_CONFIDENT )
			continue;

		objectID = (int)recv_det->id;
		x1 = (recv_det->left * 4096) + 4096;
		y1 = 4096 - (recv_det->top * 4096);
		x2 = (recv_det->right * 4096) + 4096;
		y2 = 4096 - (recv_det->bottom * 4096);
		int x = (x1 * 1000)/8192;
		int y = (y1 * 1000)/8192;
		int w = ((x2-x1) * 1000)/8192;
		int h = ((y2-y1) * 1000)/8192;

		if( dataRotation == 180 ) {
			x = 1000 - x - w;
			y = 1000 - y - h;
		}

		if( dataRotation == 90 ) {
			int t = h;
			h = w;
			w = t;
			t = x;
			x = 1000 - y - w;
			y = t;
		}

		if( dataRotation == 270 ) {
			int t = h;
			h = w;
			w = t;
			t = y;
			y = 1000 - x - h;
			x = t;
		}

		int cx = x + (w/2);
		int cy = y + (h/2);
		
		if( centerOfGravity == 1 )
			cy = y + h;

		int confidence = (unsigned)(recv_det->score);
		int classID = (int)recv_det->det_class;

		sprintf(idString,"%d",objectID);
		detection = cJSON_GetObjectItem(activeDetections,idString);
		if( detection ) {
			LOG_TRACE("Found\n");
			cJSON_GetObjectItem(detection,"visible")->valueint = 5;
			cJSON_ReplaceItemInObject(detection,"timestamp",cJSON_CreateNumber(timestamp));

			if( classID != cJSON_GetObjectItem(detection,"type")->valueint ) {
				cJSON_ReplaceItemInObject(detection,"class",cJSON_CreateString(video_object_detection_subscriber_det_class_name(classifications[classID])));
				cJSON_ReplaceItemInObject(detection,"type",cJSON_CreateNumber(classID));
			}
			cJSON_ReplaceItemInObject(detection,"x",cJSON_CreateNumber(x));
			cJSON_ReplaceItemInObject(detection,"y",cJSON_CreateNumber(y));
			cJSON_ReplaceItemInObject(detection,"w",cJSON_CreateNumber(w));
			cJSON_ReplaceItemInObject(detection,"h",cJSON_CreateNumber(h));
			cJSON_ReplaceItemInObject(detection,"cx",cJSON_CreateNumber(cx));
			cJSON_ReplaceItemInObject(detection,"cy",cJSON_CreateNumber(cy));
			double dx = cx - cJSON_GetObjectItem(detection,"bx")->valueint;
			double dy = cy - cJSON_GetObjectItem(detection,"by")->valueint;
			cJSON_ReplaceItemInObject(detection,"dx",cJSON_CreateNumber((int)dx));
			cJSON_ReplaceItemInObject(detection,"dy",cJSON_CreateNumber((int)dy));
			if( confidence > cJSON_GetObjectItem(detection,"confidence")->valueint )
				cJSON_ReplaceItemInObject(detection,"confidence",cJSON_CreateNumber(confidence));
			double age = round((timestamp - cJSON_GetObjectItem(detection,"birth")->valuedouble)/1000.0);
			cJSON_ReplaceItemInObject(detection,"age",cJSON_CreateNumber(age));
		} else {
			detection = cJSON_CreateObject();
			cJSON_AddStringToObject(detection,"id",idString);
			cJSON_AddNumberToObject(detection,"type",classID);
			cJSON_AddStringToObject(detection,"class",video_object_detection_subscriber_det_class_name(classifications[classID]));
			cJSON_AddNumberToObject(detection,"birth",timestamp);
			cJSON_AddNumberToObject(detection,"age",0);				
			cJSON_AddNumberToObject(detection,"bx",cx);
			cJSON_AddNumberToObject(detection,"by",cy);
			cJSON_AddNumberToObject(detection,"confidence",confidence);
			cJSON_AddTrueToObject(detection,"active");
			cJSON_AddNumberToObject(detection,"visible", 5);
			cJSON_AddFalseToObject(detection,"ignore");
			cJSON_AddNumberToObject(detection,"timestamp",timestamp);
			cJSON_AddNumberToObject(detection,"x",x);
			cJSON_AddNumberToObject(detection,"y",y);
			cJSON_AddNumberToObject(detection,"w",w);
			cJSON_AddNumberToObject(detection,"h",h);
			cJSON_AddNumberToObject(detection,"cx",cx);
			cJSON_AddNumberToObject(detection,"cy",cy);
			cJSON_AddNumberToObject(detection,"dx",0);
			cJSON_AddNumberToObject(detection,"dy",0);
			cJSON_AddNumberToObject(detection,"distance",0);
			cJSON_AddNumberToObject(detection,"topVelocity",0);
			cJSON_AddStringToObject(detection,"color","");
			cJSON_AddStringToObject(detection,"color2","");
			cJSON_AddStringToObject(detection,"hat","");
			cJSON_AddFalseToObject(detection,"face");
			cJSON_AddItemToObject(detection,"attributes",cJSON_CreateArray());
			cJSON_AddItemToObject( activeDetections, idString, detection);
		}

		LOG_TRACE("Attributes\n");

		int c;
		cJSON* attributes = cJSON_GetObjectItem(detection,"attributes");
		for( c = 0; c < recv_det->n_attributes; c++ ) {
			VOD__Attribute *attrib = recv_det->attributes[c];
			if( detection && attrib && attributes ) {
				int type = attrib->type;
				int score = -1;
				int value = -1;
				if( attrib->has_class_case )
					value = attrib->attr_class;
				if( attrib->has_score_case )
					score = attrib->score;
				if( score >= 0 && value >= 0 ) {
					cJSON* attribute = attributes->child;
					int found = 0;
					while( attribute ){
						if( cJSON_GetObjectItem(attribute,"type")->valueint == type ) {
							found = 1;
							if( score > cJSON_GetObjectItem(attribute,"score")->valueint ) {
								cJSON_GetObjectItem(attribute,"score")->valueint = score;
								cJSON_GetObjectItem(attribute,"score")->valuedouble = score;
								cJSON_GetObjectItem(attribute,"value")->valueint = value;
								cJSON_GetObjectItem(attribute,"value")->valuedouble = value;
							}
						}
						attribute = attribute->next;
					}
					if( !found ) {
						cJSON* attribute = cJSON_CreateObject();
						cJSON_AddNumberToObject( attribute,"type",type );
						cJSON_AddNumberToObject( attribute,"value",value );
						cJSON_AddNumberToObject( attribute,"score",score );
						cJSON_AddItemToArray(attributes,attribute);
					}
				}
			}
		}
		ProcessAttributes(detection);
    }

	LOG_TRACE("Events\n");

	for (i = 0; i < recv_scene->n_events; i++) {
		VOD__Event *recv_event = recv_scene->events[i];
		if( recv_event->action == 0 ) {
			sprintf(idString,"%d",(int)recv_event->object_id);
			detection = cJSON_GetObjectItem(activeDetections,idString);
			if( detection ) {
				cJSON_GetObjectItem(detection,"active")->type = cJSON_False;
				if( cJSON_GetObjectItem(detection,"ignore")->type == cJSON_True )
					cJSON_DeleteItemFromObject(activeDetections,idString);
			}
		} else {
			LOG_TRACE("Operations\n");
			cJSON* operations = cJSON_GetObjectItem(detection,"operations");
			if(!operations ) {
				operations = cJSON_CreateArray();
				cJSON_AddItemToObject(detection,"operations",operations);
			}
			cJSON* operation = cJSON_CreateObject();
			cJSON* objects = cJSON_CreateArray();
			cJSON_AddNumberToObject(operation,"type",recv_event->action);
			cJSON_AddItemToObject(operation,"objects",objects);
			for( int j = 0; j < recv_event->n_object_ids; j++ )
				cJSON_AddItemToArray(objects,cJSON_CreateNumber(recv_event->object_ids[j]));
		}
	}

    vod__scene__free_unpacked(recv_scene, NULL);

	LOG_TRACE("Active\n");
	cJSON* list = cJSON_CreateArray();
	detection = activeDetections->child;
	while(detection) {
		if( cJSON_GetObjectItem(detection,"age")->valueint > maxAge )
			cJSON_GetObjectItem(detection,"ignore")->type = cJSON_True;

		if( cJSON_GetObjectItem(detection,"ignore")->type == cJSON_False ) {
			if( cJSON_GetObjectItem(detection,"visible")->valueint <= 0 )
				cJSON_GetObjectItem(detection,"active")->type = cJSON_False;
			if( cJSON_GetObjectItem(detection,"confidence")->valueint >= minimumConfidence )
				cJSON_AddItemReferenceToArray(list,detection);
		} else {
			if( cJSON_GetObjectItem(detection,"active")->type == cJSON_True ) {
				cJSON_GetObjectItem(detection,"active")->type = cJSON_False;
				cJSON_AddItemReferenceToArray(list,detection);
			}
		}
		detection = detection->next;
	}


	LOG_TRACE("Publish\n");

	if( cJSON_GetArraySize( list ) > 0 ) {
		lastDetectionWasEmpty = 0;
		ObjectDetection_UserCallback( list );
	} else {
		if( lastDetectionWasEmpty == 0 )
			ObjectDetection_UserCallback( list );
		lastDetectionWasEmpty = 1;
	}

	LOG_TRACE("Cleanup\n");
	cJSON_Delete(list);

	detection = activeDetections->child;
	while(detection) {
		cJSON* nextObject = detection->next;
		const char * id = cJSON_GetObjectItem(detection,"id")->valuestring;
		if( cJSON_GetObjectItem(detection,"active")->type == cJSON_False && cJSON_GetObjectItem(detection,"ignore")->type == cJSON_False )
			cJSON_DeleteItemFromObject(activeDetections, id );
		detection = nextObject;
	}
	LOG_TRACE("Done\n");
}

void *ObjectDetection_some_user_data = 0;

void
ObjectDetection_Set( int confidence, int rotation, int cog, int maxAgeInSeconds, int tracker_confidence ) {
	minimumConfidence = confidence;
	centerOfGravity = cog;
	dataRotation = rotation;
	maxAge = maxAgeInSeconds;
	low_tracker_confidence = tracker_confidence;
}

int
ObjectDetection_Init( ObjectDetection_Callback callback ) {
	int status = 0;


	attributes = ACAP_FILE_Read( "settings/attributes.json" );
	if( !attributes ) {
		LOG_WARN("Missing attributes\n");
	} else {
		ACAP_Set_Config("attributes", attributes);
	}
	
	int num_classes = video_object_detection_subscriber_det_classes_get(&classifications);
	LOG_TRACE("%s: Model has %d classes\n",__func__,num_classes);
	
	if( callback == 0 ) {
		ObjectDetection_UserCallback = 0;
		return 1;
	}

	if( ObjectDetection_UserCallback || ObjectDetection_Handler ) {
		return 1;
	}

	ObjectDetection_UserCallback = callback;
	activeDetections = cJSON_CreateArray();
	
	if( video_object_detection_subscriber_create(&ObjectDetection_Handler, &ObjectDetection_some_user_data, 0) != 0 ) {
		LOG_TRACE("%s: Cannot open channel to ObjectDetection\n",__func__);
		ObjectDetection_Handler = 0;
		exit(EXIT_FAILURE);
	}

    if (video_object_detection_subscriber_set_get_detection_callback(ObjectDetection_Handler, ObjectDetection_Scene_Callback) != 0) {
		LOG_TRACE("Could not set callback\n");
		exit(EXIT_FAILURE);
	}

    video_object_detection_subscriber_set_receive_empty_hits(ObjectDetection_Handler, 1);

	status = video_object_detection_subscriber_subscribe(ObjectDetection_Handler);
    if ( status != 0) {
		LOG_TRACE("ObjectDetection_Init: Failed to set subscribe. %d\n", status);
		exit(EXIT_FAILURE);
    }

	return 1;
} 
