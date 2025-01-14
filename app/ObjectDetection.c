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
#include <assert.h>
#include <glib.h>
#include "TIME.h"
#include "ObjectDetection.h"
#include "cJSON.h"
#include "video_object_detection_subscriber.h"
#include "video_object_detection.pb-c.h"

//#define LOG_TRACE(fmt, args...)    { printf(fmt, ## args); }//
#define LOG_TRACE(fmt, args...)    {}

static video_object_detection_subscriber_t *ObjectDetection_Handler = NULL;
ObjectDetection_Callback ObjectDetection_UserCallback = NULL;
video_object_detection_subscriber_class_t **classifications;
cJSON* ObjectDetections = 0;
int minimumConfidence = 30;
int centerOfGravity = 0;
int dataRotation = 0;

static void
ObjectDetection_Scene_Callback(const uint8_t *detection_data, size_t data_size, void *user_data) {
	cJSON*	detection;
    unsigned int i,objectID;
	int ignore;
	char idString[16];

	if(!ObjectDetections)
		ObjectDetections = cJSON_CreateObject();

	LOG_TRACE("%s:\n",__func__);

	double x1,y1,x2,y2;
	if( !data_size || !detection_data ) {
		return;
	}

	if( ObjectDetection_UserCallback == 0 ) {
		return;
	}

	double timestamp = TIME_Timestamp();
	
	VOD__Scene *recv_scene;
	recv_scene = vod__scene__unpack(NULL, data_size, detection_data);
	if (recv_scene == NULL) {
		return;
	}

    for (i = 0; i < recv_scene->n_detections; i++) {

        VOD__Detection *recv_det = recv_scene->detections[i];

		if( recv_det->detection_status != 2 ) //Ignore predictions
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
		int cx = x + (w/2);
		int cy = y + (h/2);
		
		if( centerOfGravity == 1 )
			cy = y + h;

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

		int confidence = (unsigned)(recv_det->score);
		int classID = (int)recv_det->det_class;
		
		sprintf(idString,"%d",objectID);
		detection = cJSON_GetObjectItem(ObjectDetections,idString);

		
		if( detection ) {
			cJSON_ReplaceItemInObject(detection,"timestamp",cJSON_CreateNumber(timestamp));
			double age = (timestamp - cJSON_GetObjectItem(detection,"birth")->valuedouble)/1000.0;
			age = round(age * 10)/10;
			cJSON_GetObjectItem(detection,"active")->type = cJSON_True;
			if( classID != cJSON_GetObjectItem(detection,"type")->valueint ) {
				cJSON_ReplaceItemInObject(detection,"class",cJSON_CreateString(video_object_detection_subscriber_det_class_name(classifications[classID])));
				cJSON_ReplaceItemInObject(detection,"type",cJSON_CreateNumber(classID));
			}
			cJSON_ReplaceItemInObject(detection,"age",cJSON_CreateNumber(age));
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
			
			if( cJSON_GetObjectItem(detection,"confidence")->type == cJSON_False ) {
				if( confidence > minimumConfidence )
					cJSON_ReplaceItemInObject(detection,"confidence",cJSON_CreateNumber(confidence));
			} else {
				if( confidence > cJSON_GetObjectItem(detection,"confidence")->valueint )
					cJSON_ReplaceItemInObject(detection,"confidence",cJSON_CreateNumber(confidence));
			}
		} else {
			detection = cJSON_CreateObject();
			cJSON_AddStringToObject(detection,"id",idString);
			cJSON_AddNumberToObject(detection,"type",classID);
			cJSON_AddStringToObject(detection,"class",video_object_detection_subscriber_det_class_name(classifications[classID]));
			cJSON_AddTrueToObject(detection,"active");
			cJSON_AddNumberToObject(detection,"birth",timestamp);
			cJSON_AddNumberToObject(detection,"age",0);				
			cJSON_AddNumberToObject(detection,"bx",cx);
			cJSON_AddNumberToObject(detection,"by",cy);
			if( confidence > minimumConfidence )
				cJSON_AddNumberToObject(detection,"confidence",confidence);
			else
			cJSON_AddFalseToObject(detection,"confidence");
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
			cJSON_AddFalseToObject(detection,"hat");
			cJSON_AddFalseToObject(detection,"face");
			cJSON_AddItemToObject(detection,"attributes",cJSON_CreateArray());
			cJSON_AddItemToObject( ObjectDetections, idString, detection);
		}
		if( detection ) {
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
		}
    }

	for (i = 0; i < recv_scene->n_events; i++) {
		VOD__Event *recv_event = recv_scene->events[i];
		sprintf(idString,"%d",(int)recv_event->object_id);
		detection = cJSON_GetObjectItem(ObjectDetections,idString);
		if( !detection )
			continue;
		if( recv_event->action == 0 ) {
			cJSON_GetObjectItem(detection,"active")->type = cJSON_False;
		} else {
			cJSON* operations = cJSON_GetObjectItem(detection,"operations");
			if(!operations ) {
				operations = cJSON_CreateArray();
				cJSON_AddItemToObject(detection,"operations",operations);
			}
			cJSON* operation = cJSON_CreateObject();
			cJSON* objects = cJSON_CreateArray();
			cJSON_AddNumberToObject(operation,"type",recv_event->action);
			cJSON_AddItemToObject(operation,"objects",objects);
			for( i = 0; i < recv_event->n_object_ids; i++ )
				cJSON_AddItemToArray(objects,cJSON_CreateNumber(recv_event->object_ids[i]));
		}
	}
	
    vod__scene__free_unpacked(recv_scene, NULL);

	cJSON* list = cJSON_CreateArray();
	detection = ObjectDetections->child;
	while(detection) {
		if( cJSON_GetObjectItem(detection,"confidence")->type == cJSON_Number )
			cJSON_AddItemReferenceToArray(list,detection);
		detection = detection->next;
	}

	if( ObjectDetection_UserCallback && cJSON_GetArraySize(list) > 0 )
		ObjectDetection_UserCallback( list );
	cJSON_Delete(list);

	detection = ObjectDetections->child;
	while(detection) {
		cJSON* nextObject = detection->next;
		if( cJSON_GetObjectItem(detection,"active")->type == cJSON_False )
			cJSON_DeleteItemFromObject(ObjectDetections,cJSON_GetObjectItem(detection,"id")->valuestring );
		detection = nextObject;
	}
}

void *ObjectDetection_some_user_data;

void
ObjectDetection_Set( int confidence, int rotation, int cog ) {
	minimumConfidence = confidence;
	centerOfGravity = cog;
	dataRotation = rotation;
}

int
ObjectDetection_Init( ObjectDetection_Callback callback ) {
	int status = 0;
	
	int num_classes = video_object_detection_subscriber_det_classes_get(&classifications);
	LOG_TRACE("%s: Classes = %d\n",__func__,num_classes);
	
	if( callback == 0 ) {
		ObjectDetection_UserCallback = 0;
		return 1;
	}

	if( ObjectDetection_UserCallback || ObjectDetection_Handler ) {
		return 1;
	}

	ObjectDetection_UserCallback = callback;
	
	
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
