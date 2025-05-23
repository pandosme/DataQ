/**
 * ObjectDetection.c
 * Fred Juhlin 2025
 */

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
cJSON* detectionCounters = 0;

int config_tracker_confidence = 1;
int config_min_confidence = 50;
int config_cog = 0;
int config_rotation = 0;
int config_max_idle = 0;
int config_min_height = 10;
int config_max_height = 10;
int config_min_width = 800;
int config_max_width = 800;
int config_x1 = 0;
int config_x2 = 1000;
int config_y1 = 0;
int config_y2 = 1000;
int config_hanging_objects = 0;
cJSON* config_blacklist = 0;
int lastDetectionWasEmpty = 0;
cJSON* attributes = 0;

void
ObjectDetection_Config( cJSON* data ) {
	LOG_TRACE("%s: Entry\n",__func__);
	if(!activeDetections)
		activeDetections = cJSON_CreateObject();

	if( !data ) {
		LOG_WARN("%s: Invlaid input\n",__func__);
		return;
	}
		
	config_min_confidence = cJSON_GetObjectItem(data,"confidence")?cJSON_GetObjectItem(data,"confidence")->valueint:40;
	config_cog = cJSON_GetObjectItem(data,"cog")?cJSON_GetObjectItem(data,"cog")->valueint:0;
	config_rotation = cJSON_GetObjectItem(data,"rotation")?cJSON_GetObjectItem(data,"rotation")->valueint:0;
	config_tracker_confidence = cJSON_GetObjectItem(data,"tracker_confidence")?cJSON_GetObjectItem(data,"tracker_confidence")->valueint:1;
	config_max_idle = cJSON_GetObjectItem(data,"maxIdle")?cJSON_GetObjectItem(data,"maxIdle")->valueint:0;
	config_blacklist = cJSON_GetObjectItem(data,"ignoreClass")?cJSON_GetObjectItem(data,"ignoreClass"):cJSON_CreateArray();
	config_min_height = cJSON_GetObjectItem(data,"minHeight")?cJSON_GetObjectItem(data,"minHeight")->valueint:10;
	config_max_height = cJSON_GetObjectItem(data,"maxHeight")?cJSON_GetObjectItem(data,"maxHeight")->valueint:800;
	config_min_width = cJSON_GetObjectItem(data,"minWidth")?cJSON_GetObjectItem(data,"minWidth")->valueint:10;
	config_max_width = cJSON_GetObjectItem(data,"maxWidth")?cJSON_GetObjectItem(data,"maxWidth")->valueint:800;
	config_hanging_objects = 0; //cJSON_GetObjectItem(data,"hanging_objects")?cJSON_GetObjectItem(data,"hanging_objects")->valueint:5;
	cJSON *aoi = cJSON_GetObjectItem(data,"aoi");
	if( aoi ) {
		config_x1 = cJSON_GetObjectItem(aoi,"x1")?cJSON_GetObjectItem(aoi,"x1")->valueint:0;
		config_x2 = cJSON_GetObjectItem(aoi,"x2")?cJSON_GetObjectItem(aoi,"x2")->valueint:1000;
		config_y1 = cJSON_GetObjectItem(aoi,"y1")?cJSON_GetObjectItem(aoi,"y1")->valueint:0;
		config_y2 = cJSON_GetObjectItem(aoi,"y2")?cJSON_GetObjectItem(aoi,"y2")->valueint:1000;
	}

	//Reset all ignore objects

	cJSON* item = activeDetections->child;
	while(item) {
		cJSON_GetObjectItem(item,"ignore")->type = cJSON_False;
		item = item->next;
	}
	LOG_TRACE("%s: Exit\n",__func__);	
}

void
ObjectDetection_Reset(){
	if( activeDetections ) {
		cJSON_Delete(activeDetections);
		activeDetections = cJSON_CreateObject();
	}
}

int
ObjectDetection_Blacklisted(char* label) {
	if(!config_blacklist || !label )
		return 0;
	cJSON* item = config_blacklist->child;
	while( item ){
		if( strcmp( label, item->valuestring ) == 0 )
			return 1;
		item = item->next;
	}
	return 0;
}

int
ObjectDetection_CacheSize() {
	return cJSON_GetArraySize(activeDetections) + cJSON_GetArraySize(detectionCounters);
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

	LOG_TRACE("%s: Entry\n",__func__);

	if(!activeDetections)
		activeDetections = cJSON_CreateObject();


	if(!detectionCounters)
		detectionCounters = cJSON_CreateObject();
/*
	//Detection counters are used to detect if an object id is not included
	// in the detection list but not recived a kill message (hanging object).
	detection = detectionCounters->child;
	while( config_hanging_objects && detection  ) {
		detection->valueint--;
		detection = detection->next;
	}
*/
	if( !data_size || !detection_data )
		return;

	double timestamp = TIME_Timestamp();
	
	VOD__Scene *recv_scene;
	recv_scene = vod__scene__unpack(NULL, data_size, detection_data);
	if (recv_scene == NULL)
		return;

    for (i = 0; i < recv_scene->n_detections; i++) {
		LOG_TRACE("Next");

        VOD__Detection *recv_det = recv_scene->detections[i];

		if( config_tracker_confidence && recv_det->detection_status != VOD__DETECTION__DETECTION_STATUS__TRACKED_CONFIDENT )
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

		if( config_rotation == 180 ) {
			x = 1000 - x - w;
			y = 1000 - y - h;
		}

		if( config_rotation == 90 ) {
			int t = h;
			h = w;
			w = t;
			t = x;
			x = 1000 - y - w;
			y = t;
		}

		if( config_rotation == 270 ) {
			int t = h;
			h = w;
			w = t;
			t = y;
			y = 1000 - x - h;
			x = t;
		}

		int cx = x + (w/2);
		int cy = y + (h/2);
		
		if( config_cog == 1 )
			cy = y + h;

		int confidence = (unsigned)(recv_det->score);
		int classID = (int)recv_det->det_class;

		sprintf(idString,"%d",objectID);
		detection = cJSON_GetObjectItem(activeDetections,idString);
		if( !detection )  {
			int accept = 1;
			if( accept && confidence < config_min_confidence ) accept = 0;
			if( accept && h < config_min_height ) accept = 0;
			if( accept && h > config_max_height ) accept = 0;
			if( accept && w < config_min_width ) accept = 0;
			if( accept && w > config_max_width ) accept = 0;
			if( accept && cx < config_x1 ) accept = 0;
			if( accept && cx > config_x2 ) accept = 0;
			if( accept && cy < config_y1 ) accept = 0;
			if( accept && cy > config_y2 ) accept = 0;
			if( accept ) {
				detection = cJSON_CreateObject();
				cJSON_AddStringToObject(detection,"id",idString);
				cJSON_AddTrueToObject(detection,"active");
				cJSON_AddFalseToObject(detection,"ignore");
				cJSON_AddNumberToObject(detection,"type",classID);
				cJSON_AddStringToObject(detection,"class",video_object_detection_subscriber_det_class_name(classifications[classID]));
				cJSON_AddNumberToObject(detection,"birth",timestamp);
				cJSON_AddNumberToObject(detection,"age",0);				
				cJSON_AddNumberToObject(detection,"bx",cx);
				cJSON_AddNumberToObject(detection,"by",cy);
				cJSON_AddNumberToObject(detection,"confidence",confidence);
				cJSON_AddNumberToObject(detection,"idle", 0);
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
//				cJSON_AddNumberToObject(detectionCounters,idString,config_hanging_objects);
			}
		} else {
			LOG_TRACE("Found\n");
//			if( cJSON_GetObjectItem(detectionCounters,idString) )
//				cJSON_GetObjectItem(detectionCounters,idString)->valueint = config_hanging_objects;

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
			if( detection )
				cJSON_GetObjectItem(detection,"active")->type = cJSON_False;
		}
	}

    vod__scene__free_unpacked(recv_scene, NULL);

	//Check hanging objects
/*	
	if( config_hanging_objects > 0 ) {
		detection = detectionCounters->child;
		while( detection ) {
			if( detection->valueint <= 0 ) {
				cJSON* hangingObject = cJSON_GetObjectItem(activeDetections,detection->string);
				if( hangingObject )
					cJSON_GetObjectItem(hangingObject,"active")->type = cJSON_False;
			}
			detection = detection->next;
		}
	}
*/

	//Make List
	cJSON* list = cJSON_CreateArray();
	detection = activeDetections->child;
	while(detection) {
		if( ObjectDetection_Blacklisted( cJSON_GetObjectItem(detection,"class")->valuestring ) == 0 )
			cJSON_AddItemReferenceToArray(list,detection);
		detection = detection->next;
	}

	if( cJSON_GetArraySize( list ) > 0 ) {
		lastDetectionWasEmpty = 0;
		ObjectDetection_UserCallback( list );
	} else {
		if( lastDetectionWasEmpty == 0 )
			ObjectDetection_UserCallback( list );
		lastDetectionWasEmpty = 1;
	}
	cJSON_Delete(list);

	detection = activeDetections->child;
	while(detection) {
		cJSON* nextObject = detection->next;
		const char * id = cJSON_GetObjectItem(detection,"id")->valuestring;
		if( cJSON_GetObjectItem(detection,"active")->type == cJSON_False ) {
			cJSON_DeleteItemFromObject(activeDetections, id );
//			cJSON_DeleteItemFromObject(detectionCounters, id);
		}
		detection = nextObject;
	}
	LOG_TRACE("Done\n");
}

void *ObjectDetection_some_user_data = 0;

int
ObjectDetection_Init( ObjectDetection_Callback callback ) {
	int status = 0;

	LOG_TRACE("%s: Entry\n",__func__);
	
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
		LOG_WARN("%s:Invalid input when configuring object detection\n",__func__);
		return 0;
	}

	if( ObjectDetection_UserCallback || ObjectDetection_Handler ) {
		LOG_TRACE("%s: Already configured\n",__func__);		
		return 1;
	}

	ObjectDetection_UserCallback = callback;
	activeDetections = cJSON_CreateArray();
	
	if( video_object_detection_subscriber_create(&ObjectDetection_Handler, &ObjectDetection_some_user_data, 0) != 0 ) {
		LOG_WARN("%s: Cannot open channel to ObjectDetection\n",__func__);
		ObjectDetection_Handler = 0;
		return 0;
	}

    if (video_object_detection_subscriber_set_get_detection_callback(ObjectDetection_Handler, ObjectDetection_Scene_Callback) != 0) {
		LOG_WARN("%s: Could not set object detection callback\n", __func__);
		return 0;
	}

    video_object_detection_subscriber_set_receive_empty_hits(ObjectDetection_Handler, 1);

	status = video_object_detection_subscriber_subscribe(ObjectDetection_Handler);
    if ( status != 0) {
		LOG_WARN("%s: Object detection subscription failed. Error Code: %d\n",__func__,  status);
		return 0;
    }
	LOG_TRACE("%s: Entry\n",__func__);
	return 1;
} 
