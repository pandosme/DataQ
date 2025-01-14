/*------------------------------------------------------------------
 *  Fred Juhlin (2023)
 *  
 *------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <glib.h>

#include "ACAP.h"
#include "ObjectDetection.h"
#include "cJSON.h"
#include "video_object_detection_subscriber.h"
#include "video_object_detection.pb-c.h"

//#define LOG_TRACE(fmt, args...)    { printf(fmt, ## args); }//
#define LOG_TRACE(fmt, args...)    {}


/*
typedef struct {
    float left;
    float top;
    float right;
    float bottom;
    int32_t id;
    int32_t det_class;
    uint32_t score;
} video_object_detection_subscriber_hit_t;

typedef struct {
    uint64_t  timestamp;
    uint32_t  nbr_of_detections;
    video_object_detection_subscriber_hit_t
    detection_list[];
} video_object_detection_subscriber_detections_t;

#define DEFAULT_ID          -1
#define DEFAULT_DET_CLASS   -1
#define DEFAULT_SCORE       0
*/

static video_object_detection_subscriber_t *ObjectDetection_Handler = NULL;
ObjectDetection_Callback ObjectDetection_UserCallback = NULL;
int ObjectDetection_CONFIDENCE = 0;
int ObjectDetection_centerOfGravity = 0;
int ObjectDetection_LAST_EMPTY = 0;
int	ObjectDetection_Rotation = 0;
cJSON* ObjectDetections = 0;
int minAttributeScore = 50;
int ObjectDectionIdleTimoutSeconds = 0;
cJSON* ObjectDetectionSettings = 0;

static void
ObjectDetection_Scene_Callback(const uint8_t *detection_data, size_t data_size, void *user_data) {
	cJSON*	detection;
    unsigned int i,objectID;
	int ignore;
	char idString[16];

	if( !data_size || !detection_data || !ObjectDetectionSettings || !ObjectDetection_UserCallback) {
		return;
	}

	cJSON* aoi = aoi
	int minWidth = cJSON_GetObjectItem(ObjectDetectionSettings,"width")->valueint;
	int minheigth = cJSON_GetObjectItem(ObjectDetectionSettings,"height")->valueint;
	int maxWidth = cJSON_GetObjectItem(ObjectDetectionSettings,"width")->valueint;
	int maxHeight = cJSON_GetObjectItem(ObjectDetectionSettings,"height")->valueint;
	int minConfidence = cJSON_GetObjectItem(ObjectDetectionSettings,"confidence")->valueint;
	int rotation = cJSON_GetObjectItem(ObjectDetectionSettings,"rotation")->valueint;
	int centerOfGravity = cJSON_GetObjectItem(ObjectDetectionSettings,"centerOfGravity")->valueint;
	cJSON* birthArea = cJSON_GetObjectItem(ObjectDetectionSettings,"birthArea");
	int X1 = cJSON_GetObjectItem(birthArea,"x1")->valueint;
	int X2 = cJSON_GetObjectItem(birthArea,"x2")->valueint;
	int Y1 = cJSON_GetObjectItem(birthArea,"y1")->valueint;
	int Y2 = cJSON_GetObjectItem(birthArea,"y2")->valueint;
	cJSON* ignoreArea = cJSON_GetObjectItem(ObjectDetectionSettings,"ignoreArea");
	int iX1 = cJSON_GetObjectItem(ignoreArea,"x1")->valueint;
	int iX2 = cJSON_GetObjectItem(ignoreArea,"x2")->valueint;
	int iY1 = cJSON_GetObjectItem(ignoreArea,"y1")->valueint;
	int iY2 = cJSON_GetObjectItem(ignoreArea,"y2")->valueint;

	if(!ObjectDetections)
		ObjectDetections = cJSON_CreateObject();

	double x1,y1,x2,y2;
	double timestamp = TIME_Timestamp();
	
	VOD__Scene *recv_scene;
	recv_scene = vod__scene__unpack(NULL, data_size, detection_data);
	if (recv_scene == NULL) {
		return;
	}
    for (i = 0; i < recv_scene->n_detections; i++) {

        VOD__Detection *recv_det = recv_scene->detections[i];

		objectID = (int)recv_det->id;
		x1 = (recv_det->left * 4096) + 4096;
		y1 = 4096 - (recv_det->top * 4096);
		x2 = (recv_det->right * 4096) + 4096;
		y2 = 4096 - (recv_det->bottom * 4096);
		double x = x1;
		double y = y1;
		double w = x2-x1;
		double h = y2-y1;

			
		if( rotation == 180 ) {
			x = 8192 - x - w;
			y = 8192 - y - h;
		}
		
		x = (x * 1000)/8192;
		y = (y * 1000)/8192;
		w = (w * 1000)/8192;
		h = (h * 1000)/8192;
			
		if( rotation == 90 ) {
			int t = h;
			h = w;
			w = t;
			t = x;
			x = 1000 - y - w;
			y = t;
		}

		if( rotation == 270 ) {
			int t = h;
			h = w;
			w = t;
			t = y;
			y = 1000 - x - h;
			x = t;
		}
			
		x1 = x;
		y1 = y;
		x2 = x + w;
		y2 = y + h;
		double cx = x + (w / 2);
		double cy = y + h;
		if( centerOfGravity == 1 )
			cy = y + ( h / 2 );

		int confidence = (unsigned)(recv_det->score);
		int classID = (int)recv_det->det_class;
		
		sprintf(idString,"%d",objectID);
		detection = cJSON_GetObjectItem(ObjectDetections,idString);
		
		if( detection ) {
			ignore = 0;
			if( ObjectDetection_Predictions == 0 && recv_det->detection_status != 2 ) ignore = 1;
			if( !ignore ) {
				cJSON_ReplaceItemInObject(detection,"timestamp",cJSON_CreateNumber(timestamp));
				double age = (timestamp - cJSON_GetObjectItem(detection,"birth")->valuedouble)/1000.0;
				cJSON_GetObjectItem(detection,"active")->type = cJSON_True;
				cJSON_ReplaceItemInObject(detection,"class",video_object_detection_subscriber_det_class_name(classifications[classID]));
				cJSON_ReplaceItemInObject(detection,"age",cJSON_CreateNumber(age));
				cJSON_ReplaceItemInObject(detection,"confidence",cJSON_CreateNumber(confidence));
				cJSON_ReplaceItemInObject(detection,"x",cJSON_CreateNumber(x));
				cJSON_ReplaceItemInObject(detection,"y",cJSON_CreateNumber(y));
				cJSON_ReplaceItemInObject(detection,"w",cJSON_CreateNumber(w));
				cJSON_ReplaceItemInObject(detection,"h",cJSON_CreateNumber(h));
				cJSON_ReplaceItemInObject(detection,"cx",cJSON_CreateNumber(cx));
				cJSON_ReplaceItemInObject(detection,"cy",cJSON_CreateNumber(cy));
			}
		} else {
			ignore = 0;
			if( !ignore && confidence < minConfidence ) ignore = 1;
			if( !ignore && recv_det->detection_status != 2 ) ignore = 1;
			if( !ignore && (w < minWidth || w > maxWidth || h < minHeight || h > maxHeight ) ) ingore = 1; 
			if( !ignore && (cx < X1 || cx > X2 || cy < Y1 || cy > Y2 ) ) ingore = 1; 
			if( !ignore && cx > iX1 && cx < iX2 && cy > iY1 && cy < iY2 ) ingore = 1; 
			if(!ignore) {
				detection = cJSON_CreateObject();
				cJSON_AddNumberToObject(detection,"id",objectID);
				cJSON_AddStringToObject(detection,"class",video_object_detection_subscriber_det_class_name(classifications[classID]));
				cJSON_AddTrueToObject(detection,"active");
				cJSON_AddNumberToObject(detection,"birth",timestamp);
				cJSON_AddNumberToObject(detection,"age",0);				
				cJSON_AddNumberToObject(detection,"bx",cx);
				cJSON_AddNumberToObject(detection,"by",cy);
				cJSON_AddNumberToObject(detection,"confidence",confidence);
				cJSON_AddNumberToObject(detection,"timestamp",timestamp);
				cJSON_AddNumberToObject(detection,"x",x);
				cJSON_AddNumberToObject(detection,"y",y);
				cJSON_AddNumberToObject(detection,"w",w);
				cJSON_AddNumberToObject(detection,"h",h);
				cJSON_AddNumberToObject(detection,"cx",cx);
				cJSON_AddNumberToObject(detection,"cy",cy);
				cJSON_AddItemToObject( detection,"attibutes",cJSON_CreateArray());
				cJSON_AddItemToObject( ObjectDetections, idString, detection);
			}
		}
		int c;
		cJSON* attributes = cJSON_CreateArray();
		for( c = 0; c < recv_det->n_attributes; c++ ) {
			VOD__Attribute *attrib = recv_det->attributes[c];
			cJSON* attribute = cJSON_CreateObject();
			if( attrib ) {
				cJSON_AddNumberToObject(attribute,"type", attrib->type );
				if( attrib->has_class_case )
					cJSON_AddNumberToObject(attribute,"class", attrib->attr_class );
				if( attrib->has_class_case )
					cJSON_AddNumberToObject(attribute,"score", attrib->score );
			}
		}
		cJSON_ReplaceItemInObject(detection,"attributes",attributes);
    }

	detection = ObjectDetections->child;
	
	while(detection) {
		int remove = 0;
		for (i = 0; i < recv_scene->n_events; i++) {
			VOD__Event *recv_event = recv_scene->events[i];
			if( recv_event->action == 0 ) {
				if( (int)recv_event->object_id == cJSON_GetObjectItem(detection,"id")->valueint ) {
					cJSON_GetObjectItem(detection,"active")->type = cJSON_False;
				}
			}
		}
		detection = detection->next;
	}
	
    vod__scene__free_unpacked(recv_scene, NULL);

	cJSON* list = cJSON_CreateArray();
	detection = ObjectDetections->child;
	while(detection) {
		cJSON_AddItemReferenceToArray(list, detection);
		detection = detection->next;
	}

	if( ObjectDetection_UserCallback && cJSON_GetArraySize(list) > 0 )
		ObjectDetection_UserCallback( list );

	cJSON_Delete(list);

	detection = ObjectDetections->child;
	while(detection) {
		cJSON* nextObject = detection->next;
		if( cJSON_GetObjectItem(detection,"active")->type == cJSON_False ) {
			sprintf(idString,"%d",cJSON_GetObjectItem(detection,"id")->valueint);
			cJSON_DeleteItemFromObject(ObjectDetections,idString);
		}
		detection = nextObject;
	}
    return;
}

void *ObjectDetection_some_user_data;

static void
ObjectDetection_HTTP_callback(const ACAP_HTTP_Response response,const ACAP_HTTP_Request request) {

    const char* method = ACAP_HTTP_Get_Method(request);
    if (!method) {
        LOG_WARN("Invalid Request Method\n");
        ACAP_HTTP_Respond_Error(response, 400, "Invalid Request Method");
        return;
    }
    
    LOG_TRACE("%s: Method=%s\n", __func__, method);

    if (strcmp(method, "GET") == 0) {
		if( ObjectDetectionSettings )
			ACAP_HTTP_Respond_JSON(response, ObjectDetectionSettings);
		else
			ACAP_HTTP_Respond_Error(resonse, 500, "Invalid Object Detection Settings");
		return
	}

    // Handle POST: Add a new timelapse profile
    if (strcmp(method, "POST") == 0) {
        const char* contentType = ACAP_HTTP_Get_Content_Type(request);
        if (!contentType || strcmp(contentType, "application/json") != 0) {
            ACAP_HTTP_Respond_Error(response, 415, "Unsupported Media Type - Use application/json");
            return;
        }

        if (!request->postData || request->postDataLength == 0) {
            ACAP_HTTP_Respond_Error(response, 400, "Missing POST data");
            return;
        }
		
		cJSON* settings = cJSON_Parse(request->postData);
        if (!profile) {
            ACAP_HTTP_Respond_Error(response, 400, "Invalid JSON data");
            return;
        }

		cJSON* setting = settings->child;
		while(setting) {
			if( cJSON_GetObjectItem(ObjectDetectionSettings,setting->string ) )
				cJSON_ReplaceItemInObject(ObjectDetectionSettings,setting->string,cJSON_Duplicate(setting,1) );
			setting = setting->next;
		}
		ACAP_FILE_Write( "localdata/ObjectDetection.json", ObjectDetectionSettings );
		cJSON_Delete(settings);
	
        ACAP_HTTP_Respond_Text(response, "Settings updated successfully");
        return;
    }
    ACAP_HTTP_Respond_Error(response, 405, "Method Not Allowed");	
}

int
ObjectDetection_Init( ObjectDetection_Callback callback ) {
	video_object_detection_subscriber_class_t **classifications;	
	int num_classes = video_object_detection_subscriber_det_classes_get(&classifications);
	LOG_TRACE("%s: Classes = %d\n",__func__,num_classes);

	int i;
	cJSON* classes = cJSON_CreateArray();
	for(i = 0; i < num_classes; i++) {
		int classId = video_object_detection_subscriber_det_class_id(classifications[i]);
		const char* className = video_object_detection_subscriber_det_class_name(classifications[i]);
		cJSON_AddItemToArray(classes, cJSON_CreateString(className));
		LOG_TRACE("%d %s\n",classID,className);
	}
	video_object_detection_subscriber_det_classes_free(classifications, num_classes);
	ACAP_STATUS_SetObject("ObjectDetection", "classes", classes);

	if( callback == 0 ) {
		ObjectDetection_UserCallback = 0;
		return 1;
	}

	if( ObjectDetection_UserCallback || ObjectDetection_Handler ) {
		return 1;
	}

	ObjectDetection_UserCallback = callback;

	ACAP_STATUS_SetObject("ObjectDetection", "status", "Initializing");
	ACAP_STATUS_SetObject("ObjectDetection", "state", false);

	if( video_object_detection_subscriber_create(&ObjectDetection_Handler, &ObjectDetection_some_user_data, 0) != 0 ) {
		ACAP_STATUS_SetObject("ObjectDetection", "status", "No channel open");
		LOG_WARN("%s: Cannot open channel to ObjectDetection\n",__func__);
		ObjectDetection_Handler = 0;
		return 0;
	}

    if (video_object_detection_subscriber_set_get_detection_callback(ObjectDetection_Handler, ObjectDetection_Scene_Callback) != 0) {
		ACAP_STATUS_SetObject("ObjectDetection", "status", "No callback");
		LOG_WARN("%s: Could not set callback\n",__func__);
		return 0;
	}

    video_object_detection_subscriber_set_receive_empty_hits(ObjectDetection_Handler, 1);
	int status = 0;

	status = video_object_detection_subscriber_subscribe(ObjectDetection_Handler);
    if ( status != 0) {
		ACAP_STATUS_SetObject("ObjectDetection", "status", "Internal subscription failed");
		LOG_TRACE("ObjectDetection_Init: Failed to set subscribe. %d\n", status);
		return 0;
    }


	ObjectDetectionSettings = ACAP_FILE_Read( "settings/subscriptions.json" );
	if(! ObjectDetectionSettings ) {
		ACAP_STATUS_SetObject("ObjectDetection", "status", "No settings found");
		LOG_WARN("%s: Invalid settings file\n",__func__);
		return 0;
	}
	
	cJSON* settings = ACAP_FILE_Read( "localdata/subscriptions.json" );
	if( settings ) {
		cJSON* setting = settings->child;
		while(setting) {
			if( cJSON_GetObjectItem(ObjectDetectionSettings,setting->string ) )
				cJSON_ReplaceItemInObject(ObjectDetectionSettings,setting->string,cJSON_Duplicate(setting,1) );
			setting = setting->next;
		}
	}

	ACAP_STATUS_SetObject("ObjectDetection", "status", "Running");
	ACAP_STATUS_SetObject("ObjectDetection", "state", true);

	return 1;
} 
