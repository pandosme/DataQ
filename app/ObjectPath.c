#define NDEBUG 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <math.h>
#include <assert.h>

#include "cJSON.h"
#include "ObjectPath.h"

//#define LOG_TRACE(fmt, args...)    { printf(fmt, ## args); }
#define LOG_TRACE(fmt, args...)    {}

cJSON *ObjectPathSettings = 0;
cJSON* ObjectPathTrackers = 0;
ObjectPath_Callback ObjectPath_UserCallback = 0;

void
ObjectPath_Input(cJSON* tracker) {
	cJSON* sample;
	cJSON* samples;

//	LOG_TRACE("%s:\n",__func__);

	if(!tracker || !ObjectPath_UserCallback )
		return;

	if(!ObjectPathTrackers)
		ObjectPathTrackers = cJSON_CreateObject();
	

	char idString[20];
	sprintf(idString,"%d", cJSON_GetObjectItem(tracker,"id")->valueint);
	cJSON* path = cJSON_GetObjectItem(ObjectPathTrackers,idString);

	if(!path) {
LOG_TRACE("Path: %s added\n",idString);		
		path = cJSON_CreateObject();
		cJSON_AddNumberToObject(path,"id",cJSON_GetObjectItem(tracker,"id")->valueint);
		cJSON_AddNumberToObject(path,"id",cJSON_GetObjectItem(tracker,"id")->valueint);
		cJSON_AddNumberToObject(path,"confidence",cJSON_GetObjectItem(tracker,"confidence")->valueint);
		cJSON_AddStringToObject(path,"class",cJSON_GetObjectItem(tracker,"class")->valuestring);
		cJSON_AddNumberToObject(path,"timestamp",cJSON_GetObjectItem(tracker,"timestamp")->valuedouble);
		cJSON_AddNumberToObject(path,"age",0);
		cJSON_AddNumberToObject(path,"previousTime",cJSON_GetObjectItem(tracker,"timestamp")->valuedouble);
		cJSON_AddNumberToObject(path,"dx",0);
		cJSON_AddNumberToObject(path,"dy",0);
		cJSON_AddNumberToObject(path,"distance",cJSON_GetObjectItem(tracker,"distance")->valueint);
		cJSON_AddNumberToObject(path,"dwell",0);
		samples = cJSON_CreateArray();
		sample = cJSON_CreateObject();
		cJSON_AddNumberToObject(sample,"x",cJSON_GetObjectItem(tracker,"cx")->valueint);
		cJSON_AddNumberToObject(sample,"y",cJSON_GetObjectItem(tracker,"cy")->valueint);
		cJSON_AddNumberToObject(sample,"d",0);
		cJSON_AddItemToArray(samples,sample);
		cJSON_AddItemToObject(path,"path",samples);
		cJSON_AddItemToObject(ObjectPathTrackers,idString,path);
	} else {
LOG_TRACE("Path: %s updated\n",idString);
		cJSON_ReplaceItemInObject( path,"confidence",cJSON_CreateNumber(cJSON_GetObjectItem(tracker,"confidence")->valueint));
		cJSON_ReplaceItemInObject( path,"timestamp",cJSON_CreateNumber(cJSON_GetObjectItem(tracker,"timestamp")->valuedouble));
		cJSON_ReplaceItemInObject( path,"age",cJSON_CreateNumber(cJSON_GetObjectItem(tracker,"age")->valuedouble));
		cJSON_ReplaceItemInObject( path,"distance",cJSON_CreateNumber(cJSON_GetObjectItem(tracker,"distance")->valuedouble));
		cJSON_ReplaceItemInObject( path,"dx",cJSON_CreateNumber(cJSON_GetObjectItem(tracker,"dx")->valueint));
		cJSON_ReplaceItemInObject( path,"dy",cJSON_CreateNumber(cJSON_GetObjectItem(tracker,"dy")->valueint));
		cJSON_ReplaceItemInObject( path,"dy",cJSON_CreateNumber(cJSON_GetObjectItem(tracker,"dy")->valueint));
		
		if( !cJSON_GetObjectItem(path,"color") && cJSON_GetObjectItem(tracker,"color") )
			cJSON_AddStringToObject(path,"color",cJSON_GetObjectItem(tracker,"color")->valuestring);

		if( !cJSON_GetObjectItem(path,"color2") && cJSON_GetObjectItem(tracker,"color2") )
			cJSON_AddStringToObject(path,"color2",cJSON_GetObjectItem(tracker,"color2")->valuestring);

		if(  !cJSON_GetObjectItem(path,"bag") && cJSON_GetObjectItem(tracker,"bag") )
			cJSON_AddStringToObject(path,"bag",cJSON_GetObjectItem(tracker,"bag")->valuestring);

		if( !cJSON_GetObjectItem(path,"hat") && cJSON_GetObjectItem(tracker,"hat")  )
			cJSON_AddStringToObject(path,"hat",cJSON_GetObjectItem(tracker,"hat")->valuestring);

		if( !cJSON_GetObjectItem(path,"face") && cJSON_GetObjectItem(tracker,"face") ) {
			if( cJSON_GetObjectItem(tracker,"face")->type == cJSON_True )
				cJSON_AddTrueToObject(path,"face");
			else
				cJSON_AddFalseToObject(path,"face");
		}

		double timestamp = cJSON_GetObjectItem(tracker,"timestamp")->valuedouble;
		double previousTime = cJSON_GetObjectItem(path,"previousTime")->valuedouble;
		double time = (timestamp - previousTime)/1000.0;
		cJSON_ReplaceItemInObject( path,"previousTime",cJSON_CreateNumber(timestamp));
		if( time > cJSON_GetObjectItem(path,"dwell")->valuedouble )
			cJSON_ReplaceItemInObject( path,"dwell",cJSON_CreateNumber(time));
		samples = cJSON_GetObjectItem(path,"path");
		if( samples ) {
			sample = cJSON_CreateObject();
			cJSON_AddNumberToObject(sample,"x",cJSON_GetObjectItem(tracker,"cx")->valueint);
			cJSON_AddNumberToObject(sample,"y",cJSON_GetObjectItem(tracker,"cy")->valueint);
			cJSON_AddNumberToObject(sample,"d",time);
			cJSON_AddItemToArray(samples,sample);
		}
	}

	if( cJSON_GetObjectItem(tracker,"active")->type == cJSON_False) {
		cJSON* payload = cJSON_DetachItemFromObject(ObjectPathTrackers,idString);
		if( payload ) {
			double age = cJSON_GetObjectItem(payload,"age")->valuedouble;
			samples = cJSON_GetObjectItem( payload,"path" );
			if( age > 2.0 && samples && cJSON_GetArraySize(samples) > 1) {
				cJSON_DeleteItemFromObject(payload,"previousTime");
				double timestamp = cJSON_GetObjectItem(path,"timestamp")->valuedouble - (age * 1000);
				cJSON_ReplaceItemInObject( payload,"timestamp",cJSON_CreateNumber(timestamp));
				ObjectPath_UserCallback(payload);
			}
			cJSON_Delete(payload);
		} else {
			LOG_TRACE("Path: %s is not a valid path\n", idString);
		}
	}
//LOG_TRACE("%s: Exit\n",__func__);
}

int 
ObjectPath_Init( ObjectPath_Callback callback ) {

	if( ObjectPath_UserCallback )
		return 1;

	ObjectPath_UserCallback = callback;

	return 1;
} 