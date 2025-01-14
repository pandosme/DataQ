#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <math.h>
#include <glib.h>

#include <assert.h>
#include "ObjectTracker.h"
#include "cJSON.h"
#include "TIME.h"

//#define LOG_TRACE(fmt, args...)    { printf(fmt, ## args); }//
#define LOG_TRACE(fmt, args...)    {}


cJSON* trackers = 0;
ObjectTracker_Callback Tracker_UserCallback = 0;
int updatingTrackers = 0;
double ObjectTracker_idleTime = 0;

void
ObjectTracker_Detections( cJSON* detections ) {
	char idString[16];
	cJSON *tracker,*detection;
	cJSON *remove = cJSON_CreateArray();

//	LOG_TRACE("%s\n",__func__);

	if(!trackers)
		trackers = cJSON_CreateObject();

	detection = detections->child;
	while(detection) {
		sprintf(idString,"%d",cJSON_GetObjectItem(detection,"id")->valueint);
		tracker = cJSON_GetObjectItem(trackers,idString);
		if( tracker )  {
			cJSON_GetObjectItem(tracker,"active")->type = cJSON_GetObjectItem(detection,"active")->type;
			cJSON_ReplaceItemInObject(tracker,"age",cJSON_CreateNumber(cJSON_GetObjectItem(detection,"age")->valuedouble));
			cJSON_ReplaceItemInObject(tracker,"confidence",cJSON_CreateNumber(cJSON_GetObjectItem(detection,"confidence")->valueint));
			cJSON_ReplaceItemInObject(tracker,"timestamp",cJSON_CreateNumber(cJSON_GetObjectItem(detection,"timestamp")->valuedouble));
			cJSON_ReplaceItemInObject(tracker,"x",cJSON_CreateNumber(cJSON_GetObjectItem(detection,"x")->valueint));
			cJSON_ReplaceItemInObject(tracker,"y",cJSON_CreateNumber(cJSON_GetObjectItem(detection,"y")->valueint));
			cJSON_ReplaceItemInObject(tracker,"w",cJSON_CreateNumber(cJSON_GetObjectItem(detection,"w")->valueint));
			cJSON_ReplaceItemInObject(tracker,"h",cJSON_CreateNumber(cJSON_GetObjectItem(detection,"h")->valueint));
			cJSON_ReplaceItemInObject(tracker,"cx",cJSON_CreateNumber(cJSON_GetObjectItem(detection,"cx")->valueint));
			cJSON_ReplaceItemInObject(tracker,"cy",cJSON_CreateNumber(cJSON_GetObjectItem(detection,"cy")->valueint));

			if( !cJSON_GetObjectItem(tracker,"color") && cJSON_GetObjectItem(detection,"color") )
				cJSON_AddStringToObject(tracker,"color",cJSON_GetObjectItem(detection,"color")->valuestring);
			if( !cJSON_GetObjectItem(tracker,"color2") && cJSON_GetObjectItem(detection,"color2") )
				cJSON_AddStringToObject(tracker,"color2",cJSON_GetObjectItem(detection,"color2")->valuestring);

			if( !cJSON_GetObjectItem(tracker,"bag") && cJSON_GetObjectItem(detection,"bag") )
				cJSON_AddStringToObject(tracker,"bag",cJSON_GetObjectItem(detection,"bag")->valuestring);

			if( !cJSON_GetObjectItem(tracker,"hat") && cJSON_GetObjectItem(detection,"hat") )
				cJSON_AddStringToObject(tracker,"hat",cJSON_GetObjectItem(detection,"hat")->valuestring);

			if( !cJSON_GetObjectItem(tracker,"face") && cJSON_GetObjectItem(detection,"face") ) {
				if( cJSON_GetObjectItem(detection,"face")->type == cJSON_True )
					cJSON_AddTrueToObject(tracker,"face");
				else
					cJSON_AddFalseToObject(tracker,"face");
			}
			
		} else {
			tracker = cJSON_Duplicate(detection,1);
			cJSON_AddNumberToObject(tracker,"px",1000);
			cJSON_AddNumberToObject(tracker,"py",1000);
			cJSON_AddNumberToObject(tracker,"dx",0);
			cJSON_AddNumberToObject(tracker,"dy",0);
			cJSON_AddNumberToObject(tracker,"distance",0);
//			cJSON_AddNumberToObject(tracker,"lastUpdate", cJSON_GetObjectItem(detection,"timestamp")->valuedouble);
			cJSON_AddItemToObject(trackers,idString,tracker);
		}

		if( cJSON_GetObjectItem(tracker,"active")->type == cJSON_False ){
			cJSON_AddItemToArray(remove,cJSON_CreateNumber(cJSON_GetObjectItem(tracker,"id")->valueint));
		}

		double dx = cJSON_GetObjectItem(tracker,"cx")->valueint - cJSON_GetObjectItem(tracker,"bx")->valueint;
		double dy = cJSON_GetObjectItem(tracker,"cy")->valueint - cJSON_GetObjectItem(tracker,"by")->valueint;
		int distance = (sqrt( (dx * dx) + (dy*dy) ) / 10.0);
		cJSON_ReplaceItemInObject(tracker,"dx",cJSON_CreateNumber(dx));
		cJSON_ReplaceItemInObject(tracker,"dy",cJSON_CreateNumber(dy));
		cJSON_ReplaceItemInObject(tracker,"distance",cJSON_CreateNumber(distance));

		dx = cJSON_GetObjectItem(tracker,"cx")->valueint - cJSON_GetObjectItem(tracker,"px")->valueint;
		dy = cJSON_GetObjectItem(tracker,"cy")->valueint - cJSON_GetObjectItem(tracker,"py")->valueint;
		distance = sqrt( (dx * dx) + (dy*dy) );
		if( distance > 50.0 && Tracker_UserCallback ) {
			cJSON_ReplaceItemInObject(tracker,"px",cJSON_CreateNumber(cJSON_GetObjectItem(detection,"cx")->valueint));
			cJSON_ReplaceItemInObject(tracker,"py",cJSON_CreateNumber(cJSON_GetObjectItem(detection,"cy")->valueint));
//			cJSON_DeleteItemFromObject(tracker,"lastUpdate");
			Tracker_UserCallback(tracker);
//			cJSON_AddNumberToObject(tracker,"lastUpdate", cJSON_GetObjectItem(detection,"timestamp")->valuedouble);
		}
		detection = detection->next;
	}
	
	cJSON* item = remove->child;
	while( item ) {
		sprintf(idString,"%d", item->valueint );
		tracker = cJSON_DetachItemFromObject(trackers,idString);
		if( tracker ) {
			Tracker_UserCallback(tracker);
			cJSON_Delete( tracker );
		}
		item = item->next;
	}
	cJSON_Delete(remove);
}



int 
ObjectTracker_Init( ObjectTracker_Callback callback ) {
	
	LOG_TRACE("ObjectTracker_Init\n");

	trackers = cJSON_CreateObject();
	
	if( Tracker_UserCallback )
		return 1;

	Tracker_UserCallback = callback;

	return 1;
} 