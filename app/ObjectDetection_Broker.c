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
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include <mdb/connection.h>
#include <mdb/error.h>
#include <mdb/subscriber.h>

#include "ACAP.h"
#include "ObjectDetection.h"
#include "cJSON.h"

#define LOG(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args);}
#define LOG_WARN(fmt, args...)    { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args);}
#define LOG_TRACE(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
//#define LOG_TRACE(fmt, args...)    {}


typedef struct channel_identifier {
    char* topic;
    char* source;
} channel_identifier_t;

mdb_error_t* error                         = NULL;
mdb_subscriber_config_t* subscriber_config = NULL;
mdb_subscriber_t* subscriber               = NULL;
mdb_connection_t* connection			   = NULL;
ObjectDetection_Callback ObjectDetection_UserCallback = NULL;
cJSON* ObjectDetections = 0;
cJSON* ObjectDetectionSettings = 0;

static void on_connection_error(const mdb_error_t* error, void* user_data) {
    (void)user_data;

    LOG_WARN("%s: Got connection error: %s, Aborting...", __func__, error->message);

    abort();
}


static void on_message(const mdb_message_t* message, void* user_data) {
    const mdb_message_payload_t* payload = mdb_message_get_payload(message);
	cJSON* data = cJSON_Parse((char*)payload->data);
	if( !data ) {
		LOG_WARN("%s: Parse error\n",__func__);
		return;
	}
	LOG_TRACE("%s\n", (char*)payload->data );
	if( ObjectDetection_UserCallback )
		ObjectDetection_UserCallback( data );
	cJSON_Delete( data );
}

static void on_done_subscriber_create(const mdb_error_t* error, void* user_data) {
    if (error != NULL) {
        syslog(LOG_ERR, "Got subscription error: %s, Aborting...", error->message);
        abort();
    }
    channel_identifier_t* channel_identifier = (channel_identifier_t*)user_data;

    LOG_TRACE("Subscribed to %s (%s)...",channel_identifier->topic,channel_identifier->source);
}

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
			ACAP_HTTP_Respond_Error(response, 500, "Invalid Object Detection Settings");
		return;
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
        if (!settings) {
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
	if( callback == 0 ) {
		ObjectDetection_UserCallback = 0;
		return 1;
	}

	if( ObjectDetection_UserCallback ) {
		//Already initialized
		return 1;
	}

	ObjectDetection_UserCallback = callback;

	ACAP_STATUS_SetString("ObjectDetection", "status", "Initializing");
	ACAP_STATUS_SetBool("ObjectDetection", "state", false);

    connection = mdb_connection_create(on_connection_error, NULL, &error);
    if (error != NULL) {
		ACAP_STATUS_SetString("ObjectDetection", "status", "Unable to create connection");
		LOG_WARN("%s: %s\n",__func__,error->message);
		mdb_error_destroy(&error);		
        return 0;
    }

    channel_identifier_t channel_identifier    = {.topic =
                                                      "com.axis.analytics_scene_description.v0.beta",
                                                  .source = "1"};

    subscriber_config = mdb_subscriber_config_create(channel_identifier.topic,
                                                     channel_identifier.source,
                                                     on_message,
                                                     &channel_identifier,
                                                     &error);
    if (error != NULL) {
		LOG_WARN("%s: %s\n",__func__,error->message);
		ACAP_STATUS_SetString("ObjectDetection", "status", "No subscriber available");
		mdb_error_destroy(&error);		
        return 0;
    }

    subscriber = mdb_subscriber_create_async(connection,
                                             subscriber_config,
                                             on_done_subscriber_create,
                                             &channel_identifier,
                                             &error);
    if (error != NULL) {
		LOG_WARN("%s: %s\n",__func__,error->message);
		ACAP_STATUS_SetString("ObjectDetection", "status", "No connection to subscriber");
		mdb_error_destroy(&error);		
        return 0;
    }

	ObjectDetectionSettings = ACAP_FILE_Read( "settings/subscriptions.json" );
	if(! ObjectDetectionSettings ) {
		ACAP_STATUS_SetString("ObjectDetection", "status", "No settings found");
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

	ACAP_STATUS_SetString("ObjectDetection", "status", "Running");
	ACAP_STATUS_SetBool("ObjectDetection", "state", 1);

	return 1;
} 

int ObjectDetection_Cleanup() {
	if( subscriber_config )
		mdb_subscriber_config_destroy(&subscriber_config);
	if( subscriber )
		mdb_subscriber_destroy(&subscriber);
	if( connection )
		mdb_connection_destroy(&connection);
	return 1;
}
