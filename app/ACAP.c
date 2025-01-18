/*
 * Copyright (c) 2024 Fred Juhlin
 * MIT License - See LICENSE file for details
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <syslog.h>
#include <string.h>
#include <errno.h>
#include <glib.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <ctype.h>
#include <sys/sysinfo.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <glib-object.h>
#include <glib.h>
#include <gio/gio.h>
#include <axsdk/axevent.h>
#include <axsdk/axhttp.h>
#include <axsdk/axparameter.h>
#include "ACAP.h"


// Logging macros
#define LOG(fmt, args...) { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_WARN(fmt, args...) { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args); }
//#define LOG_TRACE(fmt, args...) { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_TRACE(fmt, args...) {}

// Global variables
static cJSON* app = NULL;
static cJSON* status_container = NULL;

cJSON* 		ACAP_STATUS(void);
void		ACAP_HTTP(void);
void		ACAP_HTTP_Process(void);
void		ACAP_HTTP_Cleanup(void);
cJSON*		ACAP_EVENTS(void);
int 		ACAP_FILE_Init(void);
cJSON* 		ACAP_DEVICE(void);
void 		ACAP_VAPIX_Init(void);

static void ACAP_ENDPOINT_settings(const ACAP_HTTP_Response response, const ACAP_HTTP_Request request);
static void ACAP_ENDPOINT_app(const ACAP_HTTP_Response response, const ACAP_HTTP_Request request);
static char ACAP_package_name[ACAP_MAX_PACKAGE_NAME];
static ACAP_Config_Update ACAP_UpdateCallback = NULL;
cJSON* SplitString(const char* input, const char* delimiter);


/*-----------------------------------------------------
 * Core Functions Implementation
 *-----------------------------------------------------*/


cJSON* ACAP(const char* package, ACAP_Config_Update callback) {
    if (!package) {
        LOG_WARN("Invalid package name\n");
        return NULL;
    }

    
    LOG_TRACE("%s: Initializing ACAP for package %s\n", __func__, package);
    
    // Store package name
    strncpy(ACAP_package_name, package, ACAP_MAX_PACKAGE_NAME - 1);
    ACAP_package_name[ACAP_MAX_PACKAGE_NAME - 1] = '\0';
    
    // Initialize subsystems
    if (!ACAP_FILE_Init()) {
        LOG_WARN("Failed to initialize file system\n");
        return NULL;
    }

    // Store callback
    ACAP_UpdateCallback = callback;

    // Create main app object
    app = cJSON_CreateObject();
    if (!app) {
        LOG_WARN("Failed to create app object\n");
        return NULL;
    }

    // Load manifest
    cJSON* manifest = ACAP_FILE_Read("manifest.json");
    if (manifest) {
        cJSON_AddItemToObject(app, "manifest", manifest);
    }

    // Load and merge settings
    cJSON* settings = ACAP_FILE_Read("settings/settings.json");
    if (!settings) {
        settings = cJSON_CreateObject();
    }
    
    cJSON* savedSettings = ACAP_FILE_Read("localdata/settings.json");
    if (savedSettings) {
        cJSON* prop = savedSettings->child;
        while (prop) {
            if (cJSON_GetObjectItem(settings, prop->string)) {
				if( prop->type == cJSON_Object ) {
					cJSON* settingsProp = cJSON_GetObjectItem(settings,prop->string);
					cJSON* subprop = prop->child;
					while( subprop ) {
						if( cJSON_GetObjectItem( settingsProp, subprop->string ) )
							cJSON_ReplaceItemInObject(settingsProp, subprop->string, cJSON_Duplicate(subprop, 1));
						subprop = subprop->next;
					}
				} else {
					cJSON_ReplaceItemInObject(settings, prop->string, cJSON_Duplicate(prop, 1));
				}
            }
            prop = prop->next;
        }
        cJSON_Delete(savedSettings);
    }

    cJSON_AddItemToObject(app, "settings", settings);

    // Initialize subsystems
    ACAP_EVENTS();
	ACAP_HTTP();
    
    // Register core services
    ACAP_Set_Config("status", ACAP_STATUS());
    ACAP_Set_Config("device", ACAP_DEVICE());
    
    // Register core endpoints
    ACAP_HTTP_Node("app", ACAP_ENDPOINT_app);
    ACAP_HTTP_Node("settings", ACAP_ENDPOINT_settings);

    // Notify about settings
    if (ACAP_UpdateCallback) {
		cJSON* setting = settings->child;
		while( setting ) {
			ACAP_UpdateCallback(setting->string, setting);
			setting = setting->next;
		}
    }

    LOG_TRACE("%s: Initialization complete\n", __func__);
    return settings;
}


static void
ACAP_ENDPOINT_app(const ACAP_HTTP_Response response, const ACAP_HTTP_Request request) {
    // Check request method
    ACAP_HTTP_Respond_JSON(response, app);
}

static void
ACAP_ENDPOINT_settings(const ACAP_HTTP_Response response, const ACAP_HTTP_Request request) {
	const char* json = ACAP_HTTP_Request_Param(request, "json");
	if(!json)
		json = ACAP_HTTP_Request_Param(request, "set");
	if( !json ) {
        ACAP_HTTP_Respond_JSON(response, cJSON_GetObjectItem(app, "settings"));
        return;
	}
	cJSON *params = cJSON_Parse(json);
	if (!params) {
		ACAP_HTTP_Respond_Error(response, 400, "Invalid JSON data");
		return;
    }
	cJSON* settings = cJSON_GetObjectItem(app, "settings");
	if(!settings) {
		ACAP_HTTP_Respond_Error(response, 400, "Settings is null");
		LOG_WARN("%s: Settings is NULL\n",__func__);
		return;
	}

	cJSON* param = params->child;
	while (param) {
		if (cJSON_GetObjectItem(settings, param->string)) {
			cJSON_ReplaceItemInObject(settings, param->string, cJSON_Duplicate(param, 1));
			if (ACAP_UpdateCallback)
				ACAP_UpdateCallback(param->string, cJSON_GetObjectItem(settings,param->string));
		}
		param = param->next;
	}
	cJSON_Delete(params);
	ACAP_FILE_Write("localdata/settings.json", settings);
    ACAP_HTTP_Respond_Text(response, "Settings updated successfully");
}

const char* ACAP_Name(void) {
	return ACAP_package_name;
}
 
int
ACAP_Set_Config(const char* service, cJSON* serviceSettings ) {
	LOG_TRACE("%s: %s\n",__func__,service);
	if( cJSON_GetObjectItem(app,service) ) {
		LOG_TRACE("%s: %s already registered\n",__func__,service);
		return 1;
	}
	cJSON_AddItemToObject( app, service, serviceSettings );
	return 1;
}

cJSON*
ACAP_Get_Config(const char* service) {
	cJSON* reqestedService = cJSON_GetObjectItem(app, service );
	if( !reqestedService ) {
		LOG_WARN("%s: %s is undefined\n",__func__,service);
		return 0;
	}
	return reqestedService;
}

/*------------------------------------------------------------------
 * HTTP Request Processing Implementation
 *------------------------------------------------------------------*/

AXHttpHandler  *ACAP_HTTP_handler = 0;
GHashTable     *ACAP_HTTP_node_table = 0; 

void
ACAP_HTTP_Close() {
}

int
ACAP_HTTP_Node( const char *name, ACAP_HTTP_Callback callback ) {
  gchar path[128];
  g_snprintf (path, 128, "/local/%s/%s", ACAP_package_name, name );
  if( !ACAP_HTTP_handler ) {
    LOG_WARN("ACAP_HTTP_cgi: HTTP handler not initialized\n");
    return 0;
  }
  
LOG_TRACE("%s:%s", __func__, path );

  if( !ACAP_HTTP_node_table )
    ACAP_HTTP_node_table = g_hash_table_new_full(g_str_hash, g_str_equal,g_free, NULL);
  g_hash_table_insert( ACAP_HTTP_node_table, g_strdup( path ), (gpointer*)callback);
  return 1;
}

int
ACAP_HTTP_Respond_String( ACAP_HTTP_Response response,const gchar *fmt, ...) {
  va_list ap;
  gchar *tmp_str;
  GDataOutputStream *stream = (GDataOutputStream *)response;
  if( !stream ) {
    LOG_WARN("ACAP_HTTP_Respond_String: Cannot send data to http.  Handler = NULL\n");
    return 0;
  }
  
  va_start(ap, fmt);
  tmp_str = g_strdup_vprintf(fmt, ap);
  g_data_output_stream_put_string((GDataOutputStream *)response, tmp_str, NULL, NULL);
  
  g_free(tmp_str);

  va_end(ap);
  return 1;
}

int	ACAP_HTTP_Respond_Data(ACAP_HTTP_Response response, size_t count, const void* data) {
  gsize data_sent;
  
  if( count == 0 || data == 0 ) {
    LOG_WARN("ACAP_HTTP_Data: Invalid data\n");
    return 0;
  }
  
  if( !g_output_stream_write_all((GOutputStream *)response, data, count, &data_sent, NULL, NULL) ) {
    LOG_WARN("ACAP_HTTP_Data: Error sending data.");
    return 0;
  }  
  return 1;
}

const char*
ACAP_HTTP_Request_Param( const ACAP_HTTP_Request request, const char* name) {
  gchar *value;
  if( !request )
    return 0;
  if(!g_hash_table_lookup_extended((GHashTable *)request, name, NULL, (gpointer*)&value)) {
    printf("ACAP_HTTP_Request_Param: Invalid option %s\n", name);
    return 0;
  }
  return value;   
}

cJSON*
ACAP_HTTP_Request_JSON( const ACAP_HTTP_Request request, const char *param ) {
  const char *jsonstring;
  cJSON *object;
  jsonstring = ACAP_HTTP_Request_Param( request, param);
  if( !jsonstring ) {
    return 0;
  }
  object = cJSON_Parse(jsonstring);
  if(!object) {
    LOG_WARN("ACAP_HTTP_Request_JSON: Invalid JSON: %s",jsonstring);
    return 0;
  }
  return object;
}

int
ACAP_HTTP_Header_XML( ACAP_HTTP_Response response ) {
  ACAP_HTTP_Respond_String( response,"Content-Type: text/xml; charset=utf-8; Cache-Control: no-cache\r\n\r\n");
  ACAP_HTTP_Respond_String( response,"<?xml version=\"1.0\"?>\r\n");
  return 1;
}

int
ACAP_HTTP_Header_JSON( ACAP_HTTP_Response response ) {
  ACAP_HTTP_Respond_String( response,"Content-Type: application/json; charset=utf-8; Cache-Control: no-cache\r\n\r\n");
  return 1;
}

int
ACAP_HTTP_Header_TEXT( ACAP_HTTP_Response response ) {
  ACAP_HTTP_Respond_String( response,"Content-Type: text/plain; charset=utf-8; Cache-Control: no-cache\r\n\r\n");
  return 1;
}

int
ACAP_HTTP_Header_FILE( const ACAP_HTTP_Response response, const char *filename, const char *contenttype, unsigned filelength ) {
  ACAP_HTTP_Respond_String( response, "HTTP/1.1 200 OK\r\n");
  ACAP_HTTP_Respond_String( response, "Pragma: public\r\n");
  ACAP_HTTP_Respond_String( response, "Content-Description: File Transfer\r\n");
  ACAP_HTTP_Respond_String( response, "Content-Type: %s\r\n", contenttype);
  ACAP_HTTP_Respond_String( response, "Content-Disposition: attachment; filename=%s\r\n", filename);
  ACAP_HTTP_Respond_String( response, "Content-Transfer-Encoding: binary\r\n");
  ACAP_HTTP_Respond_String( response, "Expires: 0\r\n");
  ACAP_HTTP_Respond_String( response, "Cache-Control: must-revalidate\r\n");
  ACAP_HTTP_Respond_String( response, "Content-Length: %u\r\n", filelength );
  ACAP_HTTP_Respond_String( response, "\r\n");
  return 1;
}

int
ACAP_HTTP_Respond_Error( ACAP_HTTP_Response response, int code, const char *message ) {
  ACAP_HTTP_Respond_String( response,"status: %d %s Error\r\nContent-Type: text/plain\r\n\r\n", code, (code < 500) ? "Client" : (code < 600) ? "Server":"");
  if( code < 500 )
    LOG_WARN("HTTP response %d: %s\n", code, message);
  if( code >= 500 )
    LOG_WARN("HTTP response %d: %s\n", code, message);
  ACAP_HTTP_Respond_String( response,"%s", message);
  return 1;
}

int
ACAP_HTTP_Respond_Text( ACAP_HTTP_Response response, const char *message ) {
  ACAP_HTTP_Header_TEXT( response );
  ACAP_HTTP_Respond_String( response,"%s", message);
  return 1;
}

int
ACAP_HTTP_Respond_JSON( ACAP_HTTP_Response response, cJSON *object) {
  char *jsonstring;
  if( !object ) {
    LOG_WARN("ACAP_HTTP_Respond_JSON: Invalid object");
    return 0;
  }
  jsonstring = cJSON_Print( object );  
  ACAP_HTTP_Header_JSON( response );
  ACAP_HTTP_Respond_String( response, jsonstring );
  free( jsonstring );
  return 0;
}

static void
ACAP_HTTP_main_callback(const gchar *path,const gchar *method, const gchar *query, GHashTable *request, GOutputStream *output_stream, gpointer user_data){
	GDataOutputStream *response;
	gchar *key;
	ACAP_HTTP_Callback callback = 0;

	(void) user_data;
	
	if( request ) {
		LOG_TRACE("HTTP request: %s?%s\n", path, query);
	} else  {
		LOG_TRACE("HTTP request: %s\n", path);
	}

	response = g_data_output_stream_new(output_stream);

	if( !ACAP_HTTP_node_table ) {
		ACAP_HTTP_Respond_Error( response, 500, "ACAP_HTTP_main_callback: Invalid table" );
		return;
	}
	if( !g_hash_table_lookup_extended( ACAP_HTTP_node_table, path, (gpointer*)&key,(gpointer*)&callback) ) {
		LOG_WARN("ACAP_HTTP_main_callback: CGI table lookup failed for %s (key = %s)\n", path, key );
	}

	if( callback ) {
		callback( (ACAP_HTTP_Response)response, (ACAP_HTTP_Request) request);
	} else {
		ACAP_HTTP_Respond_Error( response,400,"ACAP_HTTP_main_callback: No valid HTTP consumer");
	}
	g_object_unref(response);
}

void ACAP_HTTP(void) {
	LOG_TRACE("%s:%s", __func__, ACAP_package_name);
	if( !ACAP_HTTP_handler )
		ACAP_HTTP_handler = ax_http_handler_new( ACAP_HTTP_main_callback, NULL);
	if( !ACAP_HTTP_handler )
		LOG_WARN("ACAP_HTTP_init: Failed to initialize HTTP\n");

	if( !ACAP_HTTP_node_table )
		ACAP_HTTP_node_table = g_hash_table_new_full(g_str_hash, g_str_equal,g_free, NULL);
}

void 
ACAP_HTTP_Cleanup(void) {
}

/*------------------------------------------------------------------
 * Status Management Implementation
 *------------------------------------------------------------------*/

static void
ACAP_ENDPOINT_status(const ACAP_HTTP_Response response, const ACAP_HTTP_Request request) {
	if(!status_container)
		status_container = cJSON_CreateObject();
    
    ACAP_HTTP_Respond_JSON(response, status_container);
}

cJSON* ACAP_STATUS(void) {
    if (!status_container) {
        status_container = cJSON_CreateObject();
		ACAP_HTTP_Node("status",ACAP_ENDPOINT_status);
	}
    return status_container;
}

cJSON* ACAP_STATUS_Group(const char* name) {
    if (!name || !status_container) {
        return NULL;
    }

    cJSON* group = cJSON_GetObjectItem(status_container, name);
    if (!group) {
        group = cJSON_CreateObject();
        if (!group) {
            LOG_WARN("Failed to create status group: %s\n", name);
            return NULL;
        }
        cJSON_AddItemToObject(status_container, name, group);
    }
    return group;
}

void ACAP_STATUS_SetBool(const char* group, const char* name, int state) {
    if (!group || !name) {
        LOG_WARN("%s: Invalid group or name parameter\n", __func__);
        return;
    }

    cJSON* groupObj = ACAP_STATUS_Group(group);
    if (!groupObj) {
		LOG_TRACE("%s: Unknown %s\n",__func__,group);
        return;
    }

    cJSON* item = cJSON_GetObjectItem(groupObj, name);
    if (item) {
        cJSON_ReplaceItemInObject(groupObj, name, cJSON_CreateBool(state));
    } else {
        cJSON_AddItemToObject(groupObj, name, cJSON_CreateBool(state));
    }
}

void ACAP_STATUS_SetNumber(const char* group, const char* name, double value) {
    if (!group || !name) {
        LOG_WARN("Invalid group or name parameter\n");
        return;
    }

    cJSON* groupObj = ACAP_STATUS_Group(group);
    if (!groupObj) {
        return;
    }

    cJSON* item = cJSON_GetObjectItem(groupObj, name);
    if (item) {
        cJSON_ReplaceItemInObject(groupObj, name, cJSON_CreateNumber(value));
    } else {
        cJSON_AddItemToObject(groupObj, name, cJSON_CreateNumber(value));
    }
}

void ACAP_STATUS_SetString(const char* group, const char* name, const char* string) {
    if (!group || !name || !string) {
        LOG_WARN("Invalid parameters\n");
        return;
    }

    cJSON* groupObj = ACAP_STATUS_Group(group);
    if (!groupObj) {
        return;
    }

    cJSON* item = cJSON_GetObjectItem(groupObj, name);
    if (item) {
        cJSON_ReplaceItemInObject(groupObj, name, cJSON_CreateString(string));
    } else {
        cJSON_AddItemToObject(groupObj, name, cJSON_CreateString(string));
    }
}

void ACAP_STATUS_SetObject(const char* group, const char* name, cJSON* data) {
    if (!group || !name || !data) {
        LOG_WARN("Invalid parameters\n");
        return;
    }

    cJSON* groupObj = ACAP_STATUS_Group(group);
    if (!groupObj) {
        return;
    }

    cJSON* item = cJSON_GetObjectItem(groupObj, name);
    if (item) {
        cJSON_ReplaceItemInObject(groupObj, name, cJSON_Duplicate(data, 1));
    } else {
        cJSON_AddItemToObject(groupObj, name, cJSON_Duplicate(data, 1));
    }
}

void ACAP_STATUS_SetNull(const char* group, const char* name) {
    if (!group || !name) {
        LOG_WARN("Invalid group or name parameter\n");
        return;
    }

    cJSON* groupObj = ACAP_STATUS_Group(group);
    if (!groupObj) {
        return;
    }

    cJSON* item = cJSON_GetObjectItem(groupObj, name);
    if (item) {
        cJSON_ReplaceItemInObject(groupObj, name, cJSON_CreateNull());
    } else {
        cJSON_AddItemToObject(groupObj, name, cJSON_CreateNull());
    }
}

/*------------------------------------------------------------------
 * Status Getters Implementation
 *------------------------------------------------------------------*/

int ACAP_STATUS_Bool(const char* group, const char* name) {
    cJSON* groupObj = ACAP_STATUS_Group(group);
    if (!groupObj) {
		LOG_TRACE("%s: Invalid group \n",__func__);
        return 0;
    }

    cJSON* item = cJSON_GetObjectItem(groupObj, name);
	if( !item ) {
		return 0;
	}
    return item->type == cJSON_True?1:0;
}

int ACAP_STATUS_Int(const char* group, const char* name) {
    cJSON* groupObj = ACAP_STATUS_Group(group);
    if (!groupObj) {
        return 0;
    }

    cJSON* item = cJSON_GetObjectItem(groupObj, name);
    return item && cJSON_IsNumber(item) ? item->valueint : 0;
}

double ACAP_STATUS_Double(const char* group, const char* name) {
    cJSON* groupObj = ACAP_STATUS_Group(group);
    if (!groupObj) {
        return 0.0;
    }

    cJSON* item = cJSON_GetObjectItem(groupObj, name);
    return item && cJSON_IsNumber(item) ? item->valuedouble : 0.0;
}

char* ACAP_STATUS_String(const char* group, const char* name) {
    cJSON* groupObj = ACAP_STATUS_Group(group);
    if (!groupObj) {
        return NULL;
    }

    cJSON* item = cJSON_GetObjectItem(groupObj, name);
    return item && cJSON_IsString(item) ? item->valuestring : NULL;
}

cJSON* ACAP_STATUS_Object(const char* group, const char* name) {
    cJSON* groupObj = ACAP_STATUS_Group(group);
    if (!groupObj) {
        return NULL;
    }

    return cJSON_GetObjectItem(groupObj, name);
}

/*------------------------------------------------------------------
 * Device Information Implementation
 *------------------------------------------------------------------*/

static cJSON* ACAP_DEVICE_Container = NULL;
AXParameter *ACAP_DEVICE_parameter_handler;

double previousTransmitted = 0;
double previousNetworkTimestamp = 0;
double previousNetworkAverage = 0;
char** string_split( char* a_str,  char a_delim);

#define BUFFER_SIZE 500
#define MIN_DATA_LENGTH 20
#define BITS_PER_BYTE 8
#define BITS_PER_KBIT 1024

typedef uint64_t network_bytes_t;

double ACAP_DEVICE_Network_Average(void) {
    FILE *fd = NULL;
    char data[BUFFER_SIZE] = {0};
    char readstr[BUFFER_SIZE] = {0};
    char **stringArray = NULL;
    double network_average = 0.0;
    network_bytes_t transmitted = 0;
    network_bytes_t rx = 0;
    
    fd = fopen("/proc/net/dev", "r");
    if (!fd) {
        LOG_WARN("Error opening /proc/net/dev");
        return previousNetworkAverage;
    }
    
    bool found_interface = false;
    while (fgets(readstr, BUFFER_SIZE - 1, fd)) {
        if (strstr(readstr, "eth0")) {
            if (snprintf(data, BUFFER_SIZE, "%s", readstr) >= BUFFER_SIZE) {
                LOG_WARN("Buffer overflow prevented");
                fclose(fd);
                return previousNetworkAverage;
            }
            found_interface = true;
            break;
        }
    }
    fclose(fd);
    
    if (!found_interface || strlen(data) < MIN_DATA_LENGTH) {
        LOG_WARN("Invalid or missing network data");
        return previousNetworkAverage;
    }
    
    stringArray = string_split(data, ' ');
    if (!stringArray) {
        LOG_WARN("String split failed");
        return previousNetworkAverage;
    }
    
    bool parse_success = false;
    char *endptr = NULL;
    
    // Count array elements first
    size_t array_size = 0;
    while (stringArray[array_size]) {
        array_size++;
    }
    
    if (array_size > 1) {
        rx = strtoull(stringArray[1], &endptr, 10);
        if (endptr != stringArray[1]) {
            parse_success = true;
        }
    }
    
    if (array_size > 9) {
        transmitted = strtoull(stringArray[9], &endptr, 10);
        if (endptr != stringArray[9]) {
            parse_success = true;
        }
    }
    
    // Free memory
    for (size_t i = 0; i < array_size; i++) {
        free(stringArray[i]);
    }
    free(stringArray);
    
    if (!parse_success) {
        LOG_WARN("Failed to parse network values");
        return previousNetworkAverage;
    }
    
    double timestamp = ACAP_DEVICE_Timestamp();
    double timeDiff = timestamp - previousNetworkTimestamp;
    
    // Validate time difference
    if (timeDiff <= 0) {
        LOG_WARN("Invalid time difference");
        return previousNetworkAverage;
    }
    
    // Handle counter wraparound
    double diff;
    if (transmitted < previousTransmitted) {
        diff = (double)((UINT64_MAX - previousTransmitted) + transmitted);
    } else {
        diff = (double)(transmitted - previousTransmitted);
    }
    
    previousTransmitted = transmitted;
    previousNetworkTimestamp = timestamp;
    
    // Convert to Kbps
    timeDiff /= 1000.0;  // Convert to seconds
    diff *= BITS_PER_BYTE;  // Convert bytes to bits
    diff /= BITS_PER_KBIT;  // Convert to Kbits
    
    network_average = diff / timeDiff;
    
    // Threshold small values to zero
    if (network_average < 0.001) {
        network_average = 0;
    }
    
    previousNetworkAverage = network_average;
    return network_average;
}

double
ACAP_DEVICE_CPU_Average() {
	double loadavg = 0;
	struct sysinfo info;
	
	sysinfo(&info);
	loadavg = (double)info.loads[2];
	loadavg /= 65536.0;
	LOG_TRACE("%f\n",loadavg);
	return loadavg; 
}

double
ACAP_DEVICE_Uptime() {
	struct sysinfo info;
	sysinfo(&info);
	return (double)info.uptime; 
};	

const char* 
ACAP_DEVICE_Prop( const char *attribute ) {
	if( !ACAP_DEVICE_Container )
		return 0;
	return cJSON_GetObjectItem(ACAP_DEVICE_Container,attribute) ? cJSON_GetObjectItem(ACAP_DEVICE_Container,attribute)->valuestring : 0;
}

int
ACAP_DEVICE_Prop_Int( const char *attribute ) {
  if( !ACAP_DEVICE_Container )
    return 0;
  return cJSON_GetObjectItem(ACAP_DEVICE_Container,attribute) ? cJSON_GetObjectItem(ACAP_DEVICE_Container,attribute)->valueint : 0;
}

cJSON* 
ACAP_DEVICE_JSON( const char *attribute ) {
  if( !ACAP_DEVICE_Container )
    return 0;
  return cJSON_GetObjectItem(ACAP_DEVICE_Container,attribute);
}

cJSON*
ACAP_DEVICE(void) {
	gchar *value = 0, *pHead,*pTail;

	LOG_TRACE("DEVICE_Init: %s\n", ACAP_package_name );

	ACAP_DEVICE_Container = cJSON_CreateObject();
	ACAP_DEVICE_parameter_handler = ax_parameter_new(ACAP_package_name, 0);
	if( !ACAP_DEVICE_parameter_handler ) {
		LOG_WARN("Cannot create parameter ACAP_DEVICE_parameter_handler");
		return 0;
	}
	

	if(ax_parameter_get(ACAP_DEVICE_parameter_handler, "root.Properties.System.SerialNumber", &value, 0)) {
		cJSON_AddItemToObject(ACAP_DEVICE_Container,"serial",cJSON_CreateString(value));
		g_free(value);
	}
  
	if(ax_parameter_get(ACAP_DEVICE_parameter_handler, "root.brand.ProdShortName", &value, 0)) {
		cJSON_AddItemToObject(ACAP_DEVICE_Container,"model",cJSON_CreateString(value));
		g_free(value);
	}

	if(ax_parameter_get(ACAP_DEVICE_parameter_handler, "root.Properties.System.Architecture", &value, 0)) {
		cJSON_AddItemToObject(ACAP_DEVICE_Container,"platform",cJSON_CreateString(value));
		g_free(value);
	}

	if(ax_parameter_get(ACAP_DEVICE_parameter_handler, "root.Properties.System.Soc", &value, 0)) {
		cJSON_AddItemToObject(ACAP_DEVICE_Container,"chip",cJSON_CreateString(value));
		g_free(value);
	}

	if(ax_parameter_get(ACAP_DEVICE_parameter_handler, "root.brand.ProdType", &value, 0)) {
		cJSON_AddItemToObject(ACAP_DEVICE_Container,"type",cJSON_CreateString(value));
		g_free(value);
	}

//	if(ax_parameter_get(ACAP_DEVICE_parameter_handler, "root.Network.VolatileHostName.HostName", &value, 0)) {
//		cJSON_AddItemToObject(ACAP_DEVICE_Container,"hostname",cJSON_CreateString(value));
//		g_free(value);
//	}
  
	int aspect = 169;

	if(ax_parameter_get(ACAP_DEVICE_parameter_handler, "root.ImageSource.I0.Sensor.AspectRatio",&value,0 )) {
		cJSON_AddItemToObject(ACAP_DEVICE_Container,"aspect",cJSON_CreateString(value));
		if(strcmp(value,"4:3") == 0)
			aspect = 43;
		if(strcmp(value,"16:10") == 0)
			aspect = 1610;
		if(strcmp(value,"1:1") == 0)
			aspect = 11;
		g_free(value);
	} else {
		cJSON_AddStringToObject(ACAP_DEVICE_Container,"aspect","16:9");
	}
  
	cJSON* resolutionList = cJSON_CreateArray();
	cJSON* resolutions = cJSON_CreateObject();
	cJSON_AddItemToObject(ACAP_DEVICE_Container,"resolutions",resolutions);
	cJSON* resolutions169 = cJSON_CreateArray();
	cJSON_AddItemToObject(resolutions,"16:9",resolutions169);
	cJSON* resolutions43 = cJSON_CreateArray();
	cJSON_AddItemToObject(resolutions,"4:3",resolutions43);
	cJSON* resolutions11 = cJSON_CreateArray();
	cJSON_AddItemToObject(resolutions,"1:1",resolutions11);
	cJSON* resolutions1610 = cJSON_CreateArray();
	cJSON_AddItemToObject(resolutions,"16:10",resolutions1610);
	if(ax_parameter_get(ACAP_DEVICE_parameter_handler, "root.Properties.Image.Resolution", &value, 0)) {
		pHead = value;
		pTail = value;
		while( *pHead ) {
			if( *pHead == ',' ) {
				*pHead = 0;
				cJSON_AddItemToArray( resolutionList, cJSON_CreateString(pTail) );
				pTail = pHead + 1;
			}
			pHead++;
		}
		cJSON_AddItemToArray( resolutionList, cJSON_CreateString(pTail) );
		g_free(value);

		int length = cJSON_GetArraySize(resolutionList);
		int index;
		char data[30];
		LOG_TRACE("Resolutions");
		int width = 0;
		int height = 0;
		for( index = 0; index < length; index++ ) {
			char* resolution = strcpy(data,cJSON_GetArrayItem(resolutionList,index)->valuestring);
			if( resolution ) {
				char* sX = resolution;
				char* sY = 0;
				while( *sX != 0 ) {
					if( *sX == 'x' ) {
						*sX = 0;
						sY = sX + 1;
					}
					sX++;
				}
				if( sY ) {
					int x = atoi(resolution);
					int y = atoi(sY);
					if( x && y ) {
						int a = (x*100)/y;
						if( a == 177 ) {
							cJSON_AddItemToArray(resolutions169, cJSON_CreateString(cJSON_GetArrayItem(resolutionList,index)->valuestring));
							if(aspect == 169 && x > width )
								width = x;
							if(aspect == 169 && y > height )
								height = y;
						}
						if( a == 133 ) {
							cJSON_AddItemToArray(resolutions43, cJSON_CreateString(cJSON_GetArrayItem(resolutionList,index)->valuestring));
							if(aspect == 43 && x > width )
								width = x;
							if(aspect == 43 && y > height )
								height = y;
						}
						if( a == 160 ) {
							cJSON_AddItemToArray(resolutions1610, cJSON_CreateString(cJSON_GetArrayItem(resolutionList,index)->valuestring));
							if(aspect == 1610 && x > width )
								width = x;
							if(aspect == 1610 && y > height )
								height = y;
						}
						if( a == 100 ) {
							cJSON_AddItemToArray(resolutions11, cJSON_CreateString(cJSON_GetArrayItem(resolutionList,index)->valuestring));
							if(aspect == 11 && x > width )
								width = x;
							if(aspect == 11 && y > height )
								height = y;
						}
					}
				}
			}
		}
		cJSON_AddItemToObject(ACAP_DEVICE_Container,"width",cJSON_CreateNumber(width));
		cJSON_AddItemToObject(ACAP_DEVICE_Container,"height",cJSON_CreateNumber(height));
		
		int a = (width*100)/height;
		if( a == 133 )
			cJSON_ReplaceItemInObject(ACAP_DEVICE_Container,"aspect",cJSON_CreateString("4:3") );
		if( a == 160 )
			cJSON_ReplaceItemInObject(ACAP_DEVICE_Container,"aspect",cJSON_CreateString("16:10") );
		if( a == 100 )
			cJSON_ReplaceItemInObject(ACAP_DEVICE_Container,"aspect",cJSON_CreateString("1:1") );
		cJSON_Delete(resolutionList);
		LOG_TRACE("Resolutions: Done");
	}
  
	if(ax_parameter_get(ACAP_DEVICE_parameter_handler, "root.Network.eth0.MACAddress",&value,0 )) {
		cJSON_AddItemToObject(ACAP_DEVICE_Container,"mac",cJSON_CreateString(value));
		g_free(value);
	}  

	if(ax_parameter_get(ACAP_DEVICE_parameter_handler, "root.ImageSource.I0.Sensor.AspectRatio",&value,0 )) {
		cJSON_AddItemToObject(ACAP_DEVICE_Container,"aspect",cJSON_CreateString(value));
		g_free(value);
	}  

	if(ax_parameter_get(ACAP_DEVICE_parameter_handler, "root.Image.I0.Appearance.Rotation",&value,0 )) {
		cJSON_AddItemToObject(ACAP_DEVICE_Container,"rotation",cJSON_CreateNumber( atoi(value) ));
		g_free(value);
	}  

	if(ax_parameter_get(ACAP_DEVICE_parameter_handler, "root.Network.eth0.IPAddress",&value,0 )) {
		cJSON_AddItemToObject(ACAP_DEVICE_Container,"IPv4",cJSON_CreateString(value));
		g_free(value);
	}  
  
	if(ax_parameter_get(ACAP_DEVICE_parameter_handler, "root.Properties.Firmware.Version",&value,0 )) {
		cJSON_AddItemToObject(ACAP_DEVICE_Container,"firmware",cJSON_CreateString(value));
		g_free(value);
	}  

	cJSON_AddStringToObject(ACAP_DEVICE_Container,"date", ACAP_DEVICE_Date() );
	cJSON_AddStringToObject(ACAP_DEVICE_Container,"time", ACAP_DEVICE_Time() );
 
	ax_parameter_free(ACAP_DEVICE_parameter_handler);
	return ACAP_DEVICE_Container;
}



/*------------------------------------------------------------------
 * Time and System Information
 *------------------------------------------------------------------*/

int
ACAP_DEVICE_Seconds_Since_Midnight() {
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);
	int seconds = tm.tm_hour * 3600;
	seconds += tm.tm_min * 60;
	seconds += tm.tm_sec;
	return seconds;
}

char ACAP_DEVICE_timestring[128] = "2020-01-01 00:00:00";
char ACAP_DEVICE_date[128] = "2023-01-01";
char ACAP_DEVICE_time[128] = "00:00:00";
char ACAP_DEVICE_isostring[128] = "2020-01-01T00:00:00+0000";

const char*
ACAP_DEVICE_Date() {
	time_t t = time(NULL);
	struct tm *tm = localtime(&t);
	sprintf(ACAP_DEVICE_date,"%d-%02d-%02d",tm->tm_year + 1900,tm->tm_mon + 1, tm->tm_mday);
	return ACAP_DEVICE_date;
}

const char*
ACAP_DEVICE_Time() {
	time_t t = time(NULL);
	struct tm *tm = localtime(&t);
	sprintf(ACAP_DEVICE_time,"%02d:%02d:%02d",tm->tm_hour,tm->tm_min,tm->tm_sec);
	return ACAP_DEVICE_time;
}


const char*
ACAP_DEVICE_Local_Time() {
	time_t t = time(NULL);
	struct tm *tm = localtime(&t);
	sprintf(ACAP_DEVICE_timestring,"%d-%02d-%02d %02d:%02d:%02d",tm->tm_year + 1900,tm->tm_mon + 1, tm->tm_mday,tm->tm_hour,tm->tm_min,tm->tm_sec);
	LOG_TRACE("Local Time: %s\n",ACAP_DEVICE_timestring);
	return ACAP_DEVICE_timestring;
}



const char*
ACAP_DEVICE_ISOTime() {
	time_t t = time(NULL);
	struct tm *tm = localtime(&t);
	strftime(ACAP_DEVICE_isostring, 50, "%Y-%m-%dT%T%z",tm);
	return ACAP_DEVICE_isostring;
}

double
ACAP_DEVICE_Timestamp(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    // Convert seconds and microseconds to milliseconds
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

/*------------------------------------------------------------------
 * File Operations Implementation
 *------------------------------------------------------------------*/

static char ACAP_FILE_Path[ACAP_MAX_PATH_LENGTH] = "";

const char* ACAP_FILE_AppPath(void) {
    return ACAP_FILE_Path;
}

int ACAP_FILE_Init(void) {
    if (!ACAP_package_name[0]) {
        LOG_WARN("Package name not initialized\n");
        return 0;
    }

    snprintf(ACAP_FILE_Path, sizeof(ACAP_FILE_Path), 
             "/usr/local/packages/%s/", ACAP_package_name);
    return 1;
}

FILE* ACAP_FILE_Open(const char* filepath, const char* mode) {
    if (!filepath ) {
        LOG_WARN("Invalid parameters for file operation\n");
        return NULL;
    }

	LOG_TRACE("%s: %s\n", __func__, filepath);

    char fullpath[ACAP_MAX_PATH_LENGTH];
    if (snprintf(fullpath, sizeof(fullpath), "%s%s", ACAP_FILE_Path, filepath) >= sizeof(fullpath)) {
        LOG_WARN("Path too long\n");
        return NULL;
    }
    
    FILE* file = fopen(fullpath, mode);
    if (!file) {
        LOG_TRACE("%s: Opening file %s failed: %s\n", __func__, fullpath, strerror(errno));
    }
    return file;
}

int ACAP_FILE_Delete(const char* filepath) {
    if (!filepath) {
        return 0;
    }

    char fullpath[ACAP_MAX_PATH_LENGTH];
    snprintf(fullpath, sizeof(fullpath), "%s%s", ACAP_FILE_Path, filepath);
    
    if (remove(fullpath) != 0) {
        LOG_WARN("Delete %s failed\n", fullpath);
        return 0;
    }
    return 1;
}

cJSON* ACAP_FILE_Read(const char* filepath) {
	LOG_TRACE("%s: %s\n",__func__,filepath);
    if (!filepath) {
        LOG_WARN("Invalid filepath\n");
        return NULL;
    }

    FILE* file = ACAP_FILE_Open(filepath, "r");
    if (!file) {
		LOG_TRACE("%s: File open error %s\n",__func__,filepath);
        return NULL;
    }

    // Get file size
    fseek(file, 0, SEEK_END);
    long size = ftell(file);
    fseek(file, 0, SEEK_SET);

    if (size < 2) {
        LOG_WARN("%s: File size error in %s\n", __func__, filepath);
        fclose(file);
        return NULL;
    }

    // Allocate memory and read file
    char* jsonString = malloc(size + 1);
    if (!jsonString) {
        LOG_WARN("Memory allocation error\n");
        fclose(file);
        return NULL;
    }

    size_t bytesRead = fread(jsonString, 1, size, file);
    fclose(file);

    if (bytesRead != (size_t)size) {
        LOG_WARN("Read error in %s\n", filepath);
        free(jsonString);
        return NULL;
    }

    jsonString[bytesRead] = '\0';
    cJSON* object = cJSON_Parse(jsonString);
    free(jsonString);

    if (!object) {
        LOG_WARN("JSON Parse error for %s\n", filepath);
        return NULL;
    }

    return object;
}

int ACAP_FILE_Write(const char* filepath, cJSON* object) {
    if (!filepath || !object) {
        LOG_WARN("Invalid parameters for file write\n");
        return 0;
    }

    FILE* file = ACAP_FILE_Open(filepath, "w");
    if (!file) {
        LOG_WARN("Error opening %s for writing\n", filepath);
        return 0;
    }

    char* jsonString = cJSON_Print(object);
    if (!jsonString) {
        LOG_WARN("JSON serialization error for %s\n", filepath);
        fclose(file);
        return 0;
    }

    int result = fputs(jsonString, file);
    free(jsonString);
    fclose(file);

    if (result < 0) {
        LOG_WARN("Could not save data to %s\n", filepath);
        return 0;
    }

    return 1;
}

/*------------------------------------------------------------------
 * Events System Implementation
 *------------------------------------------------------------------*/

ACAP_EVENTS_Callback EVENT_USER_CALLBACK = 0;
static void ACAP_EVENTS_Main_Callback(guint subscription, AXEvent *event, gpointer user_data);
cJSON* ACAP_EVENTS_SUBSCRIPTIONS = 0;
cJSON* ACAP_EVENTS_DECLARATIONS = 0;
AXEventHandler *ACAP_EVENTS_HANDLER = 0;


double ioFilterTimestamp0 = 0;
double ioFilterTimestamp1 = 0;
double ioFilterTimestamp2 = 0;
double ioFilterTimestamp3 = 0;

char ACAP_EVENTS_PACKAGE[64];
char ACAP_EVENTS_APPNAME[64];


cJSON* SubscriptionsArray = 0;
typedef struct S_AXEventKeyValueSet {
	GHashTable *key_values;
} T_ValueSet;

typedef struct S_NamespaceKeyPair {
  gchar *name_space;
  gchar *key;
} T_KeyPair;

typedef struct S_ValueElement {
  gint int_value;
  gboolean bool_value;
  gdouble double_value;
  gchar *str_value;
  AXEventElementItem *elem_value;
  gchar *elem_str_value;
  GList *tags;
  gchar *key_nice_name;
  gchar *value_nice_name;

  gboolean defined;
  gboolean onvif_data;

  AXEventValueType value_type;
} T_ValueElement;


cJSON*
ACAP_EVENTS() {
	LOG_TRACE("%s:\n",__func__);
	//Get the ACAP package ID and Nice Name
	cJSON* manifest = ACAP_Get_Config("manifest");
	if(!manifest)
		return cJSON_CreateNull();
	cJSON* acapPackageConf = cJSON_GetObjectItem(manifest,"acapPackageConf");
	if(!acapPackageConf)
		return cJSON_CreateNull();
	cJSON* setup = cJSON_GetObjectItem(acapPackageConf,"setup");
	if(!setup)
		return cJSON_CreateNull();
	sprintf(ACAP_EVENTS_PACKAGE,"%s", cJSON_GetObjectItem(setup,"appName")->valuestring);
	sprintf(ACAP_EVENTS_APPNAME,"%s", cJSON_GetObjectItem(setup,"friendlyName")->valuestring);
	
	LOG_TRACE("%s: %s %s\n",__func__,ACAP_EVENTS_PACKAGE,ACAP_EVENTS_APPNAME);

	ACAP_EVENTS_SUBSCRIPTIONS = cJSON_CreateArray();
	ACAP_EVENTS_DECLARATIONS = cJSON_CreateObject();
	ACAP_EVENTS_HANDLER = ax_event_handler_new();
	
	cJSON* events = ACAP_FILE_Read( "settings/events.json" );
	if(!events)
		LOG_WARN("Cannot load even event list\n")
	cJSON* event = events?events->child:0;
	while( event ) {
		ACAP_EVENTS_Add_Event_JSON( event );
		event = event->next;
	}
	return events;
}

int
ACAP_EVENTS_SetCallback( ACAP_EVENTS_Callback callback ){
	LOG_TRACE("%s: Entry\n",__func__);
	EVENT_USER_CALLBACK = callback;
	return 1;
}

cJSON*
ACAP_EVENTS_Parse( AXEvent *axEvent ) {
	LOG_TRACE("%s: Entry\n",__func__);
	
	const T_ValueSet *set = (T_ValueSet *)ax_event_get_key_value_set(axEvent);
	
	cJSON *object = cJSON_CreateObject();
	GHashTableIter iter;
	T_KeyPair *nskp;
	T_ValueElement *value_element;
	char topics[6][32];
	char topic[200];
	int i;
	for(i=0;i<6;i++)
		topics[i][0]=0;
	
	g_hash_table_iter_init(&iter, set->key_values);
//	LOG_TRACE("ACAP_EVENTS_ParsePayload:\n");
	while (g_hash_table_iter_next(&iter, (gpointer*)&nskp,(gpointer*)&value_element)) {
		int isTopic = 0;
		
		if( strcmp(nskp->key,"topic0") == 0 ) {
//			LOG_TRACE("Parse: Topic 0: %s\n",(char*)value_element->str_value);
			if( strcmp((char*)value_element->str_value,"CameraApplicationPlatform") == 0 )
				sprintf(topics[0],"acap");
			else
				sprintf(topics[0],"%s",(char*)value_element->str_value);
			isTopic = 1;
		}
		if( strcmp(nskp->key,"topic1") == 0 ) {
//			LOG_TRACE("Parse: Topic 1: %s\n",(char*)value_element->str_value);
			sprintf(topics[1],"%s",value_element->str_value);
			isTopic = 1;
		}
		if( strcmp(nskp->key,"topic2") == 0 ) {
//			LOG_TRACE("Parse: Topic 2: %s\n",(char*)value_element->str_value);
			sprintf(topics[2],"%s",value_element->str_value);
			isTopic = 1;
		}
		if( strcmp(nskp->key,"topic3") == 0 ) {
//			LOG_TRACE("Parse: Topic 3: %s\n",(char*)value_element->str_value);
			sprintf(topics[3],"%s",value_element->str_value);
			isTopic = 1;
		}
		if( strcmp(nskp->key,"topic4") == 0 ) {
			sprintf(topics[4],"%s",value_element->str_value);
			isTopic = 1;
		}
		if( strcmp(nskp->key,"topic5") == 0 ) {
			sprintf(topics[5],"%s",value_element->str_value);
			isTopic = 1;
		}
		
		if( isTopic == 0 ) {
			if (value_element->defined) {
				switch (value_element->value_type) {
					case AX_VALUE_TYPE_INT:
//						LOG_TRACE("Parse: Int %s = %d\n", nskp->key, value_element->int_value);
						cJSON_AddNumberToObject(object,nskp->key,(double)value_element->int_value);
					break;

					case AX_VALUE_TYPE_BOOL:
						LOG_TRACE("Bool %s = %d\n", nskp->key, value_element->bool_value);
//						if( strcmp(nskp->key,"state") != 0 )
//							cJSON_AddNumberToObject(object,nskp->key,(double)value_element->bool_value);
//						if( !value_element->bool_value )
//							cJSON_ReplaceItemInObject(object,"state",cJSON_CreateFalse());
						cJSON_AddBoolToObject(object,nskp->key,value_element->bool_value);
					break;

					case AX_VALUE_TYPE_DOUBLE:
						LOG_TRACE("Parse: Double %s = %f\n", nskp->key, value_element->double_value);
						cJSON_AddNumberToObject(object,nskp->key,(double)value_element->double_value);
					break;

					case AX_VALUE_TYPE_STRING:
						LOG_TRACE("Parse: String %s = %s\n", nskp->key, value_element->str_value);
						cJSON_AddStringToObject(object,nskp->key, value_element->str_value);
					break;

					case AX_VALUE_TYPE_ELEMENT:
						LOG_TRACE("Parse: Element %s = %s\n", nskp->key, value_element->str_value);
						cJSON_AddStringToObject(object,nskp->key, value_element->elem_str_value);
					break;

					default:
						LOG_TRACE("Parse: Undefined %s = %s\n", nskp->key, value_element->str_value);
						cJSON_AddNullToObject(object,nskp->key);
					break;
				}
			}
		}
	}
	strcpy(topic,topics[0]);
	for(i=1;i<6;i++) {
		if( strlen(topics[i]) > 0 ) {
			strcat(topic,"/");
			strcat(topic,topics[i]);
		}
	}

	//Special Device Event Filter
	if( strcmp(topic,"Device/IO/Port") == 0 ) {
		int port = cJSON_GetObjectItem(object,"port")?cJSON_GetObjectItem(object,"port")->valueint:-1;
		if( port == -1 ) {
			cJSON_Delete(object);
			return 0;
		}
		if( port == 0 ) {
			if( ACAP_DEVICE_Timestamp() - ioFilterTimestamp0 < 1000 ) {
				cJSON_Delete(object);
				return 0;
			}
			ioFilterTimestamp0 = ACAP_DEVICE_Timestamp();
		}
		
		if( port == 1 ) {
			if( ACAP_DEVICE_Timestamp() - ioFilterTimestamp1 < 1000 ) {
				cJSON_Delete(object);
				return 0;
			}
			ioFilterTimestamp1 = ACAP_DEVICE_Timestamp();
		}
		
		if( port == 2 ) {
			if( ACAP_DEVICE_Timestamp() - ioFilterTimestamp2 < 1000 ) {
				cJSON_Delete(object);
				return 0;
			}
			ioFilterTimestamp2 = ACAP_DEVICE_Timestamp();
		}

		if( port == 3 ) {
			if( ACAP_DEVICE_Timestamp() - ioFilterTimestamp3 < 1000 ) {
				cJSON_Delete(object);
				return 0;
			}
			ioFilterTimestamp3 = ACAP_DEVICE_Timestamp();
		}
	}

	cJSON_AddStringToObject(object,"event",topic);
	return object;
}


// gpointer points to the name used when subscribing to event
static void
ACAP_EVENTS_Main_Callback(guint subscription, AXEvent *axEvent, gpointer user_data) {
	LOG_TRACE("%s:\n",__func__);

	cJSON* eventData = ACAP_EVENTS_Parse(axEvent);
	if( !eventData )
		return;
	cJSON_AddItemReferenceToObject(eventData, "source", (cJSON*)user_data);	
	if( EVENT_USER_CALLBACK )
		EVENT_USER_CALLBACK( eventData, (void*)user_data );
	cJSON_Delete(eventData);
}

/*
event structure
{
	"name": "All ACAP Events",
	"topic0": {"tnsaxis":"CameraApplicationPlatform"},
	"topic1": {"tnsaxis":"someService"},
	"topic2": {"tnsaxis":"someEvent"}
}
 
*/

int
ACAP_EVENTS_Subscribe( cJSON *event, void* user_data ) {
	AXEventKeyValueSet *keyset = 0;	
	cJSON *topic;
	guint declarationID = 0;

	if(!ACAP_EVENTS_HANDLER) {
		LOG_WARN("%s: Event handler not initialize\n",__func__);
		return 0;
	}

	char *json = cJSON_PrintUnformatted(event);
	if( json ) {
		LOG_TRACE("%s: %s\n",__func__,json);
		free(json);
	}

	if( !event ) {
		LOG_WARN("%s: Invalid event\n",__func__);
		return 0;
	}


	if( !cJSON_GetObjectItem( event,"name" ) || !cJSON_GetObjectItem( event,"name" )->valuestring || !strlen(cJSON_GetObjectItem( event,"name" )->valuestring) ) {
		LOG_WARN("%s: Event declaration is missing name\n",__func__);
		return 0;
	}

	if( !cJSON_GetObjectItem( event,"topic0" ) ) {
		LOG_WARN("%s: Event declaration is missing topic0\n",__func__);
		return 0;
	}
	
	keyset = ax_event_key_value_set_new();
	if( !keyset ) {
		LOG_WARN("ACAP_EVENTS_Subscribe: Unable to create keyset\n");
		return 0;
	}


	// ----- TOPIC 0 ------
	topic = cJSON_GetObjectItem( event,"topic0" );
	if(!topic) {
		LOG_WARN("ACAP_EVENTS_Subscribe: Invalid tag for topic 0");
		return 0;
	}
	if( !ax_event_key_value_set_add_key_value( keyset, "topic0", topic->child->string, topic->child->valuestring, AX_VALUE_TYPE_STRING,NULL) ) {
		LOG_WARN("ACAP_EVENTS_Subscribe: Topic 0 keyset error");
		ax_event_key_value_set_free(keyset);
		return 0;
	}
	LOG_TRACE("%s: topic0 %s:%s\n",__func__, topic->child->string, topic->child->valuestring );
	
	// ----- TOPIC 1 ------
	if( cJSON_GetObjectItem( event,"topic1" ) ) {
		topic = cJSON_GetObjectItem( event,"topic1" );
		if( !ax_event_key_value_set_add_key_value( keyset, "topic1", topic->child->string, topic->child->valuestring, AX_VALUE_TYPE_STRING,NULL) ) {
			LOG_WARN("ACAP_EVENTS_Subscribe: Unable to subscribe to event (1)");
			ax_event_key_value_set_free(keyset);
			return 0;
		}
		LOG_TRACE("%s: topic1 %s:%s\n",__func__, topic->child->string, topic->child->valuestring );
	}
	//------ TOPIC 2 -------------
	if( cJSON_GetObjectItem( event,"topic2" ) ) {
		topic = cJSON_GetObjectItem( event,"topic2" );
		if( !ax_event_key_value_set_add_key_value( keyset, "topic2", topic->child->string, topic->child->valuestring, AX_VALUE_TYPE_STRING,NULL) ) {
			LOG_WARN("ACAP_EVENTS_Subscribe: Unable to subscribe to event (2)");
			ax_event_key_value_set_free(keyset);
			return 0;
		}
		LOG_TRACE("%s: topic2 %s:%s\n",__func__, topic->child->string, topic->child->valuestring );
	}
	
	if( cJSON_GetObjectItem( event,"topic3" ) ) {
		LOG_TRACE("%s: topic3:%s:%s", __func__,topic->child->string,topic->child->valuestring);
		topic = cJSON_GetObjectItem( event,"topic3" );
		if( !ax_event_key_value_set_add_key_value( keyset, "topic3", topic->child->string, topic->child->valuestring, AX_VALUE_TYPE_STRING,NULL) ) {
			LOG_WARN("ACAP_EVENTS_Subscribe: Unable to subscribe to event (3)");
			ax_event_key_value_set_free(keyset);
			return 0;
		}
		LOG_TRACE("%s: topic3 %s %s\n",__func__, topic->child->string, topic->child->valuestring );
	}

	int ax = ax_event_handler_subscribe(
		ACAP_EVENTS_HANDLER, 
		keyset, 
		&declarationID,
		ACAP_EVENTS_Main_Callback,
		(gpointer)user_data,
		NULL
	);
	
	ax_event_key_value_set_free(keyset);
	if( !ax ) {
		LOG_WARN("ACAP_EVENTS_Subscribe: Unable to subscribe to event\n");
		return 0;
	}
	cJSON_AddItemToArray(ACAP_EVENTS_SUBSCRIPTIONS,cJSON_CreateNumber(declarationID));
	return declarationID;
}

int ACAP_EVENTS_Unsubscribe(int id) {
    LOG_TRACE("%s: Unsubscribing id=%d\n", __func__, id);
    
    if (!ACAP_EVENTS_SUBSCRIPTIONS) {
        return 0;
    }

    if (id == 0) {  // Unsubscribe all
        cJSON* event = ACAP_EVENTS_SUBSCRIPTIONS->child;
        while (event) {
            ax_event_handler_unsubscribe(ACAP_EVENTS_HANDLER, (guint)event->valueint, 0);
            event = event->next;
        }
        cJSON_Delete(ACAP_EVENTS_SUBSCRIPTIONS);
        ACAP_EVENTS_SUBSCRIPTIONS = cJSON_CreateArray();
    } else {
        // Find and remove specific subscription
        cJSON* event = ACAP_EVENTS_SUBSCRIPTIONS->child;
        cJSON* prev = NULL;
        
        while (event) {
            if (event->valueint == id) {
                ax_event_handler_unsubscribe(ACAP_EVENTS_HANDLER, (guint)id, 0);
                if (prev) {
                    prev->next = event->next;
                } else {
                    ACAP_EVENTS_SUBSCRIPTIONS->child = event->next;
                }
                cJSON_Delete(event);
                break;
            }
            prev = event;
            event = event->next;
        }
    }
    
    return 1;
}


int
ACAP_EVENTS_Add_Event( const char *id, const char* name, int state ) {
	AXEventKeyValueSet *set = NULL;
	int dummy_value = 0;
	guint declarationID;

	if( !ACAP_EVENTS_HANDLER || !id || !name || !ACAP_EVENTS_DECLARATIONS) {
		LOG_WARN("ACAP_EVENTS_Add_Event: Invalid input\n");
		return 0;
	}

	set = ax_event_key_value_set_new();
	
	ax_event_key_value_set_add_key_value( set, "topic0", "tnsaxis", "CameraApplicationPlatform", AX_VALUE_TYPE_STRING,NULL);
	ax_event_key_value_set_add_key_value( set, "topic1", "tnsaxis", ACAP_EVENTS_PACKAGE , AX_VALUE_TYPE_STRING,NULL);
	ax_event_key_value_set_add_nice_names( set, "topic1", "tnsaxis", ACAP_EVENTS_PACKAGE, ACAP_EVENTS_APPNAME, NULL);
	ax_event_key_value_set_add_key_value( set, "topic2", "tnsaxis", id, AX_VALUE_TYPE_STRING,NULL);
	ax_event_key_value_set_add_nice_names( set, "topic2", "tnsaxis", id, name, NULL);

	int ax = 0;
	if( state ) {
		ax_event_key_value_set_add_key_value(set,"state", NULL, &dummy_value, AX_VALUE_TYPE_BOOL,NULL);
		ax_event_key_value_set_mark_as_data(set, "state", NULL, NULL);
		ax = ax_event_handler_declare(ACAP_EVENTS_HANDLER, set, 0,&declarationID,NULL,NULL,NULL);
	} else {
		ax_event_key_value_set_add_key_value(set,"value", NULL, &dummy_value, AX_VALUE_TYPE_INT,NULL);
		ax_event_key_value_set_mark_as_data(set, "value", NULL, NULL);
		ax = ax_event_handler_declare(ACAP_EVENTS_HANDLER, set, 1,&declarationID,NULL,NULL,NULL);
	}

	if( !ax ) {
		LOG_WARN("Error declaring event\n");
		ax_event_key_value_set_free(set);
		return 0;
	}
	LOG_TRACE("%s: %s %s %s\n",__func__,id, name, state?"Stateful":"Stateless");

	cJSON_AddNumberToObject(ACAP_EVENTS_DECLARATIONS,id,declarationID);
	ax_event_key_value_set_free(set);
	return declarationID;
}	

int
ACAP_EVENTS_Remove_Event(const char *id ) {
	LOG_TRACE("%s: %s",__func__,id);
	if( !ACAP_EVENTS_HANDLER || !id || !ACAP_EVENTS_DECLARATIONS) {
		LOG_TRACE("ACAP_EVENTS_Remove_Event: Invalid input\n");
		return 0;
	}
	
	cJSON *event = cJSON_DetachItemFromObject(ACAP_EVENTS_DECLARATIONS,id);
	if( !event ) {
		LOG_WARN("Error remving event %s.  Event not found\n",id);
		return 0;
	}

	ax_event_handler_undeclare( ACAP_EVENTS_HANDLER, event->valueint, NULL);
	cJSON_Delete( event);
	return 1;
}


int
ACAP_EVENTS_Fire( const char* id ) {
	GError *error = NULL;
	
	LOG_TRACE("%s: %s\n", __func__, id );
	
	if( !ACAP_EVENTS_DECLARATIONS || !id ) {
		LOG_WARN("EVENTs_Fire_State: Error send event\n");
		return 0;
	}

	cJSON *event = cJSON_GetObjectItem(ACAP_EVENTS_DECLARATIONS, id );
	if(!event) {
		LOG_WARN("%s: Event %s not found\n",__func__, id);
		return 0;
	}

	AXEventKeyValueSet* set = ax_event_key_value_set_new();

	guint value = 1;
	if( !ax_event_key_value_set_add_key_value(set,"value",NULL, &value,AX_VALUE_TYPE_INT,&error) ) {
		ax_event_key_value_set_free(set);
		LOG_WARN("%s: %s error %s\n", __func__,id,error->message);
		g_error_free(error);
		return 0;
	}

	AXEvent* axEvent = ax_event_new2(set,NULL);
	ax_event_key_value_set_free(set);

	if( !ax_event_handler_send_event(ACAP_EVENTS_HANDLER, event->valueint, axEvent, &error) )  {
		LOG_WARN("%s: Could not send event %s %s\n",__func__, id, error->message);
		ax_event_free(axEvent);
		g_error_free(error);
		return 0;
	}
	ax_event_free(axEvent);
	LOG_TRACE("%s: %s fired %d\n",__func__,  id,value );
	return 1;
}

int
ACAP_EVENTS_Fire_State( const char* id, int value ) {
	if( value && ACAP_STATUS_Bool("events",id) )
		return 1;  //The state is already high
	if( !value && !ACAP_STATUS_Bool("events",id) )
		return 1;  //The state is already low

	LOG_TRACE("%s: %s %d\n", __func__, id, value );
	
	if( !ACAP_EVENTS_DECLARATIONS || !id ) {
		LOG_WARN("EVENTs_Fire_State: Error send event\n");
		return 0;
	}

	cJSON *event = cJSON_GetObjectItem(ACAP_EVENTS_DECLARATIONS, id );
	if(!event) {
		LOG_WARN("Error sending event %s.  Event not found\n", id);
		return 0;
	}

	AXEventKeyValueSet* set = ax_event_key_value_set_new();

	if( !ax_event_key_value_set_add_key_value(set,"state",NULL , &value,AX_VALUE_TYPE_BOOL,NULL) ) {
		ax_event_key_value_set_free(set);
		LOG_WARN("EVENT_Fire_State: Could not send event %s.  Internal error\n", id);
		return 0;
	}

	AXEvent* axEvent = ax_event_new2(set, NULL);
	ax_event_key_value_set_free(set);

	if( !ax_event_handler_send_event(ACAP_EVENTS_HANDLER, event->valueint, axEvent, NULL) )  {
		LOG_WARN("EVENT_Fire_State: Could not send event %s\n", id);
		ax_event_free(axEvent);
		return 0;
	}
	ax_event_free(axEvent);
	ACAP_STATUS_SetBool("events",id,value);
	LOG_TRACE("EVENT_Fire_State: %s %d fired\n", id,value );
	return 1;
}


int
ACAP_EVENTS_Fire_JSON( const char* Id, cJSON* data ) {
	AXEventKeyValueSet *set = NULL;
	int boolValue;
	GError *error = NULL;
	if(!data) {
		LOG_WARN("%s: Invalid data",__func__);
		return 0;
	}

	cJSON *event = cJSON_GetObjectItem(ACAP_EVENTS_DECLARATIONS, Id );
	if(!event) {
		LOG_WARN("%s: Error sending event %s.  Event not found\n",__func__,Id);
		return 0;
	}
	

	set = ax_event_key_value_set_new();
	int success = 0;
	cJSON* property = data->child;
	while(property) {
		if(property->type == cJSON_True ) {
			boolValue = 1;
			success = ax_event_key_value_set_add_key_value(set,property->string,NULL , &boolValue,AX_VALUE_TYPE_BOOL,NULL);
			LOG_TRACE("%s: Set %s %s = True\n",__func__,Id,property->string);
		}
		if(property->type == cJSON_False ) {
			boolValue = 0;
			success = ax_event_key_value_set_add_key_value(set,property->string,NULL , &boolValue,AX_VALUE_TYPE_BOOL,NULL);
			LOG_TRACE("%s: Set %s %s = False\n",__func__,Id,property->string);
		}
		if(property->type == cJSON_String ) {
			success = ax_event_key_value_set_add_key_value(set,property->string,NULL , property->valuestring,AX_VALUE_TYPE_STRING,NULL);
			LOG_TRACE("%s: Set %s %s = %s\n",__func__,Id,property->string,property->valuestring);
		}
		if(property->type == cJSON_Number ) {
			success = ax_event_key_value_set_add_key_value(set,property->string,NULL , &property->valuedouble,AX_VALUE_TYPE_DOUBLE,NULL);
			LOG_TRACE("%s: Set %s %s = %f\n",__func__,Id,property->string,property->valuedouble);
		}
		if(!success)
			LOG_WARN("%s: Unable to add property\n",__func__);
		property = property->next;
	}

	AXEvent* axEvent = ax_event_new2(set, NULL);
	success = ax_event_handler_send_event(ACAP_EVENTS_HANDLER, event->valueint, axEvent, &error);
	ax_event_key_value_set_free(set);
	ax_event_free(axEvent);
	if(!success)  {
		LOG_WARN("%s: Could not send event %s id = %d %s\n",__func__, Id, event->valueint, error->message);
		g_error_free(error);
		return 0;
	}
	return 1;
}

int
ACAP_EVENTS_Add_Event_JSON( cJSON* event ) {
	AXEventKeyValueSet *set = NULL;
	GError *error = NULL;
	guint declarationID;
	int success = 0;	
	set = ax_event_key_value_set_new();
	
	char *eventID = cJSON_GetObjectItem(event,"id")?cJSON_GetObjectItem(event,"id")->valuestring:0;
	char *eventName = cJSON_GetObjectItem(event,"name")?cJSON_GetObjectItem(event,"name")->valuestring:0;
	
	if(!eventID) {
		LOG_WARN("%s: Invalid id\n",__func__);
		return 0;
	}

	ax_event_key_value_set_add_key_value( set, "topic0", "tnsaxis", "CameraApplicationPlatform", AX_VALUE_TYPE_STRING,NULL);
	ax_event_key_value_set_add_key_value( set, "topic1", "tnsaxis", ACAP_EVENTS_PACKAGE , AX_VALUE_TYPE_STRING,NULL);
	ax_event_key_value_set_add_nice_names( set, "topic1", "tnsaxis", ACAP_EVENTS_PACKAGE, ACAP_EVENTS_APPNAME, NULL);
	ax_event_key_value_set_add_key_value( set, "topic2", "tnsaxis", eventID, AX_VALUE_TYPE_STRING,NULL);
	if( eventName )
		ax_event_key_value_set_add_nice_names( set, "topic2", "tnsaxis", eventID, eventName, NULL);

	if( cJSON_GetObjectItem(event,"show") && cJSON_GetObjectItem(event,"show")->type == cJSON_False )
		ax_event_key_value_set_mark_as_user_defined(set,eventID,"tnsaxis","isApplicationData",NULL);

	int defaultInt = 0;
	double defaultDouble = 0;
	char defaultString[] = "";
	cJSON* source = cJSON_GetObjectItem(event,"source")?cJSON_GetObjectItem(event,"source")->child:0;
	while( source ) {
		cJSON* property = source->child;
		if( property ) {
			if(strcmp(property->valuestring,"string") == 0 )
				success = ax_event_key_value_set_add_key_value(set,property->string,NULL, &defaultString,AX_VALUE_TYPE_STRING,NULL);
			if(strcmp(property->valuestring,"int") == 0 )
				success = ax_event_key_value_set_add_key_value(set,property->string,NULL, &defaultInt,AX_VALUE_TYPE_INT,NULL);
			if(strcmp(property->valuestring,"bool") == 0 )
				success = ax_event_key_value_set_add_key_value(set,property->string,NULL, &defaultInt,AX_VALUE_TYPE_BOOL,NULL);
			ax_event_key_value_set_mark_as_source(set, property->string, NULL, NULL);
			LOG_TRACE("%s: %s Source %s %s\n",__func__,eventID,property->string,property->valuestring);
		}
		source = source->next;
	}

	cJSON* data = cJSON_GetObjectItem(event,"data")?cJSON_GetObjectItem(event,"data")->child:0;
	
	int propertyCounter = 0;
	while( data ) {
		cJSON* property = data->child;
		if( property ) {
			propertyCounter++;
			if(strcmp(property->valuestring,"string") == 0 ) {
				if( !ax_event_key_value_set_add_key_value(set,property->string,NULL, &defaultString,AX_VALUE_TYPE_STRING,&error) ) {
					LOG_WARN("%s: Unable to add string %s %s\n",__func__,property->string,error->message);
					g_error_free(error);
				} else {
					LOG_TRACE("%s: Added string %s\n",__func__,property->string);
				}
			}
			if(strcmp(property->valuestring,"int") == 0 )
				ax_event_key_value_set_add_key_value(set,property->string,NULL, &defaultInt,AX_VALUE_TYPE_INT,NULL);
			if(strcmp(property->valuestring,"double") == 0 )
				ax_event_key_value_set_add_key_value(set,property->string,NULL, &defaultDouble,AX_VALUE_TYPE_DOUBLE,NULL);
			if(strcmp(property->valuestring,"bool") == 0 )
				ax_event_key_value_set_add_key_value(set,property->string,NULL, &defaultInt,AX_VALUE_TYPE_BOOL,NULL);
			ax_event_key_value_set_mark_as_data(set, property->string, NULL, NULL);
			LOG_TRACE("%s: %s Data %s %s\n",__func__,eventID,property->string,property->valuestring);
		}
		data = data->next;
	}

	if( propertyCounter == 0 )  //An empty event requires at least one property
		ax_event_key_value_set_add_key_value(set,"value",NULL, &defaultInt,AX_VALUE_TYPE_INT,NULL);
	
	if( cJSON_GetObjectItem(event,"state") && cJSON_GetObjectItem(event,"state")->type == cJSON_True ) {
		ax_event_key_value_set_add_key_value(set,"state",NULL, &defaultInt,AX_VALUE_TYPE_BOOL,NULL);
		ax_event_key_value_set_mark_as_data(set, "state", NULL, NULL);
		LOG_TRACE("%s: %s is stateful\n",__func__,eventID);
		success = ax_event_handler_declare(ACAP_EVENTS_HANDLER, set, 0,&declarationID,NULL,NULL,NULL);
	} else {
		success = ax_event_handler_declare(ACAP_EVENTS_HANDLER, set, 1,&declarationID,NULL,NULL,NULL);
	}
	LOG_TRACE("%s: %s ID = %d\n",__func__,eventID,declarationID);
	if( !success ) {
		ax_event_key_value_set_free(set);
		LOG_WARN("Unable to register event %s\n",eventID);
		return 0;
	}
	cJSON_AddNumberToObject(ACAP_EVENTS_DECLARATIONS,eventID,declarationID);
	ax_event_key_value_set_free(set);
	return declarationID;
}

/*------------------------------------------------------------------
 * Cleanup Implementation
 *------------------------------------------------------------------*/

void ACAP_Cleanup(void) {
    // Clean up other resources
    ACAP_HTTP_Cleanup();
	
    if (status_container) {
        cJSON_Delete(status_container);
        status_container = NULL;
    }

	LOG_TRACE("%s:",__func__);
    if (app) {
        cJSON_Delete(app);
        app = NULL;
    }
    
    if (ACAP_DEVICE_Container) {
        cJSON_Delete(ACAP_DEVICE_Container);
        ACAP_DEVICE_Container = NULL;
    }
    
    ACAP_UpdateCallback = NULL;
}

/*------------------------------------------------------------------
 * Error Handling Implementation
 *------------------------------------------------------------------*/

const char* ACAP_Get_Error_String(ACAP_Status status) {
    switch (status) {
        case ACAP_SUCCESS:
            return "Success";
        case ACAP_ERROR_INIT:
            return "Initialization error";
        case ACAP_ERROR_PARAM:
            return "Invalid parameter";
        case ACAP_ERROR_MEMORY:
            return "Memory allocation error";
        case ACAP_ERROR_IO:
            return "I/O error";
        case ACAP_ERROR_HTTP:
            return "HTTP error";
        default:
            return "Unknown error";
    }
}

/*------------------------------------------------------------------
 * Helper functions
 *------------------------------------------------------------------*/

cJSON* SplitString(const char* input, const char* delimiter) {
    if (!input || !delimiter) {
        return NULL; // Invalid input
    }

    // Create a cJSON array to hold the split strings
    cJSON* json_array = cJSON_CreateArray();
    if (!json_array) {
        return NULL; // Failed to create JSON array
    }

    // Make a copy of the input string because strtok modifies it
    char* input_copy = strdup(input);
    if (!input_copy) {
        cJSON_Delete(json_array);
        return NULL; // Memory allocation failure
    }

    // Remove trailing newline character, if present
    size_t len = strlen(input_copy);
    if (len > 0 && input_copy[len - 1] == '\n') {
        input_copy[len - 1] = '\0';
    }

    // Use strtok to split the string by the delimiter
    char* token = strtok(input_copy, delimiter);
    while (token != NULL) {
        // Add each token to the JSON array
        cJSON* json_token = cJSON_CreateString(token);
        if (!json_token) {
            free(input_copy);
            cJSON_Delete(json_array);
            return NULL; // Memory allocation failure for JSON string
        }
        cJSON_AddItemToArray(json_array, json_token);

        // Get the next token
        token = strtok(NULL, delimiter);
    }

    free(input_copy); // Free the temporary copy of the input string
    return json_array;
}

char**
string_split( char* a_str,  char a_delim) {
    char** result    = 0;
    size_t count     = 0;
    const char* tmp  = a_str;
    const char* last_comma = 0;
    char delim[2];
    delim[0] = a_delim;
    delim[1] = 0;
    /* Count how many elements will be extracted. */
    while (*tmp)
    {
        if (a_delim == *tmp)
        {
            count++;
            last_comma = tmp;
        }
        tmp++;
    }

    /* Add space for trailing token. */
    count += last_comma < (a_str + strlen(a_str) - 1);

    /* Add space for terminating null string so caller
       knows where the list of returned strings ends. */
    count++;
    result = malloc(sizeof(char*) * count);
    if (result) {
        size_t idx  = 0;
        char* token = strtok(a_str, delim);
        while (token) {
//            assert(idx < count);
            *(result + idx++) = strdup(token);
            token = strtok(0, delim);
        }
//        assert(idx == count - 1);
        *(result + idx) = 0;
    }
    return result;
}

