/*------------------------------------------------------------------
 *  Copyright Fred Juhlin (2021)
 *------------------------------------------------------------------*/
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include "ACAP.h"

#define LOG(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_WARN(fmt, args...)    { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args); }
//#define LOG_TRACE(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_TRACE(fmt, args...)    {}


cJSON *CERTS_SETTINGS = 0;
char CERTS_CA_STORE[] = "/etc/ssl/certs/ca-certificates.crt";
char CERTS_LOCAL_PATH[256];

char*
CERTS_Get_CA() {
	if( !CERTS_SETTINGS ) {
		LOG_WARN("CERTS: Not initialized");
		return CERTS_CA_STORE;
	}
	cJSON* path = cJSON_GetObjectItem(CERTS_SETTINGS,"cafile");
	if( !path ) {
		LOG_TRACE("CERTS_CAPath: %s\n", CERTS_SETTINGS);
		return CERTS_CA_STORE;
	}
	LOG_TRACE("CERTS_CAPath: %s\n", path->valuestring);
	return path->valuestring;
}

char*
CERTS_Get_Cert() {
	if( !CERTS_SETTINGS ) {
		LOG_WARN("CERTS is not initiailiaized\n");
		return 0;
	}
	cJSON* cert = cJSON_GetObjectItem(CERTS_SETTINGS,"certfile");
	cJSON* key = cJSON_GetObjectItem(CERTS_SETTINGS,"keyfile");
	if( !cert || !key ) {
		LOG_TRACE("CERTS_CertFile: 0\n");
		return 0;
	}
	LOG_TRACE("CERTS_CertFile: %s\n", cert->valuestring);
	return cert->valuestring;
}

char*
CERTS_Get_Key() {
	if( !CERTS_SETTINGS ) {
		LOG_WARN("CERTS is not initiailiaized\n");
		return 0;
	}
	cJSON* cert = cJSON_GetObjectItem(CERTS_SETTINGS,"certfile");
	cJSON* key = cJSON_GetObjectItem(CERTS_SETTINGS,"keyfile");
	if( !cert || !key ) {
		LOG_TRACE("CERTS_KeyFile: 0\n");
		return 0;
	}
	LOG_TRACE("CERTS_KeyFile: %s\n", key->valuestring);
	return key->valuestring;
}

char*
CERTS_Get_Password() {
	if( !CERTS_SETTINGS ) {
		LOG_WARN("CERTS is not initiailiaized\n");
		return 0;
	}
	cJSON* password = cJSON_GetObjectItem(CERTS_SETTINGS,"password");
	if( !password ) {
		LOG_TRACE("CERTS_Password: 0\n");
		return 0;
	}
	LOG_TRACE("CERTS_Password: %s\n", password->valuestring);
	return password->valuestring;
}

void
CERTS_HTTP (const  ACAP_HTTP_Response response,const  ACAP_HTTP_Request request) {
	const char *pem,*type;

	if( !CERTS_SETTINGS ) {
		LOG_WARN("CERTS is not initiailiaized\n");
		 ACAP_HTTP_Respond_Error( response, 400, "Certificate service is not initializied" );
		return ;
	}


	pem =  ACAP_HTTP_Request_Param( request, "pem");
	type =  ACAP_HTTP_Request_Param( request, "type");
	
	LOG_TRACE("%s: Start\n",__func__);
	
	if( !type || !pem ) {
		LOG_TRACE("CERTS_HTTP: No action.  Respond with data\n");
		cJSON* copy = cJSON_Duplicate( CERTS_SETTINGS,1 );
		if( cJSON_GetObjectItem( copy,"password") ) {
			cJSON_ReplaceItemInObject( copy,"password",cJSON_CreateString("") );
		}
		 ACAP_HTTP_Respond_JSON(response, copy );
		return;
	}

	if(!type) {
		LOG_WARN("CERT: Type is missing\n");
		 ACAP_HTTP_Respond_Error( response, 400, "Missing type" );
		return;
	}
	
	if(!pem) {
		LOG_WARN("CERT: PEM is missing\n");
		 ACAP_HTTP_Respond_Error( response, 400, "Missing pem" );
		return;
	}

	if( !(strcmp(type,"ca")==0 || strcmp(type,"cert")==0 || strcmp(type,"key")==0) ) {
		LOG_WARN("CERTS: Invalid type %s\n", type);
		 ACAP_HTTP_Respond_Error( response, 400, "Missing type" );
		return;
	}

	if( strcmp(type,"ca") == 0 ) {
		if( strlen(pem) < 500 ) {
			if( !cJSON_GetObjectItem(CERTS_SETTINGS,"cafile") ) {
				 ACAP_HTTP_Respond_Text(response,"OK");
				return;
			}
			if(  ACAP_FILE_Delete( "localdata/ca.pem" ) ) {
				cJSON_DeleteItemFromObject(CERTS_SETTINGS,"cafile");
				 ACAP_STATUS_SetBool("certificate","CA",0);
				 ACAP_HTTP_Respond_Text(response,"OK");
			} else {
				LOG_WARN("Unable to remove CA file\n");
				 ACAP_HTTP_Respond_Error(response,500,"Failed to remove");
				return;
			}
		}
		if( strstr( pem, "-----BEGIN CERTIFICATE-----") == 0 ) {
			 ACAP_HTTP_Respond_Error( response, 400, "Invalid PEM data" );
			return;
		}
	}

	if( strcmp(type,"cert") == 0 ) {
		if( strlen(pem) < 500 ) {
			if( !cJSON_GetObjectItem(CERTS_SETTINGS,"certfile") ) {
				 ACAP_HTTP_Respond_Text(response,"OK");
				return;
			}
			if(  ACAP_FILE_Delete( "localdata/cert.pem" ) ) {
				cJSON_DeleteItemFromObject(CERTS_SETTINGS,"certfile");
				if(  ACAP_FILE_Delete( "localdata/key.pem" ) ) {
					cJSON_DeleteItemFromObject(CERTS_SETTINGS,"keyfile");
					if( cJSON_GetObjectItem(CERTS_SETTINGS,"password") ) {
						 ACAP_FILE_Delete( "localdata/ph.txt" );
						cJSON_DeleteItemFromObject(CERTS_SETTINGS,"password");
					}
				}				
				 ACAP_STATUS_SetBool("certificate","cert", 0);
				 ACAP_HTTP_Respond_Text(response,"OK");
			} else {
				LOG_WARN("Unable to remove certificate file\n");
				 ACAP_HTTP_Respond_Error(response,500,"Failed to remove");
				return;
			}
		}
		if( strstr( pem, "-----BEGIN CERTIFICATE-----") == 0 ) {
			 ACAP_HTTP_Respond_Error( response, 400, "Invalid PEM data" );
			return;
		}
	}

	if( strcmp(type,"key")==0 ) {
		if( strlen(pem) < 500 ) {
			if( !cJSON_GetObjectItem(CERTS_SETTINGS,"keyfile") ) {
				 ACAP_HTTP_Respond_Text(response,"OK");
				return;
			}
			if(  ACAP_FILE_Delete( "localdata/key.pem" ) ) {
				 ACAP_FILE_Delete( "localdata/ph.txt" );
				cJSON_DeleteItemFromObject(CERTS_SETTINGS,"keyfile");
				if( cJSON_GetObjectItem(CERTS_SETTINGS,"password" ) ) {
					cJSON_DeleteItemFromObject(CERTS_SETTINGS,"password");
					 ACAP_STATUS_SetBool("certificate","password",0);
				}
				 ACAP_STATUS_SetBool("certificate","key",0);
				 ACAP_HTTP_Respond_Text(response,"OK");
			} else {
				LOG_WARN("Unable to remove key file\n");
				 ACAP_HTTP_Respond_Error(response,500,"Failed to remove");
				return;
			}
		}
		if( strstr( pem, "-----BEGIN RSA PRIVATE KEY-----") == 0 ) {
			 ACAP_HTTP_Respond_Error( response, 400, "Invalid PEM data" );
			return;
		}
	}
	
	LOG_TRACE("CERTS_HTTP: Updateing %s\n",type);

	char filepath[128];
	sprintf(filepath,"localdata/%s.pem",type);
	LOG_TRACE("CERTS_HTTP: Opening %d for writing\n",filepath);
	FILE* file =  ACAP_FILE_Open(filepath, "w" );
	if(!file) {
		LOG_WARN("CERTS: Cannot open %s for witing\n",filepath);
		 ACAP_HTTP_Respond_Error( response, 500, "Failed saving data");
		return;
	}
	size_t pemSize = strlen(pem);
	LOG_TRACE("CERTS_HTTP: Saving data size=%d\n",pemSize);
	size_t length = fwrite( pem, pemSize, 1, file );
	fclose(file);

	if( length < 1 ) {
		LOG_WARN("CERTS: Could not save cert data\n");
		 ACAP_HTTP_Respond_Error( response, 500, "Failed saving data" );
		return;
	} else {
		LOG_TRACE("CERTS_HTTP: Data saveed in %s\n",filepath);
	}
	char fullpath[256]="";
	sprintf(fullpath,"%s%s", ACAP_FILE_AppPath(),filepath);
	if( strcmp(type,"cert") == 0 ) {
		if( cJSON_GetObjectItem(CERTS_SETTINGS,"certfile") )
			cJSON_ReplaceItemInObject(CERTS_SETTINGS,"certfile",cJSON_CreateString(fullpath));
		else
			cJSON_AddStringToObject(CERTS_SETTINGS,"certfile",fullpath);
		 ACAP_STATUS_SetBool("certificate","cert",1);
	}

	if( strcmp(type,"key") == 0 ) {
		if( cJSON_GetObjectItem(CERTS_SETTINGS,"keyfile") )
			cJSON_ReplaceItemInObject(CERTS_SETTINGS,"keyfile",cJSON_CreateString(fullpath));
		else
			cJSON_AddStringToObject(CERTS_SETTINGS,"keyfile",fullpath);
		
		if( cJSON_GetObjectItem(CERTS_SETTINGS,"password") )
			cJSON_DeleteItemFromObject(CERTS_SETTINGS,"password");
		const char *password =  ACAP_HTTP_Request_Param( request, "password");
		if( password ) {
			LOG_TRACE("Key with password\n");
			cJSON_AddStringToObject(CERTS_SETTINGS,"password",password);
			 ACAP_STATUS_SetBool("certificate","password",1);
			
			FILE* file =  ACAP_FILE_Open("localdata/ph.txt", "w" );
			if(file) {
				LOG_TRACE("Saving password\n");
				size_t length = fwrite( password, sizeof(char), strlen(password), file );
				if( length < 1 )
					LOG_WARN("Could not save password\n");
				fclose(file);
			}
		} else {
			 ACAP_STATUS_SetBool("certificate","password",0);
			LOG_TRACE("No password set for key file\n");
		}
		 ACAP_STATUS_SetBool("certificate","key",1);
	}

	if( strcmp(type,"ca") == 0 ) {
		if( cJSON_GetObjectItem(CERTS_SETTINGS,"cafile") )
			cJSON_ReplaceItemInObject(CERTS_SETTINGS,"cafile",cJSON_CreateString(fullpath));
		else
			cJSON_AddStringToObject(CERTS_SETTINGS,"cafile",fullpath);
		 ACAP_STATUS_SetBool("certificate","ca",1);
	}
	 ACAP_HTTP_Respond_Text( response, "OK" );
}

int
CERTS_Init(){
	CERTS_SETTINGS = cJSON_CreateObject();

	char * buffer;
	size_t length;

	LOG_TRACE("CERTS_Init: Start\n");

	 ACAP_STATUS_SetBool("certificate","ca",0);
	 ACAP_STATUS_SetBool("certificate","cert",0);
	 ACAP_STATUS_SetBool("certificate","key",0);
	 ACAP_STATUS_SetBool("certificate","password",0);

	FILE* file =  ACAP_FILE_Open("localdata/cert.pem", "r" );
	if(file) {
		LOG_TRACE("Loading cert.pem\n");
		fseek(file , 0 , SEEK_END);
		long lSize = ftell(file);
		if( lSize < 9000 ) {
			buffer = (char*) malloc (sizeof(char)*lSize);
			if(!buffer) {
				fclose(file);
				LOG_WARN("CERT:  Memory allocation error\n");
				return 0;
			}	
			rewind(file);
			length = fread ( buffer, sizeof(char), lSize, file );
			if( length > 50 ) {
				if( strstr( buffer, "-----BEGIN CERTIFICATE-----") > 0 ) {
					char filepath[256];
					sprintf(filepath,"%s%s", ACAP_FILE_AppPath(),"localdata/cert.pem");
					cJSON_AddStringToObject(CERTS_SETTINGS, "certfile", filepath );
					LOG_TRACE("Client certificate loaded\n");
					 ACAP_STATUS_SetBool("certificate","cert",0);
				} else {
					LOG_WARN("Could not load certificate file.  File corrupt or empty\n");
				}
			} else {
				LOG_WARN("Could not load certificate file.  File corrupt or empty\n");
			}
			free(buffer);
		} else {
			LOG_WARN("Could not load certificate file.  File corrupt or empty\n");
		}
		fclose(file);
	} else {
		LOG_TRACE("No client certificate loaded\n");
	}

	file =  ACAP_FILE_Open("localdata/key.pem", "r" );
	if(file) {
		LOG_TRACE("Loading key.pem\n");
		fseek(file , 0 , SEEK_END);
		long lSize = ftell(file);
		if( lSize < 9000 ) {
			buffer = (char*) malloc (sizeof(char)*lSize);
			if(!buffer) {
				fclose(file);
				LOG_WARN("CERT:  Memory allocation error\n");
				return 0;
			}	
			rewind(file);
			length = fread ( buffer, sizeof(char), lSize, file );
			if( length > 50 ) {
				if( strstr( buffer, "-----BEGIN RSA PRIVATE KEY-----") > 0 ) {
					char filepath[256];
					sprintf(filepath,"%s%s", ACAP_FILE_AppPath(),"localdata/key.pem");
					cJSON_AddStringToObject(CERTS_SETTINGS, "keyfile", filepath );
					LOG_TRACE("Private certificate key loaded");
					 ACAP_STATUS_SetBool("certificate","key",1);
				} else {
					LOG_WARN("Could not load key file.  PEM file is corrupt\n");
				}
			} else {
				LOG_WARN("Could not load key file.  File corrupt or empty\n");
			}
			free(buffer);
		} else {
			LOG_WARN("Could not load key file.  File corrupt or empty\n");
		}
		fclose(file);
	} else {
		LOG_TRACE("No client certificate private loaded\n");
	}

	file =  ACAP_FILE_Open("localdata/ca.pem", "r" );
	if(file) {
		LOG_TRACE("Loading ca.pem\n");
		fseek(file , 0 , SEEK_END);
		long lSize = ftell(file);
		if( lSize < 9000 ) {
			buffer = (char*) malloc (sizeof(char)*lSize);
			if(!buffer) {
				fclose(file);
				LOG_WARN("Could not load CA file.  Memory allocation\n");
				return 0;
			}	
			rewind(file);
			length = fread ( buffer, sizeof(char), lSize, file );
			if( length > 50 ) {
				if( strstr( buffer, "-----BEGIN CERTIFICATE-----") > 0 ) {
					char filepath[256];
					sprintf(filepath,"%s%s", ACAP_FILE_AppPath(),"localdata/ca.pem");
					cJSON_AddStringToObject(CERTS_SETTINGS, "cafile", filepath );
					LOG_TRACE("CA Certificate loaded");
					 ACAP_STATUS_SetBool("certificate","ca",1);
				} else {
					LOG_WARN("Could not load CA file.  Invalid or corrupt\n");
				}
			} else {
				LOG_WARN("Could not load CA file.  Invalid or corrupt\n");
			}
			free(buffer);
		} else {
			LOG_WARN("Could not load CA file.  Invalid or corrupt\n");
		}
		fclose(file);
	} else {
		LOG_TRACE("CA file not avaialable\n");
	}

	 ACAP_STATUS_SetBool("certificate","password",0);
	file =  ACAP_FILE_Open("localdata/ph.txt", "r" );
	if(file) {
		fseek(file , 0 , SEEK_END);
		long lSize = ftell(file);
		if( lSize < 100 ) {
			buffer = (char*) malloc (sizeof(char)*lSize);
			if(!buffer) {
				fclose(file);
				LOG_WARN("CERT:  Memory allocation error\n");
				return 0;
			}	
			rewind(file);
			length = fread ( buffer, sizeof(char), lSize, file );
			if( length > 3 ) {
				cJSON_AddStringToObject(CERTS_SETTINGS, "password", buffer );
				 ACAP_STATUS_SetBool("certificate","password",0);
			} else {
				LOG_WARN("CERT: Invalid or currupt file\n");
			}
			free(buffer);
		}
		fclose(file);
	}
	LOG_TRACE("%s: setting http node\n",__func__);
	 ACAP_HTTP_Node( "certs", CERTS_HTTP );
	return 1;
}