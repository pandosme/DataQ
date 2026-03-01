/*------------------------------------------------------------------
 *  Fred Juhlin (2023)
 *  ObjectDetection for Radar
 *------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <glib.h>
#include <string.h>
#include <syslog.h>
#include <dlfcn.h>
#include <dirent.h>
#include <math.h>
#include "cJSON.h"
#include "ACAP.h"
#include "RadarDetection.h"
#include "libradar_scene_subscriber.h"
#include "radarscene.pb-c.h"


#define LOG(fmt, args...)      { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_WARN(fmt, args...) { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args); }
//#define LOG_TRACE(fmt, args...) { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_TRACE(fmt, args...) {}

/* Round to one decimal place for clean JSON output */
#define R1(x) (round((x) * 10.0) / 10.0)

static radar_scene_subscriber_t *RadarDetection_Handler = NULL;
static void *lib_handle = NULL;
static radar_scene_subscriber_t *(*radar_create)(int channel, data_format_t data_format, void *user_data) = NULL;
static int (*radar_set_callback)(radar_scene_subscriber_t *subscriber,on_message_arrived_callback_t callback) = NULL;
static int (*radar_disconnect_callback)(radar_scene_subscriber_t *subscriber,on_disconnect_callback_t callback) = NULL;
static int (*radar_subscribe)(radar_scene_subscriber_t *subscriber) = NULL;
static int (*radar_unsubscribe)(radar_scene_subscriber_t *subscriber) = NULL;
static int (*radar_delete)(radar_scene_subscriber_t *subscriber) = NULL;
static int (*radar_boundingbox)(radar_scene_subscriber_t *subscriber) = NULL;
static int (*radar_velocity)(radar_scene_subscriber_t *subscriber) = NULL;
/* Optional: look up class name by id from library's internal table.
 * The radar often sends label id+confidence but NOT the name string in the proto.
 * get_object_classifications (plural) returns the full array; we store it as a table.
 * get_object_classification  (singular) looks up one entry by enum — kept as fallback. */
static const char* (*radar_get_classification)(radar_scene_subscriber_t *subscriber, unsigned class_enum) = NULL;
static int         (*radar_get_classifications)(radar_scene_subscriber_t *subscriber, const char ***classifications) = NULL;

/* Classification table populated from the library on the first received scene frame.
 * Hardcoded fallback is used when library lookup is unavailable or returns nothing.
 * Verified mapping from D2110 firmware 12.x via get_object_classifications:
 *   0=Raw, 1=RawAssociated, 2=Unknown, 3=Human, 4=Vehicle */
static const char *class_table_hardcoded[] = {
    "Raw",            /* 0 */
    "RawAssociated",  /* 1 */
    "Unknown",        /* 2 */
    "Human",          /* 3 */
    "Vehicle",        /* 4 */
};
#define CLASS_TABLE_HARDCODED_N ((int)(sizeof(class_table_hardcoded)/sizeof(class_table_hardcoded[0])))

static const char **class_table   = NULL;
static int          class_table_n  = 0;
static int          class_table_ok = 0;  /* set to 1 once populated */

int Some_user_data = 42;  //Yes, the final answer...

ObjectDetection_Callback RadarDetection_Detection = NULL;
TrackerDetection_Callback RadarDetection_Tracker = NULL;


static int config_units = 1;
static int config_min_confidence = 50;
static int config_x1 = 0;
static int config_x2 = 1000;
static int config_y1 = 0;
static int config_y2 = 1000;
static cJSON* config_blacklist = 0;
cJSON* radarCache = 0;



void
RadarDetection_Scene(char *protoData, void *user_data) {
	LOG_TRACE("%s: Entry\n", __func__);
	int i;
	cJSON* item = 0;

	/* Populate classification table once on first live frame */
	if (!class_table_ok && radar_get_classifications && RadarDetection_Handler) {
		class_table_ok = 1;
		const char **names = NULL;
		int n = radar_get_classifications(RadarDetection_Handler, &names);
		LOG("%s: get_object_classifications -> %d\n", __func__, n);
		if (n > 0 && names) {
			class_table   = names;
			class_table_n = n;
			for (int ci = 0; ci < n; ci++) {
				LOG("%s:   class[%d] = %s\n", __func__, ci, names[ci] ? names[ci] : "(null)");
			}
		}
	}
	/* Fix: read header as unsigned bytes to avoid signed-char sign-extension.
	 * Each byte >= 0x80 would otherwise become negative, corrupting size.
	 * The library header documents: 32-bit big-endian total size (includes
	 * the 4-byte header itself), followed by protobuf payload. */
	unsigned char *buf = (unsigned char *)protoData;
	size_t total = ((size_t)buf[0] << 24) | ((size_t)buf[1] << 16)
	             | ((size_t)buf[2] <<  8) | (size_t)buf[3];
	if (total < 4) {
		LOG_WARN("%s: Invalid frame size %zu\n", __func__, total);
		return;
	}
	size_t size = total - 4;
	LOG_TRACE("%s: Frame total=%zu payload=%zu\n", __func__, total, size);

    Radar__Scene *scene = radar__scene__unpack(NULL, size, (const uint8_t *)(protoData + 4));

	if( scene == NULL )
		return;
	
	double now = ACAP_DEVICE_Timestamp();

	if( !radarCache )
		radarCache = cJSON_CreateObject();

	if( scene->n_objects > 0 )
		LOG_TRACE("Scene: %d object(s)\n", (int)scene->n_objects);

	item = radarCache->child;
	while(item) {
		cJSON_GetObjectItem(item,"active")->type = cJSON_False;
		item = item->next;
	}

	for (i = 0; i < scene->n_objects; i++) {
		Radar__Object *object = scene->objects[i];
		if( object ) {
			Radar__BoundingBox *box = object->bounding_box;
			Radar__Point *position = object->position;
			Radar__GeoPosition *geoposition = object->geoposition;
			Radar__Velocity *velocity = object->velocity;
			Radar__Labels *labels = object->labels;
			char id[32] = "not_set";
			const char* labelName = "Undefined";
			int x = 0;
			int y = 0;
			double speed = 0;
			double maxSpeed = 0;
			float confidence = -1.0f;
			int type = 0;
			float lat = 0;
			float lon = 0;
			double radarDistance = 0;
			double radarAngle = 0;
			double direction = 0;

			//Process

			if (object->has_object_id_case == RADAR__OBJECT__HAS_OBJECT_ID_ID) {
				sprintf(id,"%u",object->id);
			}

			if (object->has_boundingbox_case == RADAR__OBJECT__HAS_BOUNDINGBOX_BOUNDING_BOX && box) {
				x = ((box->left + 1) / 2) * 1000;
				y = ((1 - box->top) / 2 ) * 1000;
			}

			if (object->has_labels_case == RADAR__OBJECT__HAS_LABELS_LABELS && labels) {
				LOG_TRACE("Labels: n=%zu\n", labels->n_labels);
				for (size_t l = 0; l < labels->n_labels; l++) {
					Radar__Label *label = labels->labels[l];
					unsigned lid = (label->has_class_id_case == RADAR__LABEL__HAS_CLASS_ID_ID) ? label->id : 0;
					float lconf  = (label->has_confidence_case == RADAR__LABEL__HAS_CONFIDENCE_CONFIDENCE) ? label->confidence : 1.0f;
					/* Name: prefer proto name field; fall back to library lookup by id.
					 * The radar typically sends id+confidence but omits the name string,
					 * leaving has_name_case=NOT_SET. The library has an internal id->name table. */
					const char *lname = NULL;
					if (label->has_name_case == RADAR__LABEL__HAS_NAME_NAME) {
						lname = label->name;
					} else if (lid < (unsigned)class_table_n && class_table && class_table[lid] && class_table[lid][0]) {
						/* Use library-fetched classification table */
						lname = class_table[lid];
					} else if (radar_get_classification && lid > 0) {
						/* Singular library lookup fallback */
						lname = radar_get_classification(RadarDetection_Handler, lid);
					}
					/* Last resort: hardcoded ID→name table */
					if (!lname && lid < (unsigned)CLASS_TABLE_HARDCODED_N) {
						lname = class_table_hardcoded[lid];
					}
					LOG_TRACE("  label[%zu] id=%u id_case=%d conf=%.3f conf_case=%d name=%s name_case=%d\n",
						l, lid,
						(int)label->has_class_id_case, lconf,
						(int)label->has_confidence_case,
						lname ? lname : "(none)",
						(int)label->has_name_case);
					/* Accept this label if it has a name (even confidence=0.0 is a valid classification;
					 * pick the label with highest confidence when multiple labels are present). */
					if (lname && lconf >= confidence) {
						confidence = lconf;
						type       = (int)lid;
						labelName  = lname;
					}
				}
			} else {
				LOG_TRACE("No labels: has_labels_case=%d\n", (int)object->has_labels_case);
			}
			/* confidence from protobuf is float 0.0-1.0; scale to 0-100 percent.
			 * If no labels arrived, default to 100 (radar reports presence not prob). */
			/* confidence starts at -1.0f (sentinel = no label received).
			 * Use >= 0.0f so conf=0.000 (Unknown class) maps to 0%, not 100%. */
			int iconfidence = (confidence >= 0.0f) ? (int)(confidence * 100.0f + 0.5f) : 100;
			if (object->has_velocity_case == RADAR__OBJECT__HAS_VELOCITY_VELOCITY && velocity) {
				speed = velocity->vx;
				direction = velocity->vy;
				if( config_units == 1 )
					speed *= 3.6;
				if( config_units == 2 )
				speed *= 2.2369362921;
				speed += 0.4;
			}

		    if (object->has_position_case == RADAR__OBJECT__HAS_POSITION_POSITION && position) {
				radarDistance = position->x;
				radarAngle = position->y;
				if( config_units == 2 )
					radarDistance *= 3.28084;
				radarDistance += 0.4;
			}


			LOG_TRACE("Object id=%s class=%s conf=%d x=%d y=%d dist=%.1fm speed=%.1f\n",
				id, labelName, iconfidence, x, y, radarDistance, speed);

			cJSON* detection = cJSON_GetObjectItem(radarCache,id);
			if( !detection ) {
				LOG("New track: id=%s class=%s\n", id, labelName);
				detection = cJSON_CreateObject();
				cJSON_AddStringToObject(detection,"id",id);
				cJSON_AddStringToObject(detection,"class",labelName);
				cJSON_AddBoolToObject(detection, "active", 1);
				cJSON_AddBoolToObject(detection, "valid", 0);
				cJSON_AddNumberToObject(detection,"type",type);
				cJSON_AddNumberToObject(detection,"confidence",iconfidence);
				cJSON_AddNumberToObject(detection,"timestamp",now);
				cJSON_AddNumberToObject(detection,"birth",now);
				cJSON_AddNumberToObject(detection,"x",x);
				cJSON_AddNumberToObject(detection,"y",y);
				cJSON_AddNumberToObject(detection,"bx",x);
				cJSON_AddNumberToObject(detection,"by",y);
				cJSON_AddNumberToObject(detection,"px",x);
				cJSON_AddNumberToObject(detection,"py",y);
				cJSON_AddNumberToObject(detection,"dx",0);
				cJSON_AddNumberToObject(detection,"dy",0);
				cJSON_AddNumberToObject(detection,"age",0);
				cJSON_AddNumberToObject(detection,"distance",0);
				cJSON_AddNumberToObject(detection,"speed",R1(speed));
				cJSON_AddNumberToObject(detection,"maxSpeed",R1(speed));
				cJSON_AddNumberToObject(detection,"direction",R1(direction));
				cJSON_AddNumberToObject(detection,"radarAngle",R1(radarAngle));
				cJSON_AddNumberToObject(detection,"radarDistance",R1(radarDistance));
				cJSON_AddNumberToObject(detection,"idle",0);
				cJSON_AddNumberToObject(detection,"lasttracker",0);
				cJSON_AddItemToObject(radarCache,id, detection);
			} else {
				int dx = x - cJSON_GetObjectItem(detection,"bx")->valueint;
				int dy = y - cJSON_GetObjectItem(detection,"by")->valueint;
				cJSON_ReplaceItemInObject(detection,"dx",cJSON_CreateNumber(dx));
				cJSON_ReplaceItemInObject(detection,"dy",cJSON_CreateNumber(dy));
				cJSON_ReplaceItemInObject(detection,"timestamp",cJSON_CreateNumber(now));
				cJSON_GetObjectItem(detection,"active")->type = cJSON_True;
				cJSON_ReplaceItemInObject(detection,"x",cJSON_CreateNumber(x));
				cJSON_ReplaceItemInObject(detection,"y",cJSON_CreateNumber(y));
				double age = round(10.0 * ((now - cJSON_GetObjectItem(detection,"birth")->valuedouble) / 1000.0)) / 10.0;
				cJSON_ReplaceItemInObject(detection,"age",cJSON_CreateNumber(age));
				cJSON_ReplaceItemInObject(detection,"speed",cJSON_CreateNumber(R1(speed)));
				double maxSpeed = cJSON_GetObjectItem(detection,"maxSpeed")->valuedouble;
				if( speed > maxSpeed )
					cJSON_ReplaceItemInObject(detection,"maxSpeed",cJSON_CreateNumber(R1(speed)));
				cJSON_ReplaceItemInObject(detection,"direction",cJSON_CreateNumber(R1(direction)));
				cJSON_ReplaceItemInObject(detection,"radarAngle",cJSON_CreateNumber(R1(radarAngle)));
				cJSON_ReplaceItemInObject(detection,"radarDistance",cJSON_CreateNumber(R1(radarDistance)));
				/* Always update class when the radar provides a definite classification.
				 * Labels may not arrive on the very first frame so a track starts as
				 * "Undefined"; update it as soon as a real label is received.
				 * Also fix typo: was "ype" instead of "type". */
				if( confidence > 0.0f ) {
					cJSON_ReplaceItemInObject(detection,"type",cJSON_CreateNumber(type));
					cJSON_ReplaceItemInObject(detection,"class",cJSON_CreateString(labelName));
					cJSON_ReplaceItemInObject(detection,"confidence",cJSON_CreateNumber(iconfidence));
				}
			}

			if( cJSON_GetObjectItem(detection,"valid")->type == cJSON_False ) {
				if( x > config_x1 && x < config_x2 && y > config_y1 && y < config_y2 ) {
					cJSON_GetObjectItem(detection,"valid")->type = cJSON_True;
				} else {
					LOG("Object id=%s filtered by AOI (x=%d y=%d aoi=[%d-%d,%d-%d])\n",
						id, x, y, config_x1, config_x2, config_y1, config_y2);
				}
			}
		}
	}
	radar__scene__free_unpacked(scene, NULL);

	/* Build the full detection list first, call tracker per object,
	 * then publish once.  Previous code called Detection + Delete list
	 * inside the loop causing double-free and use-after-free. */
	cJSON* list = cJSON_CreateArray();
	item = radarCache->child;
	while(item) {
		int item_valid = cJSON_GetObjectItem(item,"valid") && cJSON_GetObjectItem(item,"valid")->type == cJSON_True;
		if( cJSON_GetObjectItem(item,"valid")->type == cJSON_True ) {
			double px = cJSON_GetObjectItem(item,"px")->valuedouble;
			double py = cJSON_GetObjectItem(item,"py")->valuedouble;
			double x = cJSON_GetObjectItem(item,"x")->valuedouble;
			double y = cJSON_GetObjectItem(item,"y")->valuedouble;
			cJSON* duplicate = cJSON_Duplicate(item, 1);
			cJSON_DeleteItemFromObject(duplicate,"valid");
			cJSON_DeleteItemFromObject(duplicate,"px");
			cJSON_DeleteItemFromObject(duplicate,"py");
			/* cx,cy = radar detection point (exact position).
			 * x,y = top-left of a synthetic bounding box centred on that point.
			 * w,h = small fixed size so downstream systems expecting a bbox work. */
			cJSON_AddNumberToObject(duplicate,"cx",x);
			cJSON_AddNumberToObject(duplicate,"cy",y);
			cJSON_AddNumberToObject(duplicate,"w",20);
			cJSON_AddNumberToObject(duplicate,"h",20);
			cJSON_GetObjectItem(duplicate,"x")->valueint -= 10;
			cJSON_GetObjectItem(duplicate,"x")->valuedouble -= 10;
			cJSON_GetObjectItem(duplicate,"y")->valueint -= 10;
			cJSON_GetObjectItem(duplicate,"y")->valuedouble -= 10;

			/* Package radar-specific data into a "radar" sub-object */
			{
				cJSON *radarObj = cJSON_CreateObject();
				cJSON *f;
				f = cJSON_DetachItemFromObject(duplicate, "radarAngle");
				cJSON_AddItemToObject(radarObj, "angle",     f ? f : cJSON_CreateNumber(0));
				f = cJSON_DetachItemFromObject(duplicate, "direction");
				cJSON_AddItemToObject(radarObj, "direction", f ? f : cJSON_CreateNumber(0));
				f = cJSON_DetachItemFromObject(duplicate, "speed");
				cJSON_AddItemToObject(radarObj, "speed",     f ? f : cJSON_CreateNumber(0));
				f = cJSON_DetachItemFromObject(duplicate, "radarDistance");
				cJSON_AddItemToObject(radarObj, "distance",  f ? f : cJSON_CreateNumber(0));
				cJSON_AddItemToObject(duplicate, "radar", radarObj);
			}
			cJSON_DeleteItemFromObject(duplicate, "lasttracker");

			double dx = px - x;
			double dy = py - y;
			double movedDist = sqrtf((float)(dx * dx + dy * dy));
			if( movedDist > 0 ) {
				/* Always accumulate total traveled distance */
				double previousDistance = cJSON_GetObjectItem(item,"distance")->valuedouble;
					cJSON_ReplaceItemInObject(item,"distance",cJSON_CreateNumber(R1(previousDistance + movedDist)));
				cJSON_ReplaceItemInObject(item,"px",cJSON_CreateNumber(x));
				cJSON_ReplaceItemInObject(item,"py",cJSON_CreateNumber(y));
			}
			/* Fire tracker once per second (time-based, not distance-based).
			 * Radar coords change only ~0.6px/frame so any pixel threshold
			 * was too high to ever trigger for normal walking speeds. */
			double lastTrackerTime = cJSON_GetObjectItem(item,"lasttracker")->valuedouble;
			if( now - lastTrackerTime >= 1000 ) {
				cJSON_ReplaceItemInObject(item,"lasttracker",cJSON_CreateNumber(now));
				const char* item_id = cJSON_GetObjectItem(item,"id") ? cJSON_GetObjectItem(item,"id")->valuestring : "?";
				LOG("Tracker fire: id=%s age=%.1f\n", item_id, cJSON_GetObjectItem(item,"age") ? cJSON_GetObjectItem(item,"age")->valuedouble : -1.0);
				/* Tracker gets a copy — list owns its own duplicate */
				cJSON* trackerCopy = cJSON_Duplicate(duplicate, 1);
				RadarDetection_Tracker(trackerCopy, 0);
			}
			cJSON_AddItemToArray(list, duplicate);
		}
		item = item->next;
	}
	/* Publish the complete list once. Detections_Data owns and frees list. */
	RadarDetection_Detection(list);

	/* Remove inactive objects from cache, firing a goodbye tracker call first
	 * so ProcessPaths can finalise the path for the disappeared object. */
	item = radarCache->child;
	while( item ) {
		cJSON* next = item->next;
		/* Compare type, not assign (was: type = cJSON_False) */
		if( cJSON_GetObjectItem(item,"active")->type == cJSON_False ) {
			/* Send final tracker call with active=false so the path is published */
			if( cJSON_GetObjectItem(item,"valid")->type == cJSON_True ) {
				double gx = cJSON_GetObjectItem(item,"x")->valuedouble;
				double gy = cJSON_GetObjectItem(item,"y")->valuedouble;
				cJSON* goodbye = cJSON_Duplicate(item, 1);
				cJSON_DeleteItemFromObject(goodbye,"valid");
				cJSON_DeleteItemFromObject(goodbye,"px");
				cJSON_DeleteItemFromObject(goodbye,"py");
				cJSON_AddNumberToObject(goodbye,"cx",gx);
				cJSON_AddNumberToObject(goodbye,"cy",gy);
				cJSON_AddNumberToObject(goodbye,"w",20);
				cJSON_AddNumberToObject(goodbye,"h",20);
				cJSON_GetObjectItem(goodbye,"x")->valueint    -= 10;
				cJSON_GetObjectItem(goodbye,"x")->valuedouble -= 10;
				cJSON_GetObjectItem(goodbye,"y")->valueint    -= 10;
				cJSON_GetObjectItem(goodbye,"y")->valuedouble -= 10;
				/* Package radar-specific data into a "radar" sub-object */
				{
					cJSON *radarObj = cJSON_CreateObject();
					cJSON *f;
					f = cJSON_DetachItemFromObject(goodbye, "radarAngle");
					cJSON_AddItemToObject(radarObj, "angle",     f ? f : cJSON_CreateNumber(0));
					f = cJSON_DetachItemFromObject(goodbye, "direction");
					cJSON_AddItemToObject(radarObj, "direction", f ? f : cJSON_CreateNumber(0));
					f = cJSON_DetachItemFromObject(goodbye, "speed");
					cJSON_AddItemToObject(radarObj, "speed",     f ? f : cJSON_CreateNumber(0));
					f = cJSON_DetachItemFromObject(goodbye, "radarDistance");
					cJSON_AddItemToObject(radarObj, "distance",  f ? f : cJSON_CreateNumber(0));
					cJSON_AddItemToObject(goodbye, "radar", radarObj);
				}
				cJSON_DeleteItemFromObject(goodbye, "lasttracker");
				/* active field is already cJSON_False — signals path completion */
				RadarDetection_Tracker(goodbye, 0);
			}
			cJSON_DetachItemFromObjectCaseSensitive(radarCache, item->string);
			cJSON_Delete(item);
		}
		item = next;
	}
}

void
RadarDetection_Disconnect(void *user_data) {
	exit(-99);
}

int
RadarDetection_LoadLibrary() {
	lib_handle = NULL;

	/* Always load the system library by full path so the device's own
	 * libradar_scene_subscriber.so.0 is used instead of any bundled copy.
	 * This ensures ABI compatibility with the running radar-scene-provider. */
	const char *lib_paths[] = {
		"/usr/lib/libradar_scene_subscriber.so.0",
		"/usr/lib/libradar_scene_subscriber.so",
		NULL
	};

	for (int p = 0; lib_paths[p] != NULL; p++) {
		lib_handle = dlopen(lib_paths[p], RTLD_LAZY | RTLD_GLOBAL);
		if (lib_handle) {
			LOG("%s: Loaded %s\n", __func__, lib_paths[p]);
			break;
		}
		LOG_WARN("%s: dlopen(%s) failed: %s\n", __func__, lib_paths[p], dlerror());
	}
	if (!lib_handle) {
		LOG_WARN("%s: Could not load libradar_scene_subscriber from system\n", __func__);
		return -9;
	}
	dlerror();
	
    radar_create = dlsym(lib_handle, "radar_scene_subscriber_create");
	if( !radar_create )
		return -10;
	
	radar_set_callback = dlsym(lib_handle, "radar_scene_subscriber_set_on_message_arrived_callback");
	if( !radar_set_callback )
		return -11;
		

	radar_disconnect_callback = dlsym(lib_handle, "radar_scene_subscriber_set_on_disconnect_callback");
	if( !radar_disconnect_callback )
		return -12;
	
    radar_boundingbox = dlsym(lib_handle, "radar_scene_subscriber_set_bounding_box_enabled");
	if( !radar_boundingbox )
		return -13;
		
    radar_velocity = dlsym(lib_handle, "radar_scene_subscriber_set_velocity_enabled");
	if( !radar_velocity )
		return -14;
		
    radar_subscribe = dlsym(lib_handle, "radar_scene_subscriber_subscribe");
	if( !radar_subscribe )
		return -15;

	gchar *dl_error;
	if((dl_error = dlerror()) != NULL)
		return -16;

	/* Optional cleanup helpers — not fatal if absent on older firmware. */
	radar_unsubscribe = dlsym(lib_handle, "radar_scene_subscriber_unsubscribe");
	dlerror();
	radar_delete = dlsym(lib_handle, "radar_scene_subscriber_delete");
	dlerror();

	/* Optional: map classification id → name. Not fatal if absent. */
	radar_get_classification = dlsym(lib_handle, "radar_scene_subscriber_get_object_classification");
	dlerror();
	radar_get_classifications = dlsym(lib_handle, "radar_scene_subscriber_get_object_classifications");
	dlerror();
	if (radar_get_classifications) {
		LOG("%s: Classification table lookup available\n", __func__);
	} else if (radar_get_classification) {
		LOG("%s: Singular classification lookup available\n", __func__);
	} else {
		LOG("%s: No classification lookup (labels must carry name string)\n", __func__);
	}
	
	RadarDetection_Handler = (*radar_create)( 0, SCENE_PROTO_1, &Some_user_data);
	if( !RadarDetection_Handler )
		return -17;
	
	if( (*radar_set_callback)( RadarDetection_Handler, RadarDetection_Scene ) != SUCCESS)
		return -18;

	if( (*radar_disconnect_callback)( RadarDetection_Handler, RadarDetection_Disconnect ) != SUCCESS)
		return -19;
	
	if( (*radar_boundingbox)( RadarDetection_Handler ) != SUCCESS )
		return -20;
	
	if( (*radar_velocity)( RadarDetection_Handler ) != SUCCESS )
		return -21;
	
	if( (*radar_subscribe)( RadarDetection_Handler ) != SUCCESS )
		return -22;
	return 1;
}


void
RadarDetection_Reset() {

}

void
RadarDetection_Cleanup(void) {
	if (RadarDetection_Handler) {
		if (radar_unsubscribe) {
			int rc = (*radar_unsubscribe)(RadarDetection_Handler);
			if (rc != SUCCESS)
				LOG_WARN("%s: unsubscribe returned %d\n", __func__, rc);
		}
		if (radar_delete) {
			int rc = (*radar_delete)(RadarDetection_Handler);
			if (rc != SUCCESS)
				LOG_WARN("%s: delete returned %d\n", __func__, rc);
		}
		RadarDetection_Handler = NULL;
	}
	if (lib_handle) {
		dlclose(lib_handle);
		lib_handle = NULL;
	}
	LOG("%s: Radar library released\n", __func__);
}


void RadarDetection_Config(cJSON* data) {
    LOG_TRACE("%s: Entry\n", __func__);
    if (!data) {
        LOG_WARN("%s: Invalid input\n", __func__);
        return;
    }

    config_min_confidence = cJSON_GetObjectItem(data, "confidence") ? cJSON_GetObjectItem(data, "confidence")->valueint : 40;
    config_blacklist = cJSON_GetObjectItem(data, "ignoreClass") ? cJSON_GetObjectItem(data, "ignoreClass") : cJSON_CreateArray();
    config_units = cJSON_GetObjectItem(data, "units") ? cJSON_GetObjectItem(data, "units")->valueint : 1;
    cJSON *aoi = cJSON_GetObjectItem(data, "aoi");
    if (aoi) {
        config_x1 = cJSON_GetObjectItem(aoi, "x1") ? cJSON_GetObjectItem(aoi, "x1")->valueint : 0;
        config_x2 = cJSON_GetObjectItem(aoi, "x2") ? cJSON_GetObjectItem(aoi, "x2")->valueint : 1000;
        config_y1 = cJSON_GetObjectItem(aoi, "y1") ? cJSON_GetObjectItem(aoi, "y1")->valueint : 0;
        config_y2 = cJSON_GetObjectItem(aoi, "y2") ? cJSON_GetObjectItem(aoi, "y2")->valueint : 1000;
    }
    LOG_TRACE("%s: Exit\n", __func__);
	RadarDetection_Reset();
}


int
RadarDetection_Init( ObjectDetection_Callback detections, TrackerDetection_Callback tracker) {
	int status = RadarDetection_LoadLibrary();
	if( status < 0 ) {
		LOG_WARN("%s: Load Library error %d",__func__,status);
		return status;
	}

	config_units = 1;
	RadarDetection_Detection = detections;
	RadarDetection_Tracker = tracker;

	cJSON* list = cJSON_CreateArray();
	cJSON_AddItemToArray(list,cJSON_CreateString("Vehicle"));
	cJSON_AddItemToArray(list,cJSON_CreateString("Human"));	
	cJSON_AddItemToArray(list,cJSON_CreateString("Undefined"));
	ACAP_STATUS_SetObject("detections","labels",list);
	cJSON_Delete(list);

	return 1;
}

