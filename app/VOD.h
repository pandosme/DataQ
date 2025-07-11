#ifndef VOD_H
#define VOD_H

#include <stdbool.h>
#include <stddef.h>
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char *name;   // Attribute type name
    char *value;  // Attribute value (class name)
} vod_attribute_t;

typedef struct {
    char id[32];        // String version of unique id
    float confidence;   // Maximum class score during tracking
    int type;           // Class type (id) of maximum confidence
    char class_name[64];// Class name of the type
    int x, y, w, h;     // [0..1000] coordinates, top-left origo
    vod_attribute_t *attributes; // Array of attributes (type/value pairs)
    size_t num_attributes;
    bool active;        // True if tracked, false if deleted
} vod_object_t;

// Callback signature: called with list of current objects
typedef void (*vod_callback_t)(const vod_object_t *objects, size_t num_objects, void *user_data);

/**
 * Initialize the VOD object detection wrapper.
 * @param channel    Channel to subscribe to.
 * @param cb         Callback to receive detected objects.
 * @param user_data  User pointer passed to callback.
 * @return 0 on success, negative on error.
 */
int VOD_Init(int channel, vod_callback_t cb, void *user_data);
cJSON* VOD_Labels_List();

/**
 * Shutdown and free all resources.
 */
void VOD_Shutdown(void);

#ifdef __cplusplus
}
#endif

#endif // VOD_H
