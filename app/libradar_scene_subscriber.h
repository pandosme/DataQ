/**
 * @file radar_scene_subscriber.h
 * @mainpage Radar Scene Subscriber
 * @brief Client API for subscribing on radar scene motion description.
 *
 * This library can be used to connect to the radar scene provider service
 * which runs the actual motion object tracking.
 *
 * Scene motion description data is serialized using protobuf and can be
 * deserialized using standard protobuf tools, see e.g.
 * [protocol-buffers](https://developers.google.com/protocol-buffers/) and
 * [protobuf-c](https://github.com/protobuf-c/protobuf-c).
 *
 * The coordinates are represented in a right-handed normalized system.
 * The coordinate system used is specified by ONVIF.
 * The x coordinate has positive direction to the right and range [-1.0,1.0].
 * The y coordinate has positive direction up and range [-1.0,1.0].
 * See [ONVIF documentation](https://www.onvif.org/specs/srv/analytics/ONVIF-Analytics-Service-Spec.pdf)
 * under "5.1.2.2 Spatial Relation" for more details.
 * The coordinates are based on camera rotation 0 and are not
 * changed even if the camera rotation is any other value.
 *
 * A scene with motion description has:
 * - time: Timestamp of radar frame closest to the tracking result.
 *         It is monotonic time in nanoseconds.
 * - objects: Tracked motion objects.
 * - events: Events of changes of tracked motion objects.
 *
 * Each tracked motion object has the following properties:
 * - id: The object's unique identification number that is a track over time.
 * - confidence: How confident an object is considered by the tracker.
 * - bounding_box: Describes the object's surrounding rectangle with the four lines
 *   (two points) left, top, right, bottom.
 * - polygon: A list of coordinate points that represents the shape and location
 *   of the object.
 * - velocity: Is the time derivative of the object's position, where
 *   the time unit is 1 second. Thus, for example a velocity of 1 in the x-direction
 *   means moving from x=0 to x=1 in one second, and the same applies for the y-direction.
 *
 * Events are used for changes in tracking of objects, such as object deleted,
 * object split, and object merge. The type of event is specified with an action
 * and refers to one or multiple  object ids.
 * - event_delete: Object is deleted. Has only one id, so the object_id refers
 *   to the object that has been deleted (and object_ids is empty).
 * - event_split: The object has split into two or more objects. The object_id
 *   is the source id and the object_ids are the target ids, i.e. one id is split
 *   into multiple ids.
 * - event_merge: Two or more objects are merged together. The object_ids are
 *   the source ids and the object_id is the target id, i.e. multiple ids are
 *   merged into one id.
 *
 * See scene.proto (found in \<sdk path\>/usr/share/protobuf/scene.proto) for a
 * detailed description of the protobuf format.
 *
 * The scene data format versions previous to the protobuf version are deprecated!
 * Use SCENE_PROTO_1.
 *
 * The radar scene motion description service is a shared resource.
 * Good practice is to save system resources by stopping subscriptions
 * when not needed.
 *
 * @ref radar_scene_subscriber.h "C API documentation"
 *
 * Copyright (C) 2017-2022 Axis Communications AB, LUND, SWEDEN
 */

#ifndef RADAR_SCENE_SUBSCRIBER_H
#define RADAR_SCENE_SUBSCRIBER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#define SUCCESS                       (0)
/**< Success */
#define ERROR_GENERAL                 (-1)
/**< General unspecified error */
#define ERROR_INVALID_RUNNING_STATE   (-2)
/**< Invalid running state */
#define ERROR_NO_VALID_PROVIDER       (-3)
/**< Radar Scene service error */
#define ERROR_INVALID_DATA_FORMAT     (-4)
/**< Invalid data format */
#define ERROR_INVALID_CHANNEL         (-5)
/**< Invalid radar channel */
#define ERROR_UNAUTHORIZED            (-6)
/**< Authorization error */
#define ERROR_INVALID_CONFIG          (-7)
/**< Configuration error */

/**
 * Data format versions.
 * This describes the format of the data but also how the data is internally transported.
 * Always use SCENE_PROTO_1. The previous versions are deprecated. The communication protocol
 * is abstracted by the library and is not necessary to expose.
 *
 * The data format in SCENE_PROTO_1 is protobuf 3 serialized data using the scene.proto API.
 * The format of the buffer is a 32 bit header directly followed by scene data payload.
 * The header is in network endian format and only contain the total size in bytes of the buffer,
 * used for memory allocation.
 * The scene data has an offset of four bytes into the buffer. To unpack the scene data,
 * use protobuf generated code of your own choice. This extracts the serialized data into useful
 * structs or classes, depending on the protobuf implementation.
 * When size is requested by the scene unpack function, do endian conversion of the header (see ntohl())
 * and reduce size with 4 bytes (the header size), this is the packed size.
 *
 * SCENELIB_1_0 - Socket communication v1.0, data decoded using libs/scene.
 * SCENELIB_1_1 - Socket communication v1.1, data decoded using libs/scene updated with low_confident.
 * SCENE_PROTO_1 - Scene data using protobuf format using scene.proto API.
 *
 */
typedef enum {
    SCENELIB_1_0 = 1,
    SCENELIB_1_1,
    SCENE_PROTO_1
} data_format_t;

/**
 * Callback notification when data is received. The callback function must return quickly and is not
 * allowed to call operations on the subscriber, such as unsubscribe or delete.
 *
 * @param scene_data Pointer to scene data which can be passed to libs/scene:scene_copy_from_area().
 *                   Caller is responsible for freeing this memory using free().
 * @param user_data  Pointer to user defined data.
 *
 */
typedef void (*on_message_arrived_callback_t)(char *scene_data,
                                              void *user_data);

/**
 * Callback notification when provider is connected. The callback function must return quickly and
 * is not allowed to call operations on the subscriber, such as trying to resubscribe or delete.
 *
 * @param user_data  Pointer to user defined data.
 *
 */
typedef void (*on_connect_callback_t)(void *user_data);

/**
 * Callback notification when provider is disconnected. The callback function must return quickly and
 * is not allowed to call operations on the subscriber, such as trying to resubscribe or delete.
 *
 * @param user_data  Pointer to user defined data.
 *
 */
typedef void (*on_disconnect_callback_t)(void *user_data);

/**
 * Opaque radar scene subscriber object.
 */
typedef struct radar_scene_subscriber_st radar_scene_subscriber_t;

/**
 *  Creates an object carry scene provider configuration and state and returns it's pointer.
 *  Callbacks are NULL and feature parameters are set to false. The caller must use the
 *  setter functions to set callbacks and enable the features it is interested in.
 *
 *  Channel indexing starts at 0.
 *
 *  The caller must call radar_scene_subscriber_delete() on the returned pointer to free resources
 *  after cancelling the created subscription. The caller must not call free() on it directly.
 *
 *  @param  channel     The channel for which radar scene data is requested. The first channel is 0. If set to -1 the lib handles channel selection internally.
 *  @param  data_format data_format_t specifying the data format which is requsted.
 *  @param  user_data   Pointer to user defined data which will be available in the callbacks.
 *
 *  @return             A pointer to the radar scene subscriber.
 *                      NULL if creation failed.
 *
 */
radar_scene_subscriber_t *
radar_scene_subscriber_create(int channel,
                              data_format_t data_format,
                              void *user_data);

/**
 *  Destroys a configuration struct to free the memory.
 *  The caller must first cancel the subscription by calling radar_scene_subscriber_unsubscribe().
 *
 *  @param  subscriber      Handle to a subscriber.
 *
 *  @return                 SUCCESS when delete is successful.
 *                          ERROR_INVALID_RUNNING_STATE if subsription is active.
 *                          ERROR_GENERAL when delete failed.
 *
 */
int
radar_scene_subscriber_delete(radar_scene_subscriber_t *subscriber);

/**
 *  Requests starting a radar scene provider. The caller must call radar_scene_subscriber_unsubscribe()
 *  to cancel the subscription, unless the on_disconnect callback has been invoked.
 *  If subscription to radar scene provider fails internally,
 *  it will retry for 180 seconds before returning.
 *
 *  @param  subscriber     The subscriber for which radar scene data is requested.
 *
 *  @return                SUCCESS if subscribing is successful.
 *                         <0 when subscribing failed.
 *                         ERROR_NO_VALID_PROVIDER if the provider is not active.
 *                         ERROR_INVALID_DATA_FORMAT if the data format is unknown.
 *
 */
int
radar_scene_subscriber_subscribe(radar_scene_subscriber_t *subscriber);

/**
 *  Cancel a valid subscription to register that the caller is no longer interested in radar
 *  scene data. This may stop the radar scene provider if no more subscriptions are active
 *  for the channel.
 *
 *  @param  subscriber      The subscriber to cancel.
 *
 *  @return                 SUCCESS when unsubscribing is successful.
 *                          <0 when unsubscribing failed.
 *                          ERROR_NO_VALID_PROVIDER if the provider is not active.
 *
 */
int
radar_scene_subscriber_unsubscribe(radar_scene_subscriber_t *subscriber);

/**
 *  Set the callback to invoke when the scene provider is connected.
 *
 *  @param  subscriber      The subscriber to set the callback on.
 *  @param  callback        The callback to set.
 *
 *  @return                 SUCCESS when setting the parameter is successful.
 *                          ERROR_GENERAL when setting the parameter failed.
 *
 */
int
radar_scene_subscriber_set_on_connect_callback(radar_scene_subscriber_t *subscriber,
                                               on_connect_callback_t callback);

/**
 *  Set the callback to invoke when the scene provider is disconnected.
 *
 *  @param  subscriber      The subscriber to set the callback on.
 *  @param  callback        The callback to set.
 *
 *  @return                 SUCCESS when setting the parameter is successful.
 *                          ERROR_GENERAL when setting the parameter failed.
 *
 */
int
radar_scene_subscriber_set_on_disconnect_callback(radar_scene_subscriber_t *subscriber,
                                                  on_disconnect_callback_t callback);

/**
 *  Set the callback to invoke when a scene message has arrived.
 *
 *  @param  subscriber      The subscriber to set the callback on.
 *  @param  callback        The callback to set.
 *
 *  @return                 SUCCESS when setting the parameter is successful.
 *                          ERROR_GENERAL when setting the parameter failed.
 *
 */
int
radar_scene_subscriber_set_on_message_arrived_callback(radar_scene_subscriber_t *subscriber,
                                                       on_message_arrived_callback_t callback);

/**
 *  Enable getting bounding boxes in the scene messages.
 *
 *  @param  subscriber      The subscriber to enable the parameter on.
 *
 *  @return                 SUCCESS when setting the parameter is successful.
 *                          ERROR_GENERAL when setting the parameter failed.
 *
 */
int
radar_scene_subscriber_set_bounding_box_enabled(radar_scene_subscriber_t *subscriber);

/**
 *  Enable getting polygon data in the scene messages.
 *
 *  @param  subscriber      The subscriber to enable the parameter on.
 *
 *  @return                 SUCCESS when setting the parameter is successful.
 *                          ERROR_GENERAL when setting the parameter failed.
 *
 */
int
radar_scene_subscriber_set_polygon_enabled(radar_scene_subscriber_t *subscriber);

/**
 *  Enable getting velocity data in the scene messages.
 *
 *  @param  subscriber      The subscriber to enable the parameter on.
 *
 *  @return                 SUCCESS when setting the parameter is successful.
 *                          ERROR_GENERAL when setting the parameter failed.
 *
 */
int
radar_scene_subscriber_set_velocity_enabled(radar_scene_subscriber_t *subscriber);

/**
 *  Enable getting raw data in the scene messages.
 *
 *  @param  subscriber      The subscriber to enable the parameter on.
 *
 *  @return                 SUCCESS when setting the parameter is successful.
 *                          ERROR_GENERAL when setting the parameter failed.
 *
 */
int
radar_scene_subscriber_set_raw_enabled(radar_scene_subscriber_t *subscriber);

/**
 *  Enable reconnect attempts if connection is lost.
 *
 *  @param  subscriber      The subscriber to enable the parameter on.
 *  @param  enable          Reconnect mode. True if enabled.
 *
 *  @return                 SUCCESS when setting the parameter is successful.
 *                          ERROR_GENERAL when setting the parameter failed.
 *
 */
int
radar_scene_subscriber_set_reconnect(radar_scene_subscriber_t *subscriber, bool enable);

/**
 *  Enable getting boxes and velocities in the image plane.
 *
 *  Note that this functionality is not supported on all devices.
 *
 *  @param  subscriber      The subscriber to enable the parameter on.
 *  @param  enable          Image transformation mode. True if enabled.
 *
 *  @return                 SUCCESS when setting the parameter is successful.
 *                          ERROR_GENERAL when setting the parameter failed.
 */
int
radar_scene_subscriber_set_image_transformation_enabled(radar_scene_subscriber_t *subscriber, bool enable);

/**
 *  Get supported classifiactions.
 *
 *  @param  subscriber      The subscriber to enable the parameter on.
 *  @param  classifications Classifications supported by the radar scene data.
 *
 *  @return                 Number of classifiactions when successful.
 *                          ERROR_GENERAL on fail to get classifiations.
 *
 */
int
radar_scene_subscriber_get_object_classifications(radar_scene_subscriber_t *subscriber, const char* **classifications);

/**
 *  Get classifiaction refered to by given class_enum.
 *
 *  @param  subscriber      The subscriber to enable the parameter on.
 *  @param  class_enum      Enumeration of classifiaction.
 *
 *  @return                 Classification.
 *
 */
const char*
radar_scene_subscriber_get_object_classification(radar_scene_subscriber_t *subscriber, unsigned class_enum);

/**
 *  Get a list of supported channels.
 *
 *  @param  subscriber      The subscriber to enable the parameter on.
 *  @param  channels        List of channels
 *
 *  @return                 Number of channels when successful.
 *                          ERROR_GENERAL on fail to get channels.
 */
int
radar_scene_subscriber_get_supported_channels(radar_scene_subscriber_t *subscriber, int **channels);

#ifdef __cplusplus
}
#endif

#endif /* RADAR_SCENE_SUBSCRIBER_H */