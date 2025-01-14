/**
 *  @file video_object_detection_subscriber.h
 *  @mainpage Video Object Detection Subscriber
 *  @brief Client API for subscribing on video object detections.
 *
 *  This library can be used to connect to the video object detection
 *  service which runs the actual video object detection.
 *
 *  Detection data is serialized using protobuf and can be deserialized using
 *  standard protobuf tools, see e.g.
 *  [protocol-buffers](https://developers.google.com/protocol-buffers/) and
 *  [protobuf-c](https://github.com/protobuf-c/protobuf-c).
 *
 *  The coordinates are represented in a right-handed normalized cartesian system.
 *  The coordinate system used is specified by ONVIF.
 *  The x coordinate has positive direction to the right and range [-1.0,1.0].
 *  The y coordinate has positive direction up and range [-1.0,1.0].
 *  See [ONVIF documentation](https://www.onvif.org/specs/srv/analytics/ONVIF-Analytics-Service-Spec.pdf)
 *  under "5.1.2.2 Spatial Relation" for more details.
 *  The coordinates are based on camera rotation 0 and are not changed even if
 *  the camera rotation is any other value. It is therefore up to the client
 *  to manually correct for the rotation if needed after receiving the detections.
 *
 *  The video object detection service is a shared resource. Good practice
 *  is to save system resources by stopping subscriptions when not needed.
 *
 *  @ref video_object_detection_subscriber.h "C API documentation"
 *
 *  Copyright (C) 2013-2019, Axis Communications AB, LUND, SWEDEN
 */

#ifndef VIDEO_OBJECT_DETECTION_SUBSCRIBER_H
#define VIDEO_OBJECT_DETECTION_SUBSCRIBER_H

#define VIDEO_OBJECT_DETECTION_SUBSCRIBER_SUCCESS                       (0)
/**< Success */
#define VIDEO_OBJECT_DETECTION_SUBSCRIBER_ERROR_GENERAL                 (-1)
/**< General unspecified error */
#define VIDEO_OBJECT_DETECTION_SUBSCRIBER_ERROR_INVALID_DBUS_SESSION    (-2)
/**< DBus session error */
#define VIDEO_OBJECT_DETECTION_SUBSCRIBER_ERROR_NO_VALID_VOD            (-3)
/**< Video Object Detection service error */
#define VIDEO_OBJECT_DETECTION_SUBSCRIBER_ERROR_INVALID_CHANNEL         (-4)
/**< Invalid video channel */
#define VIDEO_OBJECT_DETECTION_SUBSCRIBER_ERROR_UNAUTHORIZED            (-5)
/**< Authorization error */
#define VIDEO_OBJECT_DETECTION_SUBSCRIBER_ERROR_INVALID_CONFIG          (-6)
/**< Configuration error */
#define VIDEO_OBJECT_DETECTION_SUBSCRIBER_ERROR_INTERNAL                (-7)
/**< Internal error in subscriber library */
#define VIDEO_OBJECT_DETECTION_SUBSCRIBER_ERROR_NO_POWER                (-8)
/**< Not enough power available for Video Object Detection service */

#define VIDEO_OBJECT_DETECTION_SUBSCRIBER_DEFAULT_RECEIVE_EMPTY_HITS false
/**< Default value for receive_empty_hits, @see video_object_detection_subscriber_set_receive_empty_hits() */

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * Opaque handle to the video object detection subscriber object.
 */
typedef struct video_object_detection_subscriber_st video_object_detection_subscriber_t;

/**
 * Opaque type for detection class object.
 */
typedef struct video_object_detection_subscriber_class_st video_object_detection_subscriber_class_t;

/**
 * Callback notification when data is received.
 *
 * The callback function must return quickly to avoid piling up callbacks.
 * Intended usage is to:
 * 1. Deserialize detection_data
 * 2. Notify some other thread that there is new data
 * 3. Return
 *
 * It is not allowed to call operations on the subscriber, such as unsubscribe or delete,
 * in the callback.
 *
 * @param detection_data  The reported detections serialized using protobuf.
 *                        Can be deserialized by client using protobuf tools generated code.
 *                        The subscriber instance owns this memory.
 * @param data_size       The size of detection_data in bytes.
 * @param user_data       Pointer to user defined data.
 */
typedef void (*video_object_detection_subscriber_get_detection_callback_t)(
    const uint8_t *detection_data,
    size_t data_size,
    void *user_data);

/**
 * Callback notification if service is closing down,
 * clients are forced to stop handling detections.
 *
 * @param user_data  Pointer to user defined data.
 */
typedef void (*video_object_detection_subscriber_teardown_callback_t)(void *user_data);

/**
 *  Create a video object detection subscriber instance.
 *
 *  This instance is owned by the client and is used to setup a subscription
 *  to the video object detection service.
 *
 *  Client implements callbacks for upward communication. Callbacks may be null.
 *
 *  User data reference may be passed for accessing internal structures
 *  in callbacks. The Video object detection subscriber never touches this,
 *  it simply forwards this pointer.
 *
 *  @param  subscriber  The subscriber instance.
 *  @param  user_data   Pointer to user defined data.
 *  @param  channel     The channel for which video object detection data is requested.
 *                      This is zero indexed. The maximum number of available
 *                      channels can be obtained by calling
 *                      video_object_detection_subscriber_get_nbr_channels.
 *
 *  @return             VIDEO_OBJECT_DETECTION_SUBSCRIBER_SUCCESS on success,
 *                      or negative error code on failure.
 */
int
video_object_detection_subscriber_create(video_object_detection_subscriber_t **subscriber,
                                         void *user_data,
                                         int channel);

/**
 *  Starts a subscription to the video object detection service.
 *
 *  If there are no previous subscribers, the video object detection service
 *  may need to run initialization tasks.
 *  Therefore this method may take several minutes to complete.
 *
 *  @param  subscriber  The subscriber instance.
 *
 *  @return             VIDEO_OBJECT_DETECTION_SUBSCRIBER_SUCCESS on success,
 *                      or negative error code on failure.
 */
int
video_object_detection_subscriber_subscribe(video_object_detection_subscriber_t *const subscriber);


/**
 *  Stop video object detection subscription.
 *  Connection to service is closed and no further data will be received from it.
 *
 *  @param  subscriber  The subscriber instance.
 *
 *  @return             VIDEO_OBJECT_DETECTION_SUBSCRIBER_SUCCESS on success,
 *                      or negative error code on failure.
 */
int
video_object_detection_subscriber_unsubscribe(video_object_detection_subscriber_t *const
                                              subscriber);

/**
 *  Free the video object detection subscriber instance.
 *  Caller is not interested in further object detections.
 *
 *  If subscriber is in running state, client needs to unsubscribe before deleting.
 *  Call will fail if passing null handle or subcriber is in running state.
 *
 *  @param  subscriber  Handle to the subscriber instance.
 *
 *  @return             VIDEO_OBJECT_DETECTION_SUBSCRIBER_SUCCESS on success,
 *                      or negative error code on failure.
 */
int
video_object_detection_subscriber_delete(video_object_detection_subscriber_t **subscriber);

/**
 *  Set the callback to invoke when data is received.
 *  @see video_object_detection_subscriber_get_detection_callback_t()
 *
 *  @param  subscriber  The subscriber instance.
 *  @param  callback    The callback to set.
 *
 *  @return             VIDEO_OBJECT_DETECTION_SUBSCRIBER_SUCCESS on success,
 *                      or negative error code on failure.
 */
int
video_object_detection_subscriber_set_get_detection_callback(video_object_detection_subscriber_t
        *subscriber,
        video_object_detection_subscriber_get_detection_callback_t callback);

/**
 *  Set the callback to invoke if service closes down.
 *  @see video_object_detection_subscriber_teardown_callback_t()
 *
 *  @param  subscriber  The subscriber instance.
 *  @param  callback    The callback to set.
 *
 *  @return             VIDEO_OBJECT_DETECTION_SUBSCRIBER_SUCCESS on success,
 *                      or negative error code on failure.
 */
int
video_object_detection_subscriber_set_teardown_callback(video_object_detection_subscriber_t
                                                        *subscriber,
                                                        video_object_detection_subscriber_teardown_callback_t callback);

/**
 *  Enable if the client wishes to receive get_detection callbacks
 *  when no hits are found. It is disabled by default.
 *
 *  @param  subscriber          The subscriber instance.
 *  @param  receive_empty_hits  Bool for receive empty hits setting.
 *
 *  @return                     VIDEO_OBJECT_DETECTION_SUBSCRIBER_SUCCESS on success,
 *                              or negative error code on failure.
 */
int
video_object_detection_subscriber_set_receive_empty_hits(video_object_detection_subscriber_t
                                                         *subscriber,
                                                         bool receive_empty_hits);

/**
 *  Get the major and minor version of Video Object Detection
 *
 *  @param  vod_version_major   The major version of video object detection
 *  @param  vod_version_minor   The minor version of video object detection
 *
 *  @return                     VIDEO_OBJECT_DETECTION_SUBSCRIBER_SUCCESS on success,
 *                              or negative error code on failure.
 */
int
video_object_detection_subscriber_get_version(int *vod_version_major, int *vod_version_minor);

/**
 *  Get number of channels in Video Object Detection
 *
 *  All channels may not be available for subscription simultaneously.
 *  Use video_object_detection_subscriber_get_nbr_channels_simultaneously to
 *  get the number of simultaneously available channels.
 *
 *  @param  nbr_channels  Number of channels of video object detection
 *
 *  @return               VIDEO_OBJECT_DETECTION_SUBSCRIBER_SUCCESS on success,
 *                        or negative error code on failure.
 */
int
video_object_detection_subscriber_get_nbr_channels(int *nbr_channels);

/**
 *  Get number of simultaneously supported channels in Video Object Detection
 *
 *  @param  nbr_channels_simultaneously  Number of simultaneously supported channels
 *
 *  @return                              VIDEO_OBJECT_DETECTION_SUBSCRIBER_SUCCESS on success,
 *                                       or negative error code on failure.
 */
int
video_object_detection_subscriber_get_nbr_channels_simultaneously(int *nbr_channels_simultaneously);

/**
 *  Get a name string for an object detection class.
 *
 *  @param  det_class   The detection class object.
 *
 *  @return             A string with the name for the detection class on success,
 *                      or NULL on failure.
 */
const char *
video_object_detection_subscriber_det_class_name(const video_object_detection_subscriber_class_t
                                                 *det_class);

/**
 *  Get an integer identifier for an object detection class used to relate the
 *  serialized detection data with the detection class objects.
 *
 *  @param  det_class   The detection class object.
 *
 *  @return             A non-negative integer identifier for the detection class,
 *                      or negative error code on failure.
 */
int
video_object_detection_subscriber_det_class_id(const video_object_detection_subscriber_class_t
                                               *det_class);

/**
 *  Get the currently available object detection classes.
 *
 *  @param  det_classes  Pointer to hold an allocated array of video object detection class objects.
 *                       The caller is responsible for releasing this memory using
 *                       video_object_detection_subscriber_det_classes_free.
 *
 *  @return              The number of detection classes in the allocated det_classes array,
 *                       or negative error code on failure.
 */
int
video_object_detection_subscriber_det_classes_get(video_object_detection_subscriber_class_t
                                                  **det_classes[]);

/**
 *  Release allocated object detection classes.
 *
 *  @param  det_classes  An array of video object detection class objects to release.
 *                       This array must have been obtained by calling
 *                       video_object_detection_subscriber_det_classes_get.
 *  @param  num_classes  The number of detection classes in det_classes, as provided by
 *                       video_object_detection_subscriber_det_classes_get.
 */
void
video_object_detection_subscriber_det_classes_free(video_object_detection_subscriber_class_t
                                                   *det_classes[],
                                                   int num_classes);

/**
 *  Get the detector information with available object classes.
 *
 *
 *  @param  buffer     Pointer to hold an allocated buffer. Will be allocated by method if successful.
 *                     The caller is responsible for releasing this memory using free().
 *  @param  size       Pointer to hold the size of the buffer, value set if successful.
 *
 *  @return            VIDEO_OBJECT_DETECTION_SUBSCRIBER_SUCCESS on success,
 *                     or negative error code on failure.
 */
int
video_object_detection_subscriber_get_detector_information(uint8_t **buffer,
                                                           size_t *size);

/**
 *  Run status of subscriber.
 *
 *  @param  subscriber  The subscriber instance.
 *
 *  @return             Current running status as a bool.
 */
bool
video_object_detection_subscriber_running(video_object_detection_subscriber_t *subscriber);

#ifdef __cplusplus
};
#endif

#endif // VIDEO_OBJECT_DETECTION_SUBSCRIBER_H
