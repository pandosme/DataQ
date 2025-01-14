// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: video_object_detection.proto

// This CPP symbol can be defined to use imports that match up to the framework
// imports needed when using CocoaPods.
#if !defined(GPB_USE_PROTOBUF_FRAMEWORK_IMPORTS)
 #define GPB_USE_PROTOBUF_FRAMEWORK_IMPORTS 0
#endif

#if GPB_USE_PROTOBUF_FRAMEWORK_IMPORTS
 #import <Protobuf/GPBProtocolBuffers.h>
#else
 #import "GPBProtocolBuffers.h"
#endif

#if GOOGLE_PROTOBUF_OBJC_VERSION < 30002
#error This file was generated by a newer version of protoc which is incompatible with your Protocol Buffer library sources.
#endif
#if 30002 < GOOGLE_PROTOBUF_OBJC_MIN_SUPPORTED_VERSION
#error This file was generated by an older version of protoc which is incompatible with your Protocol Buffer library sources.
#endif

// @@protoc_insertion_point(imports)

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"

CF_EXTERN_C_BEGIN

@class Calibration;
@class Detection;
@class Event;
@class ObjectClass;

NS_ASSUME_NONNULL_BEGIN

#pragma mark - Enum EventAction

/**
 * *
 * Event action used by an event.
 **/
typedef GPB_ENUM(EventAction) {
  /**
   * Value used if any message's field encounters a value that is not defined
   * by this enum. The message will also have C functions to get/set the rawValue
   * of the field.
   **/
  EventAction_GPBUnrecognizedEnumeratorValue = kGPBUnrecognizedEnumeratorValue,
  EventAction_EventDelete = 0,
};

GPBEnumDescriptor *EventAction_EnumDescriptor(void);

/**
 * Checks to see if the given value is defined by the enum or was not known at
 * the time this source was generated.
 **/
BOOL EventAction_IsValidValue(int32_t value);

#pragma mark - Enum Detection_DetectionStatus

typedef GPB_ENUM(Detection_DetectionStatus) {
  /**
   * Value used if any message's field encounters a value that is not defined
   * by this enum. The message will also have C functions to get/set the rawValue
   * of the field.
   **/
  Detection_DetectionStatus_GPBUnrecognizedEnumeratorValue = kGPBUnrecognizedEnumeratorValue,
  Detection_DetectionStatus_Untracked = 0,
  Detection_DetectionStatus_TrackedUnconfident = 1,
  Detection_DetectionStatus_TrackedConfident = 2,
};

GPBEnumDescriptor *Detection_DetectionStatus_EnumDescriptor(void);

/**
 * Checks to see if the given value is defined by the enum or was not known at
 * the time this source was generated.
 **/
BOOL Detection_DetectionStatus_IsValidValue(int32_t value);

#pragma mark - VideoObjectDetectionRoot

/**
 * Exposes the extension registry for this file.
 *
 * The base class provides:
 * @code
 *   + (GPBExtensionRegistry *)extensionRegistry;
 * @endcode
 * which is a @c GPBExtensionRegistry that includes all the extensions defined by
 * this file and all files that it depends on.
 **/
@interface VideoObjectDetectionRoot : GPBRootObject
@end

#pragma mark - Detection

typedef GPB_ENUM(Detection_FieldNumber) {
  Detection_FieldNumber_Left = 1,
  Detection_FieldNumber_Top = 2,
  Detection_FieldNumber_Right = 3,
  Detection_FieldNumber_Bottom = 4,
  Detection_FieldNumber_Id_p = 5,
  Detection_FieldNumber_DetClass = 6,
  Detection_FieldNumber_Score = 7,
  Detection_FieldNumber_DetectionStatus = 8,
};

@interface Detection : GPBMessage

@property(nonatomic, readwrite) float left;

@property(nonatomic, readwrite) float top;

@property(nonatomic, readwrite) float right;

@property(nonatomic, readwrite) float bottom;

@property(nonatomic, readwrite) uint32_t id_p;

@property(nonatomic, readwrite) uint32_t detClass;

@property(nonatomic, readwrite) uint32_t score;

@property(nonatomic, readwrite) Detection_DetectionStatus detectionStatus;

@end

/**
 * Fetches the raw value of a @c Detection's @c detectionStatus property, even
 * if the value was not defined by the enum at the time the code was generated.
 **/
int32_t Detection_DetectionStatus_RawValue(Detection *message);
/**
 * Sets the raw value of an @c Detection's @c detectionStatus property, allowing
 * it to be set to a value that was not defined by the enum at the time the code
 * was generated.
 **/
void SetDetection_DetectionStatus_RawValue(Detection *message, int32_t value);

#pragma mark - Event

typedef GPB_ENUM(Event_FieldNumber) {
  Event_FieldNumber_Action = 1,
  Event_FieldNumber_ObjectId = 2,
};

/**
 * *
 * Event is an object that holds information about an event. The type
 * of event is specified with an action and refers to one object id.
 * A delete event only has one id, so the object_id refers
 * to the object that has been deleted.
 **/
@interface Event : GPBMessage

/** Event identifier */
@property(nonatomic, readwrite) EventAction action;

/** Object id that this event refers to */
@property(nonatomic, readwrite) int32_t objectId;

@end

/**
 * Fetches the raw value of a @c Event's @c action property, even
 * if the value was not defined by the enum at the time the code was generated.
 **/
int32_t Event_Action_RawValue(Event *message);
/**
 * Sets the raw value of an @c Event's @c action property, allowing
 * it to be set to a value that was not defined by the enum at the time the code
 * was generated.
 **/
void SetEvent_Action_RawValue(Event *message, int32_t value);

#pragma mark - Scene

typedef GPB_ENUM(Scene_FieldNumber) {
  Scene_FieldNumber_Timestamp = 1,
  Scene_FieldNumber_DetectionsArray = 2,
  Scene_FieldNumber_EventsArray = 3,
};

@interface Scene : GPBMessage

@property(nonatomic, readwrite) uint64_t timestamp;

@property(nonatomic, readwrite, strong, null_resettable) NSMutableArray<Detection*> *detectionsArray;
/** The number of items in @c detectionsArray without causing the array to be created. */
@property(nonatomic, readonly) NSUInteger detectionsArray_Count;

@property(nonatomic, readwrite, strong, null_resettable) NSMutableArray<Event*> *eventsArray;
/** The number of items in @c eventsArray without causing the array to be created. */
@property(nonatomic, readonly) NSUInteger eventsArray_Count;

@end

#pragma mark - Calibration

typedef GPB_ENUM(Calibration_FieldNumber) {
  Calibration_FieldNumber_Precision = 1,
  Calibration_FieldNumber_Recall = 2,
  Calibration_FieldNumber_Threshold = 3,
};

@interface Calibration : GPBMessage

@property(nonatomic, readwrite) float precision;

@property(nonatomic, readwrite) float recall;

@property(nonatomic, readwrite) float threshold;

@end

#pragma mark - ObjectClass

typedef GPB_ENUM(ObjectClass_FieldNumber) {
  ObjectClass_FieldNumber_Id_p = 1,
  ObjectClass_FieldNumber_Name = 2,
  ObjectClass_FieldNumber_CalibrationsArray = 3,
};

@interface ObjectClass : GPBMessage

@property(nonatomic, readwrite) uint32_t id_p;

@property(nonatomic, readwrite, copy, null_resettable) NSString *name;

@property(nonatomic, readwrite, strong, null_resettable) NSMutableArray<Calibration*> *calibrationsArray;
/** The number of items in @c calibrationsArray without causing the array to be created. */
@property(nonatomic, readonly) NSUInteger calibrationsArray_Count;

@end

#pragma mark - DetectorInformation

typedef GPB_ENUM(DetectorInformation_FieldNumber) {
  DetectorInformation_FieldNumber_ClassesArray = 1,
};

@interface DetectorInformation : GPBMessage

@property(nonatomic, readwrite, strong, null_resettable) NSMutableArray<ObjectClass*> *classesArray;
/** The number of items in @c classesArray without causing the array to be created. */
@property(nonatomic, readonly) NSUInteger classesArray_Count;

@end

NS_ASSUME_NONNULL_END

CF_EXTERN_C_END

#pragma clang diagnostic pop

// @@protoc_insertion_point(global_scope)
