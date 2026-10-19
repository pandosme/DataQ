const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { execFileSync } = require('node:child_process');
const test = require('node:test');

test('stops VOD initialization and releases resources at each failed stage', () => {
    const app = path.resolve(__dirname, '../app');
    const source = fs.readFileSync(path.join(app, 'VOD.c'), 'utf8');
    const start = source.indexOf('int VOD_Init(');
    const end = source.indexOf('void VOD_Shutdown(', start);
    assert(start >= 0 && end > start);
    const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'dataq-vod-test-'));
    const harness = `
#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "VOD.h"
#include "video_object_detection_subscriber.h"
#include "video_object_detection.pb-c.h"
#define LOG(...) ((void)0)
#define LOG_WARN(...) ((void)0)
#define LOG_TRACE(...) ((void)0)
static int locked, class_queries, failure_stage, deleted, classes_freed, info_freed;
static int vod_mutex, vod_predictions, consecutive_timeouts, watchdog_timer_id;
static double last_detection_time, DETECTION_TIMEOUT_SECONDS;
static int MAX_CONSECUTIVE_TIMEOUTS;
static bool recovery_in_progress;
static video_object_detection_subscriber_t *subscriber;
static struct {
    vod_callback_t cb;
    void *user_data;
    int num_classes;
    video_object_detection_subscriber_class_t **det_classes;
    VOD__DetectorInformation *det_info;
} g_ctx;
#define pthread_mutex_lock(...) (++locked)
#define pthread_mutex_unlock(...) (--locked)
#define g_get_real_time() 0
#define g_timeout_add_seconds(...) 1
int video_object_detection_subscriber_get_version(int *major, int *minor) {
    *major = 0; *minor = 0;
    return failure_stage == 0 ? VIDEO_OBJECT_DETECTION_SUBSCRIBER_ERROR_UNAUTHORIZED : 0;
}
int video_object_detection_subscriber_det_classes_get(video_object_detection_subscriber_class_t ***classes) {
    ++class_queries;
    if (failure_stage == 1) return -5;
    *classes = (video_object_detection_subscriber_class_t **)1;
    return 1;
}
int video_object_detection_subscriber_get_detector_information(uint8_t **buffer, size_t *size) {
    *buffer = NULL; *size = 0;
    return failure_stage == 2 ? -5 : 0;
}
#define vod__detector_information__unpack(...) (failure_stage == 3 ? NULL : (VOD__DetectorInformation *)1)
int video_object_detection_subscriber_create(video_object_detection_subscriber_t **handle, void *data, int channel) {
    (void)data;
    assert(channel == 0);
    if (failure_stage == 4) return VIDEO_OBJECT_DETECTION_SUBSCRIBER_ERROR_VOD_DISABLED;
    *handle = (video_object_detection_subscriber_t *)1;
    return 0;
}
#define video_object_detection_subscriber_set_get_detection_callback(...) (failure_stage == 5 ? -5 : 0)
#define video_object_detection_subscriber_set_receive_empty_hits(handle, enabled) (assert(enabled == true), failure_stage == 6 ? -5 : 0)
#define video_object_detection_subscriber_subscribe(...) (failure_stage == 7 ? -5 : 0)
int video_object_detection_subscriber_delete(video_object_detection_subscriber_t **handle) {
    ++deleted; *handle = NULL; return 0;
}
void video_object_detection_subscriber_det_classes_free(video_object_detection_subscriber_class_t **classes, int count) {
    (void)classes; assert(count == 1); ++classes_freed;
}
void vod__detector_information__free_unpacked(VOD__DetectorInformation *info, ProtobufCAllocator *allocator) {
    (void)info; (void)allocator; ++info_freed;
}
static void callback(const vod_object_t *objects, size_t count, void *data) {
    (void)objects; (void)count; (void)data;
}
${source.slice(start, end)}
int main(void) {
    for (failure_stage = 0; failure_stage < 8; ++failure_stage) {
        class_queries = deleted = classes_freed = info_freed = 0;
        int result = VOD_Init(0, callback, NULL, 0);
        assert(result != 0);
        if (failure_stage == 4) {
            assert(result == VIDEO_OBJECT_DETECTION_SUBSCRIBER_ERROR_VOD_DISABLED);
        }
        assert(class_queries == (failure_stage > 0));
        assert(classes_freed == (failure_stage >= 2));
        assert(info_freed == (failure_stage >= 4));
        assert(deleted == (failure_stage >= 5));
        assert(subscriber == NULL && g_ctx.det_classes == NULL && g_ctx.det_info == NULL);
        assert(locked == 0);
    }
    return 0;
}
`;
    try {
        const input = path.join(directory, 'test.c');
        const executable = path.join(directory, 'test');
        fs.writeFileSync(input, harness);
        execFileSync('cc', ['-std=c99', '-I', app, input, '-o', executable]);
        execFileSync(executable);
    } finally {
        fs.rmSync(directory, { recursive: true, force: true });
    }
});

test('publishes VOD metadata labels even when subscribing fails', () => {
    const app = path.resolve(__dirname, '../app');
    const source = fs.readFileSync(path.join(app, 'ObjectDetection.c'), 'utf8');
    const start = source.indexOf('int ObjectDetection_Init(');
    const end = source.indexOf('cJSON* ObjectDetection_Labels(', start);
    assert(start >= 0 && end > start);
    const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'dataq-label-test-'));
    const harness = `
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "cJSON.h"
#include "ObjectDetection.h"
#include "VOD.h"
#define LOG(...) ((void)0)
#define LOG_WARN(...) ((void)0)
#define LOG_TRACE(...) ((void)0)
#define g_mutex_lock(...) ((void)0)
#define g_mutex_unlock(...) ((void)0)
#define g_hash_table_new_full(...) ((void *)1)
#define g_timeout_add_seconds(...) (++timers)
static void *detectionCache;
static ObjectDetection_Callback detectionsCallback;
static TrackerDetection_Callback trackerCallback;
static int subscription_result, timers, metadata_available = 1;
static cJSON *stored_labels;
static cJSON *ACAP_Get_Config(const char *name) { (void)name; return NULL; }
static const char *NiceName(const char *name) {
    if (name && strcmp(name, "human") == 0) return "Human";
    if (name && strcmp(name, "car") == 0) return "Car";
    return name;
}
static void ACAP_STATUS_SetObject(const char *group, const char *key, cJSON *value) {
    assert(strcmp(group, "detections") == 0 && strcmp(key, "labels") == 0);
    cJSON_Delete(stored_labels);
    stored_labels = cJSON_Duplicate(value, 1);
}
cJSON *VOD_Label_List(void) {
    if (!metadata_available) return NULL;
    return cJSON_Parse("[{\\"id\\":\\"human\\"},{\\"id\\":\\"car\\"}]");
}
static void VOD_Data(const vod_object_t *objects, size_t count, void *data) {
    (void)objects; (void)count; (void)data;
}
int VOD_Init(int channel, vod_callback_t callback, void *data, int predictions) {
    (void)channel; (void)callback; (void)data; (void)predictions;
    assert(stored_labels != NULL);
    return subscription_result;
}
${source.slice(start, end)}
int main(void) {
    subscription_result = -5;
    assert(ObjectDetection_Init(NULL, NULL) == 0);
    assert(timers == 0);
    assert(cJSON_GetArraySize(stored_labels) == 2);
    assert(strcmp(cJSON_GetArrayItem(stored_labels, 0)->valuestring, "Human") == 0);
    assert(strcmp(cJSON_GetArrayItem(stored_labels, 1)->valuestring, "Car") == 0);
    subscription_result = 0;
    assert(ObjectDetection_Init(NULL, NULL) == 1);
    assert(timers == 1 && cJSON_GetArraySize(stored_labels) == 2);
    metadata_available = 0;
    subscription_result = -5;
    assert(ObjectDetection_Init(NULL, NULL) == 0);
    assert(cJSON_GetArraySize(stored_labels) == 0);
    cJSON_Delete(stored_labels);
    return 0;
}
`;
    try {
        const input = path.join(directory, 'test.c');
        const executable = path.join(directory, 'test');
        fs.writeFileSync(input, harness);
        execFileSync('cc', ['-std=c99', '-Wall', '-Wextra', '-Werror', '-Wno-misleading-indentation',
            '-I', app, input, path.join(app, 'cJSON.c'), '-lm', '-o', executable]);
        execFileSync(executable);
    } finally {
        fs.rmSync(directory, { recursive: true, force: true });
    }
});
