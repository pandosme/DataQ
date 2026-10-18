const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { execFileSync } = require('node:child_process');
const test = require('node:test');

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
