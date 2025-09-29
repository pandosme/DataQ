#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <glib.h>
#include <time.h>
#include <glib-unix.h>
#include <math.h>
#include "Stitch.h"
#include "cJSON.h"

#define LOG(fmt, args...)      { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_WARN(fmt, args...) { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args); }
#define LOG_TRACE(fmt, args...) {}

typedef struct _StitchHeldPath {
    cJSON* path;
    GTimer* expiry_timer;
} StitchHeldPath;

static gint stitch_active = 0;
static gint stitch_duration = 5;
static gint stitch_x1 = 250, stitch_x2 = 750, stitch_y1 = 250, stitch_y2 = 750;
static double stitch_angle_threshold = 40.0;
static stitch_callback stitch_publish = NULL;

static GMutex stitch_mutex;
static GList* held_paths = NULL;

static double vec_angle(int x1, int y1, int x2, int y2) {
    double dx = x2-x1, dy = y2-y1;
    return atan2(dy, dx);
}

static double angle_between(double a1, double a2) {
    double diff = a1 - a2;
    while(diff > M_PI)  diff -= 2*M_PI;
    while(diff < -M_PI) diff += 2*M_PI;
    return fabs(diff) * 180.0 / M_PI;
}

static double point_distance(int x1, int y1, int x2, int y2) {
    double dx = (x1-x2), dy = (y1-y2);
    return sqrt(dx*dx + dy*dy);
}

static gboolean publish_timeout_cb(gpointer data) {
    StitchHeldPath* hp = (StitchHeldPath*)data;
    g_mutex_lock(&stitch_mutex);
    stitch_publish(hp->path);
    held_paths = g_list_remove(held_paths, hp);
    g_timer_destroy(hp->expiry_timer);
    free(hp);
    g_mutex_unlock(&stitch_mutex);
    return G_SOURCE_REMOVE;
}

static int class_match(cJSON* a, cJSON* b) {
    const char* classA = cJSON_GetObjectItem(a, "class")->valuestring;
    const char* classB = cJSON_GetObjectItem(b, "class")->valuestring;
    return strcmp(classA, classB) == 0;
}

static void get_vecs_for_angle(cJSON* path, int first, double* ang_out) {
    cJSON* arr = cJSON_GetObjectItem(path, "path");
    int n = cJSON_GetArraySize(arr);
    if (first && n >= 2) {
        cJSON* p0 = cJSON_GetArrayItem(arr, 0);
        cJSON* p1 = cJSON_GetArrayItem(arr, 1);
        int x0 = cJSON_GetObjectItem(p0, "x")->valueint;
        int y0 = cJSON_GetObjectItem(p0, "y")->valueint;
        int x1 = cJSON_GetObjectItem(p1, "x")->valueint;
        int y1 = cJSON_GetObjectItem(p1, "y")->valueint;
        *ang_out = vec_angle(x0, y0, x1, y1);
    } else if (!first && n >= 2) {
        cJSON* p0 = cJSON_GetArrayItem(arr, n-2);
        cJSON* p1 = cJSON_GetArrayItem(arr, n-1);
        int x0 = cJSON_GetObjectItem(p0, "x")->valueint;
        int y0 = cJSON_GetObjectItem(p0, "y")->valueint;
        int x1 = cJSON_GetObjectItem(p1, "x")->valueint;
        int y1 = cJSON_GetObjectItem(p1, "y")->valueint;
        *ang_out = vec_angle(x0, y0, x1, y1);
    } else {
        *ang_out = 0;
    }
}

static cJSON* merge_paths(cJSON* a, cJSON* b) {
    // Create new path object (deep copy of a for basis)
    cJSON* merged = cJSON_Duplicate(a, 1);

    // Merge path arrays
    cJSON* arrA = cJSON_GetObjectItem(a, "path");
    cJSON* arrB = cJSON_GetObjectItem(b, "path");
    cJSON* merged_arr = cJSON_CreateArray();
    int nA = cJSON_GetArraySize(arrA), nB = cJSON_GetArraySize(arrB);
    for(int i=0;i<nA;i++) cJSON_AddItemReferenceToArray(merged_arr, cJSON_GetArrayItem(arrA,i));
    for(int i=0;i<nB;i++) cJSON_AddItemReferenceToArray(merged_arr, cJSON_GetArrayItem(arrB,i));
    cJSON_ReplaceItemInObjectCaseSensitive(merged,"path",merged_arr);

    // age = age_a + age_b
    double ageA = cJSON_GetObjectItem(a,"age")->valuedouble;
    double ageB = cJSON_GetObjectItem(b,"age")->valuedouble;
    cJSON_ReplaceItemInObjectCaseSensitive(merged,"age", cJSON_CreateNumber(ageA+ageB));

    // dx = x_last - x_first, dy = y_last - y_first
    cJSON* p0 = cJSON_GetArrayItem(merged_arr, 0);
    cJSON* p1 = cJSON_GetArrayItem(merged_arr, nA+nB-1);
    int x0 = cJSON_GetObjectItem(p0, "x")->valueint;
    int y0 = cJSON_GetObjectItem(p0, "y")->valueint;
    int x1 = cJSON_GetObjectItem(p1, "x")->valueint;
    int y1 = cJSON_GetObjectItem(p1, "y")->valueint;
    cJSON_ReplaceItemInObjectCaseSensitive(merged,"dx", cJSON_CreateNumber(x1-x0));
    cJSON_ReplaceItemInObjectCaseSensitive(merged,"dy", cJSON_CreateNumber(y1-y0));

    // bx/by of merged are from the first item
    cJSON_ReplaceItemInObjectCaseSensitive(merged,"bx",cJSON_CreateNumber(x0));
    cJSON_ReplaceItemInObjectCaseSensitive(merged,"by",cJSON_CreateNumber(y0));

    // timestamp is the timestamp of a
    cJSON_ReplaceItemInObjectCaseSensitive(merged,"timestamp", cJSON_GetObjectItem(a,"timestamp"));

    // dwell = max d in merged
    double maxd = 0.0;
    for(int i=0;i<(nA+nB);i++) {
        cJSON* pt = cJSON_GetArrayItem(merged_arr, i);
        double d = cJSON_GetObjectItem(pt, "d")->valuedouble;
        if(d > maxd) maxd = d;
    }
    cJSON_ReplaceItemInObjectCaseSensitive(merged,"dwell", cJSON_CreateNumber(maxd));

    // distance = a.distance + b.distance + vector length between end of a and start of b / 10
    double distA = cJSON_GetObjectItem(a, "distance")->valuedouble;
    double distB = cJSON_GetObjectItem(b, "distance")->valuedouble;
    cJSON* lastA = cJSON_GetArrayItem(arrA, nA-1);
    cJSON* firstB = cJSON_GetArrayItem(arrB, 0);
    int la_x = cJSON_GetObjectItem(lastA,"x")->valueint;
    int la_y = cJSON_GetObjectItem(lastA,"y")->valueint;
    int fb_x = cJSON_GetObjectItem(firstB,"x")->valueint;
    int fb_y = cJSON_GetObjectItem(firstB,"y")->valueint;
    double between = point_distance(la_x, la_y, fb_x, fb_y) / 10.0;
    cJSON_ReplaceItemInObjectCaseSensitive(merged, "distance", cJSON_CreateNumber(distA+distB+between));
    // Add a merged id or flag if you wish here

    return merged;
}

void STICH_Path(cJSON* path) {
    if(!path) { stitch_publish(path); return; }
    if(!stitch_active) { stitch_publish(path); return; }

    cJSON* arr = cJSON_GetObjectItem(path, "path");
    int n = cJSON_GetArraySize(arr);
    if(n < 1) { stitch_publish(path); return; }

    cJSON* start = cJSON_GetArrayItem(arr, 0);
    cJSON* end = cJSON_GetArrayItem(arr, n-1);

    int sx = cJSON_GetObjectItem(start, "x")->valueint;
    int sy = cJSON_GetObjectItem(start, "y")->valueint;
    int ex = cJSON_GetObjectItem(end, "x")->valueint;
    int ey = cJSON_GetObjectItem(end, "y")->valueint;

    bool birth_in = (sx >= stitch_x1 && sx <= stitch_x2 && sy >= stitch_y1 && sy <= stitch_y2);
    bool death_in = (ex >= stitch_x1 && ex <= stitch_x2 && ey >= stitch_y1 && ey <= stitch_y2);

    if(!birth_in && !death_in) {
        stitch_publish(path);
        return;
    }

    g_mutex_lock(&stitch_mutex);

    // Try to match if both in area OR if only birth in area
    if((birth_in && death_in) || (birth_in && !death_in)) {
LOG("STICH: Finding Match...");
        GList* node = held_paths;
        StitchHeldPath* match = NULL;
        while (node) {
            StitchHeldPath* hp = node->data;
            cJSON* candidate = hp->path;
            if (!class_match(path, candidate)) { node = node->next; continue; }

            double ang_held = 0, ang_incoming = 0;
            get_vecs_for_angle(candidate, 0, &ang_held);
            get_vecs_for_angle(path, 1, &ang_incoming);
            double ang_diff = angle_between(ang_held, ang_incoming);
            if(ang_diff > stitch_angle_threshold) { node = node->next; continue; }

            cJSON* held_arr = cJSON_GetObjectItem(candidate, "path");
            int held_n = cJSON_GetArraySize(held_arr);
            cJSON* held_last = cJSON_GetArrayItem(held_arr, held_n-1);
            cJSON* incom_arr = cJSON_GetObjectItem(path, "path");
            cJSON* incom_first = cJSON_GetArrayItem(incom_arr, 0);
            double time_held = cJSON_GetObjectItem(held_last, "t")->valuedouble;
            double time_incoming = cJSON_GetObjectItem(incom_first, "t")->valuedouble;
            if(fabs(time_incoming-time_held) > stitch_duration) { node = node->next; continue; }

            match = hp;
            break;
        }

        if(match) {
LOG("STICH: Match found");
            cJSON* merged = merge_paths(match->path, path);
            if(!cJSON_GetObjectItem(merged,"stitched"))
                cJSON_AddTrueToObject(merged,"stitched");
            g_source_remove_by_user_data(match);
            held_paths = g_list_remove(held_paths, match);
            g_timer_destroy(match->expiry_timer);
            free(match);
            cJSON_Delete(path);
            // Check last position of merged path
            cJSON* merged_arr = cJSON_GetObjectItem(merged, "path");
            int m_n = cJSON_GetArraySize(merged_arr);
            cJSON* merged_end = cJSON_GetArrayItem(merged_arr, m_n-1);
            int mex = cJSON_GetObjectItem(merged_end, "x")->valueint;
            int mey = cJSON_GetObjectItem(merged_end, "y")->valueint;
            bool merged_death_in = (mex >= stitch_x1 && mex <= stitch_x2 && mey >= stitch_y1 && mey <= stitch_y2);

            if(merged_death_in) {
                // Hold merged path for future matching
                StitchHeldPath* hp_new = malloc(sizeof(StitchHeldPath));
                hp_new->path = merged;
                hp_new->expiry_timer = g_timer_new();
                held_paths = g_list_append(held_paths, hp_new);
                g_timeout_add_seconds(stitch_duration, publish_timeout_cb, hp_new);
            } else {
                stitch_publish(merged);
            }
            g_mutex_unlock(&stitch_mutex);
            return;
        }

        // If both in area but no match, publish immediately
        if(birth_in && death_in) {
            stitch_publish(path);
            g_mutex_unlock(&stitch_mutex);
            return;
        }
        // If birth in and death out, publish immediately if no match found
        if(birth_in && !death_in) {
            stitch_publish(path);
            g_mutex_unlock(&stitch_mutex);
            return;
        }
    }

    // If death in, birth out: hold for matching
    if(!birth_in && death_in) {
LOG("STICH: Hold to match...");
        StitchHeldPath* hp = malloc(sizeof(StitchHeldPath));
        hp->path = path;
        hp->expiry_timer = g_timer_new();
        held_paths = g_list_append(held_paths, hp);
        g_timeout_add_seconds(stitch_duration, publish_timeout_cb, hp);
        g_mutex_unlock(&stitch_mutex);
        return;
    }

    g_mutex_unlock(&stitch_mutex); // Cover all code paths
}

int  STICH_Settings(cJSON* settings) {
    if(!settings) return 0;
    stitch_active = cJSON_GetObjectItem(settings,"active")?cJSON_GetObjectItem(settings,"active")->type==cJSON_True?1:0:0;
    stitch_duration = cJSON_GetObjectItem(settings,"duration")?cJSON_GetObjectItem(settings,"duration")->valueint:5;
    stitch_x1 = cJSON_GetObjectItem(settings,"x1")?cJSON_GetObjectItem(settings,"x1")->valueint:250;
    stitch_x2 = cJSON_GetObjectItem(settings,"x2")?cJSON_GetObjectItem(settings,"x2")->valueint:750;
    stitch_y1 = cJSON_GetObjectItem(settings,"y1")?cJSON_GetObjectItem(settings,"y1")->valueint:250;
    stitch_y2 = cJSON_GetObjectItem(settings,"y2")?cJSON_GetObjectItem(settings,"y2")->valueint:750;
    stitch_angle_threshold = cJSON_GetObjectItem(settings,"angle_threshold")?
        cJSON_GetObjectItem(settings,"angle_threshold")->valuedouble:40.0;
    return 1;
}

int  STICH_Init(stitch_callback cb) {
    if(!cb) return 0;
    stitch_publish = cb;
    g_mutex_init(&stitch_mutex);
    held_paths = NULL;
    return 1;
}
