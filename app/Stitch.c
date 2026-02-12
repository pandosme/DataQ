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
static gint stitch_allow_class_switch = 0;
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
    if(!a || !b) return 0;
    cJSON* clsA = cJSON_GetObjectItem(a, "class");
    cJSON* clsB = cJSON_GetObjectItem(b, "class");
    if(!clsA || !clsB || !clsA->valuestring || !clsB->valuestring) return 0;
    return strcmp(clsA->valuestring, clsB->valuestring) == 0;
}

static void get_vecs_for_angle(cJSON* path, int first, double* ang_out) {
    if(!path || !ang_out) return;
    cJSON* arr = cJSON_GetObjectItem(path, "path");
    if(!arr) { *ang_out = 0; return; }
    int n = cJSON_GetArraySize(arr);
    if (first && n >= 2) {
        cJSON* p0 = cJSON_GetArrayItem(arr, 0);
        cJSON* p1 = cJSON_GetArrayItem(arr, 1);
        if(!p0 || !p1) { *ang_out = 0; return; }
        cJSON* x0_obj = cJSON_GetObjectItem(p0, "x");
        cJSON* y0_obj = cJSON_GetObjectItem(p0, "y");
        cJSON* x1_obj = cJSON_GetObjectItem(p1, "x");
        cJSON* y1_obj = cJSON_GetObjectItem(p1, "y");
        if(!x0_obj || !y0_obj || !x1_obj || !y1_obj) { *ang_out = 0; return; }
        int x0 = x0_obj->valueint;
        int y0 = y0_obj->valueint;
        int x1 = x1_obj->valueint;
        int y1 = y1_obj->valueint;
        *ang_out = vec_angle(x0, y0, x1, y1);
    } else if (!first && n >= 2) {
        cJSON* p0 = cJSON_GetArrayItem(arr, n-2);
        cJSON* p1 = cJSON_GetArrayItem(arr, n-1);
        if(!p0 || !p1) { *ang_out = 0; return; }
        cJSON* x0_obj = cJSON_GetObjectItem(p0, "x");
        cJSON* y0_obj = cJSON_GetObjectItem(p0, "y");
        cJSON* x1_obj = cJSON_GetObjectItem(p1, "x");
        cJSON* y1_obj = cJSON_GetObjectItem(p1, "y");
        if(!x0_obj || !y0_obj || !x1_obj || !y1_obj) { *ang_out = 0; return; }
        int x0 = x0_obj->valueint;
        int y0 = y0_obj->valueint;
        int x1 = x1_obj->valueint;
        int y1 = y1_obj->valueint;
        *ang_out = vec_angle(x0, y0, x1, y1);
    } else {
        *ang_out = 0;
    }
}

static cJSON* merge_paths(cJSON* a, cJSON* b) {
    if(!a || !b) return NULL;

    // Create new path object (deep copy of a for basis)
    cJSON* merged = cJSON_Duplicate(a, 1);
    if(!merged) return NULL;

    // Merge path arrays - MUST use Duplicate to create copies, not references!
    cJSON* arrA = cJSON_GetObjectItem(a, "path");
    cJSON* arrB = cJSON_GetObjectItem(b, "path");
    if(!arrA || !arrB) {
        cJSON_Delete(merged);
        return NULL;
    }
    cJSON* merged_arr = cJSON_CreateArray();
    if(!merged_arr) {
        cJSON_Delete(merged);
        return NULL;
    }
    int nA = cJSON_GetArraySize(arrA), nB = cJSON_GetArraySize(arrB);
    for(int i=0;i<nA;i++) {
        cJSON* item = cJSON_GetArrayItem(arrA, i);
        if(item) cJSON_AddItemToArray(merged_arr, cJSON_Duplicate(item, 1));
    }
    for(int i=0;i<nB;i++) {
        cJSON* item = cJSON_GetArrayItem(arrB, i);
        if(item) cJSON_AddItemToArray(merged_arr, cJSON_Duplicate(item, 1));
    }
    cJSON_ReplaceItemInObjectCaseSensitive(merged,"path",merged_arr);

    // age = age_a + age_b
    cJSON* ageA_obj = cJSON_GetObjectItem(a,"age");
    cJSON* ageB_obj = cJSON_GetObjectItem(b,"age");
    double ageA = ageA_obj ? ageA_obj->valuedouble : 0.0;
    double ageB = ageB_obj ? ageB_obj->valuedouble : 0.0;
    cJSON_ReplaceItemInObjectCaseSensitive(merged,"age", cJSON_CreateNumber(ageA+ageB));

    // If allow_class_switch is enabled, use class from path with highest confidence
    if(stitch_allow_class_switch) {
        cJSON* confA_obj = cJSON_GetObjectItem(a,"confidence");
        cJSON* confB_obj = cJSON_GetObjectItem(b,"confidence");
        int confA = confA_obj ? confA_obj->valueint : 0;
        int confB = confB_obj ? confB_obj->valueint : 0;

        // If B has higher confidence, use its class
        if(confB > confA) {
            cJSON* classB = cJSON_GetObjectItem(b,"class");
            if(classB && classB->valuestring) {
                cJSON_ReplaceItemInObjectCaseSensitive(merged,"class", cJSON_CreateString(classB->valuestring));
            }
        }
        // Also update confidence to the maximum
        cJSON_ReplaceItemInObjectCaseSensitive(merged,"confidence", cJSON_CreateNumber(confA > confB ? confA : confB));
    }

    // dx = x_last - x_first, dy = y_last - y_first
    int totalPoints = cJSON_GetArraySize(merged_arr);
    if(totalPoints < 1) {
        cJSON_Delete(merged);
        return NULL;
    }
    cJSON* p0 = cJSON_GetArrayItem(merged_arr, 0);
    cJSON* p1 = cJSON_GetArrayItem(merged_arr, totalPoints-1);
    if(!p0 || !p1) {
        cJSON_Delete(merged);
        return NULL;
    }
    cJSON* x0_obj = cJSON_GetObjectItem(p0, "x");
    cJSON* y0_obj = cJSON_GetObjectItem(p0, "y");
    cJSON* x1_obj = cJSON_GetObjectItem(p1, "x");
    cJSON* y1_obj = cJSON_GetObjectItem(p1, "y");
    if(!x0_obj || !y0_obj || !x1_obj || !y1_obj) {
        cJSON_Delete(merged);
        return NULL;
    }
    int x0 = x0_obj->valueint;
    int y0 = y0_obj->valueint;
    int x1 = x1_obj->valueint;
    int y1 = y1_obj->valueint;
    cJSON_ReplaceItemInObjectCaseSensitive(merged,"dx", cJSON_CreateNumber(x1-x0));
    cJSON_ReplaceItemInObjectCaseSensitive(merged,"dy", cJSON_CreateNumber(y1-y0));

    // bx/by of merged are from the first item
    cJSON_ReplaceItemInObjectCaseSensitive(merged,"bx",cJSON_CreateNumber(x0));
    cJSON_ReplaceItemInObjectCaseSensitive(merged,"by",cJSON_CreateNumber(y0));

    // timestamp is the timestamp of a - must duplicate, not reference!
    cJSON* timestamp_a = cJSON_GetObjectItem(a,"timestamp");
    if(timestamp_a)
        cJSON_ReplaceItemInObjectCaseSensitive(merged,"timestamp", cJSON_Duplicate(timestamp_a, 1));

    // dwell = max d in merged
    double maxd = 0.0;
    for(int i=0;i<totalPoints;i++) {
        cJSON* pt = cJSON_GetArrayItem(merged_arr, i);
        if(pt) {
            cJSON* d_obj = cJSON_GetObjectItem(pt, "d");
            if(d_obj) {
                double d = d_obj->valuedouble;
                if(d > maxd) maxd = d;
            }
        }
    }
    cJSON_ReplaceItemInObjectCaseSensitive(merged,"dwell", cJSON_CreateNumber(maxd));

    // distance = a.distance + b.distance + vector length between end of a and start of b / 10
    cJSON* distA_obj = cJSON_GetObjectItem(a, "distance");
    cJSON* distB_obj = cJSON_GetObjectItem(b, "distance");
    double distA = distA_obj ? distA_obj->valuedouble : 0.0;
    double distB = distB_obj ? distB_obj->valuedouble : 0.0;
    double between = 0.0;
    if(nA > 0 && nB > 0) {
        cJSON* lastA = cJSON_GetArrayItem(arrA, nA-1);
        cJSON* firstB = cJSON_GetArrayItem(arrB, 0);
        if(lastA && firstB) {
            cJSON* la_x_obj = cJSON_GetObjectItem(lastA,"x");
            cJSON* la_y_obj = cJSON_GetObjectItem(lastA,"y");
            cJSON* fb_x_obj = cJSON_GetObjectItem(firstB,"x");
            cJSON* fb_y_obj = cJSON_GetObjectItem(firstB,"y");
            if(la_x_obj && la_y_obj && fb_x_obj && fb_y_obj) {
                int la_x = la_x_obj->valueint;
                int la_y = la_y_obj->valueint;
                int fb_x = fb_x_obj->valueint;
                int fb_y = fb_y_obj->valueint;
                between = point_distance(la_x, la_y, fb_x, fb_y) / 10.0;
            }
        }
    }
    cJSON_ReplaceItemInObjectCaseSensitive(merged, "distance", cJSON_CreateNumber(distA+distB+between));
    // Add a merged id or flag if you wish here

    return merged;
}

void Stitch_Path(cJSON* path) {
    if(!path) { stitch_publish(path); return; }
    if(!stitch_active) { stitch_publish(path); return; }

    cJSON* arr = cJSON_GetObjectItem(path, "path");
    if(!arr) { stitch_publish(path); return; }
    int n = cJSON_GetArraySize(arr);
    if(n < 2) { stitch_publish(path); return; }  // Require minimum 2 points for direction matching

    cJSON* start = cJSON_GetArrayItem(arr, 0);
    cJSON* end = cJSON_GetArrayItem(arr, n-1);
    if(!start || !end) { stitch_publish(path); return; }

    cJSON* sx_obj = cJSON_GetObjectItem(start, "x");
    cJSON* sy_obj = cJSON_GetObjectItem(start, "y");
    cJSON* ex_obj = cJSON_GetObjectItem(end, "x");
    cJSON* ey_obj = cJSON_GetObjectItem(end, "y");
    if(!sx_obj || !sy_obj || !ex_obj || !ey_obj) { stitch_publish(path); return; }

    int sx = sx_obj->valueint;
    int sy = sy_obj->valueint;
    int ex = ex_obj->valueint;
    int ey = ey_obj->valueint;

    bool birth_in = (sx >= stitch_x1 && sx <= stitch_x2 && sy >= stitch_y1 && sy <= stitch_y2);
    bool death_in = (ex >= stitch_x1 && ex <= stitch_x2 && ey >= stitch_y1 && ey <= stitch_y2);

    if(!birth_in && !death_in) {
        stitch_publish(path);
        return;
    }

    g_mutex_lock(&stitch_mutex);

    // Try to match if both in area OR if only birth in area
    if((birth_in && death_in) || (birth_in && !death_in)) {
        GList* node = held_paths;
        StitchHeldPath* match = NULL;
        while (node) {
            StitchHeldPath* hp = node->data;
            if(!hp) { LOG_WARN("STICH: NULL hp in list\n"); node = node->next; continue; }
            cJSON* candidate = hp->path;
            if(!candidate) { LOG_WARN("STICH: NULL candidate path\n"); node = node->next; continue; }
            // Only check class match if allow_class_switch is disabled
            if (!stitch_allow_class_switch && !class_match(path, candidate)) { node = node->next; continue; }

            // Check angle match only if angle_threshold > 0 (0 = disabled)
            if(stitch_angle_threshold > 0.0) {
                double ang_held = 0, ang_incoming = 0;
                get_vecs_for_angle(candidate, 0, &ang_held);
                get_vecs_for_angle(path, 1, &ang_incoming);
                double ang_diff = angle_between(ang_held, ang_incoming);
                if(ang_diff > stitch_angle_threshold) { node = node->next; continue; }
            }

            cJSON* held_arr = cJSON_GetObjectItem(candidate, "path");
            cJSON* incom_arr = cJSON_GetObjectItem(path, "path");
            if(!held_arr || !incom_arr) { node = node->next; continue; }
            int held_n = cJSON_GetArraySize(held_arr);
            if(held_n < 2) { node = node->next; continue; }  // Require minimum 2 points
            cJSON* held_last = cJSON_GetArrayItem(held_arr, held_n-1);
            cJSON* incom_first = cJSON_GetArrayItem(incom_arr, 0);
            if(!held_last || !incom_first) { node = node->next; continue; }
            cJSON* t_held = cJSON_GetObjectItem(held_last, "t");
            cJSON* t_incom = cJSON_GetObjectItem(incom_first, "t");
            if(!t_held || !t_incom) { node = node->next; continue; }
            double time_held = t_held->valuedouble;
            double time_incoming = t_incom->valuedouble;
            if(fabs(time_incoming-time_held) > stitch_duration) { node = node->next; continue; }

            match = hp;
            break;
        }

        if(match) {
            cJSON* merged = merge_paths(match->path, path);
            if(!merged) {
                LOG_WARN("STICH: merge_paths failed\n");
                g_mutex_unlock(&stitch_mutex);
                stitch_publish(path);
                return;
            }
            if(!cJSON_GetObjectItem(merged,"stitched"))
                cJSON_AddTrueToObject(merged,"stitched");
            g_source_remove_by_user_data(match);
            held_paths = g_list_remove(held_paths, match);
            g_timer_destroy(match->expiry_timer);
            cJSON_Delete(match->path);  // Free the held path's cJSON object
            free(match);
            cJSON_Delete(path);
            // Check last position of merged path
            cJSON* merged_arr = cJSON_GetObjectItem(merged, "path");
            if(!merged_arr || cJSON_GetArraySize(merged_arr) < 1) {
                LOG_WARN("STICH: Invalid merged path array\n");
                cJSON_Delete(merged);
                g_mutex_unlock(&stitch_mutex);
                return;
            }
            int m_n = cJSON_GetArraySize(merged_arr);
            cJSON* merged_end = cJSON_GetArrayItem(merged_arr, m_n-1);
            if(!merged_end) {
                LOG_WARN("STICH: Invalid merged path end\n");
                cJSON_Delete(merged);
                g_mutex_unlock(&stitch_mutex);
                return;
            }
            cJSON* mex_obj = cJSON_GetObjectItem(merged_end, "x");
            cJSON* mey_obj = cJSON_GetObjectItem(merged_end, "y");
            if(!mex_obj || !mey_obj) {
                LOG_WARN("STICH: Invalid merged path end coordinates\n");
                cJSON_Delete(merged);
                g_mutex_unlock(&stitch_mutex);
                return;
            }
            int mex = mex_obj->valueint;
            int mey = mey_obj->valueint;
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

        // If both in area but no match, publish immediately (short path that stays in area)
        if(birth_in && death_in) {
            stitch_publish(path);
            g_mutex_unlock(&stitch_mutex);
            return;
        }
        // If birth in and death out without match: fall through to hold for matching
    }

    // Hold path for future matching
    // This handles:
    //   1. !birth_in && death_in (path entering area)
    //   2. birth_in && !death_in without match (path leaving area)
    StitchHeldPath* hp = malloc(sizeof(StitchHeldPath));
    hp->path = path;
    hp->expiry_timer = g_timer_new();
    held_paths = g_list_append(held_paths, hp);
    g_timeout_add_seconds(stitch_duration, publish_timeout_cb, hp);
    g_mutex_unlock(&stitch_mutex);
}

int  Stitch_Settings(cJSON* settings) {
    if(!settings) return 0;

    g_mutex_lock(&stitch_mutex);

    // Clear all held paths when settings change to avoid inconsistent state
    GList* node = held_paths;
    while (node) {
        StitchHeldPath* hp = node->data;
        GList* next = node->next;

        // Publish the held path immediately
        if (hp->path && stitch_publish) {
            stitch_publish(hp->path);
        }

        // Clean up
        g_source_remove_by_user_data(hp);
        g_timer_destroy(hp->expiry_timer);
        free(hp);

        node = next;
    }
    g_list_free(held_paths);
    held_paths = NULL;

    // Update settings
    stitch_active = cJSON_GetObjectItem(settings,"active")?cJSON_GetObjectItem(settings,"active")->type==cJSON_True?1:0:0;
    stitch_duration = cJSON_GetObjectItem(settings,"duration")?cJSON_GetObjectItem(settings,"duration")->valueint:5;
    stitch_x1 = cJSON_GetObjectItem(settings,"x1")?cJSON_GetObjectItem(settings,"x1")->valueint:250;
    stitch_x2 = cJSON_GetObjectItem(settings,"x2")?cJSON_GetObjectItem(settings,"x2")->valueint:750;
    stitch_y1 = cJSON_GetObjectItem(settings,"y1")?cJSON_GetObjectItem(settings,"y1")->valueint:250;
    stitch_y2 = cJSON_GetObjectItem(settings,"y2")?cJSON_GetObjectItem(settings,"y2")->valueint:750;
    stitch_angle_threshold = cJSON_GetObjectItem(settings,"angle_threshold")?
        cJSON_GetObjectItem(settings,"angle_threshold")->valuedouble:40.0;
    stitch_allow_class_switch = cJSON_GetObjectItem(settings,"allow_class_switch")?
        cJSON_GetObjectItem(settings,"allow_class_switch")->type==cJSON_True?1:0:0;

    g_mutex_unlock(&stitch_mutex);

    LOG("Stitch settings updated: active=%d, area=[%d,%d,%d,%d], duration=%d, angle=%0.1f, allow_class_switch=%d\n",
        stitch_active, stitch_x1, stitch_y1, stitch_x2, stitch_y2, stitch_duration, stitch_angle_threshold, stitch_allow_class_switch);

    return 1;
}

int  Stitch_Init(stitch_callback cb) {
    if(!cb) return 0;
    stitch_publish = cb;
    g_mutex_init(&stitch_mutex);
    held_paths = NULL;
    return 1;
}
