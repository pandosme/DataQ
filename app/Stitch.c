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

// Multiplier for how long to keep a held path in memory.
// stitch_duration controls the *matching window* (max time hidden behind obstruction).
// The hold timeout must be much larger to outlive the person's full journey across the scene.
#define STITCH_HOLD_TIMEOUT_MULTIPLIER 24

typedef struct _StitchHeldPath {
    cJSON* path;
    GTimer* expiry_timer;
    bool is_birth_in;   // true = born inside stitch area (re-emergence); false = died inside (pre-occlusion)
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

// Compute travel direction using up to MAX_ANGLE_PTS consecutive pairs
// (either at the start or end of a path) and return their circular mean.
// This is far more robust than using just a single noisy pair.
#define MAX_ANGLE_PTS 5
static void get_vecs_for_angle(cJSON* path, int first, double* ang_out) {
    if(!path || !ang_out) { if(ang_out) *ang_out = 0; return; }
    cJSON* arr = cJSON_GetObjectItem(path, "path");
    if(!arr) { *ang_out = 0; return; }
    int n = cJSON_GetArraySize(arr);
    if(n < 2) { *ang_out = 0; return; }

    int pairs = n - 1;
    if(pairs > MAX_ANGLE_PTS) pairs = MAX_ANGLE_PTS;

    double sin_sum = 0.0, cos_sum = 0.0;
    int count = 0;

    for(int i = 0; i < pairs; i++) {
        int idx0, idx1;
        if(first) {
            idx0 = i;
            idx1 = i + 1;
        } else {
            idx0 = n - 2 - i;
            idx1 = n - 1 - i;
        }
        cJSON* p0 = cJSON_GetArrayItem(arr, idx0);
        cJSON* p1 = cJSON_GetArrayItem(arr, idx1);
        if(!p0 || !p1) continue;
        cJSON* x0o = cJSON_GetObjectItem(p0, "x"); cJSON* y0o = cJSON_GetObjectItem(p0, "y");
        cJSON* x1o = cJSON_GetObjectItem(p1, "x"); cJSON* y1o = cJSON_GetObjectItem(p1, "y");
        if(!x0o || !y0o || !x1o || !y1o) continue;
        double a = vec_angle(x0o->valueint, y0o->valueint, x1o->valueint, y1o->valueint);
        sin_sum += sin(a);
        cos_sum += cos(a);
        count++;
    }

    if(count == 0) { *ang_out = 0; return; }
    *ang_out = atan2(sin_sum / count, cos_sum / count);
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

/*
 * Stitch_Path — core stitching logic
 *
 * Terminology:
 *   death_in path  — started outside the stitch area, tracker lost inside (pre-occlusion leg)
 *   birth_in path  — tracker born inside the stitch area, exited outside (post-occlusion leg)
 *
 * Both types are HELD in held_paths when no immediate match is found.
 * Matching can occur in either order (death_in before birth_in is the normal case,
 * but birth_in can arrive first due to processing timing).
 *
 * The hold timeout is stitch_duration * STITCH_HOLD_TIMEOUT_MULTIPLIER seconds —
 * much larger than stitch_duration itself, because a person can walk across the
 * scene for a long time AFTER reappearing, and Stitch_Path is only called when
 * the post-occlusion tracker DIES.  The embedded "t" timestamps in path points
 * are what determine the actual matching window (must be <= stitch_duration apart).
 *
 * Best-match scoring: all candidates are evaluated; the one with the lowest
 * combined angular + temporal penalty is chosen.
 */
void Stitch_Path(cJSON* path) {
    if(!path) return;
    if(!stitch_active) { stitch_publish(path); return; }

    cJSON* arr = cJSON_GetObjectItem(path, "path");
    if(!arr) { stitch_publish(path); return; }
    int n = cJSON_GetArraySize(arr);
    if(n < 2) { stitch_publish(path); return; }

    cJSON* start = cJSON_GetArrayItem(arr, 0);
    cJSON* end   = cJSON_GetArrayItem(arr, n-1);
    if(!start || !end) { stitch_publish(path); return; }

    cJSON* sx_obj = cJSON_GetObjectItem(start, "x"); cJSON* sy_obj = cJSON_GetObjectItem(start, "y");
    cJSON* ex_obj = cJSON_GetObjectItem(end,   "x"); cJSON* ey_obj = cJSON_GetObjectItem(end,   "y");
    if(!sx_obj || !sy_obj || !ex_obj || !ey_obj) { stitch_publish(path); return; }

    int sx = sx_obj->valueint, sy = sy_obj->valueint;
    int ex = ex_obj->valueint, ey = ey_obj->valueint;

    bool birth_in = (sx >= stitch_x1 && sx <= stitch_x2 && sy >= stitch_y1 && sy <= stitch_y2);
    bool death_in = (ex >= stitch_x1 && ex <= stitch_x2 && ey >= stitch_y1 && ey <= stitch_y2);

    /* Path touches neither side of stitch area → pass through unchanged */
    if(!birth_in && !death_in) {
        stitch_publish(path);
        return;
    }

    g_mutex_lock(&stitch_mutex);

    /* ------------------------------------------------------------------
     * Find the BEST candidate from held_paths.
     *
     * Normal  order: held=death_in  + incoming=birth_in
     *   → person disappeared into area, now re-emerging
     *   → temporal check: incoming's first "t" minus held's last "t" in [0, stitch_duration]
     *
     * Reversed order: held=birth_in + incoming=death_in
     *   → race condition: post-occlusion path was born and finished before the
     *     pre-occlusion path was finalized
     *   → temporal check: held's first "t" minus incoming's last "t" in [0, stitch_duration]
     * ------------------------------------------------------------------ */
    StitchHeldPath* best_match = NULL;
    double           best_score = 1e18;

    for(GList* node = held_paths; node != NULL; node = node->next) {
        StitchHeldPath* hp = node->data;
        if(!hp || !hp->path) { LOG_WARN("STICH: NULL entry in held_paths\n"); continue; }
        cJSON* candidate = hp->path;

        cJSON* cand_arr = cJSON_GetObjectItem(candidate, "path");
        if(!cand_arr) continue;
        int cn = cJSON_GetArraySize(cand_arr);
        if(cn < 2) continue;

        /* Determine which is the older / newer path and derive the time gap */
        cJSON* old_path = NULL;
        cJSON* new_path = NULL;
        double time_diff = -1.0;

        if(!hp->is_birth_in && birth_in) {
            /* Normal: held=death_in (older), incoming=birth_in (newer) */
            cJSON* t_held_last  = cJSON_GetObjectItem(cJSON_GetArrayItem(cand_arr, cn-1), "t");
            cJSON* t_incom_first = cJSON_GetObjectItem(cJSON_GetArrayItem(arr,      0),   "t");
            if(!t_held_last || !t_incom_first) continue;
            time_diff = t_incom_first->valuedouble - t_held_last->valuedouble;
            old_path = candidate;
            new_path = path;

        } else if(hp->is_birth_in && death_in) {
            /* Reversed: held=birth_in (newer path arrived early), incoming=death_in (older) */
            cJSON* t_incom_last  = cJSON_GetObjectItem(cJSON_GetArrayItem(arr,      n-1), "t");
            cJSON* t_held_first  = cJSON_GetObjectItem(cJSON_GetArrayItem(cand_arr, 0),   "t");
            if(!t_incom_last || !t_held_first) continue;
            time_diff = t_held_first->valuedouble - t_incom_last->valuedouble;
            old_path = path;        /* incoming death_in is the older segment */
            new_path = candidate;   /* held birth_in is the newer segment */

        } else {
            continue; /* same type (both death_in or both birth_in) — skip */
        }

        /* Time must be non-negative and within the matching window */
        if(time_diff < 0.0 || time_diff > (double)stitch_duration) continue;

        /* Class constraint */
        if(!stitch_allow_class_switch && !class_match(path, candidate)) continue;

        /* Angle between the exit direction of the older path and the entry
         * direction of the newer path — using circular mean of up to 5 vectors */
        double ang_diff = 0.0;
        if(stitch_angle_threshold > 0.0) {
            double ang_old = 0.0, ang_new = 0.0;
            get_vecs_for_angle(old_path, 0, &ang_old); /* last direction  */
            get_vecs_for_angle(new_path, 1, &ang_new); /* first direction */
            ang_diff = angle_between(ang_old, ang_new);
            if(ang_diff > stitch_angle_threshold) continue;
        }

        /* Score: lower is better — weight time gap more than angle */
        double score = ang_diff + time_diff * 5.0;
        if(score < best_score) {
            best_score  = score;
            best_match  = hp;
        }
    }

    /* ------------------------------------------------------------------
     * Match found → merge old + new, then decide whether to hold or publish
     * ------------------------------------------------------------------ */
    if(best_match) {
        cJSON* candidate = best_match->path;

        /* Establish merge order (always old segment first) */
        cJSON* old_path;
        cJSON* new_path;
        if(!best_match->is_birth_in && birth_in) {
            old_path = candidate; new_path = path;
        } else {
            old_path = path;      new_path = candidate;
        }

        cJSON* merged = merge_paths(old_path, new_path);
        if(!merged) {
            LOG_WARN("STICH: merge_paths failed\n");
            g_mutex_unlock(&stitch_mutex);
            stitch_publish(path);
            return;
        }
        if(!cJSON_GetObjectItem(merged, "stitched"))
            cJSON_AddTrueToObject(merged, "stitched");

        /* Remove held entry and free resources */
        g_source_remove_by_user_data(best_match);
        held_paths = g_list_remove(held_paths, best_match);
        g_timer_destroy(best_match->expiry_timer);
        cJSON_Delete(best_match->path);
        free(best_match);
        cJSON_Delete(path); /* original incoming path consumed by merge */

        /* Check whether the merged path's END is still inside the stitch area */
        cJSON* merged_arr = cJSON_GetObjectItem(merged, "path");
        if(!merged_arr || cJSON_GetArraySize(merged_arr) < 1) {
            LOG_WARN("STICH: Invalid merged path array\n");
            cJSON_Delete(merged);
            g_mutex_unlock(&stitch_mutex);
            return;
        }
        int m_n = cJSON_GetArraySize(merged_arr);
        cJSON* merged_end = cJSON_GetArrayItem(merged_arr, m_n - 1);
        if(!merged_end) {
            LOG_WARN("STICH: Invalid merged path end\n");
            cJSON_Delete(merged);
            g_mutex_unlock(&stitch_mutex);
            return;
        }
        cJSON* mex_obj = cJSON_GetObjectItem(merged_end, "x");
        cJSON* mey_obj = cJSON_GetObjectItem(merged_end, "y");
        if(!mex_obj || !mey_obj) {
            LOG_WARN("STICH: Invalid merged path end coords\n");
            cJSON_Delete(merged);
            g_mutex_unlock(&stitch_mutex);
            return;
        }
        int mex = mex_obj->valueint, mey = mey_obj->valueint;
        bool merged_death_in = (mex >= stitch_x1 && mex <= stitch_x2 &&
                                 mey >= stitch_y1 && mey <= stitch_y2);
        if(merged_death_in) {
            /* Still ends inside → hold as a (death_in) segment for further stitching */
            int hold_timeout = stitch_duration * STITCH_HOLD_TIMEOUT_MULTIPLIER;
            if(hold_timeout < 60) hold_timeout = 60;
            StitchHeldPath* hp_new = malloc(sizeof(StitchHeldPath));
            hp_new->path        = merged;
            hp_new->expiry_timer = g_timer_new();
            hp_new->is_birth_in = false;
            held_paths = g_list_append(held_paths, hp_new);
            g_timeout_add_seconds(hold_timeout, publish_timeout_cb, hp_new);
        } else {
            stitch_publish(merged);
        }
        g_mutex_unlock(&stitch_mutex);
        return;
    }

    /* ------------------------------------------------------------------
     * No match found → HOLD this path.
     *
     * Key fix: previously birth_in paths with no match were published
     * immediately.  This caused split paths when the post-occlusion path
     * arrived (via Stitch_Path) before the pre-occlusion path was
     * finalized — a common race condition.  Now ALL paths that touch the
     * stitch area are held, with a long hold timeout so they survive the
     * full scene crossing.
     * ------------------------------------------------------------------ */
    int hold_timeout = stitch_duration * STITCH_HOLD_TIMEOUT_MULTIPLIER;
    if(hold_timeout < 60) hold_timeout = 60;

    StitchHeldPath* hp = malloc(sizeof(StitchHeldPath));
    hp->path         = path;
    hp->expiry_timer = g_timer_new();
    hp->is_birth_in  = birth_in && !death_in; /* true only when purely birth_in */
    held_paths = g_list_append(held_paths, hp);
    g_timeout_add_seconds(hold_timeout, publish_timeout_cb, hp);
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
