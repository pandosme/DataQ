/*------------------------------------------------------------------
 *  Fred Juhlin (2025)
 *------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <syslog.h>
#include "ACAP.h"
#include "cJSON.h"
#include "GeoSpace.h"
#include "linmatrix/inc/lm_lib.h"

#define LOG(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args);}
#define LOG_WARN(fmt, args...)    { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args);}
//#define LOG_TRACE(fmt, args...)    { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args);}
#define LOG_TRACE(fmt, args...)    {}

typedef struct {
    lm_mat_t H;
    lm_mat_elem_t h_data[9];
    bool initialized;
} GeoSpace_Matrix_t;

static GeoSpace_Matrix_t gMatrix = {
    .initialized = false,
    .h_data = {0}
};

int GeoSpace_Matrix(cJSON* matrix) {
	LOG_TRACE("%s: Entry\n",__func__);
    if (!matrix || !cJSON_IsArray(matrix) || cJSON_GetArraySize(matrix) != 9) {
        LOG("%s: Matrix not configured\n", __func__);
        return 0;
    }

    // Fill matrix data
    for (int i = 0; i < 9; i++) {
        cJSON* element = cJSON_GetArrayItem(matrix, i);
        gMatrix.h_data[i] = (lm_mat_elem_t)(element ? element->valuedouble : 0.0);
    }
    
    if (!gMatrix.initialized) {
        if (lm_mat_set(&gMatrix.H, 3, 3, gMatrix.h_data, 9) != LM_SUCCESS) {
            LOG_WARN("%s: Failed to initialize matrix\n", __func__);
            return 0;
        }
        gMatrix.initialized = true;
    } else {
        // Update existing matrix data
        memcpy(gMatrix.H.elem.ptr, gMatrix.h_data, 9 * sizeof(lm_mat_elem_t));
    }
	LOG_TRACE("%s: Exit\n",__func__);
    return 1;
}


int
GeoSpace_transform(int x, int y, double *lat, double *lon) {
    LOG_TRACE("%s: Enter\n",__func__);

   // Initialize output parameters
    if (lat) *lat = 0;
    if (lon) *lon = 0;

    if (!gMatrix.initialized || !lat || !lon) {
        LOG_WARN("Invalid parameters or matrix not initialized\n");
        return 0;
    }
    
    // Create point vector [x, y, 1]
    lm_mat_t p;
    lm_mat_elem_t p_data[3] = {(lm_mat_elem_t)x, (lm_mat_elem_t)y, 1.0f};
    if (lm_mat_set(&p, 3, 1, p_data, 3) != LM_SUCCESS) {
        LOG_WARN("Failed to set point vector\n");
        return 0;
    }
    
    // Result vector
    lm_mat_t result;
    lm_mat_elem_t result_data[3] = {0};
    if (lm_mat_set(&result, 3, 1, result_data, 3) != LM_SUCCESS) {
        LOG_WARN("Failed to set result vector\n");
        return 0;
    }
    
    // Compute transformation using stored matrix
    if (lm_oper_gemm(false, false, 1.0f, &gMatrix.H, &p, 0.0f, &result) != LM_SUCCESS) {
        LOG_WARN("Matrix multiplication failed\n");
        return 0;
    }
    
    // Convert to homogeneous coordinates
    lm_mat_elem_t w = result_data[2];
    if (fabsf(w) > 1e-10f) {
        *lon = (double)(result_data[0] / w);
        *lat = (double)(result_data[1] / w);
    }
    LOG_TRACE("%s: Exit\n",__func__);
	return 1;
}

static void
GeoSpace_HTTP_transform(const ACAP_HTTP_Response response, const ACAP_HTTP_Request request) {
	LOG_TRACE("%s: Enter\n",__func__);
	
    if (!gMatrix.initialized) {
        ACAP_HTTP_Respond_Error(response, 500, "No valid transformation matrix");
        return;
    }

    const char* xParam = ACAP_HTTP_Request_Param(request, "x");
    const char* yParam = ACAP_HTTP_Request_Param(request, "y");

    if(!xParam || !yParam) {
        ACAP_HTTP_Respond_Error(response, 400, "Invalid input");
        return;
    }
    
    double x = atof(xParam);
    double y = atof(yParam);
    double lat, lon;

    GeoSpace_transform(x, y, &lat, &lon);
    
    cJSON *locationData = cJSON_CreateObject();
    cJSON_AddNumberToObject(locationData, "lat", lat);
    cJSON_AddNumberToObject(locationData, "lon", lon);    

    ACAP_HTTP_Respond_JSON(response, locationData);
	cJSON_Delete(locationData);
	LOG_TRACE("%s: Exit %f, %f\n",__func__,lat,lon);
}

void
GeoSpace_Init() {
    LOG_TRACE("%s: Enter\n",__func__);
    //Initialize HTTP endpoint
    ACAP_HTTP_Node("geospace", GeoSpace_HTTP_transform);
    LOG_TRACE("%s: Enter\n",__func__);
}
