/*------------------------------------------------------------------
 *  Fred Juhlin (2025)
 *------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <syslog.h>
#include "ACAP.h"
#include "cJSON.h"
#include "location.h"
#include "linmatrix/inc/lm_lib.h"

#define LOG(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args);}
#define LOG_WARN(fmt, args...)    { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args);}
//#define LOG_TRACE(fmt, args...)    { syslog(LOG_WARNING, fmt, ## args); printf(fmt, ## args);}
#define LOG_TRACE(fmt, args...)    {}

typedef struct {
    lm_mat_t H;
    lm_mat_elem_t h_data[9];
    bool initialized;
} LOCATION_Matrix_t;

static LOCATION_Matrix_t gMatrix = {
    .initialized = false,
    .h_data = {0}
};

int LOCATION_Matrix(cJSON* matrix) {

    if (!matrix || !cJSON_IsArray(matrix) || cJSON_GetArraySize(matrix) != 9) {
        LOG_WARN("%s: Invalid matrix format\n", __func__);
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
    
    return 1;
}


int
LOCATION_transform(int x, int y, double *lat, double *lng) {
    LOG_TRACE("%s: Enter\n",__func__);

   // Initialize output parameters
    if (lat) *lat = 0;
    if (lng) *lng = 0;

    if (!gMatrix.initialized || !lat || !lng) {
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
        *lng = (double)(result_data[0] / w);
        *lat = (double)(result_data[1] / w);
    }
	return 1;
}

static void
LOCATION_HTTP_transform(const ACAP_HTTP_Response response, const ACAP_HTTP_Request request) {
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
    double lat, lng;

    LOCATION_transform(x, y, &lat, &lng);
    
    cJSON *locationData = cJSON_CreateObject();
    cJSON_AddNumberToObject(locationData, "lat", lat);
    cJSON_AddNumberToObject(locationData, "lng", lng);    

    ACAP_HTTP_Respond_JSON(response, locationData);
	LOG_TRACE("%s: Exit %f, %f\n",__func__,lat,lng);
}

void
LOCATION_Init() {
    //Initialize HTTP endpoint
    ACAP_HTTP_Node("transform", LOCATION_HTTP_transform);
}
