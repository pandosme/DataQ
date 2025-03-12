/*
 * Copyright (c) 2025 Fred Juhlin
 * MIT License - See LICENSE file for details
 * Version 1.0
 */

#ifndef _GEOSPACE_H_
#define _GEOSPACE_H_

#ifdef __cplusplus
extern "C" {
#endif

// Add these declarations
#define MATRIX_SIZE 9
typedef double HomographyMatrix[MATRIX_SIZE];

void GeoSpace_Init();
int  GeoSpace_transform(int x, int y, double *lat, double *lon);
int  GeoSpace_Matrix( cJSON* matrix);

#ifdef __cplusplus
}
#endif

#endif
