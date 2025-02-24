/*
 * Copyright (c) 2025 Fred Juhlin
 * MIT License - See LICENSE file for details
 * Version 1.0
 */

#ifndef _LOCATION_H_
#define _LOCATION_H_

#ifdef __cplusplus
extern "C" {
#endif

// Add these declarations
#define MATRIX_SIZE 9
typedef double HomographyMatrix[MATRIX_SIZE];

void LOCATION_Init();
int  LOCATION_transform(int x, int y, double *lat, double *lng);
int  LOCATION_Matrix( cJSON* matrix);
int  LOCATION_CalculateHomography(const double points[][4], int numPoints, HomographyMatrix matrix);

#ifdef __cplusplus
}
#endif

#endif
