/*------------------------------------------------------------------
 *  Fred Juhlin (2025)
 *------------------------------------------------------------------*/
 
#ifndef ObjectDetection_H
#define ObjectDetection_H

#include "cJSON.h"

typedef void (*ObjectDetection_Callback)( cJSON *detections  );
//Subscribers are responsible for deleting the cJSON object.

int		ObjectDetection_Init( ObjectDetection_Callback detections, ObjectDetection_Callback tracker);
void	ObjectDetection_Config( cJSON* data );
void	ObjectDetection_Reset();

#endif