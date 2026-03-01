/*------------------------------------------------------------------
 *  Fred Juhlin (2024)
 *
 *------------------------------------------------------------------*/
 
#ifndef RadarDetection_H
#define RadarDetection_H

#include "cJSON.h"

typedef void (*ObjectDetection_Callback)( cJSON *detections  );
typedef void (*TrackerDetection_Callback)( cJSON *detections, int timer );

int		RadarDetection_Init( ObjectDetection_Callback detections, TrackerDetection_Callback tracker);
void	RadarDetection_Config( cJSON* data );
void	RadarDetection_Reset();
void	RadarDetection_Cleanup(void);
#endif