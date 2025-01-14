/*------------------------------------------------------------------
 *  Fred Juhlin (2025)
 *------------------------------------------------------------------*/
 
#ifndef ObjectDetection_H
#define ObjectDetection_H

#include "cJSON.h"

typedef void (*ObjectDetection_Callback)( cJSON *detections  );
//Consumer needs delete list;

int		ObjectDetection_Init( ObjectDetection_Callback callback );
void	ObjectDetection_Set( int confidence, int rotation, int cog );

#endif
