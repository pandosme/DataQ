/*------------------------------------------------------------------
 *  Fred Juhlin (2022)
 *------------------------------------------------------------------*/

#ifndef _ObjectTracker_H_
#define _ObjectTracker_H_

#include <stdio.h>
#include "cJSON.h"

#ifdef  __cplusplus
extern "C" {
#endif

typedef void (*ObjectTracker_Callback) (cJSON* tracker);

/* ObjectTracker
{
	"id": Unique object ID,
	"active": bool,	True = still tracking, False = Object tracking lost
	"timestamp": number, Last position timestamp (EPOCH milli seconds)
	"birth": number, Birth timestamp (EPOCH milli seconds)
	"x": 0-1000,
	"y": 0-1000,
	"w": 0-1000,
	"h": 0-1000,
	"cx": 0-1000,  Center of gravity.  Placement depends on configuration
	"cy": 0-1000,   Either in the center of the box or bottom-middle (default)
	"bx": 0-1000,  Birth X (cx)
	"by": 0-1000,  Birth Y (cy)
	"px": 0-1000,  Previous cx
	"py": 0-1000,  Previous cy
	"dx": 0-1000,  Total delta x movement from birth ( positive => right, negative => left
	"dy": 0-1000,  Total delta y movement from birth ( positive => down, negative => up
	"distance": number  Percent of camer view 
	"age": number seconds
}
*/

int		ObjectTracker_Init( ObjectTracker_Callback callback );
void	ObjectTracker_Detections( cJSON* detections );
cJSON*  ObjectTracker_Settings();

#ifdef  __cplusplus
}
#endif

#endif
