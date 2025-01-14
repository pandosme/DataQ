/*------------------------------------------------------------------
 *  Copyright Fred Juhlin (2022)
 *  License:
 *  All rights reserved.  This code may not be used in commercial
 *  products without permission from Fred Juhlin.
 *------------------------------------------------------------------*/
 
#ifndef ObjectPath_H
#define ObjectPath_H

#include "cJSON.h"

typedef void (*ObjectPath_Callback)( cJSON* ObjectPath );
/*

*/

int  ObjectPath_Init(ObjectPath_Callback callback);
void ObjectPath_Input( cJSON* tracker );


#endif
