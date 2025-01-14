/*------------------------------------------------------------------
 *  Fred Juhlin (2023)
 *------------------------------------------------------------------
 */
 
 #ifndef _TIME_H_
#define _TIME_H_

#include "cJSON.h"

#ifdef  __cplusplus
extern "C" {
#endif

double		TIME_Timestamp();  //UTC in milliseconds
int			TIME_Seconds_Since_Midnight();
const char* TIME_Local_Time(); //YYYY-MM-DD HH:MM:SS
const char* TIME_ISOTime(); //YYYY-MM-DDTHH:MM:SS+0000
const char* TIME_Date(); //YYYY-MM-DD
const char* TIME_Time(); //HH:MM:SS
double		TIME_Uptime();

#ifdef  __cplusplus
}
#endif

#endif