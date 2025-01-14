
/*------------------------------------------------------------------
 *  Fred Juhlin (2023)
 *------------------------------------------------------------------*/
 
#include <stdlib.h>
#include <stdio.h>
#include <glib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <ctype.h>
#include <sys/sysinfo.h>
#include <sys/time.h>

#include "cJSON.h"

//#define LOG_TRACE(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOG_TRACE(fmt, args...)    {}


int
TIME_Seconds_Since_Midnight() {
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);
	int seconds = tm.tm_hour * 3600;
	seconds += tm.tm_min * 60;
	seconds += tm.tm_sec;
	return seconds;
}

char TIME_date[128] = "2023-01-01";
char TIME_time[128] = "00:00:00";

const char*
TIME_Date() {
	time_t t = time(NULL);
	struct tm *tm = localtime(&t);
	sprintf(TIME_date,"%d-%02d-%02d",tm->tm_year + 1900,tm->tm_mon + 1, tm->tm_mday);
	return TIME_date;
}

const char*
TIME_Time() {
	time_t t = time(NULL);
	struct tm *tm = localtime(&t);
	sprintf(TIME_time,"%02d:%02d:%02d",tm->tm_hour,tm->tm_min,tm->tm_sec);
	return TIME_time;
}

char TIME_timestring[128] = "2020-01-01 00:00:00";

const char*
TIME_Local_Time() {
	time_t t = time(NULL);
	struct tm *tm = localtime(&t);
	sprintf(TIME_timestring,"%d-%02d-%02d %02d:%02d:%02d",tm->tm_year + 1900,tm->tm_mon + 1, tm->tm_mday,tm->tm_hour,tm->tm_min,tm->tm_sec);
	LOG_TRACE("Local Time: %s\n",TIME_timestring);
	return TIME_timestring;
}

char TIME_isostring[128] = "2020-01-01T00:00:00+0000";

const char*
TIME_ISOTime() {
	time_t t = time(NULL);
	struct tm *tm = localtime(&t);
	strftime(TIME_isostring, 50, "%Y-%m-%dT%T%z",tm);
	return TIME_isostring;
}

double
TIME_Timestamp(void) {
	long ms;
	time_t s;
	struct timespec spec;
	double timestamp;
	clock_gettime(CLOCK_REALTIME, &spec);
	s  = spec.tv_sec;
	ms = round(spec.tv_nsec / 1.0e6); // Convert nanoseconds to milliseconds
	if (ms > 999) {
		s++;
		ms -= 999;
	}
	timestamp = (double)s;
	timestamp *= 1000;
	timestamp += (double)ms;
	return timestamp;  
}

double
TIME_Uptime() {
	struct sysinfo info;
	sysinfo(&info);
	return (double)info.uptime; 
};	

