#ifndef DR_GPS_h
#define DR_GPS_h

#include "Arduino.h"
#include <stdint.h>

#define MAX_QUEUE_SIZE 15

#ifdef __cplusplus
extern "C"
{
#endif

// #define GPS_VERBOSE
String getGPSmeasure(int timeout_min=5);
void disableGPS();
void enableGPS();
void printGPSData();
void gpsloop();
bool getGPSstatus();
void bypassGPS();
double getGPSlat();
double getGPSlong();

#ifdef __cplusplus
}
#endif

#endif