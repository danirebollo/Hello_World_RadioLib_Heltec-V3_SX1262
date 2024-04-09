#ifndef DR_TOOLS_h
#define DR_TOOLS_h

#include "Arduino.h"
#include <stdint.h>
#include "Wire.h"
#include "pinout.h"


#ifdef __cplusplus
extern "C"
{
#endif


#define MAX_QUEUE_SIZE 15
#define uS_TO_S_FACTOR 1000000 //Conversion factor for micro seconds to seconds

void i2cScanner();
void changeCPUFreq(int freq);
void esp32hibernation(int min2hibernate=20);
void setTimetoSleep_min(int timetosleep0);
void lightsleepesp(bool deepsleep);
void playBuzzer(int freq, int duration);

#ifdef __cplusplus
}
#endif

#endif