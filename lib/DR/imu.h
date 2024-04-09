#ifndef DR_IMU_h
#define DR_IMU_h

#include "Arduino.h"
#include <stdint.h>

#define MAX_QUEUE_SIZE 15

#ifdef __cplusplus
extern "C"
{
#endif


void enableIMU();
void disableIMU();
void imuReadRegs();
bool getIMUstatus();
String getIMUmeasure();
void IMUread();
bool readTemp();
bool checkIMUAvailable(bool force = false);
void setDefaults();

#define DSO_SA0_LOW_ADDRESS  0b1101010

#ifdef __cplusplus
}
#endif

#endif