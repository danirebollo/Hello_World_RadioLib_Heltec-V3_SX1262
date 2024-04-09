#ifndef DR_APP_h
#define DR_APP_h

#include "Arduino.h"
#include <stdint.h>
#include <SPI.h>
#include "dr_lorawan.h"
#include <Wire.h>
#include <LSM6.h>
#include "gasgauge.h"
#include "gps.h"
#include "imu.h"
#include "pinout.h"
#include "lora.h"
#include "tools.h"
#include <WiFi.h>
#include "driver/adc.h"

#define MAX_QUEUE_SIZE 15

#define ST(A) #A
//#define FWVERSION ST(FIRMWARE_VERSION)

#ifdef __cplusplus
extern "C"
{
#endif
void enableRadio();
void disableRadio();
void lowpowermode();
void normalmode();
void forceOneCore();
bool post_test();
void partial_post(String component, bool result);

#ifdef __cplusplus
}
#endif

#endif