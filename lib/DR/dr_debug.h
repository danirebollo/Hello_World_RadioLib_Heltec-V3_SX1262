#ifndef DR_DEBUG_h
#define DR_DEBUG_h

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
#include "dr_app.h"

#define MAX_QUEUE_SIZE 15

#ifdef __cplusplus
extern "C"
{
#endif

void UARTloop();

#ifdef __cplusplus
}
#endif

#endif