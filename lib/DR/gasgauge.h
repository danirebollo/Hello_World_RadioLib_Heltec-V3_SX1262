#ifndef DR_GASGAUGE_h
#define DR_GASGAUGE_h

#include "Arduino.h"
#include <stdint.h>

#define MAX_QUEUE_SIZE 15

#ifdef __cplusplus
extern "C"
{
#endif

void startGasGauge();
void stopGasGauge();
String readgasgauge();
void resetGasGauge();
uint16_t getTEMPS(int force=false);
uint16_t getVBATS(int force=false);
uint16_t getmaBATS(int force=false);
uint16_t getChargeCountBATS(int nop=false);
uint16_t ResetGasGauge(int nop=false);
uint16_t GasGaugeInit();
bool ReadBatteryData();
void ReadRegisters();
bool gasGaugeAvailable(bool force = false);

#ifdef __cplusplus
}
#endif

#endif