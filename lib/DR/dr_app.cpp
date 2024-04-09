#include "dr_app.h"

#include "Arduino.h"
#include <stdint.h>
#include <SPI.h>
//#include <cayenneLPP.h>
#include "dr_lorawan.h"
#include <Wire.h>
#include <LSM6.h>
//#include <BluetoothSerial.h>
#include "gasgauge.h"
#include "gps.h"
#include "imu.h"
#include "pinout.h"
#include "lora.h"
#include "tools.h"
#include <WiFi.h>
#include "driver/adc.h"
#include "dr_debug.h"

void enableRadio()
{
    enableLoRa();
    enableGPS();
}
void disableRadio()
{
    disableGPS();
    disableLoRa();
}

void lowpowermode()
{
    log_i("Disabling LoRa");
    disableLoRa();
    log_i("Disabling GPS");
    disableGPS();
    log_i("Disabling IMU");
    disableIMU();
    
    //log_i("Disabling adc");
    //adc_power_off();
    ////WiFi.disconnect(true);  // Disconnect from the network
    //log_i("Disabling WiFi");
    ////WiFi.mode(WIFI_MODE_NULL );    // Switch WiFi off
    //WiFi.disconnect(true); 
    //WiFi.mode(WIFI_OFF); 
    //esp_wifi_stop(); 
    //btStop(); 
    //esp_bt_controller_disable(); //esp_bluedroid_disable();
    //esp_wifi_set_ps(WIFI_PS_NONE);
    //WiFi.setSleep(false);
}

void normalmode()
{
    log_i("Enabling LoRa");
    enableLoRa();
    log_i("Enabling GPS");
    enableGPS();
    //log_i("Enabling IMU");
    // enableIMU();
    //adc_power_on();
    //WiFi.disconnect(false);  // Reconnect the network
    //WiFi.mode(WIFI_STA);    // Switch WiFi off
}

void forceOneCore()
{
    //// Set the CPU core to run the loop code
    //// on core 0
    //xTaskCreatePinnedToCore(
    //    loopTask,   /* Function to implement the task */
    //    "loopTask", /* Name of the task */
    //    10000,      /* Stack size in words */
    //    NULL,       /* Task input parameter */
    //    0,          /* Priority of the task */
    //    NULL,       /* Task handle. */
    //    0);         /* Core where the task should run */

}


void partial_post(String component, bool result)
{
  if (result)
    log_d("%-45s%5s", component.c_str(), "\033[0;32mPASS\033[0m");
  else
    log_d("%-45s%5s", component.c_str(), "\033[0;31mFAIL\033[0m");
}

int readADC()
{
    int adcValue = 0;
    //adc_power_on();
    //adcValue = analogRead(ADC1);
    //adc_power_off();
    return adcValue;
}


String getDeviceID()
{
  char deviceID[DEVICEIDSIZE];
  uint64_t chipid;
  chipid = ESP.getEfuseMac();
  uint16_t chip = (uint16_t)(chipid >> 32);
  //log_d("HWID: %s" );
#ifdef FORCEDEVICEID
  snprintf(deviceID, DEVICEIDSIZE, "m000001110003"); //movusbike-50e2b73a7d80
  //snprintf(did, DEVICEIDSIZE, "50e2b73a7d80");
#else
  snprintf(deviceID, DEVICEIDSIZE, "m%04x%08x", chip, (uint32_t)chipid);
  //log_w("DeviceID: %d", deviceID);
  //snprintf(did, 23, "%04x%08x", chip, (uint32_t)chipid);
#endif

  String str(deviceID);
  return str;
}

bool post_test()
{
    log_d("Running POST test...");
  bool generalresult = true;
  String result_string = "PASS";

  String modemfirmware = ""; // TODO. save modemfirmware into EEPROM and recover. drconnect_1.getFirmwareVersion();
    
    int vbat=0;
    int mabat=0;
    int chargecount=0;

    bool gasgaugefail = false;
    if(gasGaugeAvailable())
    {
        log_d("Gas gauge available");
        startGasGauge();
        ReadBatteryData();
        //log_d("read vbat");
        vbat = getVBATS();
        //delay(200);
        //log_d("read mabat");
        mabat = getmaBATS();
        chargecount = getChargeCountBATS();
    }
    else
    {
        log_e("Gas gauge not available");
        gasgaugefail = true;
    }
    bool gpsfail = false;
    if(!getGPSstatus())
    {
        log_e("GPS not available");
        gpsfail = true;
    }
    else
    {
        log_d("GPS available");
    }
    bool imufail = false;
    if(!getIMUstatus())
    {
        log_e("IMU not available");
        imufail = true;
    }
    else
    {
        log_d("IMU available");
    }

  //log_d("read adc1");
  //int vic = readADC(); //not connected yet

  //log_d("█████████████████████████████████████████████████████████████████████████████████████████");
  //log_ut("POST");
  log_d("");
  log_d("# Ixorigue eartrack. DeviceId:'%s' Firmware Version: '%s'", getDeviceID().c_str(), FIRMWARE_VERSION);
  log_d("▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀");
  log_d("█████████████████████████████████████████ POST ██████████████████████████████████████████");
  log_d("▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄");
  log_d("-Testing SPI devices");

  bool result = getLoRaStatus();
  partial_post((String) "-Testing LoRa transceiver", result);
  if (!result)
    generalresult = false;


  log_d("-Testing I2C devices");
  result = true;

  //result = drutilities_1.LookForI2CAddress(I2CADDRESS_GASGAUGE_STC);


    if(gasgaugefail)
    {
        //log_e("Gas gauge not available");
        result = false;
    }
  if (vbat < 100)
      result = false;
  partial_post((String) "Testing Gas gauge (Address 0x" + (String)I2CADDRESS_GASGAUGE_STC + (String) ")", result);
  if (!result)
  {
    generalresult = false;
    gasgaugefail = true;
  }
    
    
  //delay(200);

  result = true;
  if(gasgaugefail)
    result = false;
  partial_post((String) "-Testing Gas gauge batery current (" + (String)mabat + (String) "mA)", result);
  if (!result)
    generalresult = false;

  result = true;
  if(gasgaugefail)
    result = false;
  partial_post((String) "-Testing Gas gauge batery coulomb (" + (String)chargecount + (String) "mA)", result);
  if (!result)
    generalresult = false;

    result = true;
    if(imufail)
        result = false;
  partial_post((String) "Testing IMU (Address 0x" + (String)I2CADDRESS_IMU + (String) ")", result);
  if (!result)
    generalresult = false;

  log_d("-Testing UART devices");

  result = true;
  if(gpsfail)
    result = false;
  partial_post("-Testing GPS", result);
  if (!result)
    generalresult = false;

log_d("-Testing other peripherals");
  result = true;
  if (vbat < 3000)
    result = false;
  partial_post((String) "-Testing ADC batery voltage (" + (String)vbat + (String) "mV)", result);
  if (!result)
    generalresult = false;


  log_d("▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀");
  if (generalresult)
    log_i("───────────────────────────────────────── PASS ──────────────────────────────────────────");
  else
    log_e("───────────────────────────────────────── FAIL ──────────────────────────────────────────");
  log_d("▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄");

  log_d("");

  //short vbatIconsumption = getChargeCountBATS(1);
  //String svalue = "GGBOOT:" + (String)vbatIconsumption;
  //log_ut("%s", svalue.c_str());

  //short gasgaugediff;
  //if (gasgaugeatdeepsleep != 0)
  //{
  //  gasgaugediff = vbatIconsumption - gasgaugeatdeepsleep;
  //  log_d("-------------> gasgauge diff deep sleep-boot: %d", (gasgaugediff));
  //  //drconnect_1.putMQTTMessage("{\"code\":\"60\", \"description\":\"gasgauge difference deep sleep-boot\", \"value\":\"" + (String)gasgaugediff + "\"}", "alert");
  //}

  return generalresult;
}
