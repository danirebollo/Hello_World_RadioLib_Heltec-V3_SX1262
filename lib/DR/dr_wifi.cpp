#include "dr_wifi.h"
#include "Arduino.h"
#include <stdint.h>
#include "WiFi.h"
#include <Wire.h>
#include <esp_wifi.h>
#include "driver/adc.h"
//#include <WiFiClient.h>
//#include <WiFiAP.h>
//#include <WiFiGeneric.h>
//#include <WiFiMulti.h>
//#include <WiFiScan.h>
//#include <WiFiServer.h>
//#include <WiFiSTA.h>
//#include <WiFiType.h>
//#include <WiFiUdp.h>
//#include <WiFiClientSecure.h>
//#include <WiFiScanClass.h>
//#include <WiFiServerSecure.h>
//#include <WiFiSTA_WPS.h>

//void enableWiFi()
//{
//    adc_power_on();
//    WiFi.disconnect(false);  // Reconnect the network
//    WiFi.mode(WIFI_STA);    // Switch WiFi off
// 
//    Serial.println("START WIFI");
//   //WiFi.begin(STA_SSID, STA_PASS);
//
//    WiFi.mode(WIFI_STA);
//    WiFi.begin();
//    delay(100);
//    // WiFi.begin("SSID", "password");
//    // while (WiFi.status() != WL_CONNECTED)
//    // {
//    //     delay(500);
//    //     Serial.print(".");
//    // }
//    // Serial.println("WiFi connected");
//    // Serial.println("IP address: ");
//    // Serial.println(WiFi.localIP());   
//}