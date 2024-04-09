
#ifndef DR_LORA_h
#define DR_LORA_h

#include "Arduino.h"
#include <stdint.h>
#include <lmic.h>
#include <hal/hal.h>
#include "dr_lorawan.h"

#define MAX_QUEUE_SIZE 15

#ifdef __cplusplus
extern "C"
{
#endif

// #define LORAMODULE_HELTEC

#ifdef LORAMODULE_HELTEC
// Pin mapping HELTEC ESP32
#define LoRa_nss 18
#define LoRa_rst 14
#define LoRa_dio0 26
#define LoRa_dio1 33
#define LoRa_dio2 32
#define LoRa_MOSI 27
#define LoRa_MISO 19
#define LoRa_SCK 5

#else
#define LoRa_nss 32
#define LoRa_rst 14
#define LoRa_dio0 5  // UNCONNECTED, unused in ESP32
#define LoRa_dio1 12 // UNCONNECTED, unused in ESP32
#define LoRa_dio2 15 // UNCONNECTED, unused in ESP32
#define LoRa_MOSI 33
#define LoRa_MISO 26
#define LoRa_SCK 27
// #define LoRa_nrst 12
// #define LoRa_busy 13
//  Pin mapping CUSTOM BOARD
#endif

// signals | ESP32 HELTEC | ESP32 CUSTOM
// BUSY   |
// DIO3   |
// DIO2   | 32    | LMIC_UNUSED_PIN
// DIO1   | 33    | LMIC_UNUSED_PIN
// DIO0   | 26    | LMIC_UNUSED_PIN
// RST    | 14    | LMIC_UNUSED_PIN
// NSS    | 18    | 32

// MISO   | 19    | 26
// MOSI   | 27    | 33
// SCK    | 5     | 27

// SPI.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI, LoRa_nss);

// static const uint8_t RST_LoRa = 14;
// static const uint8_t DIO0 = 26;
// static const uint8_t DIO1 = 33;
// static const uint8_t DIO2 = 32;

/*
Starting
nss: 8
rst: 12
dio[0]: 26
dio[1]: 33
dio[2]: 32
*/

void onEvent(ev_t ev);
void sendNextMessage();
void sendData(Message message);
void requestMsg();
void addKeyToIntJson(String &jsonString, const char *key, int value);
void do_send(osjob_t *j, Message message);
void sendMsg(Message message);
void enableLoRa();
void disableLoRa();
void incrementCurrentRetryCount_0();
void loraloop();
bool getPendingACK();
int getMessageQueueLength();
bool getLoRaStatus();
void sendloramessage();
unsigned long getLastLoraMessageTime();
void loraloop_2();

#ifdef __cplusplus
}
#endif

#endif