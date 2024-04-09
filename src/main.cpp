// set 	-D LORAMODULE_HELTEC (or not) in platformio.ini

// TODO
// add WHOIS to Gas gauge and disable if not found after 2 attempts
// same with IMU
// replace IMU lib with STM32duino LSM6DSO
// enable pedometer
// disable wifi & BLE & other non used peripherals

//disable ESP32 and SIM800 on debug board.
// Fix other boards GPS antenna section
// Test LoRa antennas (chip, Helical, IPEX)

// # define LMIC_DR_LEGACY
// # define CFG_eu868
// # define LMIC_REGION_EU868
// # define CFG_LMIC_REGION_MASK 0 //LMIC_REGION_EU868

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

//#define USEBUZZER


//void UARTloop();
unsigned long currenttime_5 = millis();
char report[80];

void enableWiFi()
{
    adc_power_on();
    WiFi.disconnect(false);  // Reconnect the network
    WiFi.mode(WIFI_STA);    // Switch WiFi off
 
    Serial.println("START WIFI");
   //WiFi.begin(STA_SSID, STA_PASS);

    WiFi.mode(WIFI_STA);
    WiFi.begin();
    delay(100);
    // WiFi.begin("SSID", "password");
    // while (WiFi.status() != WL_CONNECTED)
    // {
    //     delay(500);
    //     Serial.print(".");
    // }
    // Serial.println("WiFi connected");
    // Serial.println("IP address: ");
    // Serial.println(WiFi.localIP());   
}

void getOneGPSMeasure(int timeout_min=5)
{
            log_i("getting gps measure");
            int startculombs=getChargeCountBATS(true);
            unsigned long starttime=millis();
            if(getGPSmeasure(timeout_min)=="")
            {
                log_e("GPS data not available. Trying again");
                if(getGPSmeasure(timeout_min)=="")
                {
                    log_e("GPS data not available. Exiting");
                }
            }
            unsigned long endtime=millis();
            int endculombs=getChargeCountBATS(true);
            log_i("Time to get GPS measure: %d ms", endtime-starttime);
            log_i("Coulombs consumed: %d", endculombs-startculombs);
}

#define LORA_ACK_TIMEOUT_SEC 30
#define GPS_TIMEOUT_MIN 5
void setup()
{
    delay(100);
    // setup the display
    // u8x8.begin();
    // u8x8.setFont(u8x8_font_chroma48medium8_r);
    // u8x8.drawString(0, 0, "WGLabz LoRa Test");
    //ESP32DualCoreClass::end();

    Serial.begin(115200);    
    //changeCPUFreq(80); //cpu already set to 80MHz
    
    log_d("Booting peripherals...");
    //Enable IMU pins
    pinMode(IMU_INT1, INPUT);
    pinMode(IMU_INT2, INPUT);
    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_CS, HIGH);
    
#ifdef LORAMODULE_HELTEC
    Serial.println(F("LORA MODULE= HELTEC"));
    // In/Out Pins
    //pinMode(ledPin, OUTPUT); //pin 12
    //pinMode(buttonPin, INPUT_PULLUP); //pin 13
    //digitalWrite(buttonPin, HIGH);
#else
    Serial.println(F("LORA MODULE= CUSTOM"));
    // SPI.begin(LoRa_SCK, 35, LoRa_MOSI, LoRa_nss); //MISO: 19, MOSI: 27, SCK: 5

    // Configure pins
    pinMode(EN_LORA, OUTPUT);
    pinMode(EN_GPS, OUTPUT);

    enableLoRa();
    enableIMU();
    enableGPS();
    
    delay(200);
    
    #ifdef USEBUZZER
    pinMode(BUZZER, OUTPUT);
    #endif
    //playBuzzer(1000, 1000);
    // pinMode(LoRa_MISO, INPUT); // impedance test
    //delay(100);
#endif

    Wire.begin();
    
    // POST TEST
    post_test();

    // execute code...

    unsigned long starttime=millis();
    getOneGPSMeasure(GPS_TIMEOUT_MIN);
    disableGPS();

    //Send message to LoRa
    String jsonstring = String("{\"rand\":") + String(random(0, 100)) + ", \"temp\":" + String(random(0, 100)) + ", \"hum\":" + String(random(0, 100)) + ", \"lat\": "+String(getGPSlat(),6)+", \"long\": "+String(getGPSlong(),6)+", \"log\": \"Hello\", \"confirm\": \"true\"}";
    Message messageStruct;
    messageStruct.message = jsonstring;
    sendData(messageStruct);

    //wait for ack
    log_d("Current pending ACK: %d", getPendingACK());
    log_d("Current pending messages: %d", getMessageQueueLength());

    
    unsigned long lora_starttime=millis();
    while(getPendingACK()==0 && millis()-lora_starttime<(LORA_ACK_TIMEOUT_SEC*1000))
    {
        if(millis()-currenttime_5 > 10000)
        {
            currenttime_5 = millis();
            log_d("Waiting for ACK: %d", (millis()-lora_starttime)/1000);
        }
        
        loraloop();
    }

    log_d("Disabling LoRa");
    disableLoRa();
    

    // hibernate 30 min
    esp32hibernation(30);

/*
CODE STOPS HERE, NOT EXECUTED, IT IS IN HIBERNATION MODE
*/

    // START GAS GAUGE & I2C
    //Serial.println(F("Starting Wire"));
    
    //Serial.println(F("STC3100 startGasGauge"));
    //startGasGauge();
    //Serial.println(F("STC3100 Started"));
    //resetGasGauge();

    // getting the battery data
    //readgasgauge();
    //log_i("stop gas gauge");
    //stopGasGauge();

    // START IMU

    //checkIMUAvailable();
    //i2cScanner();


    // LMIC init &RESET
    // os_init_ex returns 0 if the LMIC was already initialized
    // os_init();
    // os_init_ex(&lmic_pins);
    
    // Probar todos los perifericos
    // enviar LoRa
    // esperar mensaje durante x sec (el mensaje puede indicar que quedan mas mensajes)
    // gestionar mensaje recibido
    // hibernar 30 min
}

void loop()
{
    // run lora loop. Automatically sends pending messages, handle disabling etc
    loraloop_2();

    // Print each 10s: lat, long, nsat, hdop, vbat, mabat, chargecount, temp, millis()-gpson_time, pending ack
    if (millis() - currenttime_5 > 10000)
    {
        //log_w("loop 1");
        
        currenttime_5 = millis();
        if(gasGaugeAvailable())
        {
            log_d("Reading battery data");
            ReadBatteryData();
        }
            
        //log_w("loop 2");
        if(getIMUstatus())
        {
            //log_d("loop 21");
            IMUread();
            snprintf(report, sizeof(report), getIMUmeasure().c_str());
        }
        else
        {
            snprintf(report, sizeof(report), "IMU not enabled");
        }

        //log_w("loop 3");
        char report2[80];
        if(getGPSstatus())
        {
            //gpsloop();
            snprintf(report2, sizeof(report2), getGPSmeasure().c_str());
        }
        else
        {
            snprintf(report2, sizeof(report), "GPS not enabled");
        }

        //log_w("loop 4");
        char report4[80];
        if(gasGaugeAvailable())
        {
            snprintf(report4, sizeof(report4),readgasgauge().c_str());
        }
        else
        {
            snprintf(report4, sizeof(report4), "Gas Gauge not enabled");
        }

        char report5[80];
        if(getLoRaStatus())
        {
            String pending = "Pending ACK:"+String(getPendingACK())+", Pending lora messages: "+String(getMessageQueueLength());
            snprintf(report5, sizeof(report5), pending.c_str());
        }
        else
        {
            snprintf(report5, sizeof(report5), "LoRa not enabled");
        }

        log_d("\n\tLoRa: %s\n\tGas Gauge: %s\n\tGPS: %s\n\tIMU: %s\n",
             report5 , report4 ,report2 , report); 
    }

    UARTloop();

    delay(500);

    //if(getCpuFrequencyMhz()==10 && millis()-downfreqtime>30000)
    //{
    //    changeCPUFreq(80);
    //    log_w("Going back to 80MHz");
    //}
}