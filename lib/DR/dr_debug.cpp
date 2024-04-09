#include "dr_debug.h"
#include "dr_app.h"


unsigned long downfreqtime = millis();

void UARTloop()
{
if (Serial.available() > 0)
    {
        char keyPressed = Serial.read();

        if(keyPressed=='0')
        {
            log_d("Beeping one time");
            playBuzzer(1000, 1000);
        }
        else if(keyPressed== '1')
        {
            // Serial.println("Sending Lora Message");
            log_i("Sending Lora Message");
            sendloramessage();
            }
        else if(keyPressed== '3')
        {
            // Serial.println("Reading GPS");
            log_i("Reading GPS");
            gpsloop();
            }
        else if(keyPressed== '4')
        {
            // enable GPS
            log_i("Enabling GPS");
            enableGPS();
            }
        else if(keyPressed== '5')
        {
            // disable GPS
            log_i("Disabling GPS");
            disableGPS();
            }
        else if(keyPressed== '6')
        {
            // enable LoRa
            log_i("Enabling LoRa");
            enableLoRa();
            }
        else if(keyPressed== '7')
        {
            // disable LoRa
            log_i("Disabling LoRa");
            disableLoRa();
            }
        else if(keyPressed== '8')
        {
            // print GPS data
            log_i("Printing GPS Data");
            printGPSData();
            }
        else if(keyPressed== '9')
        {
            //shutdown ESP32 during 60s
            log_i("Shutting down ESP32");
            esp_deep_sleep(20*60*uS_TO_S_FACTOR);
            }
        else if(keyPressed== 'a')
        {
            log_i("Setting CPU to 10MHz");
            changeCPUFreq(10);
            }
        else if(keyPressed== 'b')
        {
            log_i("Setting CPU to 40MHz");
            changeCPUFreq(40);
            }
        else if(keyPressed== 'c')
        {
            log_i("Setting CPU to 80MHz");
            changeCPUFreq(80);
            }
        else if(keyPressed== 'd')
        {
            log_i("Setting CPU to 160MHz");
            changeCPUFreq(160);
            }
        else if(keyPressed== 'e')
        {
            log_i("Enabling low power mode");
            lowpowermode();
            }
        else if(keyPressed== 'f')
        {
            log_i("Disable IMU");
            disableIMU();            
            }
        else if(keyPressed== 'g')
        {
            log_i("Enable IMU");
            enableIMU();
            }
        else if(keyPressed== 'h')
        {
            log_i("Setting CPU to only one core");
            forceOneCore();
            }
        else if(keyPressed== 'i')
        {
            //lowpowermode();
            log_i("Entering into hibernation mode");
            esp32hibernation();
            }
        else if(keyPressed== 'j')
        {
            log_i("Read IMU Registers");
            imuReadRegs();
            }
        else if(keyPressed== 'k')
        {
            log_i("getting gps measure");
            int startculombs=getChargeCountBATS(true);
            unsigned long starttime=millis();
            getGPSmeasure();
            unsigned long endtime=millis();
            int endculombs=getChargeCountBATS(true);
            log_i("Time to get GPS measure: %d ms", endtime-starttime);
            log_i("Coulombs consumed: %d", endculombs-startculombs);
            }
        else if(keyPressed== 'l')
        {
            Serial.print("Current CPU Freq: ");
            Serial.println(getCpuFrequencyMhz());
            }
        else if(keyPressed== 'm')
        {
            log_i("downfreq to 10MHz during 30sec");
            changeCPUFreq(10);
            downfreqtime=millis();
            }
        else if(keyPressed== 'n')
        {
            log_i("stop gas gauge");
            stopGasGauge();
            }
        else if(keyPressed== 'o')
        {
            log_i("start gas gauge");
            startGasGauge();
            }
        else if(keyPressed== 'p')
        {
            log_i("stc3100_1.ReadBatteryData");
            ReadBatteryData();
            }
        else if(keyPressed== 'q')
        {
            log_i("read gas gauge registers");
            ReadRegisters();
            }
        else if(keyPressed== 'r')
        {
            log_i("Reading Gas Gauge");
            readgasgauge();
            }
        else if(keyPressed== 's')
        {
            log_i("start gauge ");
            Serial.println(F("STC3100 startGasGauge"));
            startGasGauge();
            Serial.println(F("STC3100 Started"));
            //resetGasGauge();
            }
        else if(keyPressed== 't')
        {
            log_i("Enable Radio");
            enableRadio();
            }
        else if(keyPressed== 'u')
        {
            log_d("Disabling Radio");
            disableRadio();
            }
        else if(keyPressed== 'v')
        {
            log_i("Reset GasGauge");
            resetGasGauge();
            }
        else 
        {
            log_w("Tecla no válida");
        }
        
    }
}