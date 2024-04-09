#include "tools.h"
void i2cScanner()
{
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("Unknow error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");

}

void changeCPUFreq(int freq)
{ 
    //240, 160, 80, 40, 20, 10
    Serial.print("Current CPU Freq: ");
    Serial.println(getCpuFrequencyMhz());
 
    setCpuFrequencyMhz(freq);
    Serial.updateBaudRate(115200);

    Serial.print("new CPU Freq: ");
    Serial.println(getCpuFrequencyMhz());
}

void esp32hibernation(int min2hibernate)
{
  log_i("Going to hibernation for %d minutes", min2hibernate);

  esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  //uint64_t sleeptime=10 * 60* uS_TO_S_FACTOR;
  //log_d(" Going to HIBERNATE. Setting time to %d", sleeptime);
  //esp_sleep_enable_timer_wakeup(timetosleep);
  setTimetoSleep_min(min2hibernate);
  esp_deep_sleep_start();
}

void setTimetoSleep_min(int timetosleep0)
{
  log_d("Setup ESP32 to sleep for every %d minutes", timetosleep0);
  if (timetosleep0 > 0)
  {
    esp_sleep_enable_timer_wakeup((uint64_t)((uint64_t)timetosleep0 * (uint64_t)(uS_TO_S_FACTOR * 60)));
  }
}

void lightsleepesp(bool deepsleep)
{
  if (deepsleep)
  {
    log_i(" esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT0);");
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT0);
  }

  log_d(" LIGHTSLEEP");
    int minutes=2;
  log_w("# esp_light_sleep for %d minutes", minutes);
  delay(500);
  esp_sleep_enable_timer_wakeup(minutes * 60 * uS_TO_S_FACTOR);
  esp_light_sleep_start();
}



void playBuzzer(int freq, int duration)
{
    tone(BUZZER, freq, duration);
}