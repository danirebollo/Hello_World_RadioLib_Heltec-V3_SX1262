#include "gps.h"

#include <TinyGPS++.h>
#include "Preferences.h"

bool gpsfirstboot = true;
TinyGPSPlus *gps = new TinyGPSPlus();
unsigned long gpson_time = 0;
bool GPS_status = false;
String gpsstring = "";
Preferences preferences_1;

uint16_t gpsloops=0;

double getGPSlat()
{
    return gps->location.lat();
}
double getGPSlong()
{
    return gps->location.lng();
}

bool getGPSstatus()
{
    return GPS_status;
}
String getGPSmeasure(int timeout_min)
{
    String gpsdata = "";
    log_d("Getting GPS measure");
    //get current from gas gauge
    
    log_d("Enabling GPS");
    //enable GPS
    enableGPS();
    //wait for 1 seconds
    delay(1000);
    //read GPS data
    log_d("Waiting GPS data");
    unsigned long starttime=millis();
    unsigned long waitingtime=millis();
    unsigned long successtime=0;
    bool gpsfail = false;
    while(gpsfirstboot && (successtime==0 || successtime+2500<millis()))
    {
        gpsloop();
        if(millis()-starttime>timeout_min*60000)
        {
            log_e("Timeout waiting GPS data");
            gpsfail = true;
            break;
        }
        //print time each 10s
        if(millis()-waitingtime>10000)
        {
            waitingtime=millis();
            log_d("Waiting GPS data: %ds (loops: %d)", (waitingtime-starttime)/1000,gpsloops);
            //log gpsfirstboot, gps->hdop.hdop(), gps->hdop.hdop(), gps->satellites.value()
            log_d("GPS: gpsfirstboot:%d, hdop:%2.2f, satellites:%d, successtime: %d", gpsfirstboot, gps->hdop.hdop(), gps->satellites.value(),successtime);
        }

        if(!gpsfirstboot)
        {
            successtime=millis();
        }
    }
    //disable GPS
    disableGPS();
    if(gpsfail)
    {
        log_e("GPS data not available");
        return "";
    }
    else
    {
        gpsdata = String("GPS data: gpsfb:") + String(gpsfirstboot) + String(", hdop:") + String(gps->hdop.hdop()) + String(", satellites:") + String(gps->satellites.value()) + String(", lat: ") + String(gps->location.lat()) + String(", long: ") + String(gps->location.lng());
        log_i("GPS data: gpsfb:%d, hdop:%2.2f, satellites:%d, lat: %f, long: %f", gpsfirstboot, gps->hdop.hdop(), gps->satellites.value(), gps->location.lat(), gps->location.lng());
    }
        return gpsdata;
}


void enableGPS()
{
    //digitalWrite(EN_GPS, HIGH);
    //delay(100);
    gps = new TinyGPSPlus();
    //Enable serial port
    int RXPin = 2, TXPin = 1; // pins for ATGM336H GPS device
    uint32_t GPSBaud = 9600;  // default baudrate of ATGM336H GPS device
    Serial2.begin(GPSBaud, SERIAL_8N1, 16, 17);
    gpson_time = millis();
    gpsfirstboot = true;
    GPS_status = true;
}
void disableGPS()
{
    if(GPS_status)
    {
    Serial2.end();
    //put serial pins in input (high impedance) mode
    pinMode(16, INPUT);
    pinMode(17, INPUT);

    //digitalWrite(EN_GPS, LOW);
    gpson_time = 0;
    gpsfirstboot = false;
    // clear gps data and create again
    if(gps == NULL || gps==nullptr )
    {
        log_w("Trying to delete GPS object that is null");
    }
    else
    {
        log_d("Deleting GPS object");
        delete gps;
    }
        
    gpsstring = "";
    GPS_status = false;
    }
    else
    {
        log_w("GPS already disabled");
    }
}

int gpsattempts = 0;
void gpsloop()
{
    if (Serial2.available() > 0)
    {
        gpsloops++;
        String data = "";
        while (Serial2.available() > 0)
        {
            // read the incoming byte:
            char inChar = Serial2.read();
            if (gps->encode(inChar) && gps->location.isValid() && gps->date.isValid() && gps->time.isValid())
            {
#define MAX_GPS_STRING_SIZE 256

                // Crear un buffer temporal para formatear la cadena
                char tempBuffer[MAX_GPS_STRING_SIZE];

                // Utilizar snprintf para formatear la cadena
                snprintf(tempBuffer, MAX_GPS_STRING_SIZE, "## -- GPS data: %d/%d/%d, %d:%d:%d, lat: %f, long: %f, nsat: %d, altitude: %3.2f, speed: %3.2f, course: %3.2f, hdop: %3.2f, timesinceon: %ds",
                         gps->date.month(), gps->date.day(), gps->date.year(), gps->time.hour(), gps->time.minute(), gps->time.second(), gps->location.lat(), gps->location.lng(), gps->satellites.value(), gps->altitude.meters(), gps->speed.kmph(), gps->course.deg(), gps->hdop.hdop(), (millis() - gpson_time) / 1000);
                // Asignar el contenido del buffer temporal a la variable gpsstring
                String gpsstring_new(tempBuffer);

                if (gpsfirstboot && gps->hdop.hdop() < 5 && gps->hdop.hdop() > 0 && gps->satellites.value() > 2)
                {
                    gpsfirstboot = false;
                    // show time from gpson_time to now
                    // Serial.println("GPS First Boot");
                    unsigned long gpson_time_1 = (millis() - gpson_time) / 1000;
                    log_i("--- --- GPS First Boot time: %d seconds (hdop:%3.2f)", (gpson_time_1), gps->hdop.hdop());
                }

                if (gpsstring_new != gpsstring)
                {
                    gpsstring = gpsstring_new;
// Serial.println(gpsstring);
#ifdef GPS_VERBOSE
                    log_i("GPS Data: %s", gpsstring.c_str());
#endif
                }

                // TODO: store into EEPROM latest GPS data (Lat, Long) and compare with the current data. If the difference is greater than a threshold, then set this data as fail

                // store gps->location.lat() into ESP32 EEPROM
                preferences_1.begin("GPSDATA", false);
                uint16_t eeprom_latitude = preferences_1.getShort("LATITUDE", 0);
                uint16_t eeprom_longitude = preferences_1.getShort("LONGITUDE", 0);

                // throw error if current latitude is greater or less than 10%
                log_d("Latitude: %f, Longitude: %f", gps->location.lat(), gps->location.lng());
                log_d("EEPROM Latitude: %d, Longitude: %d", eeprom_latitude, eeprom_longitude);
                log_d("Latitude difference is %f", abs(gps->location.lat()/eeprom_latitude));
                log_d("Longitude difference is %f", abs(gps->location.lng()/eeprom_longitude));

                bool gpsfail = false;
                if(eeprom_latitude != 0 && eeprom_longitude != 0)
                {
                if (abs(gps->location.lat()/eeprom_latitude) > 0.2)
                {
                    log_e("Latitude difference is greater than 10%%: %f", abs(gps->location.lat()/eeprom_latitude));
                    gpsfail = true;
                }
                if (abs(gps->location.lng()/eeprom_longitude) > 0.2)
                {
                    log_e("Longitude difference is greater than 10%%: %f", abs(gps->location.lng()/eeprom_longitude));
                    gpsfail = true;
                }

                if (gpsfail)
                {
                    log_e("GPS data is not valid. Attempt: %d",gpsattempts);
                    gpsattempts++;
                }
                else
                {
                    log_i("GPS data is valid");
                    gpsattempts = 0;
                    //preferences_1.putShort("LATITUDE", gps->location.lat());
                    //preferences_1.putShort("LONGITUDE", gps->location.lng());
                }
                }

                preferences_1.end();
            }
        }
    }
}

void printGPSData()
{
    log_d("GPS Data: %s", gpsstring.c_str());
}

void bypassGPS()
{
    if (0)
    {
        Serial2.begin(9600, SERIAL_8N1, 16, 17);
        // pinMode(EN_GPS, OUTPUT);
        // gpson_time=millis();
        // digitalWrite(EN_GPS, HIGH);
        // Serial.begin(9600);
        while (0)
        {
            if (Serial2.available() > 0)
            {
                String data = "";
                // log_i("GPS Data: ");
                // Serial.println("\n-- GPS Data -- ");
                while (Serial2.available() > 0)
                {
                    // read the incoming byte:
                    int inChar = Serial2.read();
                    Serial.print((char)inChar);
                }
            }
        }
    }
}