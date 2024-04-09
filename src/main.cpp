// Works on Heltec, and custom

// set 	-D LORAMODULE_HELTEC in platformio.ini

// TODO
// add WHOIS to Gas gauge and disable if not found after 2 attempts
// same with IMU
// replace IMU lib with STM32duino LSM6DSO
// enable pedometer
// downclock to 80mhz
// disable wifi & BLE & other non used peripherals
// disable GPS after GPS was given 

//disable ESP32 and SIM800 on debug board.
// Fix other boards GPS antenna section
// Test LoRa antennas (chip, Helical, IPEX)


// # define LMIC_DR_LEGACY
// # define CFG_eu868
// # define LMIC_REGION_EU868
// # define CFG_LMIC_REGION_MASK 0 //LMIC_REGION_EU868

/* print all GPS data */
// #define GPS_VERBOSE

#include "Arduino.h"
#include <stdint.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <U8x8lib.h>
#include <cayenneLPP.h>

#include "dr_lorawan.h"
#include "stc3100.h"
#include <Wire.h>
#include <TinyGPS++.h>
#include <LSM6.h>

#include <WiFi.h>
#include <BluetoothSerial.h>
#include <esp_wifi.h>
#include "driver/adc.h"

#define EN_LORA 4
#define EN_GPS 13
#define BUZZER 25

#define IMU_INT1 18
#define IMU_INT2 23
#define IMU_CS 19

TinyGPSPlus *gps = new TinyGPSPlus();
STC3100 stc3100_1;
bool gpsfirstboot = true;
String gpsstring = "";
LSM6 imu;

bool LoRa_status = false;
bool GPS_status = false;
bool IMU_status = false;
void esp32hibernation();
void enableIMU();
void disableIMU();
uint16_t getTEMPS_private(int force)
{
    int v_read = 0;
    int status;
    v_read = stc3100_1.stc3100_getTEMP(force); // readRegister_2b(I2CADDRESS_GASGAUGE_STC, 8); //stc3100_getVbat_o();
    return v_read;
}

uint16_t getVBATS_private(int force)
{
    int v_read = 0;
    int status;
    v_read = stc3100_1.stc3100_getVbat(force); // readRegister_2b(I2CADDRESS_GASGAUGE_STC, 8); //stc3100_getVbat_o();
    if (v_read > 6000 || v_read < 2000)
    {
        log_w("trying again to get vbat");
        v_read = stc3100_1.stc3100_getVbat(1);
    }
    return v_read;
}
uint16_t getmaBATS_private(int force)
{
    int mA_read = 0;
    mA_read = stc3100_1.stc3100_getmAbat(force);
    return (uint16_t)mA_read;
}
uint16_t getChargeCountBATS_private(int nop)
{
    int value = 0;
    bool force = false;
    if (nop != 0)
        force = true;

    value = stc3100_1.stc3100_getChargeCount(force);
    return (uint16_t)value;
}
uint16_t ResetGasGauge_private(int nop)
{
    return stc3100_1.STC3100_resetGauge();
}
uint16_t GasGaugeInit_woScheduler_beforefifo(int a)
{
    stc3100_1.STC3100_Startup();
    return 1;
}
// #define LORAMODULE_HELTEC

/*
Configurate OAA mode:
# define NOTOTAA
//Create applications / devices into Chipstack with DevEUI = 7248d956e61c5982 ( PROGMEM DEVEUI[8] reversed since it is little endian)
//Copy 2B7E151628AED2A6ABF7158809CF4F3C (APPKEY[16]) into chipstack device OTAA keys

*/

#define TIMEOUT_MS 10000
#define REQUEST_MS 20000

// String messageQueue[MAX_QUEUE_SIZE];

unsigned long lastMessageTime = 0;

const unsigned TX_INTERVAL = 10;

static const PROGMEM u1_t NWKSKEY[16] = {0xCB, 0x46, 0x63, 0xCD, 0xDD, 0x22, 0x8B, 0x16, 0x1E, 0xD3, 0x6A, 0x1E, 0xB9, 0x98, 0x2C, 0x76};
// static const PROGMEM u1_t APPSKEY[16] ={ 0xBE, 0x06, 0x39, 0x85, 0x83, 0xC3, 0xB8, 0x46, 0xCF, 0x91, 0xB0, 0x30, 0x33, 0x64, 0x49, 0x37 };
// 0x72, 0x48, 0xD9, 0x56, 0xE6, 0x1C, 0x59, 0x82
static const PROGMEM u1_t APPSKEY[8] = {0x72, 0x48, 0xD9, 0x56, 0xE6, 0x1C, 0x59, 0x82};

static const u4_t DEVADDR = 0x84439d4; // <-- Change this address for every node!

// CayenneLPP lpp(51);

#ifdef NOTOTAA
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}
#else
// https://descartes.co.uk/CreateEUIKey.html
// 7248D956E61C5982
// 0x72, 0x48, 0xD9, 0x56, 0xE6, 0x1C, 0x59, 0x82

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// static const u1_t PROGMEM APPEUI[8]={ 0x72, 0x48, 0xD9, 0x56, 0xE6, 0x1C, 0x59, 0x82 };

void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
// static const u1_t PROGMEM DEVEUI[8]={ 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// static const u1_t PROGMEM DEVEUI[8]={ 0x72, 0x48, 0xD9, 0x56, 0xE6, 0x1C, 0x59, 0x82 };
static const u1_t PROGMEM DEVEUI[8] = {0x82, 0x59, 0x1C, 0xE6, 0x56, 0xD9, 0x48, 0x72};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }
#endif

void do_send(osjob_t *j, Message message);
void sendMsg(Message message);

static uint8_t mydata[] = "Hi from WGLabz!";
static osjob_t sendjob;

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
const lmic_pinmap lmic_pins = {
    .nss = LoRa_nss, // CS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LoRa_rst, //
    .dio = {LoRa_dio0, LoRa_dio1, LoRa_dio2},
    .spi_freq = F_CPU / 15};
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
const lmic_pinmap lmic_pins = {
    .nss = LoRa_nss, // CS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LoRa_rst, //
    .dio = {LoRa_dio0, LoRa_dio1, LoRa_dio2},
    .spi_freq = F_CPU / 15};
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
// OLED Declaration
// U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

DRLORAWAN drlw_1;

// LED and Button Pins
int buttonPin = 13;
int ledPin = 12;
int boardLED = 25;
int lastState = 0;

int currenttime = millis();
int currenttime2 = millis();
bool retrying = false;

void ledFLash(int flashes)
{
    int lastStateLED = digitalRead(ledPin);
    for (int i = 0; i < flashes; i++)
    {
        digitalWrite(ledPin, HIGH);
        delay(300);
        digitalWrite(ledPin, LOW);
        delay(300);
    }
    digitalWrite(ledPin, lastStateLED);
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));

        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
        LMIC_setLinkCheckMode(0);
        break;
    case EV_RFU1:
        Serial.println(F("EV_RFU1"));
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
        break;
    case EV_TXCOMPLETE:
        // u8x8.drawString(0, 2, "Data Sent");
        Serial.println(F("EV_TXCOMPLETE"));

        // Serial.print(F("LMIC.txrxFlags: "));
        // Serial.print(LMIC.txrxFlags);
        // Serial.print(F("\t[TXRX_ACK: "));
        // Serial.print(TXRX_ACK);
        // Serial.print(F(", TXRX_NACK: "));
        // Serial.print(TXRX_NACK);
        // Serial.print(F(", TXRX_NOPORT: "));
        // Serial.print(TXRX_NOPORT);
        // Serial.print(F(", TXRX_PORT: "));
        // Serial.print(TXRX_PORT);
        // Serial.print(F(", TXRX_DNW1: "));
        // Serial.print(TXRX_DNW1);
        // Serial.print(F(", TXRX_DNW2: "));
        // Serial.print(TXRX_DNW2);
        // Serial.print(F(", TXRX_PING: "));
        // Serial.print(TXRX_PING);
        // Serial.println(F("]"));

        if (LMIC.txrxFlags & TXRX_ACK)
        {
            Serial.println("[Received ack for message " + String(drlw_1.getCurrentMessageId()) + "]");
            drlw_1.pendingack_drlora = false;
        }
        if (LMIC.dataLen)
        {
            Serial.print(F("Received "));
            Serial.print(LMIC.dataLen);
            Serial.print(F(" bytes of payload: "));
            // u8x8.drawString(0, 3, "Data Received: ");

            Serial.print(F("0x"));
            for (int i = 0; i < LMIC.dataLen; i++)
            {
                if (LMIC.frame[LMIC.dataBeg + i] < 0x10)
                {
                    Serial.print(F("0"));
                }
                Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
            }
            Serial.println();
        }

        // Schedule next transmission
        // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
        // do_send(&sendjob);

        // delay(2000);
        // u8x8.clearLine(1);
        // u8x8.clearLine(2);
        // u8x8.clearLine(3);
        // u8x8.clearLine(4);

        break;

    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("---->>> EV_RXCOMPLETE"));

        // print data of the packet
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
        // u8x8.drawString(0, 3, "Data Received: ");
        Serial.print(F("0x"));
        for (int i = 0; i < LMIC.dataLen; i++)
        {
            if (LMIC.frame[LMIC.dataBeg + i] < 0x10)
            {
                Serial.print(F("0"));
            }
            Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
        }
        Serial.println();

        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;

    default:
        Serial.print(F("Unknown event: 0x"));
        Serial.println(ev, HEX);
        // u8x8.drawString(0, 2, "Unknown event");
        break;
    }
}

void sendNextMessage()
{
    if (!drlw_1.pendingack_drlora && drlw_1.messageQueueLength() > 0)
    {
        Message nextMessage = drlw_1.dequeueMessage();
        drlw_1.pendingack_drlora = nextMessage.confirmed;
        // do_send(&sendjob, nextMessage, true);
        sendMsg(nextMessage);
        lastMessageTime = millis();
    }
}

void addKeyToIntJson(String &jsonString, const char *key, int value)
{
    // Busca la posición del último corchete que cierra el JSON
    int lastBracket = jsonString.lastIndexOf('}');

    // Verifica si la cadena JSON está vacía
    if (lastBracket != -1)
    {
        // Extrae la parte del JSON antes del último corchete que cierra
        String jsonStringBefore = jsonString.substring(0, lastBracket);

        // Añade la nueva clave y valor
        jsonString = jsonStringBefore + ", \"" + String(key) + "\":" + String(value) + "}";
    }
}

// Función para enviar datos a través de LMIC
void sendData(Message message)
{
    if (message.retryCount == 0)
    {
        // messageid++;
        message.messageId = drlw_1.getTail();
    }
    String &data = message.message;
    // Envía los datos con LMIC_setTxData2
    Serial.println();
    Serial.println("Sending data: " + data + " Confirmed: " + (message.confirmed ? "true" : "false") + " MessageId: " + String(message.messageId));
    int messageid = message.messageId;
    // add messageid to the data json
    addKeyToIntJson(data, "messageid", messageid);
    LMIC_setTxData2(1, (uint8_t *)data.c_str(), data.length(), message.confirmed);
}

unsigned long timewithoutack = millis();
void requestMsg()
{
    if (drlw_1.messageQueueLength() != 0)
    {
        // Serial.println("Pending messages ("+String(drlw_1.messageQueueLength())+"). Not requesting yet...");
        log_i("Pending messages. '%s' Not requesting yet...", String(drlw_1.messageQueueLength()));
        return;
    }

    if (drlw_1.pendingack_drlora)
    {
        log_i("Pending ACK. Not requesting...");
        ////u8x8.drawString(0, 4, "Pending ACK");
        if (millis() - timewithoutack > TIMEOUT_MS)
        {
            // TODO
            log_e("Abandoning pending ACK.");
            timewithoutack = millis();
            drlw_1.pendingack_drlora = false;
        }
        return;
    }
    // Serial.println(F("Requesting Data"));
    String jsonstring = String("{\"type\":\"requestdata\"}");
    Message msg;
    msg.message = jsonstring;
    msg.confirmed = false;

    do_send(&sendjob, msg);
}

// TODO : create FIFO queue, wait ack for each message.
// If not received, retry sending after random time between 1 and 5 seg. Retry 3 times. Erase if not received after 3 times
// send message request only when there is no pending message
void sendMsg(Message message)
{
    drlw_1.pendingack_drlora = true;
    timewithoutack = millis();
    message.confirmed = true;
    message.timestamp = millis();
    do_send(&sendjob, message);
    // send again if not received notify and send again
}

void incrementCurrentRetryCount_0()
{
    drlw_1.incrementCurrentRetryCount();
}

void do_send(osjob_t *j, Message message)
{

    // Check if there is not a current TX/RX job running
    // u8x8.drawString(0, 4, "Sending Data");
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
        // u8x8.drawString(0, 1, "OP_TXRXPEND, not sending");
    }
    else
    {
        // Prepare upstream data transmission at the next possible time.
        sendData(message);
        // u8x8.drawString(0, 1, "Packet queued");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

unsigned long gpson_time = 0;
void playBuzzer(int freq, int duration)
{
    tone(BUZZER, freq, duration);
}

void readgasgauge()
{
    int batdata = stc3100_1.ReadBatteryData();

    int vbat = stc3100_1.stc3100_getVbat();
    int mabat = stc3100_1.stc3100_getmAbat();
    int chargecount = stc3100_1.stc3100_getChargeCount();
    int temp = stc3100_1.stc3100_getTEMP();
    log_d("Battery Data Readed: %d, Vbat: %d, mAbat: %d, ChargeCount: %d, Temp: %d", batdata, vbat, mabat, chargecount, temp);
}

/*
RED    => "\033[31m",
GREEN  => "\033[32m",
YELLOW => "\033[33m",
BLUE   => "\033[34m",
PURPLE => "\033[35m",
CYAN   => "\033[36m",
WHITE  => "\033[37m",

# background color
BLACKB  => "\033[40m",
REDB    => "\033[41m",
GREENB  => "\033[42m",
YELLOWB => "\033[43m",
BLUEB   => "\033[44m",
PURPLEB => "\033[45m",
CYANB   => "\033[46m",
WHITEB  => "\033[47m",

# bold
B    => "\033[1m",
BOFF => "\033[22m",

# italics
I => "\033[3m",
IOFF => "\033[23m",

# underline
U => "\033[4m",
UOFF => "\033[24m",

# invert
R => "\033[7m",
ROFF => "\033[27m",

# reset
RESET  => "\033[0m",
*/


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

#define LORA_EN_STATUS true
void enableLoRa()
{
    digitalWrite(EN_LORA, LORA_EN_STATUS);
    delay(100);
    //SPI enable
    while (os_init() == 0)
    ////if(os_init_ex(&lmic_pins)==0) //(os_init()==0)
    {
        Serial.println("OS Init Failed");
        Serial.print("MOSI: ");
        Serial.println(MOSI);
        Serial.print("MISO: ");
        Serial.println(MISO);
        Serial.print("SCK: ");
        Serial.println(SCK);
        Serial.print("SS: ");
        Serial.println(SS);
        delay(1000);
        // return;
    }

    Serial.println("OS Init OK");
    Serial.print("MOSI: ");
    Serial.println(MOSI);
    Serial.print("MISO: ");
    Serial.println(MISO);
    Serial.print("SCK: ");
    Serial.println(SCK);
    Serial.print("SS: ");
    Serial.println(SS);

    LMIC_reset();
    LoRa_status = true;

    
#ifdef NOTOTAA
// Set static session parameters. Instead of dynamically establishing a session
// by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);
#else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band

    // Disable link check validation
    LMIC_setLinkCheckMode(0);
    LMIC.dn2Dr = DR_SF9;
#endif
    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7, 14);

    // In my place we are not using 868 MHz
    // LMIC_disableChannel(0);
    // LMIC_disableChannel(1);
    // LMIC_disableChannel(2);

    String jsonstring = String("{\"message\":\"Booting device\"}");
    Message msg;
    msg.message = jsonstring;
    sendMsg(msg);

    // Serial.println(F("LORA Initialized"));
    log_i("LORA Initialized");
}
void disableLoRa()
{
    //LMIC_reset();
    //LMIC_shutdown();
    spi_stop();

    //setting SPI lora pins to input mode
    pinMode(LoRa_MOSI, INPUT);
    pinMode(LoRa_MISO, INPUT);
    pinMode(LoRa_SCK, INPUT);
    pinMode(LoRa_nss, INPUT);
    pinMode(LoRa_rst, INPUT);
    pinMode(LoRa_dio0, INPUT);
    pinMode(LoRa_dio1, INPUT);
    pinMode(LoRa_dio2, INPUT);

    digitalWrite(EN_LORA, !LORA_EN_STATUS);
    //digitalWrite(LoRa_nss, LOW);
    // LMIC_reset();
    // LMIC_close();
    LoRa_status = false;

}

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

bool gasGaugeON = false;
void startGasGauge()
{
    stc3100_1.STC3100_Startup();
    //stc3100_1.STC3100_resetGauge();
    gasGaugeON = true;
}
void stopGasGauge()
{
    stc3100_1.STC3100_Powerdown();
    gasGaugeON = false;
}
void setup()
{
    delay(100);
    // setup the display
    // u8x8.begin();
    // u8x8.setFont(u8x8_font_chroma48medium8_r);
    // u8x8.drawString(0, 0, "WGLabz LoRa Test");
    //ESP32DualCoreClass::end();

    Serial.begin(115200);
    // Serial.println(F("Starting"));
    // printf("\033[1;31m[E] RED\033[0m;");

    /* bypass between Serial Serial2 */
    
    changeCPUFreq(80);

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
    log_i("Starting");

    // START GAS GAUGE & I2C
    Serial.println(F("Starting Wire"));
    Wire.begin();

    Serial.println(F("STC3100 startGasGauge"));
    startGasGauge();
    Serial.println(F("STC3100 Started"));
    stc3100_1.STC3100_resetGauge();
    // getting the battery data
    readgasgauge();
    log_i("stop gas gauge");
    stopGasGauge();

    // START IMU
    //Enable IMU pins
    pinMode(IMU_INT1, INPUT);
    pinMode(IMU_INT2, INPUT);
    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_CS, HIGH);

    #define DSO_SA0_LOW_ADDRESS  0b1101010

    log_d("Testing IMU: %d", imu.testReg(DSO_SA0_LOW_ADDRESS, LSM6::WHO_AM_I));

    //i2cScanner();

#ifdef LORAMODULE_HELTEC
    Serial.println(F("LORA MODULE= HELTEC"));
    // In/Out Pins
    pinMode(ledPin, OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP);
    digitalWrite(buttonPin, HIGH);
#else
    Serial.println(F("LORA MODULE= CUSTOM"));
    // SPI.begin(LoRa_SCK, 35, LoRa_MOSI, LoRa_nss); //MISO: 19, MOSI: 27, SCK: 5

    // Configure pins
    pinMode(EN_LORA, OUTPUT);
    pinMode(EN_GPS, OUTPUT);


    disableLoRa();
    disableGPS();
    //enableIMU();
    delay(200);

    //enableLoRa();
    //enableGPS();

    pinMode(BUZZER, OUTPUT);
    //playBuzzer(1000, 1000);

    // pinMode(LoRa_MISO, INPUT); // impedance test
    delay(100);
#endif

    // LMIC init &RESET
    // os_init_ex returns 0 if the LMIC was already initialized

    // os_init();
    // os_init_ex(&lmic_pins);
    
while(0)
{
    log_i("Going to hibernation");
    esp32hibernation();
}
}

void gpsloop()
{
    if (Serial2.available() > 0)
    {
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
            }
        }
    }
}

void printGPSData()
{
    log_d("GPS Data: %s", gpsstring.c_str());
}

void loraloop()
{
    if (drlw_1.messageQueueLength() > 0)
    {
        if (drlw_1.pendingack_drlora == false)
        {
            retrying = false;
            Serial.println("Current queue length: " + String(drlw_1.messageQueueLength()));
            sendNextMessage();
        }
        else
        {
            // Serial.println("waiting time: "+String(millis()-lastMessageTime));

            if (millis() - lastMessageTime > 2 * TIMEOUT_MS)
            {
                Serial.println("clearCurrentMessage: 2 x Timeout reached. Queue length: " + String(drlw_1.messageQueueLength()) + "");
                drlw_1.clearCurrentMessage();
                retrying = false;
            }
            else if (millis() - lastMessageTime > TIMEOUT_MS && !retrying)
            {
                Serial.println("Resending message. Timeout reached. Queue length: " + String(drlw_1.messageQueueLength()));
                // retryCurrentMessage();
                // if (millis() - lastMessageTime > TIMEOUT_MS) {
                //  Timeout reached, retry the current message
                drlw_1.pendingack_drlora = true;
                // do_send(&sendjob, messageQueue[head], true);
                drlw_1.incrementCurrentRetryCount();
                sendMsg(drlw_1.getCurrentMessage());
                // lastMessageTime = millis();
                //}

                retrying = true;
            }
        }
    }
    else
    {
        if (millis() - currenttime2 > REQUEST_MS)
        {
            currenttime2 = millis();
            requestMsg();
        }
    }
}

unsigned long currenttime_3 = millis();
unsigned long currenttime_4 = millis();
unsigned long currenttime_5 = millis();

void sendloramessage()
{
    // https://gist.github.com/kamito/704813
    currenttime = millis();
    String jsonstring = String("{\"rand\":") + String(random(0, 100)) + ", \"temp\":" + String(random(0, 100)) + ", \"hum\":" + String(random(0, 100)) + ", \"log\": \"Hello\", \"confirm\": \"true\"}";
    // sendMsg(jsonstring);
    // drlw_1.enqueueMessage_s(jsonstring);
    Message messageStruct;
    messageStruct.message = jsonstring;
    // drlw_1.enqueueMessage(messageStruct);
}

void imuReadRegs()
{
    // Read all the registers
    for (int i = 0; i < 0x6C; i++)
    {
        Serial.print("0x");
        Serial.print(i, HEX);
        Serial.print(": 0x");
        Serial.println(imu.readReg(i), HEX);
    }

}
void disableIMU()
{
    imu.DisableIMU();
    // set CS, INT1, INT2 to input mode
    pinMode(IMU_CS, INPUT);
    pinMode(IMU_INT1, INPUT);
    pinMode(IMU_INT2, INPUT);
    IMU_status=false;
    
}
void enableIMU()
{
    if (!imu.init(LSM6::device_auto, LSM6::sa0_low))
    {
        Serial.println("Failed to detect and initialize IMU!");
        while (1);
    }
    imu.enableDefault();
    IMU_status=true;
}
char report[80];
    int uS_TO_S_FACTOR = 1000000; //Conversion factor for micro seconds to seconds
void setTimetoSleep(int timetosleep0)
{
  log_d("Setup ESP32 to sleep for every %d minutes", timetosleep0);
  if (timetosleep0 > 0)
  {

    esp_sleep_enable_timer_wakeup((uint64_t)((uint64_t)timetosleep0 * (uint64_t)(uS_TO_S_FACTOR * 60)));

    //timetosleep = timetosleep0;
  }
}
void esp32hibernation()
{
  log_d(" ESP32HIBERNATION 1");

  log_d(" ESP32HIBERNATION 2");

  esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  //uint64_t sleeptime=10 * 60* uS_TO_S_FACTOR;
  //log_d(" Going to HIBERNATE. Setting time to %d", sleeptime);
  //esp_sleep_enable_timer_wakeup(timetosleep);
  setTimetoSleep(20);
  esp_deep_sleep_start();
}


void lightsleepesp(bool deepsleep)
{
  //enablesleepwithuart();
  //log_i(" ica1.SetICAwakeModeBeforeSleep();");
  if (deepsleep)
  {
    log_i(" esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT0);");
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT0);
    //disableAllISR();
  }

  log_d(" LIGHTSLEEP");
    int minutes=2;
  log_w("# esp_light_sleep for %d minutes", minutes);
  delay(500);
  //drconnect_1.resetkccounter0();
  esp_sleep_enable_timer_wakeup(minutes * 60 * uS_TO_S_FACTOR);
  esp_light_sleep_start();
}

void getGPSmeasure()
{
    log_d("Getting GPS measure");
    //get current from gas gauge
    stc3100_1.ReadBatteryData();
    int startculombs=stc3100_1.stc3100_getChargeCount();
    unsigned long starttime=millis();

    log_d("Enabling GPS");
    //enable GPS
    enableGPS();
    //wait for 10 seconds
    delay(1000);
    //read GPS data
    log_d("Waiting GPS data");
    unsigned long waitingtime=millis();
    while(gpsfirstboot)
    {
        gpsloop();

        //print time each 10s
        if(millis()-waitingtime>10000)
        {
            waitingtime=millis();
            log_d("Waiting GPS data: %ds", (waitingtime-starttime)/1000);
            //log gpsfirstboot, gps->hdop.hdop(), gps->hdop.hdop(), gps->satellites.value()
            log_d("GPS: gpsfirstboot:%d, hdop:%2.2f, satellites:%d", gpsfirstboot, gps->hdop.hdop(), gps->satellites.value());
        }
    }
    //disable GPS
    disableGPS();
    stc3100_1.ReadBatteryData();
    int endculombs=stc3100_1.stc3100_getChargeCount();
    unsigned long endtime=millis();
    log_i("GPS data: on time: %ds, culombs: %d, gpsfb:%d, hdop:%2.2f, satellites:%d, lat: %f, long: %f", (waitingtime-starttime)/1000,endculombs-startculombs, gpsfirstboot, gps->hdop.hdop(), gps->satellites.value(), gps->location.lat(), gps->location.lng());



}

bool bussy=false;
unsigned long downfreqtime=millis();
void loop()
{
    if(LoRa_status)
    {
        // Run LORA loop
        os_runloop_once();

        // send LORA data every 10 seconds
        if (millis() - currenttime > 10000)
        {
            sendloramessage();
        }
        loraloop();
        bussy=true;
    }

    // read gas gauge each 10 seconds
    if (millis() - currenttime_3 > 10000)
    {
        currenttime_3 = millis();
        //    readgasgauge();
    }
    
    //if(GPS_status)
    //    gpsloop();

    // Print each 10s: lat, long, nsat, hdop, vbat, mabat, chargecount, temp, millis()-gpson_time, pending ack
    if (millis() - currenttime_5 > 10000)
    {
        log_d("Reading battery data");
        currenttime_5 = millis();
        if(gasGaugeON)
            stc3100_1.ReadBatteryData();
        if(IMU_status)
        {
            imu.read();
            snprintf(report, sizeof(report), "A: %6d %6d %6d    G: %6d %6d %6d",
            imu.a.x, imu.a.y, imu.a.z,
            imu.g.x, imu.g.y, imu.g.z);
        }
        else
        {
            snprintf(report, sizeof(report), "IMU not enabled");
        }
        char report2[80];
        if(GPS_status)
        {
            gpsloop();
            
            snprintf(report2, sizeof(report2), "time gps on: %ds, lat: %f, long: %f, nsat: %d, hdop: %2.2f",
            (millis() - gpson_time) / 1000,gps->location.lat(), gps->location.lng(), gps->satellites.value(), gps->hdop.hdop());
        }
        else
        {
            snprintf(report2, sizeof(report), "GPS not enabled");
        }
        char report4[80];
        if(gasGaugeON)
        {
            snprintf(report4, sizeof(report4), "Vbat: %d, mAbat: %d, ChargeCount: %d, Temp: %d",
            stc3100_1.stc3100_getVbat(), stc3100_1.stc3100_getmAbat(), stc3100_1.stc3100_getChargeCount(), stc3100_1.stc3100_getTEMP());
        }
        else
        {
            snprintf(report4, sizeof(report4), "Gas Gauge not enabled");
        }
        log_d("ACK...");
        
        log_d("\n\tPending ACK: %d, Pending lora messages: %d, \n\t%s, \n\tGPS: %s\n\tIMU: %s\n",
              drlw_1.pendingack_drlora, drlw_1.messageQueueLength(), report4 ,report2 , report);
            
        bussy=true;
    }

    // read GPS

    // read accelerometer

    if (Serial.available() > 0)
    {
        char keyPressed = Serial.read();

        switch (keyPressed)
        {
        case '0':
            log_d("Beeping one time");
            playBuzzer(1000, 1000);
            break;
        case '1':
            // Serial.println("Sending Lora Message");
            log_i("Sending Lora Message");
            sendloramessage();
            break;

        case '3':
            // Serial.println("Reading GPS");
            log_i("Reading GPS");
            gpsloop();
            break;
        case '4':
            // enable GPS
            log_i("Enabling GPS");
            enableGPS();
            break;
        case '5':
            // disable GPS
            log_i("Disabling GPS");
            disableGPS();
            break;
        case '6':
            // enable LoRa
            log_i("Enabling LoRa");
            enableLoRa();
            break;
        case '7':
            // disable LoRa
            log_i("Disabling LoRa");
            disableLoRa();
            break;
        case '8':
            // print GPS data
            log_i("Printing GPS Data");
            printGPSData();
            break;

        case '9':
            //shutdown ESP32 during 60s
            log_i("Shutting down ESP32");
            esp_deep_sleep(60000000);
            break;

        case 'a':
        log_i("Setting CPU to 10MHz");
            changeCPUFreq(10);
            break;

        case 'b':
        log_i("Setting CPU to 40MHz");
            changeCPUFreq(40);
            break;
        
        case 'c':
        log_i("Setting CPU to 80MHz");
            changeCPUFreq(80);
            break;
        case 'd': 
            log_i("Setting CPU to 160MHz");
            changeCPUFreq(160);
        
        case 'e':
            log_i("Enabling low power mode");
            lowpowermode();
            break;
        case 'f':
            log_i("Disable IMU");
            disableIMU();            
            break;
        case 'g':
            log_i("Enable IMU");
            enableIMU();
            break;
        case 'h':
            log_i("Setting CPU to only one core");
            forceOneCore();
            break;
        case 'i':
            //log_i("Enabling low power mode");
            //lowpowermode();
            log_i("Entering into hibernation mode");
            esp32hibernation();
            break;
        case 'j':
            log_i("Read IMU Registers");
            imuReadRegs();
            break;
        case 'k':
            log_i("getting gps measure");
            getGPSmeasure();
        case 'l':
            Serial.print("Current CPU Freq: ");
            Serial.println(getCpuFrequencyMhz());
            break;
        case 'm':
            log_i("downfreq to 10MHz during 30sec");
            changeCPUFreq(10);
            downfreqtime=millis();
            break;
        case 'n':
            log_i("stop gas gauge");
            stopGasGauge();
            break;
        case 'o':
            log_i("start gas gauge");
            startGasGauge();
            break;
        case 'p':
            log_i("stc3100_1.ReadBatteryData");
            stc3100_1.ReadBatteryData();
            break;
        case 'q':
            log_i("read gas gauge registers");
            stc3100_1.ReadRegisters();
            break;
        case '2':
            // Serial.println("Reading Gas Gauge");
            log_i("Reading Gas Gauge");
            readgasgauge();
            break;
        case 'r':
            log_i("start gauge ");
            Serial.println(F("STC3100 startGasGauge"));
            startGasGauge();
            Serial.println(F("STC3100 Started"));
            stc3100_1.STC3100_resetGauge();
            break;
        case 's':
            log_i("Enable Radio");
            enableRadio();
            break;
        case 't':
            log_d("Disabling Radio");
            disableRadio();
            break;
        default:
            //Serial.println("Tecla no válida");
            log_w("Tecla no válida");
            break;
        }
        
        
        //0x10: 00010000
        //0x0c: 00001100
        bussy=true;
    }

    //if(!bussy)
    //{
        delay(500);
    //}

    //if(getCpuFrequencyMhz()==10 && millis()-downfreqtime>30000)
    //{
    //    changeCPUFreq(80);
    //    log_w("Going back to 80MHz");
    //}
}


// Consumo de unos 200uA (0,2mA) en hibernation. Deberia ser unos 3uA