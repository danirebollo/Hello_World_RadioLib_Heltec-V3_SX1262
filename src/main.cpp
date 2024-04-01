//Works on Heltec, not on custom (almost)

// set 	-D LORAMODULE_HELTEC in platformio.ini

/***
 * Original Code: https://github.com/matthijskooijman/arduino-lmic/blob/master/examples/ttn-abp/ttn-abp.ino
 * Modified By: Bikash Narayan Panda (weargeniuslabs@gmail.com) 
 * ***/

//# define LMIC_DR_LEGACY
//# define CFG_eu868
//# define LMIC_REGION_EU868
//# define CFG_LMIC_REGION_MASK 0 //LMIC_REGION_EU868
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

STC3100 stc3100_1;

uint16_t getTEMPS_private(int force)
{
    int v_read = 0;
    int status;
    v_read = stc3100_1.stc3100_getTEMP(force); //readRegister_2b(I2CADDRESS_GASGAUGE_STC, 8); //stc3100_getVbat_o();
    return v_read;
}

uint16_t getVBATS_private(int force)
{
    int v_read = 0;
    int status;
    v_read = stc3100_1.stc3100_getVbat(force); //readRegister_2b(I2CADDRESS_GASGAUGE_STC, 8); //stc3100_getVbat_o();
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
    stc3100_1.STC3100_resetGauge();
}
uint16_t GasGaugeInit_woScheduler_beforefifo(int a)
{
    stc3100_1.STC3100_Startup();
    return 1;
}
//#define LORAMODULE_HELTEC

/*
Configurate OAA mode: 
# define NOTOTAA
//Create applications / devices into Chipstack with DevEUI = 7248d956e61c5982 ( PROGMEM DEVEUI[8] reversed since it is little endian)
//Copy 2B7E151628AED2A6ABF7158809CF4F3C (APPKEY[16]) into chipstack device OTAA keys

*/

#define TIMEOUT_MS 10000
#define REQUEST_MS 20000

//String messageQueue[MAX_QUEUE_SIZE];

unsigned long lastMessageTime = 0;

const unsigned TX_INTERVAL = 10;

static const PROGMEM u1_t NWKSKEY[16] ={ 0xCB, 0x46, 0x63, 0xCD, 0xDD, 0x22, 0x8B, 0x16, 0x1E, 0xD3, 0x6A, 0x1E, 0xB9, 0x98, 0x2C, 0x76 };
//static const PROGMEM u1_t APPSKEY[16] ={ 0xBE, 0x06, 0x39, 0x85, 0x83, 0xC3, 0xB8, 0x46, 0xCF, 0x91, 0xB0, 0x30, 0x33, 0x64, 0x49, 0x37 };
//0x72, 0x48, 0xD9, 0x56, 0xE6, 0x1C, 0x59, 0x82
static const PROGMEM u1_t APPSKEY[8]={ 0x72, 0x48, 0xD9, 0x56, 0xE6, 0x1C, 0x59, 0x82 };

static const u4_t DEVADDR = 0x84439d4; // <-- Change this address for every node!

//CayenneLPP lpp(51);

#ifdef NOTOTAA
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
#else
//https://descartes.co.uk/CreateEUIKey.html
//7248D956E61C5982
//0x72, 0x48, 0xD9, 0x56, 0xE6, 0x1C, 0x59, 0x82

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//static const u1_t PROGMEM APPEUI[8]={ 0x72, 0x48, 0xD9, 0x56, 0xE6, 0x1C, 0x59, 0x82 };

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
//static const u1_t PROGMEM DEVEUI[8]={ 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//static const u1_t PROGMEM DEVEUI[8]={ 0x72, 0x48, 0xD9, 0x56, 0xE6, 0x1C, 0x59, 0x82 };
static const u1_t PROGMEM DEVEUI[8]={ 0x82, 0x59, 0x1C, 0xE6, 0x56, 0xD9, 0x48, 0x72};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
#endif

void do_send(osjob_t* j, Message message);
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
  .spi_freq = F_CPU/15
};
#else
#define LoRa_nss 32
#define LoRa_rst 14
#define LoRa_dio0 5 //UNCONNECTED, unused in ESP32
#define LoRa_dio1 12 //UNCONNECTED, unused in ESP32
#define LoRa_dio2 15 //UNCONNECTED, unused in ESP32
#define LoRa_MOSI 33
#define LoRa_MISO 26
#define LoRa_SCK 27
//#define LoRa_nrst 12
//#define LoRa_busy 13
// Pin mapping CUSTOM BOARD
const lmic_pinmap lmic_pins = {
  .nss = LoRa_nss, // CS
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LoRa_rst, // 
  .dio = {LoRa_dio0, LoRa_dio1, LoRa_dio2},
  .spi_freq = F_CPU/15
};
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
    
    //SPI.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI, LoRa_nss);

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
//OLED Declaration 
//U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

DRLORAWAN drlw_1;

//LED and Button Pins
int buttonPin = 13;
int ledPin=12;
int boardLED=25;
int lastState=0;

int currenttime=millis();
int currenttime2=millis();
bool retrying=false;

void ledFLash(int flashes){
    int lastStateLED=digitalRead(ledPin);
    for(int i=0;i<flashes;i++){
        digitalWrite(ledPin, HIGH);
        delay(300);
        digitalWrite(ledPin, LOW);
        delay(300);
    }
    digitalWrite(ledPin,lastStateLED);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
            //u8x8.drawString(0, 2, "Data Sent");
            Serial.println(F("EV_TXCOMPLETE"));

            //Serial.print(F("LMIC.txrxFlags: "));
            //Serial.print(LMIC.txrxFlags);
            //Serial.print(F("\t[TXRX_ACK: "));
            //Serial.print(TXRX_ACK);
            //Serial.print(F(", TXRX_NACK: "));
            //Serial.print(TXRX_NACK);
            //Serial.print(F(", TXRX_NOPORT: "));
            //Serial.print(TXRX_NOPORT);
            //Serial.print(F(", TXRX_PORT: "));
            //Serial.print(TXRX_PORT);
            //Serial.print(F(", TXRX_DNW1: "));
            //Serial.print(TXRX_DNW1);
            //Serial.print(F(", TXRX_DNW2: "));
            //Serial.print(TXRX_DNW2);
            //Serial.print(F(", TXRX_PING: "));
            //Serial.print(TXRX_PING);
            //Serial.println(F("]"));

            if (LMIC.txrxFlags & TXRX_ACK)
            {
                Serial.println("[Received ack for message " + String(drlw_1.getCurrentMessageId()) + "]");
                drlw_1.pendingack_drlora=false;
            }
            if (LMIC.dataLen) 
            {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.print(F(" bytes of payload: "));
              //u8x8.drawString(0, 3, "Data Received: ");
              
              
               Serial.print(F("0x"));
               for (int i = 0; i < LMIC.dataLen; i++) {
                   if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
                       Serial.print(F("0"));
                   }
                   Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
               }
               Serial.println();
            }


            // Schedule next transmission
           //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
           //do_send(&sendjob);

           //delay(2000);
           //u8x8.clearLine(1);
           //u8x8.clearLine(2);
           //u8x8.clearLine(3);
           //u8x8.clearLine(4);

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
            //u8x8.drawString(0, 3, "Data Received: ");
            Serial.print(F("0x"));
            for (int i = 0; i < LMIC.dataLen; i++) {
                if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
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
            //u8x8.drawString(0, 2, "Unknown event");
            break;
    }
}

void sendNextMessage() {
    if (!drlw_1.pendingack_drlora && drlw_1.messageQueueLength()>0) {
        Message nextMessage = drlw_1.dequeueMessage();
        drlw_1.pendingack_drlora = nextMessage.confirmed;
        //do_send(&sendjob, nextMessage, true);
        sendMsg(nextMessage);
        lastMessageTime = millis();
    }
}

void addKeyToIntJson(String& jsonString, const char* key, int value) {
    // Busca la posición del último corchete que cierra el JSON
    int lastBracket = jsonString.lastIndexOf('}');

    // Verifica si la cadena JSON está vacía
    if (lastBracket != -1) {
        // Extrae la parte del JSON antes del último corchete que cierra
        String jsonStringBefore = jsonString.substring(0, lastBracket);

        // Añade la nueva clave y valor
        jsonString = jsonStringBefore + ", \"" + String(key) + "\":" + String(value) + "}";
    }
}

// Función para enviar datos a través de LMIC
void sendData(Message message) {
    if(message.retryCount==0)
    {
    //messageid++;
    message.messageId=drlw_1.getTail();
    }
    String& data=message.message;
  // Envía los datos con LMIC_setTxData2
  Serial.println();
  Serial.println("Sending data: " + data+ " Confirmed: " + (message.confirmed ? "true" : "false")+ " MessageId: "+String(message.messageId));
    int messageid=message.messageId;
  //add messageid to the data json
    addKeyToIntJson(data, "messageid", messageid);
  LMIC_setTxData2(1, (uint8_t*)data.c_str(), data.length(), message.confirmed);
}

void requestMsg()
{   
    if(drlw_1.messageQueueLength() != 0)
    {
        Serial.println("Pending messages ("+String(drlw_1.messageQueueLength())+"). Not requesting yet...");
        return;
    }

    if(drlw_1.pendingack_drlora){
        Serial.println(F("Pending ACK. Not requesting..."));
        ////u8x8.drawString(0, 4, "Pending ACK");
        return;
    }
    //Serial.println(F("Requesting Data"));
    String jsonstring=String("{\"type\":\"requestdata\"}");
    Message msg;
    msg.message=jsonstring;
    msg.confirmed=false;

    do_send(&sendjob, msg);
}

// TODO : create FIFO queue, wait ack for each message. 
// If not received, retry sending after random time between 1 and 5 seg. Retry 3 times. Erase if not received after 3 times
// send message request only when there is no pending message
void sendMsg(Message message)
{
     drlw_1.pendingack_drlora=true;
     message.confirmed=true;

    do_send(&sendjob, message);
    //send again if not received notify and send again
}

void incrementCurrentRetryCount_0() {
    drlw_1.incrementCurrentRetryCount();
}

void do_send(osjob_t* j, Message message) {

    // Check if there is not a current TX/RX job running
    //u8x8.drawString(0, 4, "Sending Data");
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        //u8x8.drawString(0, 1, "OP_TXRXPEND, not sending");
    } else {
        // Prepare upstream data transmission at the next possible time.
        sendData(message);
        //u8x8.drawString(0, 1, "Packet queued");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

    #define EN_LORA 4
    #define EN_GPS 13
    #define BUZZER 25

void playBuzzer(int freq, int duration){
    tone(BUZZER, freq, duration);
}

void readgasgauge()
{
        int batdata=stc3100_1.ReadBatteryData();
    
    int vbat=stc3100_1.stc3100_getVbat();
    int mabat=stc3100_1.stc3100_getmAbat();
    int chargecount=stc3100_1.stc3100_getChargeCount();
    int temp=stc3100_1.stc3100_getTEMP();

    Serial.println("Battery Data Readed: "+String(batdata));
    Serial.println("Vbat: "+String(vbat));
    Serial.println("mAbat: "+String(mabat));
    Serial.println("ChargeCount: "+String(chargecount));
    Serial.println("Temp: "+String(temp)); 
}
void setup() {
    delay(100);
    //setup the display
    //u8x8.begin();
    //u8x8.setFont(u8x8_font_chroma48medium8_r);
    //u8x8.drawString(0, 0, "WGLabz LoRa Test");

    Serial.begin(115200);
    Serial.println(F("Starting"));

    //#define LoRa_nss 32
    //#define LoRa_dio1 14
    ////#define LoRa_nrst 12
    //#define LoRa_busy 13

    //SPI.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI, LoRa_nss); //MISO: 19, MOSI: 27, SCK: 5
//#define LoRa_MOSI 33
//#define LoRa_MISO 26
//#define LoRa_SCK 27
//#define LoRa_nss 32

    // START GAS GAUGE & I2C 
    Serial.println(F("Starting Wire"));
    Wire.begin();
    Serial.println(F("Wire Started"));
    stc3100_1.STC3100_Startup();
    Serial.println(F("STC3100 Started"));
    stc3100_1.STC3100_resetGauge();
    // getting the battery data

    readgasgauge();

#ifdef LORAMODULE_HELTEC
    Serial.println(F("LORA MODULE= HELTEC"));
    //In/Out Pins
    pinMode(ledPin, OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP);
    digitalWrite(buttonPin, HIGH);
#else
    Serial.println(F("LORA MODULE= CUSTOM"));
    //SPI.begin(LoRa_SCK, 35, LoRa_MOSI, LoRa_nss); //MISO: 19, MOSI: 27, SCK: 5

    pinMode(EN_LORA, OUTPUT);
    digitalWrite(EN_LORA, HIGH);
    pinMode(EN_GPS, OUTPUT);
    digitalWrite(EN_GPS, LOW);
    pinMode(BUZZER, OUTPUT);

    playBuzzer(1000, 1000);

    
    //pinMode(LoRa_MISO, INPUT); // impedance test
    delay(100);
#endif

    // LMIC init &RESET
    // os_init_ex returns 0 if the LMIC was already initialized

    //os_init();
    //os_init_ex(&lmic_pins);
    while(os_init()==0)
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
        //return;
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
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly 
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

    // Disable link check validation
    LMIC_setLinkCheckMode(0);
    LMIC.dn2Dr = DR_SF9;
    #endif
    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // In my place we are not using 868 MHz
    //LMIC_disableChannel(0);
    //LMIC_disableChannel(1);
    //LMIC_disableChannel(2);

    String jsonstring=String("{\"message\":\"Booting device\"}");
    Message msg;
    msg.message=jsonstring;
    sendMsg(msg);
}

void loraloop() {
    
    if(drlw_1.messageQueueLength()>0)
    {
        if(drlw_1.pendingack_drlora==false)
        {
            retrying=false;
            Serial.println("Current queue length: "+String(drlw_1.messageQueueLength()));
            sendNextMessage();
            
        }
        else
        {
            //Serial.println("waiting time: "+String(millis()-lastMessageTime));

            if(millis()-lastMessageTime>2*TIMEOUT_MS)
            {
                Serial.println("clearCurrentMessage: 2 x Timeout reached. Queue length: "+String(drlw_1.messageQueueLength())+"");
                drlw_1.clearCurrentMessage();
                retrying=false;
            }
            else if(millis()-lastMessageTime>TIMEOUT_MS && !retrying)
            {
                Serial.println("Resending message. Timeout reached. Queue length: "+String(drlw_1.messageQueueLength()));
                //retryCurrentMessage();
                    //if (millis() - lastMessageTime > TIMEOUT_MS) {
                    // Timeout reached, retry the current message
                    drlw_1.pendingack_drlora = true;
                    //do_send(&sendjob, messageQueue[head], true);
                    drlw_1.incrementCurrentRetryCount();
                    sendMsg(drlw_1.getCurrentMessage());
                    //lastMessageTime = millis();
                //}

                retrying=true;
            }
        }
    }
    else
    {
        if(millis()-currenttime2>REQUEST_MS){
        currenttime2=millis();
        requestMsg();
        }
    }
}

unsigned long currenttime_3=millis();

void loop() {
    os_runloop_once();

    // send data every 10 seconds
    if(millis()-currenttime>10000){
        currenttime=millis();
        String jsonstring=String("{\"rand\":")+String(random(0,100))+", \"temp\":"+String(random(0,100))+", \"hum\":"+String(random(0,100))+", \"log\": \"Hello\", \"confirm\": \"true\"}";
        //sendMsg(jsonstring);
        //drlw_1.enqueueMessage_s(jsonstring);
        Message messageStruct;
        messageStruct.message = jsonstring;
        //drlw_1.enqueueMessage(messageStruct);
    }
    loraloop();

    // read gas gauge each 10 seconds
    if(millis()-currenttime_3>10000){
        currenttime_3=millis();
        readgasgauge();
    }
}