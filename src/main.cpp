/***
 * Original Code: https://github.com/matthijskooijman/arduino-lmic/blob/master/examples/ttn-abp/ttn-abp.ino
 * Modified By: Bikash Narayan Panda (weargeniuslabs@gmail.com) 
 * ***/

//# define LMIC_DR_LEGACY
//# define CFG_eu868
//# define LMIC_REGION_EU868
//# define CFG_LMIC_REGION_MASK 0 //LMIC_REGION_EU868
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <U8x8lib.h>
#include <cayenneLPP.h>

/*
Configurate OAA mode: 
# define NOTOTAA
//Create applications / devices into Chipstack with DevEUI = 7248d956e61c5982 ( PROGMEM DEVEUI[8] reversed since it is little endian)
//Copy 2B7E151628AED2A6ABF7158809CF4F3C (APPKEY[16]) into chipstack device OTAA keys

*/
#define MAX_QUEUE_SIZE 15
#define TIMEOUT_MS 10000
#define REQUEST_MS 20000

int messagenumber = 0;

int messageid = 0;
typedef struct {
    int messageId=0;
    String message;
    bool confirmed;
    unsigned long timestamp;
    bool ackReceived;
} Message;

Message messages[MAX_QUEUE_SIZE];
//String messageQueue[MAX_QUEUE_SIZE];
int head = 0;
int tail = 0;
bool pendingAck = false;
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
Message getCurrentMessage();

static uint8_t mydata[] = "Hi from WGLabz!";
static osjob_t sendjob;


// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32}
};
//OLED Declaration 
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

//LED and Button Pins
int buttonPin = 13;
int ledPin=12;
int boardLED=25;
int lastState=0;

bool pendingack=false;

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
            u8x8.drawString(0, 2, "Data Sent");
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
                Serial.println("[Received ack for message " + String(messageid) + "]");
                pendingack=false;
            }
            if (LMIC.dataLen) 
            {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.print(F(" bytes of payload: "));
              u8x8.drawString(0, 3, "Data Received: ");
              
              
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
           u8x8.clearLine(1);
           u8x8.clearLine(2);
           u8x8.clearLine(3);
           u8x8.clearLine(4);

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
            u8x8.drawString(0, 3, "Data Received: ");
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
            u8x8.drawString(0, 2, "Unknown event");
            break;
    }
}

void enqueueMessage(Message message) {
    if ((tail + 1) % MAX_QUEUE_SIZE != head) {
        messages[tail] = message;
        //messageid++;
        //messages[tail].messageId = messageid;
        tail = (tail + 1) % MAX_QUEUE_SIZE;
    }
    else
    {
        Serial.println("Queue is full. Cannot enqueue message");
    }
}

void enqueueMessage(String message) {
    Message messageStruct;
    messageStruct.message = message;
    enqueueMessage(messageStruct);
}



Message dequeueMessage() {
    Message message = messages[head];
    head = (head + 1) % MAX_QUEUE_SIZE;
    return message;
}

void sendNextMessage() {
    if (!pendingAck && head != tail) {
        Message nextMessage = dequeueMessage();
        pendingAck = nextMessage.confirmed;
        //do_send(&sendjob, nextMessage, true);
        sendMsg(nextMessage);
        lastMessageTime = millis();
    }
}

Message getCurrentMessage() {
    return messages[head];
}

void retryCurrentMessage() {
    //if (millis() - lastMessageTime > TIMEOUT_MS) {
        // Timeout reached, retry the current message
        pendingAck = true;
        //do_send(&sendjob, messageQueue[head], true);
        sendMsg(messages[head]);
        //lastMessageTime = millis();
    //}
}

void clearCurrentMessage() {
    // Remove the current message from the queue
    head = (head + 1) % MAX_QUEUE_SIZE;
    pendingAck = false;
}

int messageQueueLength() {
    return (tail - head + MAX_QUEUE_SIZE) % MAX_QUEUE_SIZE;
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
    if(messageQueueLength() != 0)
    {
        Serial.println("Pending messages ("+String(messageQueueLength())+"). Not requesting yet...");
        return;
    }

    if(pendingack){
        Serial.println(F("Pending ACK. Not requesting..."));
        //u8x8.drawString(0, 4, "Pending ACK");
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
     pendingack=true;
     message.confirmed=true;

    do_send(&sendjob, message);
    //send again if not received notify and send again
}

void do_send(osjob_t* j, Message message) {
    messageid++;
    message.messageId=messageid;

    // Check if there is not a current TX/RX job running
    u8x8.drawString(0, 4, "Sending Data");
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        u8x8.drawString(0, 1, "OP_TXRXPEND, not sending");
    } else {
        // Prepare upstream data transmission at the next possible time.
        sendData(message);
        u8x8.drawString(0, 1, "Packet queued");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    //setup the display
    u8x8.begin();
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.drawString(0, 0, "WGLabz LoRa Test");

    Serial.begin(115200);
    Serial.println(F("Starting"));

    //In/Out Pins
    pinMode(ledPin, OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP);
    digitalWrite(buttonPin, HIGH);

    // LMIC init &RESET
    os_init();
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

int currenttime=millis();
int currenttime2=millis();

bool retrying=false;

void loraloop() {
    
    if(messageQueueLength()>0)
    {
        if(pendingAck==false)
        {
            retrying=false;
            Serial.println("Current queue length: "+String(messageQueueLength()));
            sendNextMessage();
            
        }
        else
        {
            //Serial.println("waiting time: "+String(millis()-lastMessageTime));

            if(millis()-lastMessageTime>2*TIMEOUT_MS)
            {
                Serial.println("clearCurrentMessage: 2 x Timeout reached. Queue length: "+String(messageQueueLength())+"");
                clearCurrentMessage();
                retrying=false;
            }
            else if(millis()-lastMessageTime>TIMEOUT_MS && !retrying)
            {
                Serial.println("Resending message. Timeout reached. Queue length: "+String(messageQueueLength()));
                retryCurrentMessage();
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

void loop() {
    os_runloop_once();

    // send data every 10 seconds
    if(millis()-currenttime>10000){
        currenttime=millis();
        String jsonstring=String("{\"rand\":")+String(random(0,100))+", \"temp\":"+String(random(0,100))+", \"hum\":"+String(random(0,100))+", \"log\": \"Hello\", \"confirm\": \"true\"}";
        //sendMsg(jsonstring);
        enqueueMessage(jsonstring);
    }
    loraloop();
}