
#include "Arduino.h"
#include <stdint.h>
#include <lmic.h>
#include <hal/hal.h>
#include "dr_lorawan.h"
#include "lora.h"
#include "pinout.h"

DRLORAWAN drlw_1;
/* LoRa CONFIG */
unsigned long lastMessageTime = 0;
const unsigned TX_INTERVAL = 10;
static const PROGMEM u1_t NWKSKEY[16] = {0xCB, 0x46, 0x63, 0xCD, 0xDD, 0x22, 0x8B, 0x16, 0x1E, 0xD3, 0x6A, 0x1E, 0xB9, 0x98, 0x2C, 0x76};
// static const PROGMEM u1_t APPSKEY[16] ={ 0xBE, 0x06, 0x39, 0x85, 0x83, 0xC3, 0xB8, 0x46, 0xCF, 0x91, 0xB0, 0x30, 0x33, 0x64, 0x49, 0x37 };
// 0x72, 0x48, 0xD9, 0x56, 0xE6, 0x1C, 0x59, 0x82
static const PROGMEM u1_t APPSKEY[8] = {0x72, 0x48, 0xD9, 0x56, 0xE6, 0x1C, 0x59, 0x82};
static const u4_t DEVADDR = 0x84439d4; // <-- Change this address for every node!
static osjob_t sendjob;
unsigned long timewithoutack = millis();
bool LoRa_status = false;
bool retrying = false;
int currenttime2 = millis();

int lastloramessage_ms = millis();

#define REQUEST_MS 20000
#define TIMEOUT_MS 10000


// #define LORAMODULE_HELTEC

/*
Configurate OAA mode:
# define NOTOTAA
//Create applications / devices into Chipstack with DevEUI = 7248d956e61c5982 ( PROGMEM DEVEUI[8] reversed since it is little endian)
//Copy 2B7E151628AED2A6ABF7158809CF4F3C (APPKEY[16]) into chipstack device OTAA keys

*/

#ifdef LORAMODULE_HELTEC
// Pin mapping HELTEC ESP32
const lmic_pinmap lmic_pins = {
    .nss = LoRa_nss, // CS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LoRa_rst, //
    .dio = {LoRa_dio0, LoRa_dio1, LoRa_dio2},
    .spi_freq = F_CPU / 15};
#else
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

bool getLoRaStatus()
{
    return LoRa_status;
}

bool getPendingACK()
{
    return drlw_1.pendingack_drlora;
}

int getMessageQueueLength()
{
    return drlw_1.messageQueueLength();
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

void incrementCurrentRetryCount_0()
{
    drlw_1.incrementCurrentRetryCount();
}


void loraloop_2()
{
    if(getLoRaStatus())
    {
        // Run LORA loop
        os_runloop_once();

        // send LORA data every 10 seconds
        if (millis() - getLastLoraMessageTime() > 10000)
        {
            sendloramessage();
        }
        loraloop();
    }
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
void sendloramessage()
{
    // https://gist.github.com/kamito/704813
    lastloramessage_ms = millis();
    String jsonstring = String("{\"rand\":") + String(random(0, 100)) + ", \"temp\":" + String(random(0, 100)) + ", \"hum\":" + String(random(0, 100)) + ", \"log\": \"Hello\", \"confirm\": \"true\"}";
    Message messageStruct;
    messageStruct.message = jsonstring;

    sendData(messageStruct);
}

unsigned long getLastLoraMessageTime()
{
    return lastloramessage_ms;
}

