
#include "Arduino.h"
#include <stdint.h>

#include "dr_lorawan.h"

int fifo_head = 0;
int fifo_tail = 0;

Message fifo_messages[MAX_QUEUE_SIZE];

int DRLORAWAN::getTail() {
    return fifo_tail;
}
void DRLORAWAN::enqueueMessage(Message message) {
    if ((fifo_tail + 1) % MAX_QUEUE_SIZE != fifo_head) {
        fifo_messages[fifo_tail] = message;
        //messageid_global++;
        //fifo_messages[fifo_tail].messageId = messageid;
        fifo_tail = (fifo_tail + 1) % MAX_QUEUE_SIZE;
    }
    else
    {
        Serial.println("Queue is full. Cannot enqueue message");
    }
}

//void DRLORAWAN::enqueueMessage_s(String message) {
//    Message fifo_messagestruct;
//    fifo_messagestruct.message = message;
//    enqueueMessage(fifo_messagestruct);
//}

Message DRLORAWAN::dequeueMessage() {
    Message message = fifo_messages[fifo_head];
    fifo_head = (fifo_head + 1) % MAX_QUEUE_SIZE;
    return message;
}

Message DRLORAWAN::getCurrentMessage() {
    return fifo_messages[fifo_head];
}

int DRLORAWAN::getCurrentMessageId() {
    return fifo_messages[fifo_head].messageId;
}

//void retryCurrentMessage() {
//    //if (millis() - lastMessageTime > TIMEOUT_MS) {
//        // Timeout reached, retry the current message
//        pendingack_drlora = true;
//        //do_send(&sendjob, messageQueue[fifo_head], true);
//        incrementCurrentRetryCount();
//        sendMsg(fifo_messages[fifo_head]);
//        //lastMessageTime = millis();
//    //}
//}

void DRLORAWAN::clearCurrentMessage() {
    // Remove the current message from the queue
    fifo_head = (fifo_head + 1) % MAX_QUEUE_SIZE;
    pendingack_drlora = false;
}

int DRLORAWAN::messageQueueLength() {
    return (fifo_tail - fifo_head + MAX_QUEUE_SIZE) % MAX_QUEUE_SIZE;
}

void DRLORAWAN::incrementCurrentRetryCount() {
    fifo_messages[fifo_head].retryCount++;
}
