#ifndef DR_LORAWAN_h
#define DR_LORAWAN_h

#include "Arduino.h"
#include <stdint.h>

#define MAX_QUEUE_SIZE 15

#ifdef __cplusplus
extern "C"
{
#endif




typedef struct {
    int messageId=0;
    String message;
    bool confirmed;
    unsigned long timestamp;
    bool ackReceived;
    int retryCount=0;
} Message;

class DRLORAWAN
{
private:
public:
    int messagenumber = 0;
    int messageid_global = 0;
    bool pendingack_drlora=false;
    void enqueueMessage(Message message);
    //void enqueueMessage_s(String message);
    Message dequeueMessage();
    Message getCurrentMessage();
    //void retryCurrentMessage();
    void clearCurrentMessage();
    int messageQueueLength();
    void incrementCurrentRetryCount();
    int getCurrentMessageId();
    int getTail();
};

#ifdef __cplusplus
}
#endif

#endif