
#ifndef DR_PINOUT_h
#define DR_PINOUT_h

#include "Arduino.h"
#include <stdint.h>
#include "stc3100.h"
//#include "lsm6.h"
#include "imu.h"

#define MAX_QUEUE_SIZE 15

#ifdef __cplusplus
extern "C"
{
#endif


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

#define LORA_EN_STATUS true

#define EN_LORA 4
#define EN_GPS 13
#define BUZZER 25

#define IMU_INT1 18
#define IMU_INT2 23
#define IMU_CS 19

#define I2CADDRESS_GASGAUGE_STC STC3100_ADDRESS
#define I2CADDRESS_IMU DSO_SA0_LOW_ADDRESS
#define DEVICEIDSIZE 25

#ifdef __cplusplus
}
#endif

#endif