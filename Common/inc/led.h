#ifndef __LED_H__
#define __LED_H__

#include "IOConfig.h"

#define ON_LED0 LED0 = 0
#define ON_LED1 LED1 = 0
#define ON_LED2 LED2 = 0
#define ON_LED3 LED3 = 0

#define OFF_LED0 LED0 = 1
#define OFF_LED1 LED1 = 1
#define OFF_LED2 LED2 = 1
#define OFF_LED3 LED3 = 1


void LEDInit(void);

#endif