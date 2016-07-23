
#ifndef __PWM_H__
#define __PWM_H__

#include "IAP15W4K61S4.h"

void PWMInit();
void PCAInit();
void PWM(unsigned int PWMa,unsigned int PWMb,unsigned int PWMc,unsigned int PWMd);

#endif