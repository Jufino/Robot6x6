#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include "stm32l1xx.h"

#define NUMBER_OF_ULTS 1

void Ultrasonic_init(void);
void ultTriger(int index);
uint16_t getUltRaw(int index);
double getUltCm(int index);
void Ultrasonic_Pin_Interrupt();
void Ultrasonic_Timer_Update_Interrupt(void);

#endif /* ULTRASONIC_H_ */
