#ifndef BUTTONS_BUTTONS_H_
#define BUTTONS_BUTTONS_H_

#include "stm32l1xx.h"

volatile uint8_t buttons[3];

void Buttons_Init(void);
void Buttons_Interrupt(void);

#endif /* BUTTONS_BUTTONS_H_ */
