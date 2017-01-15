#ifndef LEDS_H_
#define LEDS_H_

#include "stm32l1xx.h"

enum State {
	ON = 1,
	OFF = 0
};

void Leds_Init(void);
void setLed(char index, uint8_t green, uint8_t red);

#endif /* LEDS_H_ */
