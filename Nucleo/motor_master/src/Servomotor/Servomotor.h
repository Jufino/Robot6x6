#ifndef SERVO_H_
#define SERVO_H_

#include "stm32l1xx.h"

#define SERVO_MIN 1500//1400
#define SERVO_MAX 5000//5300

void InitializePWMServo(void);

void setServoRaw(uint32_t value);

void setServo(int16_t angle);

#endif /* SERVO_H_ */
