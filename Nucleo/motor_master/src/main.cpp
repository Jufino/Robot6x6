#include <Motor/Motor.h>

#define numberOfMotors 6
Motor *motor1 = new Motor(MOTOR1);
Motor *motor2 = new Motor(MOTOR2);
Motor *motor3 = new Motor(MOTOR3);
Motor *motor4 = new Motor(MOTOR4);
Motor *motor5 = new Motor(MOTOR5);
Motor *motor6 = new Motor(MOTOR6);
Motor *getMotor(int i) {
	switch (i) {
	case 1:
		return motor1;
	case 2:
		return motor2;
	case 3:
		return motor3;
	case 4:
		return motor4;
	case 5:
		return motor5;
	case 6:
		return motor6;
	}
}

extern "C" {
#include <I2Clib/i2clib.h>
#include "stm32l1xx.h"

void DMA1_Channel5_IRQHandler(void) {
	if (DMA_GetFlagStatus(DMA1_FLAG_TC5)) {
		DMA_ClearFlag(DMA1_FLAG_TC5);
		I2C_DMACmd(I2C2, DISABLE);
		I2C_GenerateSTOP(I2C2, ENABLE);
		DMA_Cmd(DMA1_Channel5, DISABLE);

		if (I2C2_getReadRegister() == GETDELTATICKSREG) {
			for (int i = 1; i <= numberOfMotors; i++) {
				if (MOTORSADDR[i-1] == I2C2_getDeviceAddress()) {
					getMotor(i)->addDeltaTicks((int16_t) I2C2_getRxBuffer(0));
					I2C2_clearReadRegister();
					I2C2_clearDeviceAddress();
					break;
				}
			}
		} else if (I2C2_getReadRegister() == GETVOLTAGEREG) {
			for (int i = 1; i <= numberOfMotors; i++) {
				if (MOTORSADDR[i-1] == I2C2_getDeviceAddress()) {
					getMotor(i)->setVoltageRaw((uint16_t) I2C2_getRxBuffer(0));
					I2C2_clearReadRegister();
					I2C2_clearDeviceAddress();
					break;
				}
			}
		} else if (I2C2_getReadRegister() == GETCURRENTREG) {
			for (int i = 1; i <= numberOfMotors; i++) {
				if (MOTORSADDR[i-1] == I2C2_getDeviceAddress()) {
					getMotor(i)->setCurrent((uint16_t) I2C2_getRxBuffer(0));
					I2C2_clearReadRegister();
					I2C2_clearDeviceAddress();
					break;
				}
			}
		} else if (I2C2_getReadRegister() == GETSPEEDREG) {
			for (int i = 1; i <= numberOfMotors; i++) {
				if (MOTORSADDR[i-1] == I2C2_getDeviceAddress()) {
					getMotor(i)->setSpeedRaw((int16_t) I2C2_getRxBuffer(0));
					I2C2_clearReadRegister();
					I2C2_clearDeviceAddress();
					break;
				}
			}
		}
		else{
			I2C2_clearReadRegister();
			I2C2_clearDeviceAddress();
		}

	}
}
}

int main(void) {
	I2C2_Init();
	getMotor(1)->setPWMMotor(200);
	while (1) {
		getMotor(1)->DMASpeedInvoke();
		double raw = getMotor(1)->getSpeedRaw();
		double speed = getMotor(1)->getSpeed();
		for (long i = 0; i < 100000; i++);

	}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
	while (1)
	{
	}
}
#endif
