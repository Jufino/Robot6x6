#include <math.h>
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

double getAverageOfSimilaryValues(double val1, double val2, double val3) {
	double diff1 = fabs(val1 - val2);
	double diff2 = fabs(val2 - val3);
	double diff3 = fabs(val1 - val3);
	if (diff1 < diff2 && diff1 < diff3) {
		return (val1 + val2) / 2;
	} else if (diff2 < diff1 && diff2 < diff3) {
		return (val2 + val3) / 2;
	} else
		return (val1 + val3) / 2;
}

void DMA1_Channel5_IRQHandler(void) {
	if (DMA_GetFlagStatus(DMA1_FLAG_TC5)) {
		DMA_ClearFlag(DMA1_FLAG_TC5);
		I2C_DMACmd(I2C2, DISABLE);
		I2C_GenerateSTOP(I2C2, ENABLE);
		DMA_Cmd(DMA1_Channel5, DISABLE);

		if (I2C2_getReadRegister() == GETDELTATICKSREG) {
			for (int i = 1; i <= numberOfMotors; i++) {
				if (MOTORSADDR[i - 1] == I2C2_getDeviceAddress()) {
					getMotor(i)->addDeltaTicks((int16_t) I2C2_getRxBuffer(0));
					I2C2_clearReadRegister();
					I2C2_clearDeviceAddress();
					break;
				}
			}
		} else if (I2C2_getReadRegister() == GETVOLTAGEREG) {
			for (int i = 1; i <= numberOfMotors; i++) {
				if (MOTORSADDR[i - 1] == I2C2_getDeviceAddress()) {
					getMotor(i)->setVoltageRaw((uint16_t) I2C2_getRxBuffer(0));
					I2C2_clearReadRegister();
					I2C2_clearDeviceAddress();
					break;
				}
			}
		} else if (I2C2_getReadRegister() == GETCURRENTREG) {
			for (int i = 1; i <= numberOfMotors; i++) {
				if (MOTORSADDR[i - 1] == I2C2_getDeviceAddress()) {
					getMotor(i)->setCurrent((uint16_t) I2C2_getRxBuffer(0));
					I2C2_clearReadRegister();
					I2C2_clearDeviceAddress();
					break;
				}
			}
		} else if (I2C2_getReadRegister() == GETSPEEDREG) {
			for (int i = 1; i <= numberOfMotors; i++) {
				if (MOTORSADDR[i - 1] == I2C2_getDeviceAddress()) {
					getMotor(i)->setSpeedRaw((int16_t) I2C2_getRxBuffer(0));
					I2C2_clearReadRegister();
					I2C2_clearDeviceAddress();
					break;
				}
			}
		} else {
			I2C2_clearReadRegister();
			I2C2_clearDeviceAddress();
		}

	}
}
}

int main(void) {
I2C2_Init();
//getMotor(5)->setSpeedRawMotor(100);
//getMotor(5)->setSpeedRawMotor(100);
//getMotor(6)->setSpeedRawMotor(100);
//getMotor(4)->setSpeedRawMotor(0);
//getMotor(5)->setSpeedRawMotor(0);
//getMotor(6)->setSpeedRawMotor(0);
while (1) {
	getMotor(1)->DMADeltaTicksInvoke();
	getMotor(2)->DMADeltaTicksInvoke();
	getMotor(3)->DMADeltaTicksInvoke();
	getMotor(4)->DMADeltaTicksInvoke();
	getMotor(5)->DMADeltaTicksInvoke();
	getMotor(6)->DMADeltaTicksInvoke();
	double distance1 = getMotor(1)->getDistance();
	double distance2 = getMotor(2)->getDistance();
	double distance3 = getMotor(3)->getDistance();
	double distance4 = getMotor(4)->getDistance();
	double distance5 = getMotor(5)->getDistance();
	double distance6 = getMotor(6)->getDistance();
	double distanceL = getAverageOfSimilaryValues(distance1,distance2,distance3);
	double distanceR = getAverageOfSimilaryValues(distance4,distance5,distance6);

	getMotor(1)->DMASpeedInvoke();
	getMotor(2)->DMASpeedInvoke();
	getMotor(3)->DMASpeedInvoke();
	getMotor(4)->DMASpeedInvoke();
	getMotor(5)->DMASpeedInvoke();
	getMotor(6)->DMASpeedInvoke();
	double speed1 = getMotor(1)->getSpeed();
	double speed2 = getMotor(2)->getSpeed();
	double speed3 = getMotor(3)->getSpeed();
	double speed4 = getMotor(4)->getSpeed();
	double speed5 = getMotor(5)->getSpeed();
	double speed6 = getMotor(6)->getSpeed();
	double speedL = getAverageOfSimilaryValues(speed1,speed2,speed3);
	double speedR = getAverageOfSimilaryValues(speed4,speed5,speed6);
	//for (long i = 0; i < 100000; i++);

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
