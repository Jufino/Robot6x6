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
#include <I2CMasterLib/I2CMasterLib.h>
#include <I2CSlaveLib/I2CSlaveLib.h>
#include "stm32l1xx.h"
}
volatile bool timePosition = false;
int timePoc = 0;
volatile double x = 0;
volatile double y = 0;
volatile double angle = 0;
#define dt 0.025

extern "C" void I2C1_ER_IRQHandler(void) {
	if (I2C_GetITStatus(I2C1, I2C_IT_AF)) {
		I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
	}
}

uint16_t i2c1_reg;
int16_t i2c1_val;
bool i2c1_recv_reg_done = false;
extern "C" void I2C1_EV_IRQHandler() {
	//ev1
	while ((I2C_SR1_ADDR & I2C1->SR1) == I2C_SR1_ADDR) {
		I2C1->SR1;
		I2C1->SR2;
	}

	//ev2
	while ((I2C_SR1_RXNE & I2C1->SR1) == I2C_SR1_RXNE) {
		if (!i2c1_recv_reg_done) {
			i2c1_reg = I2C1->DR;
			i2c1_recv_reg_done = true;
			i2c1_val = 0;
		} else {
			i2c1_val = (i2c1_val<<8) | I2C1->DR;
			if(i2c1_reg == 100){
				getMotor(1)->setSpeedMotor((double)i2c1_val);
			}
		}

	}

	//ev3
	while ((I2C_SR1_TXE & I2C1->SR1) == I2C_SR1_TXE) {
		if (i2c1_reg == 1) {
			I2C1->DR = 0xF0;
		} else {
			I2C1->DR = 0xFF;
		}
		//I2C1->SR1 |= I2C_SR1_AF;
	}

	//ev4
	while ((I2C1->SR1 & I2C_SR1_STOPF) == I2C_SR1_STOPF) {
		i2c1_recv_reg_done = false;
		I2C1->SR1;
		I2C1->CR1 |= 0x1;
	}

}

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

extern "C" void DMA1_Channel5_IRQHandler(void) {
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

extern "C" void TIM2_IRQHandler() {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		timePosition = true;
	}
}

void InitializeTimer() {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef timerInitStructure; //nastavit 40 hz
	timerInitStructure.TIM_Prescaler = 999;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 49;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2, &timerInitStructure);
	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
}

double lastDistance[6];
int main(void) {
	I2C2_Init();
	I2C1_Init();
	InitializeTimer();
	while (1) {
		if (timePosition) {
			double speeds[6];
			for (int i = 0; i < 6; i++) {
				speeds[i] = (getMotor(i)->getDistance() - lastDistance[i]);
				lastDistance[i] = getMotor(i)->getDistance();
			}

			double vL = getAverageOfSimilaryValues(speeds[1], speeds[2],
					speeds[3]);
			double vR = getAverageOfSimilaryValues(speeds[4], speeds[5],
					speeds[6]);
			double vT = (vL + vR) / 2;
			double omegaT = (vR - vL) / lengthBetweenLeftAndRightWheel;
			angle += omegaT;
			x += vT * cos(angle);
			y += vT * sin(angle);
			timePosition = false;
		} else {
			getMotor(1)->DMADeltaTicksInvoke();
			getMotor(2)->DMADeltaTicksInvoke();
			getMotor(3)->DMADeltaTicksInvoke();
			getMotor(4)->DMADeltaTicksInvoke();
			getMotor(5)->DMADeltaTicksInvoke();
			getMotor(6)->DMADeltaTicksInvoke();
			for (int i = 0; i < 1000; i++)
				;
		}
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
