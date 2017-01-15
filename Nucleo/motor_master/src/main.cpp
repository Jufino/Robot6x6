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
	default:
		return NULL;
	}
}

extern "C" {
#include <Leds/Leds.h>
#include <Ultrasonic/Ultrasonic.h>
#include <Servo/Servo.h>
#include <Buttons/Buttons.h>
#include <I2CMasterLib/I2CMasterLib.h>
#include <I2CSlaveLib/I2CSlaveLib.h>
#include "stm32l1xx.h"
}

#define dt 0.025

enum Direction {
	FORWARD, BACKWARD, ROTATE_CLOCKWISE, ROTATE_ANTICLOCKWISE, STOP
};

#define I2C1_MAX_SEND_BUFFER 4
uint16_t I2C1_RegValue;
bool I2C1_RecvRegDone = false;
int16_t I2C1_Val;
uint16_t I2C1_NumberOfRecvValues = 0;
uint8_t I2C_IndexSendBuffer = 0;
uint8_t I2C_SendBuffer[I2C1_MAX_SEND_BUFFER];

uint16_t timePoc = 0;

volatile bool timePosition = false;
volatile double x = 0;
volatile double y = 0;
volatile double z = 0;
volatile double yaw = 0;
volatile double roll = 0;
volatile double pitch = 0;

void InitializeTimer(void) {
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

void longToI2CBuffer(long number) {
	for (int i = 0; i < 4; i++) {
		I2C_SendBuffer[i] = (number >> ((3 - i) * 8)) & 0xFF;
	}
}

void intToI2CBuffer(int16_t number) {
	for (int i = 0; i < 2; i++) {
		I2C_SendBuffer[i] = (number >> ((1 - i) * 8)) & 0xFF;
	}
}

void uintToI2CBuffer(uint16_t number) {
	for (int i = 0; i < 2; i++) {
		I2C_SendBuffer[i] = (number >> ((1 - i) * 8)) & 0xFF;
	}
}

void speedL(double mmPerSec) {
	getMotor(1)->setSpeedMotor(mmPerSec);
	getMotor(2)->setSpeedMotor(mmPerSec);
	getMotor(3)->setSpeedMotor(mmPerSec);
}

void speedR(double mmPerSec) {
	getMotor(4)->setSpeedMotor(mmPerSec);
	getMotor(5)->setSpeedMotor(mmPerSec);
	getMotor(6)->setSpeedMotor(mmPerSec);
}

void goDirection(Direction dir, double mmPerSec) {
	switch (dir) {
	case FORWARD:
		speedL(mmPerSec);
		speedR(mmPerSec);
		break;
	case BACKWARD:
		speedL(-mmPerSec);
		speedR(-mmPerSec);
		break;
	case ROTATE_CLOCKWISE:
		speedL(mmPerSec);
		speedR(-mmPerSec);
		break;
	case ROTATE_ANTICLOCKWISE:
		speedL(-mmPerSec);
		speedR(mmPerSec);
		break;
	case STOP:
		speedL(0);
		speedR(0);
		break;
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

extern "C" void EXTI1_IRQHandler(void) {
	Ultrasonic_Pin_Interrupt();
}

extern "C" void TIM4_IRQHandler(void) {
	Ultrasonic_Timer_Update_Interrupt();
}

extern "C" void I2C1_ER_IRQHandler(void) {
	if (I2C_GetITStatus(I2C1, I2C_IT_AF)) {
		I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
	}
}

extern "C" void I2C1_EV_IRQHandler(void) {
	//ev1
	while ((I2C_SR1_ADDR & I2C1->SR1) == I2C_SR1_ADDR) {
		I2C1->SR1;
		I2C1->SR2;
	}
	//ev2
	while ((I2C_SR1_RXNE & I2C1->SR1) == I2C_SR1_RXNE) {
		if (!I2C1_RecvRegDone) {
			I2C1_RegValue = I2C1->DR;
			I2C1_RecvRegDone = true;
			I2C1_Val = 0;
			I2C1_NumberOfRecvValues = 0;
			I2C_IndexSendBuffer = 0;
			switch (I2C1_RegValue) {
			case 100:
				longToI2CBuffer(x);
				break;
			case 101:
				longToI2CBuffer(y);
				break;
			case 102:
				longToI2CBuffer(z);
				break;
			case 103:
				longToI2CBuffer(roll * 10000);
				break;
			case 104:
				longToI2CBuffer(pitch * 10000);
				break;
			case 105:
				longToI2CBuffer(yaw * 10000);
				break;
			case 106:
				I2C_SendBuffer[0] = 0;
				I2C_SendBuffer[1] = 0;
				I2C_SendBuffer[2] = 0;
				I2C_SendBuffer[3] = 0;
				if (buttons[0])
					I2C_SendBuffer[0] |= 1;
				if (buttons[1])
					I2C_SendBuffer[0] |= 2;
				if (buttons[2])
					I2C_SendBuffer[0] |= 4;
				break;
			case 107:
				uintToI2CBuffer(getUltRaw(0));
				break;
			}
		} else {
			I2C1_Val = (I2C1_Val << 8) | I2C1->DR;
			I2C1_NumberOfRecvValues++;
			if (I2C1_NumberOfRecvValues == 2) {
				switch (I2C1_RegValue) {
				case 1:
					getMotor(1)->setSpeedMotor((double) I2C1_Val);
					break;
				case 2:
					getMotor(2)->setSpeedMotor((double) I2C1_Val);
					break;
				case 3:
					getMotor(3)->setSpeedMotor((double) I2C1_Val);
					break;
				case 4:
					getMotor(4)->setSpeedMotor((double) I2C1_Val);
					break;
				case 5:
					getMotor(5)->setSpeedMotor((double) I2C1_Val);
					break;
				case 6:
					getMotor(6)->setSpeedMotor((double) I2C1_Val);
					break;
				case 7:
					goDirection(FORWARD, (double) I2C1_Val);
					break;
				case 8:
					goDirection(BACKWARD, (double) I2C1_Val);
					break;
				case 9:
					goDirection(ROTATE_CLOCKWISE, (double) I2C1_Val);
					break;
				case 10:
					goDirection(ROTATE_ANTICLOCKWISE, (double) I2C1_Val);
					break;
				case 11:
					goDirection(STOP, (double) I2C1_Val);
					break;
				case 12:
					setLed(0, (I2C1_Val & 0x01) == 0x01 ? ON : OFF,
							(I2C1_Val & 0x02) == 0x02 ? ON : OFF);
					setLed(1, (I2C1_Val & 0x04) == 0x04 ? ON : OFF,
							(I2C1_Val & 0x08) == 0x08 ? ON : OFF);
					setLed(2, (I2C1_Val & 0x10) == 0x10 ? ON : OFF,
							(I2C1_Val & 0x20) == 0x20 ? ON : OFF);
					break;
				case 13:
					setServo((double) I2C1_Val);
				}
			}
		}
	}
//ev3
	while ((I2C_SR1_TXE & I2C1->SR1) == I2C_SR1_TXE) {
		if (I2C_IndexSendBuffer < I2C1_MAX_SEND_BUFFER) {
			I2C1->DR = I2C_SendBuffer[I2C_IndexSendBuffer++];
		} else {
			I2C1->SR1 |= I2C_SR1_AF;
		}
	}
//ev4
	while ((I2C1->SR1 & I2C_SR1_STOPF) == I2C_SR1_STOPF) {
		I2C1_RecvRegDone = false;
		I2C1->SR1;
		I2C1->CR1 |= 0x1;
	}
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
		} else if (I2C2_getReadRegister() == GETWHO_I_AM) {
			for (int i = 1; i <= numberOfMotors; i++) {
				if (MOTORSADDR[i - 1] == I2C2_getDeviceAddress()) {
					getMotor(i)->setTestValue((int8_t) I2C2_getRxBuffer(0));
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

extern "C" void TIM2_IRQHandler(void) {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		timePosition = true;
	}
}

extern "C" void EXTI9_5_IRQHandler(void) {
	Buttons_Interrupt();
}

double lastDistance[6];
int main(void) {
	InitializePWMServo();
	Leds_Init();
	Buttons_Init();
	I2C2_Init();
	I2C1_Init();
	InitializeTimer();
	Ultrasonic_init();
	while (1) {
		ultTriger(0);
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
			yaw += omegaT;
			if (yaw > 2 * M_PI)
				yaw -= 2 * M_PI;
			else if (yaw < 0)
				yaw += 2 * M_PI;
			x += vT * cos(yaw);
			y += vT * sin(yaw);
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
