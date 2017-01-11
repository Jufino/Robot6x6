#ifndef MOTOR_H_
#define MOTOR_H_

extern "C"{
	#include "stm32l1xx.h"
	#include <I2CMasterLib/I2CMasterLib.h>
#include <Math.h>
}

//------------------------------------------------------------------
//motors address
#define MOTOR1 2
#define MOTOR2 4
#define MOTOR3 6
#define MOTOR4 8
#define MOTOR5 10
#define MOTOR6 12
const uint8_t MOTORSADDR[] = {MOTOR1,MOTOR2,MOTOR3,MOTOR4,MOTOR5,MOTOR6};
//------------------------------------------------------------------
//set register
#define SETSPEEDREG 100
#define SETCURRENTREG 99
#define SETPWMREG 101
//------------------------------------------------------------------
//get register
#define GETSPEEDREG 1
#define GETCURRENTREG 2
#define GETDELTATICKSREG 3
#define GETVOLTAGEREG 4
//------------------------------------------------------------------
#define wheelDiameter 86
#define lengthBetweenLeftAndRightWheel 250
#define numberTicksOfWheel 120 //overit
#define periodSpeedRegulator 0.25
//------------------------------------------------------------------
class Motor {
private:
	uint8_t motorAddr;
	long ticks;
	int16_t speedRaw;
	uint16_t voltageRaw;
	uint16_t current;
public:
	Motor(uint8_t motorAddr);
	//------------------------------------------------------------------
	virtual ~Motor();

	void setSpeedMotor(double mPerSec);
	//------------------------------------------------------------------
	void setAngleSpeedMotor(double anglePerSec);
	//------------------------------------------------------------------
	void setSpeedRawMotor(int16_t data);
	//------------------------------------------------------------------
	void setCurrentMotor(int16_t data);
	//------------------------------------------------------------------
	void setPWMMotor(int16_t data);
	//------------------------------------------------------------------
	void DMADeltaTicksInvoke(void);
	//------------------------------------------------------------------
	void DMACurrentInvoke(void);
	//------------------------------------------------------------------
	void DMAVoltageInvoke(void);
	//------------------------------------------------------------------
	void DMASpeedInvoke(void);
	//------------------------------------------------------------------
	void addDeltaTicks(int16_t data);
	//------------------------------------------------------------------
	double getDistance(void);
	//------------------------------------------------------------------
	long getTicks(void);
	//------------------------------------------------------------------
	void setVoltageRaw(uint16_t voltageRaw);
	uint16_t getVoltageRaw(void);
	//------------------------------------------------------------------
	double getVoltage(void);
	//------------------------------------------------------------------
	void setCurrent(uint16_t current);
	uint16_t getCurrent(void);
	//------------------------------------------------------------------
	void setSpeedRaw(int16_t speedRaw);
	int16_t getSpeedRaw(void);
	//------------------------------------------------------------------
	double getAngleSpeed(void);
	//------------------------------------------------------------------
	double getSpeed(void);

};

#endif /* MOTOR_H_ */
