/*
 * IMU.h
 *
 *  Created on: 15. 3. 2017
 *      Author: Juraj
 */

#ifndef IMU_IMU_H_
#define IMU_IMU_H_

#include "stm32l1xx.h"
#include <I2CMasterLib/I2CMasterLib.h>
#define imuAddr (0x11<<1)

float getRoll(void);
float getPitch(void);
float getYaw(void);
void calibrateCompassStart(void);
void calibrateCompassStop(void);
void calibrateGyroscope(void);

#endif /* IMU_IMU_H_ */
