#include <IMU/IMU.h>

float getRoll(void){
	return ((float)((int16_t)I2C2_ReadRegister16(imuAddr, 1)))/10000;
}
float getPitch(void){
	return ((float)((int16_t)I2C2_ReadRegister16(imuAddr, 2)))/10000;
}
float getYaw(void){
	return ((float)((int16_t)I2C2_ReadRegister16(imuAddr, 3)))/10000;
}
void calibrateCompassStart(void){
	I2C2_WriteRegister(imuAddr,99);
}
void calibrateCompassStop(void){
	I2C2_WriteRegister(imuAddr,100);
}
void calibrateGyroscope(void){
	I2C2_WriteRegister(imuAddr,101);
}

