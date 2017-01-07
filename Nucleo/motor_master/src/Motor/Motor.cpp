#include <Motor/Motor.h>
//------------------------------------------------------------------
Motor::Motor(uint8_t motorAddr) {
	this->motorAddr = motorAddr;
	this->ticks = 0;
	this->speedRaw = 0;
	this->current = 0;
	this->voltageRaw = 0;
}
//------------------------------------------------------------------
Motor::~Motor() {

}
//------------------------------------------------------------------
void Motor::setSpeedMotor(double mmPerSec){
	int16_t speedRaw = (mmPerSec*(numberTicksOfWheel/(M_PI*wheelDiameter)))/periodSpeedRegulator;
	setSpeedRawMotor(speedRaw);
}
//------------------------------------------------------------------
void Motor::setAngleSpeedMotor(double anglePerSec){
	setSpeedRawMotor((int16_t)((anglePerSec*(numberTicksOfWheel/360))/periodSpeedRegulator));
}
//------------------------------------------------------------------
void Motor::setSpeedRawMotor(int16_t data) {
	uint8_t tmp[2];
	tmp[1] = data & 0xFF;
	tmp[0] = (data >> 8) && 0xFF;
	I2C2_BytesWrite(this->motorAddr, tmp, 2, SETSPEEDREG);
}
//------------------------------------------------------------------
void Motor::setCurrentMotor(int16_t data) {
	uint8_t tmp[2];
	tmp[1] = data & 0xFF;
	tmp[0] = (data >> 8) && 0xFF;
	I2C2_BytesWrite(this->motorAddr, tmp, 2, SETCURRENTREG);
}
//------------------------------------------------------------------
void Motor::setPWMMotor(int16_t data) {
	uint8_t tmp[2];
	tmp[1] = data & 0xFF;
	tmp[0] = (data >> 8) && 0xFF;
	I2C2_BytesWrite(this->motorAddr, tmp, 2, SETPWMREG);
}
//------------------------------------------------------------------
void Motor::DMADeltaTicksInvoke(void) {
	I2C2_DMA_Read(this->motorAddr, GETDELTATICKSREG, 2);
}
//------------------------------------------------------------------
void Motor::DMACurrentInvoke(void) {
	I2C2_DMA_Read(this->motorAddr, GETCURRENTREG, 2);
}
//------------------------------------------------------------------
void Motor::DMAVoltageInvoke(void) {
	I2C2_DMA_Read(this->motorAddr, GETVOLTAGEREG, 2);
}
//------------------------------------------------------------------
void Motor::DMASpeedInvoke(void) {
	I2C2_DMA_Read(this->motorAddr, GETSPEEDREG, 2);
}
//------------------------------------------------------------------
void Motor::addDeltaTicks(int16_t data) {
	this->ticks += data;
}
//------------------------------------------------------------------
long Motor::getTicks(void) {
	while (I2C2_getReadRegister() == GETDELTATICKSREG)
		;
	return this->ticks;
}
//------------------------------------------------------------------
void Motor::setSpeedRaw(int16_t speedRaw) {
	this->speedRaw=speedRaw;
}
//------------------------------------------------------------------
int16_t Motor::getSpeedRaw(void) {
	while (I2C2_getReadRegister() == GETSPEEDREG)
		;
	return this->speedRaw;
}
//------------------------------------------------------------------
double Motor::getAngleSpeed(void) {
	return (getSpeedRaw()*periodSpeedRegulator)*(360/numberTicksOfWheel);
}
//------------------------------------------------------------------
double Motor::getSpeed(void) {
	return (getSpeedRaw()*periodSpeedRegulator)*((wheelDiameter*M_PI)/numberTicksOfWheel);
}
//------------------------------------------------------------------
void Motor::setVoltageRaw(uint16_t voltageRaw) {
	this->voltageRaw=voltageRaw;
}
//------------------------------------------------------------------
uint16_t Motor::getVoltageRaw(void) {
	while (I2C2_getReadRegister() == GETVOLTAGEREG)
		;
	return this->voltageRaw;
}
//------------------------------------------------------------------
double Motor::getVoltage(void) {
	while (I2C2_getReadRegister() == GETVOLTAGEREG)
		;
	return (double)this->voltageRaw * (25 / 1023);
}
//------------------------------------------------------------------
void Motor::setCurrent(uint16_t current) {
	this->current=current;
}
//------------------------------------------------------------------
uint16_t Motor::getCurrent(void) {
	while (I2C2_getReadRegister() == GETCURRENTREG)
		;
	return this->current;
}
//------------------------------------------------------------------
