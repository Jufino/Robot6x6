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

void Motor::setSpeedRegulator(double P,double I, double D){
	uint8_t tmp[2];
	uint16_t toSend = P*10000;

	tmp[1] = toSend & 0xFF;
	tmp[0] = (toSend >> 8) & 0xFF;
	I2C2_BytesWrite(this->motorAddr, tmp, 2, SETPSPEEDREG);

	toSend = I*10000;
	tmp[1] = toSend & 0xFF;
	tmp[0] = (toSend >> 8) & 0xFF;
	I2C2_BytesWrite(this->motorAddr, tmp, 2, SETISPEEDREG);

	toSend = D*10000;
	tmp[1] = toSend & 0xFF;
	tmp[0] = (toSend >> 8) & 0xFF;
	I2C2_BytesWrite(this->motorAddr, tmp, 2, SETDSPEEDREG);
	I2C2_WriteRegister(this->motorAddr, CALCSPEEDREG);
}



void Motor::setCurrentRegulator(double P,double I, double D){
	uint8_t tmp[2];
	uint16_t toSend = P*10000;

	tmp[1] = toSend & 0xFF;
	tmp[0] = (toSend >> 8) & 0xFF;
	I2C2_BytesWrite(this->motorAddr, tmp, 2, SETPCURRENTREG);

	toSend = I*10000;
	tmp[1] = toSend & 0xFF;
	tmp[0] = (toSend >> 8) & 0xFF;
	I2C2_BytesWrite(this->motorAddr, tmp, 2, SETICURRENTREG);

	toSend = D*10000;
	tmp[1] = toSend & 0xFF;
	tmp[0] = (toSend >> 8) & 0xFF;
	I2C2_BytesWrite(this->motorAddr, tmp, 2, SETDCURRENTREG);
	I2C2_WriteRegister(this->motorAddr, CALCCURRENTREG);
}
//------------------------------------------------------------------
void Motor::setSpeedMotor(int16_t mmPerSec) {
	int16_t speedRaw =
			((double)mmPerSec * (numberTicksOfWheel*periodSpeedRegulator) / (M_PI * wheelDiameter));

	setSpeedRawMotor(speedRaw);
}
//------------------------------------------------------------------
void Motor::setAngleSpeedMotor(int16_t anglePerSec) {
	setSpeedRawMotor(
			(int16_t) (((double)anglePerSec * periodSpeedRegulator) * (numberTicksOfWheel / 360)));
}
//------------------------------------------------------------------
void Motor::setSpeedRawMotor(int16_t data) {
	uint8_t tmp[2];
	tmp[1] = data & 0xFF;
	tmp[0] = (data >> 8) & 0xFF;
	I2C2_BytesWrite(this->motorAddr, tmp, 2, SETSPEEDREG);
}
//------------------------------------------------------------------
void Motor::setCurrentMotor(int16_t data) {
	uint8_t tmp[2];
	tmp[1] = data & 0xFF;
	tmp[0] = (data >> 8) & 0xFF;
	I2C2_BytesWrite(this->motorAddr, tmp, 2, SETCURRENTREG);
}
//------------------------------------------------------------------
void Motor::setPWMMotor(int16_t data) {
	uint8_t tmp[2];
	tmp[1] = data & 0xFF;
	tmp[0] = (data >> 8) & 0xFF;
	I2C2_BytesWrite(this->motorAddr, tmp, 2, SETPWMREG);
}
//------------------------------------------------------------------
void Motor::DMADeltaTicksInvoke(void) {
	addDeltaTicks(I2C2_ReadRegister16(this->motorAddr, GETDELTATICKSREG));
}
//------------------------------------------------------------------
void Motor::DMACurrentInvoke(void) {
	setCurrent(I2C2_ReadRegister16(this->motorAddr, GETCURRENTREG));
}
//------------------------------------------------------------------
void Motor::DMAVoltageInvoke(void) {
	setVoltageRaw(I2C2_ReadRegister16(this->motorAddr, GETVOLTAGEREG));
}
//------------------------------------------------------------------
void Motor::DMASpeedInvoke(void) {
	setSpeedRaw(I2C2_ReadRegister16(this->motorAddr, GETSPEEDREG));
}
//------------------------------------------------------------------
void Motor::addDeltaTicks(int16_t data) {
	this->ticks += data;
}
//------------------------------------------------------------------
long Motor::getTicks(void) {
	return this->ticks;
}
//------------------------------------------------------------------
void Motor::setTicks(long ticks){
	this->ticks = ticks;
}
//------------------------------------------------------------------
double Motor::getDistance(void) {
	return (double) getTicks() * ((wheelDiameter * M_PI) / (numberTicksOfWheel));
}
//------------------------------------------------------------------
void Motor::setSpeedRaw(int16_t speedRaw) {
	this->speedRaw = speedRaw;
}
//------------------------------------------------------------------
int16_t Motor::getSpeedRaw(void) {
	return this->speedRaw;
}
//------------------------------------------------------------------
double Motor::getAngleSpeed(void) {
	return getSpeedRaw() * (360 / (periodSpeedRegulator * numberTicksOfWheel));
}
//------------------------------------------------------------------
double Motor::getSpeed(void) {
	return getSpeedRaw()
			* ((wheelDiameter * M_PI)
					/ (periodSpeedRegulator * numberTicksOfWheel));
}
//------------------------------------------------------------------
void Motor::setVoltageRaw(uint16_t voltageRaw) {
	this->voltageRaw = voltageRaw;
}
//------------------------------------------------------------------
uint16_t Motor::getVoltageRaw(void) {
	return this->voltageRaw;
}
//------------------------------------------------------------------
double Motor::getVoltage(void) {
	return (double) this->voltageRaw * (25 / 1023);
}
//------------------------------------------------------------------
void Motor::setCurrent(uint16_t current) {
	this->current = current;
}
//------------------------------------------------------------------
uint16_t Motor::getCurrent(void) {
	return this->current;
}
//------------------------------------------------------------------
bool Motor::isTestOk(void) {
	uint8_t addr = I2C2_ReadRegister8(this->motorAddr, GETWHO_I_AM);
	return addr == (this->motorAddr>>1);
}
//------------------------------------------------------------------

