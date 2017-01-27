/*
 * GY87.cpp
 *
 *  Created on: 26. 1. 2017
 *      Author: Juraj
 */

#include <GY87/GY87.h>

GY87::GY87(unsigned char I2CAddressMPU6050, unsigned char I2CAddressHMC5883L,
		unsigned char I2CAddressBMP180) {
	this->I2CAddressMPU6050 = I2CAddressMPU6050;
	this->I2CAddressHMC5883L = I2CAddressHMC5883L;
	this->I2CAddressBMP180 = I2CAddressBMP180;

	setMPU6050ScaleSetting(MPU6050_SCALE_2000DPS);
	setMPU6050RangeSetting(MPU6050_RANGE_2G);
	setMPU6050DLPFModeSetting(MPU6050_DLPF_3);
	setMPU6050I2CMasterModeEnabledSetting(false);
	setMPU6050I2CBypassEnabledSetting(true) ;
	setMPU6050SleepEnabledSetting(false);

	setHMC5883LMeasurementSetting(HMC5883L_NORMAL);
	setHMC5883LSampleSetting(HMC5883L_SAMPLES_8);
	setHMC5883LRateSetting(HMC5883L_DATARATE_30HZ);
	setHMC5883LRangeSetting(HMC5883L_RANGE_1_3GA);
	setHMC5883LReadModeSetting(HMC5883L_CONTINOUS);
	setHMC5883LHighI2CSpeedSetting(false);
}

bool GY87::HMC5883LTestConnection(void) {
	char identA = I2C2_ReadRegister8(this->I2CAddressHMC5883L, HMC5883L_REG_IDENT_A);
	char identB = I2C2_ReadRegister8(this->I2CAddressHMC5883L, HMC5883L_REG_IDENT_B);
	char identC = I2C2_ReadRegister8(this->I2CAddressHMC5883L, HMC5883L_REG_IDENT_C);
	return identA == 'H' && identB == '4' && identC == '3';
}
/*
hmc5883l_measurement_t GY87::getHMC5883LMeasurementSetting(void) {
	return (hmc5883l_measurement_t)(
			(I2C2_ReadRegister8(this->I2CAddressHMC5883L,  HMC5883L_REG_CONFIG_A) & 0b00000011));
}
*/
void GY87::setHMC5883LMeasurementSetting(hmc5883l_measurement_t measurement) {
	char oldRegister = I2C2_ReadRegister8(this->I2CAddressHMC5883L, HMC5883L_REG_CONFIG_A) & 0b00000011;
	I2C2_WriteRegisterValue(this->I2CAddressHMC5883L,HMC5883L_REG_CONFIG_A, oldRegister | (measurement));
}
/*
GY87::hmc5883l_dataRate_t getHMC5883LSampleSetting(void) {
	return (hmc5883l_dataRate_t)(
			(I2C2_ReadRegister8(this->I2CAddressHMC5883L, HMC5883L_REG_CONFIG_A) & 0b11100011)
					>> 2);
}
*/
void GY87::setHMC5883LSampleSetting(hmc5883l_samples_t sample) {
	char oldRegister = I2C2_ReadRegister8(this->I2CAddressHMC5883L, HMC5883L_REG_CONFIG_A) & 0b00011111;
	I2C2_WriteRegisterValue(this->I2CAddressHMC5883L,HMC5883L_REG_CONFIG_A, oldRegister | (sample << 5) | 0b10000000);
}
/*
GY87::hmc5883l_dataRate_t getHMC5883LRateSetting(void) {
	return (hmc5883l_dataRate_t)(
			(I2C2_ReadRegister8(this->I2CAddressHMC5883L, HMC5883L_REG_CONFIG_A) & 0b11100011)
					>> 2);
}*/

void GY87::setHMC5883LRateSetting(hmc5883l_dataRate_t datarate) {
	char oldRegister = 0;//readRegister8(HMC5883L_ADDRESS, HMC5883L_REG_CONFIG_A) & 0b11100011;
	I2C2_WriteRegisterValue(this->I2CAddressHMC5883L,HMC5883L_REG_CONFIG_A, oldRegister | (datarate << 2));
}

void GY87::setHMC5883LRangeSetting(hmc5883l_range_t range) {
	switch (range) {
		case HMC5883L_RANGE_0_88GA:
		this->mgPerDigit = 0.73f;
		break;
		case HMC5883L_RANGE_1_3GA:
		this->mgPerDigit = 0.92f;
		break;
		case HMC5883L_RANGE_1_9GA:
		this->mgPerDigit = 1.22f;
		break;
		case HMC5883L_RANGE_2_5GA:
		this->mgPerDigit = 1.52f;
		break;
		case HMC5883L_RANGE_4GA:
		this->mgPerDigit = 2.27f;
		break;
		case HMC5883L_RANGE_4_7GA:
		this->mgPerDigit = 2.56f;
		break;
		case HMC5883L_RANGE_5_6GA:
		this->mgPerDigit = 3.03f;
		break;
		case HMC5883L_RANGE_8_1GA:
		this->mgPerDigit = 4.35f;
		break;
	}
	I2C2_WriteRegisterValue(this->I2CAddressHMC5883L,HMC5883L_REG_CONFIG_B,range << 5);
}
/*
hmc5883l_mode_t GY87::getHMC5883LReadModeSetting(void) {
	return (hmc5883l_mode_t)((I2C2_ReadRegister8(this->I2CAddressHMC5883L, HMC5883L_REG_MODE) & 0b00011000)>> 3);
}
*/
void GY87::setHMC5883LReadModeSetting(hmc5883l_mode_t mode) {
	char oldRegister = 0;//readRegister8(HMC5883L_ADDRESS, HMC5883L_REG_MODE) & 0b10000000;
	I2C2_WriteRegisterValue(this->I2CAddressHMC5883L,HMC5883L_REG_MODE, oldRegister | mode);
}

bool GY87::getHMC5883LHighI2CSpeedSetting(bool status) {
	return I2C2_ReadRegisterBit(this->I2CAddressHMC5883L, HMC5883L_REG_CONFIG_B, 7);
}

void GY87::setHMC5883LHighI2CSpeedSetting(bool status) {
	char oldRegister = 0;//readRegister8(HMC5883L_ADDRESS, HMC5883L_REG_CONFIG_B) & 0b00000011;
	if (status)
	oldRegister |= 0x80;
	I2C2_WriteRegisterValue(this->I2CAddressHMC5883L,HMC5883L_REG_CONFIG_B, oldRegister);
}

/*
GY87::HMC5883L_struct getHMC5883LRaw(void) {
	HMC5883L_struct HMC5883L;

	HMC5883L.compassAxis.x = (double) readRegister16s(HMC5883L_ADDRESS,
			HMC5883L_REG_OUT_X_M);
	HMC5883L.compassAxis.y = (double) readRegister16s(HMC5883L_ADDRESS,
			HMC5883L_REG_OUT_Y_M);
	HMC5883L.compassAxis.z = (double) readRegister16s(HMC5883L_ADDRESS,
			HMC5883L_REG_OUT_Z_M);

	return HMC5883L;
}

GY87::HMC5883L_struct getHMC5883LNorm(void) {
	HMC5883L_struct HMC5883L = getHMC5883LRaw();

	if (CALLIBRATE_DATA_CALCULATE) {
		if (HMC5883L.compassAxis.x < minAxis.x)
			minAxis.x = HMC5883L.compassAxis.x;
		if (HMC5883L.compassAxis.x > maxAxis.x)
			maxAxis.x = HMC5883L.compassAxis.x;
		if (HMC5883L.compassAxis.y < maxAxis.y)
			maxAxis.y = HMC5883L.compassAxis.y;
		if (HMC5883L.compassAxis.y > maxAxis.y)
			maxAxis.y = HMC5883L.compassAxis.y;
		if (HMC5883L.compassAxis.z < maxAxis.z)
			maxAxis.z = HMC5883L.compassAxis.z;
		if (HMC5883L.compassAxis.z > maxAxis.z)
			maxAxis.z = HMC5883L.compassAxis.z;

		semWait(sem_id, CALLIBRATE);
		callibrate.HMC5883LOffsetAxis.x = (maxAxis.x + minAxis.x) / 2;
		callibrate.HMC5883LOffsetAxis.y = (maxAxis.y + minAxis.y) / 2;
		callibrate.HMC5883LOffsetAxis.z = (maxAxis.z + minAxis.z) / 2;
		semPost(sem_id, CALLIBRATE);
	}

	HMC5883L.compassAxis.x = (HMC5883L.compassAxis.x HMC5883L_OFFSET_X) * mgPerDigit;
	HMC5883L.compassAxis.y = (HMC5883L.compassAxis.y HMC5883L_OFFSET_Y) * mgPerDigit;
	HMC5883L.compassAxis.z = (HMC5883L.compassAxis.z HMC5883L_OFFSET_Z) * mgPerDigit;

	double declinationAngle = (HMC5883L_DEGREE + (HMC5883L_MINUTES / 60.0))
			/ (180 / M_PI); //posun magnetickeho pola podla zemepisnej sirky a dlzky
	HMC5883L.yaw = atan2(HMC5883L.compassAxis.y, HMC5883L.compassAxis.x)
			+ declinationAngle;

	if (HMC5883L.yaw < 0) {
		HMC5883L.yaw += 2 * M_PI;
	} else if (HMC5883L.yaw > 2 * M_PI) {
		HMC5883L.yaw = 2 * M_PI;
	}

	return HMC5883L;
}

GY87::bool MPU6050TestConnection(void) {
	return readRegister8(MPU6050_ADDRESS, MPU6050_REG_WHO_AM_I) && MPU6050_ADDRESS;
}

GY87::void callibrateMPU6050Gyroscope(int samples) {
	double sumX = 0;
	double sumY = 0;
	double sumZ = 0;

	double sigmaX = 0;
	double sigmaY = 0;
	double sigmaZ = 0;

	for (unsigned char i = 0; i < samples; ++i)
	{
		Axis_struct gyAxis = getMPU6050GyRaw();
		sumX += gyAxis.x;
		sumY += gyAxis.y;
		sumZ += gyAxis.z;

		sigmaX += gyAxis.x * gyAxis.x;
		sigmaY += gyAxis.y * gyAxis.y;
		sigmaZ += gyAxis.z * gyAxis.z;

		usleep(5);
	}

	semWait(sem_id, CALLIBRATE);
	callibrate.MPU6050GyOffsetAxis.x = sumX / samples;
	callibrate.MPU6050GyOffsetAxis.y = sumY / samples;
	callibrate.MPU6050GyOffsetAxis.z = sumZ / samples;

	callibrate.MPU6050GyThresholdAxis.x = sqrt((sigmaX / samples) (callibrate.MPU6050GyOffsetAxis.x * callibrate.MPU6050GyOffsetAxis.x));
	callibrate.MPU6050GyThresholdAxis.y = sqrt((sigmaY / samples) (callibrate.MPU6050GyOffsetAxis.y * callibrate.MPU6050GyOffsetAxis.y));
	callibrate.MPU6050GyThresholdAxis.z = sqrt((sigmaZ / samples) (callibrate.MPU6050GyOffsetAxis.z * callibrate.MPU6050GyOffsetAxis.z));
	semPost(sem_id, CALLIBRATE);
}

GY87::void callibrateMPU6050Accelerometer(int samples) {
	double sumX = 0;
	double sumY = 0;
	double sumZ = 0;

	for (unsigned char i = 0; i < samples; ++i)
	{
		Axis_struct accAxis = getMPU6050AccRaw();

		sumX += accAxis.x;
		sumY += accAxis.y;
		sumZ += accAxis.z;

		usleep(100);
	}

	semWait(sem_id, CALLIBRATE);
	callibrate.MPU6050AccOffsetAxis.x = sumX / samples;
	callibrate.MPU6050AccOffsetAxis.y = sumY / samples;
	callibrate.MPU6050AccOffsetAxis.z = sumZ / samples;
	semPost(sem_id, CALLIBRATE);
}
*/

void GY87::setMPU6050ScaleSetting(mpu6050_dps_t scale)
{
	switch (scale) {
		case MPU6050_SCALE_250DPS:
		dpsPerDigit = .007633f;
		break;
		case MPU6050_SCALE_500DPS:
		dpsPerDigit = .015267f;
		break;
		case MPU6050_SCALE_1000DPS:
		dpsPerDigit = .030487f;
		break;
		case MPU6050_SCALE_2000DPS:
		dpsPerDigit = .060975f;
		break;
	}

	char oldRegister = I2C2_ReadRegister8(this->I2CAddressMPU6050, MPU6050_REG_GYRO_CONFIG) & 0b11100111;
	oldRegister |= (scale << 3);
	I2C2_WriteRegisterValue(this->I2CAddressMPU6050,MPU6050_REG_GYRO_CONFIG, oldRegister);
}
/*
GY87::mpu6050_dps_t getMPU6050ScaleSetting(void) {
	return (mpu6050_dps_t)(
			I2C2_ReadRegister8(this->I2CAddressMPU6050, MPU6050_REG_GYRO_CONFIG)
					& 0b00011000);
}
*/
void GY87::setMPU6050RangeSetting(mpu6050_range_t range) {
	switch (range) {
		case MPU6050_RANGE_2G:
		rangePerDigit = .000061f;
		break;
		case MPU6050_RANGE_4G:
		rangePerDigit = .000122f;
		break;
		case MPU6050_RANGE_8G:
		rangePerDigit = .000244f;
		break;
		case MPU6050_RANGE_16G:
		rangePerDigit = .0004882f;
		break;
	}

	char oldRegister = I2C2_ReadRegister8(this->I2CAddressMPU6050, MPU6050_REG_ACCEL_CONFIG) & 0b11100111;
	oldRegister |= (range << 3);
	I2C2_WriteRegisterValue(this->I2CAddressMPU6050,MPU6050_REG_ACCEL_CONFIG, oldRegister);
}

/*
GY87::mpu6050_range_t getMPU6050RangeSetting(void) {
	return (mpu6050_range_t)(
			(I2C2_ReadRegister8(this->I2CAddressMPU6050, MPU6050_REG_ACCEL_CONFIG)
					& 0b00011000) >> 3);
}
*/

void GY87::setMPU6050DHPFModeSetting(mpu6050_dhpf_t dhpf)
{
	char oldRegister = 0;//readRegister8(MPU6050_ADDRESS, MPU6050_REG_ACCEL_CONFIG) & 0b11111000;
	oldRegister |= dhpf;
	I2C2_WriteRegisterValue(this->I2CAddressMPU6050,MPU6050_REG_ACCEL_CONFIG, oldRegister);
}

void GY87::setMPU6050DLPFModeSetting(mpu6050_dlpf_t dlpf)
{
	char oldRegister = 0;//readRegister8(MPU6050_ADDRESS, MPU6050_REG_CONFIG) & 0b11111000;
	oldRegister |= dlpf;
	I2C2_WriteRegisterValue(this->I2CAddressMPU6050,MPU6050_REG_CONFIG, oldRegister);
}

void GY87::setMPU6050ClockSourceSetting(mpu6050_clockSource_t source)
{
	char oldRegister = 0;//readRegister8(MPU6050_ADDRESS, MPU6050_REG_PWR_MGMT_1) & 0b11111000;
	oldRegister |= source;
	I2C2_WriteRegisterValue(this->I2CAddressMPU6050,MPU6050_REG_PWR_MGMT_1, oldRegister);
}

/*
mpu6050_clockSource_t GY87::getMPU6050ClockSourceSetting(void) {
	return (mpu6050_clockSource_t)(I2C2_ReadRegister8(this->I2CAddressMPU6050, MPU6050_REG_PWR_MGMT_1) & 0b00000111);
}
*/
bool GY87::getMPU6050SleepEnabledSetting(void)
{
	return I2C2_ReadRegisterBit(this->I2CAddressMPU6050, MPU6050_REG_PWR_MGMT_1, 6);
}


void GY87::setMPU6050SleepEnabledSetting(bool state)
{
	char oldRegister = I2C2_ReadRegister8(this->I2CAddressMPU6050, MPU6050_REG_PWR_MGMT_1) & !(1 << 6);
	if (state)
	oldRegister |= (1 << 6);
	I2C2_WriteRegisterValue(this->I2CAddressMPU6050,MPU6050_REG_PWR_MGMT_1, oldRegister);
}

bool GY87::getMPU6050I2CMasterModeEnabledSetting(void)
{
	return I2C2_ReadRegisterBit(this->I2CAddressMPU6050, MPU6050_REG_USER_CTRL, 5);
}

void GY87::setMPU6050I2CMasterModeEnabledSetting(bool state)
{
	char oldRegister = I2C2_ReadRegister8(this->I2CAddressMPU6050, MPU6050_REG_USER_CTRL) & !(1 << 5);
	if (state)
	oldRegister |= (1 << 5);
	I2C2_WriteRegisterValue(this->I2CAddressMPU6050,MPU6050_REG_USER_CTRL, oldRegister);
}

void GY87::setMPU6050I2CBypassEnabledSetting(bool state)
{
	char oldRegister = I2C2_ReadRegister8(this->I2CAddressMPU6050, MPU6050_REG_INT_PIN_CFG) & !(1 << 1);
	if (state)
	oldRegister |= (1 << 1);
	I2C2_WriteRegisterValue(this->I2CAddressMPU6050,MPU6050_REG_INT_PIN_CFG, oldRegister);
}

bool GY87::getMPU6050I2CBypassEnabledSetting(void)
{
	return I2C2_ReadRegisterBit(this->I2CAddressMPU6050, MPU6050_REG_INT_PIN_CFG, 1);
}

/*
GY87::Axis_struct getMPU6050GyRaw(void) {
	Axis_struct gy;

	gy.x = (double) readRegister16s(MPU6050_ADDRESS, MPU6050_REG_GYRO_XOUT_H);
	gy.y = (double) readRegister16s(MPU6050_ADDRESS, MPU6050_REG_GYRO_YOUT_H);
	gy.z = (double) readRegister16s(MPU6050_ADDRESS, MPU6050_REG_GYRO_ZOUT_H);

	return gy;
}

GY87::Axis_struct getMPU6050AccRaw(void) {
	Axis_struct acc;

	acc.x = (double) readRegister16s(MPU6050_ADDRESS, MPU6050_REG_ACCEL_XOUT_H);
	acc.y = (double) readRegister16s(MPU6050_ADDRESS, MPU6050_REG_ACCEL_YOUT_H);
	acc.z = (double) readRegister16s(MPU6050_ADDRESS, MPU6050_REG_ACCEL_ZOUT_H);

	return acc;
}
GY87::double getMPU6050TempRaw(void) {
	return (double)readRegister16s(MPU6050_ADDRESS, MPU6050_REG_TEMP_OUT_H);
}

GY87::double getMPU6050TempNorm(void) {
	return getMPU6050TempRaw() / 340 + 36.53;
}

GY87::Axis_struct getMPU6050AccNorm(void) {
	Axis_struct acc = getMPU6050AccRaw();

	acc.x = (acc.x callibrate.MPU6050AccOffsetAxis.x) * rangePerDigit * 9.80665f;
	acc.y = (acc.y callibrate.MPU6050AccOffsetAxis.y) * rangePerDigit * 9.80665f;
	acc.z = (acc.z callibrate.MPU6050AccOffsetAxis.z) * rangePerDigit * 9.80665f;

	return acc;
}

GY87::Axis_struct getMPU6050GyNorm(void) {
	Axis_struct gy = getMPU6050GyRaw();

	gy.x = (gy.x - callibrate.MPU6050GyOffsetAxis.x) * dpsPerDigit;
	gy.y = (gy.y - callibrate.MPU6050GyOffsetAxis.y) * dpsPerDigit;
	gy.z = (gy.z - callibrate.MPU6050GyOffsetAxis.z) * dpsPerDigit;

	if (abs(gy.x) < callibrate.MPU6050GyThresholdAxis.x)
		gy.x = 0;
	if (abs(gy.y) < callibrate.MPU6050GyThresholdAxis.y)
		gy.y = 0;
	if (abs(gy.z) < callibrate.MPU6050GyThresholdAxis.z)
		gy.z = 0;

	return gy;
}

*/


// acc = angle measured with atan2 using the accelerometer
// gy = angle measured using the gyro
//http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/comment-page-1/
//http://robottini.altervista.org/kalman-filter-vs-complementary-filter
double GY87::kalmanCalculate(int indexKalman, double acc, double gy, double dt)
{
	kalman[indexKalman].x_angle += dt * (gy - kalman[indexKalman].x_bias);
	kalman[indexKalman].P_00 += dt * (kalman[indexKalman].P_10 + kalman[indexKalman].P_01) + Q_angle * dt;
	kalman[indexKalman].P_01 += dt * kalman[indexKalman].P_11;
	kalman[indexKalman].P_10 += dt * kalman[indexKalman].P_11;
	kalman[indexKalman].P_11 += + Q_gyro * dt;

	kalman[indexKalman].y = acc - kalman[indexKalman].x_angle;
	kalman[indexKalman].S = kalman[indexKalman].P_00 + R_angle;
	kalman[indexKalman].K_0 = kalman[indexKalman].P_00 / kalman[indexKalman].S;
	kalman[indexKalman].K_1 = kalman[indexKalman].P_10 / kalman[indexKalman].S;

	kalman[indexKalman].x_angle += kalman[indexKalman].K_0 * kalman[indexKalman].y;
	kalman[indexKalman].x_bias += kalman[indexKalman].K_1 * kalman[indexKalman].y;
	kalman[indexKalman].P_00= kalman[indexKalman].K_0 * kalman[indexKalman].P_00;
	kalman[indexKalman].P_01= kalman[indexKalman].K_0 * kalman[indexKalman].P_01;
	kalman[indexKalman].P_10= kalman[indexKalman].K_1 * kalman[indexKalman].P_00;
	kalman[indexKalman].P_11= kalman[indexKalman].K_1 * kalman[indexKalman].P_01;

	return kalman[indexKalman].x_angle;
}

// Tilt compensation
double GY87::tiltCompensate(double magX,double magY,double magZ, double roll, double pitch)
{
	double xh = magX * cos(roll) + magY * sin(roll) * sin(pitch) + magZ * sin(roll) * cos(pitch);
	double yh = magY * cos(pitch) + magZ * sin(pitch);
	return atan2(-yh, xh);
}

GY87::~GY87() {

}

