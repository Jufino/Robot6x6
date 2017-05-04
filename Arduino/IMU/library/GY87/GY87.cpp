/*
MPU6050.cpp - Class file for the MPU6050 Triple Axis Gyroscope & Accelerometer Arduino Library.

Version: 1.0.3
(c) 2014-2015 Korneliusz Jarzebski
www.jarzebski.pl

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


#define SOFTWARE_I2C 1 // ci chceme softwerove i2c alebo hardverove
#if SOFTWARE_I2C >= 1
  #define SCL_PIN 2 
  #define SCL_PORT PORTC 
  #define SDA_PIN 1
  #define SDA_PORT PORTC 
  #define I2C_FASTMODE 1  // Limit to 25kHz
  #define I2C_TIMEOUT 10  // 10ms timeout
  #include <SoftI2CMaster.h>
  #define ADDR(x) (x << 1)
#else
  #include <Wire.h>
#endif

#include <math.h>
#include <GY87.h>

bool GY87::begin(mpu6050_dps_t scale, mpu6050_range_t range, int mpua, int hmcla)
{
    // Set Address
    mpuAddress = mpua;
    hmclAddress = hmcla;
    
    #if SOFTWARE_I2C >= 1
    i2c_init();
    #else
    Wire.begin();
    #endif

    // Reset calibrate values
    dg.XAxis = 0;
    dg.YAxis = 0;
    dg.ZAxis = 0;
    useCalibrate = false;

    // Reset threshold values
    tg.XAxis = 0;
    tg.YAxis = 0;
    tg.ZAxis = 0;
    actualThreshold = 0;

    // Check MPU6050 Who Am I Register
    if (fastRegister8(mpuAddress,MPU6050_REG_WHO_AM_I) != 0x68)
    {
	return false;
    }

  setI2CMasterModeEnabled(false);
  setI2CBypassEnabled(true) ;
  setSleepEnabled(false);

        if ((fastRegister8(hmclAddress,HMC5883L_REG_IDENT_A) != 0x48)
    || (fastRegister8(hmclAddress,HMC5883L_REG_IDENT_B) != 0x34)
    || (fastRegister8(hmclAddress,HMC5883L_REG_IDENT_C) != 0x33))
    {
	return false;
    }

    setRangeCompass(HMC5883L_RANGE_1_3GA);
    setMeasurementMode(HMC5883L_CONTINOUS);
    setDataRate(HMC5883L_DATARATE_15HZ);
    setSamples(HMC5883L_SAMPLES_1);

    mgPerDigit = 0.92f;

    // Set Clock Source
    setClockSource(MPU6050_CLOCK_PLL_XGYRO);

    // Set Scale & Range
    setScale(scale);
    setRangeAccel(range);

    // Disable Sleep Mode
    setSleepEnabled(false);

    return true;
}

void GY87::setScale(mpu6050_dps_t scale)
{
    uint8_t value;

    switch (scale)
    {
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
	default:
	    break;
    }

    value = readRegister8(mpuAddress,MPU6050_REG_GYRO_CONFIG);
    value &= 0b11100111;
    value |= (scale << 3);
    writeRegister8(mpuAddress,MPU6050_REG_GYRO_CONFIG, value);
}

mpu6050_dps_t GY87::getScale(void)
{
    uint8_t value;
    value = readRegister8(mpuAddress,MPU6050_REG_GYRO_CONFIG);
    value &= 0b00011000;
    value >>= 3;
    return (mpu6050_dps_t)value;
}

void GY87::setRangeAccel(mpu6050_range_t range)
{
    uint8_t value;

    switch (range)
    {
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
	default:
	    break;
    }

    value = readRegister8(mpuAddress,MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11100111;
    value |= (range << 3);
    writeRegister8(mpuAddress,MPU6050_REG_ACCEL_CONFIG, value);
}

mpu6050_range_t GY87::getRangeAccel(void)
{
    uint8_t value;
    value = readRegister8(mpuAddress,MPU6050_REG_ACCEL_CONFIG);
    value &= 0b00011000;
    value >>= 3;
    return (mpu6050_range_t)value;
}

void GY87::setDHPFMode(mpu6050_dhpf_t dhpf)
{
    uint8_t value;
    value = readRegister8(mpuAddress,MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11111000;
    value |= dhpf;
    writeRegister8(mpuAddress,MPU6050_REG_ACCEL_CONFIG, value);
}

void GY87::setDLPFMode(mpu6050_dlpf_t dlpf)
{
    uint8_t value;
    value = readRegister8(mpuAddress,MPU6050_REG_CONFIG);
    value &= 0b11111000;
    value |= dlpf;
    writeRegister8(mpuAddress,MPU6050_REG_CONFIG, value);
}

void GY87::setClockSource(mpu6050_clockSource_t source)
{
    uint8_t value;
    value = readRegister8(mpuAddress,MPU6050_REG_PWR_MGMT_1);
    value &= 0b11111000;
    value |= source;
    writeRegister8(mpuAddress,MPU6050_REG_PWR_MGMT_1, value);
}

mpu6050_clockSource_t GY87::getClockSource(void)
{
    uint8_t value;
    value = readRegister8(mpuAddress,MPU6050_REG_PWR_MGMT_1);
    value &= 0b00000111;
    return (mpu6050_clockSource_t)value;
}

bool GY87::getSleepEnabled(void)
{
    return readRegisterBit(mpuAddress,MPU6050_REG_PWR_MGMT_1, 6);
}

void GY87::setSleepEnabled(bool state)
{
    writeRegisterBit(mpuAddress,MPU6050_REG_PWR_MGMT_1, 6, state);
}

bool GY87::getIntZeroMotionEnabled(void)
{
    return readRegisterBit(mpuAddress,MPU6050_REG_INT_ENABLE, 5);
}

void GY87::setIntZeroMotionEnabled(bool state)
{
    writeRegisterBit(mpuAddress,MPU6050_REG_INT_ENABLE, 5, state);
}

bool GY87::getIntMotionEnabled(void)
{
    return readRegisterBit(mpuAddress,MPU6050_REG_INT_ENABLE, 6);
}

void GY87::setIntMotionEnabled(bool state)
{
    writeRegisterBit(mpuAddress,MPU6050_REG_INT_ENABLE, 6, state);
}

bool GY87::getIntFreeFallEnabled(void)
{
    return readRegisterBit(mpuAddress,MPU6050_REG_INT_ENABLE, 7);
}

void GY87::setIntFreeFallEnabled(bool state)
{
    writeRegisterBit(mpuAddress,MPU6050_REG_INT_ENABLE, 7, state);
}

uint8_t GY87::getMotionDetectionThreshold(void)
{
    return readRegister8(mpuAddress,MPU6050_REG_MOT_THRESHOLD);
}

void GY87::setMotionDetectionThreshold(uint8_t threshold)
{
    writeRegister8(mpuAddress,MPU6050_REG_MOT_THRESHOLD, threshold);
}

uint8_t GY87::getMotionDetectionDuration(void)
{
    return readRegister8(mpuAddress,MPU6050_REG_MOT_DURATION);
}

void GY87::setMotionDetectionDuration(uint8_t duration)
{
    writeRegister8(mpuAddress,MPU6050_REG_MOT_DURATION, duration);
}

uint8_t GY87::getZeroMotionDetectionThreshold(void)
{
    return readRegister8(mpuAddress,MPU6050_REG_ZMOT_THRESHOLD);
}

void GY87::setZeroMotionDetectionThreshold(uint8_t threshold)
{
    writeRegister8(mpuAddress,MPU6050_REG_ZMOT_THRESHOLD, threshold);
}

uint8_t GY87::getZeroMotionDetectionDuration(void)
{
    return readRegister8(mpuAddress,MPU6050_REG_ZMOT_DURATION);
}

void GY87::setZeroMotionDetectionDuration(uint8_t duration)
{
    writeRegister8(mpuAddress,MPU6050_REG_ZMOT_DURATION, duration);
}

uint8_t GY87::getFreeFallDetectionThreshold(void)
{
    return readRegister8(mpuAddress,MPU6050_REG_FF_THRESHOLD);
}

void GY87::setFreeFallDetectionThreshold(uint8_t threshold)
{
    writeRegister8(mpuAddress,MPU6050_REG_FF_THRESHOLD, threshold);
}

uint8_t GY87::getFreeFallDetectionDuration(void)
{
    return readRegister8(mpuAddress,MPU6050_REG_FF_DURATION);
}

void GY87::setFreeFallDetectionDuration(uint8_t duration)
{
    writeRegister8(mpuAddress,MPU6050_REG_FF_DURATION, duration);
}

bool GY87::getI2CMasterModeEnabled(void)
{
    return readRegisterBit(mpuAddress,MPU6050_REG_USER_CTRL, 5);
}

void GY87::setI2CMasterModeEnabled(bool state)
{
    writeRegisterBit(mpuAddress,MPU6050_REG_USER_CTRL, 5, state);
}

void GY87::setI2CBypassEnabled(bool state)
{
    return writeRegisterBit(mpuAddress,MPU6050_REG_INT_PIN_CFG, 1, state);
}

bool GY87::getI2CBypassEnabled(void)
{
    return readRegisterBit(mpuAddress,MPU6050_REG_INT_PIN_CFG, 1);
}

void GY87::setAccelPowerOnDelay(mpu6050_onDelay_t delay)
{
    uint8_t value;
    value = readRegister8(mpuAddress,MPU6050_REG_MOT_DETECT_CTRL);
    value &= 0b11001111;
    value |= (delay << 4);
    writeRegister8(mpuAddress,MPU6050_REG_MOT_DETECT_CTRL, value);
}

mpu6050_onDelay_t GY87::getAccelPowerOnDelay(void)
{
    uint8_t value;
    value = readRegister8(mpuAddress,MPU6050_REG_MOT_DETECT_CTRL);
    value &= 0b00110000;
    return (mpu6050_onDelay_t)(value >> 4);
}

uint8_t GY87::getIntStatus(void)
{
    return readRegister8(mpuAddress,MPU6050_REG_INT_STATUS);
}

Activites GY87::readActivites(void)
{
    uint8_t data = readRegister8(mpuAddress,MPU6050_REG_INT_STATUS);

    a.isOverflow = ((data >> 4) & 1);
    a.isFreeFall = ((data >> 7) & 1);
    a.isInactivity = ((data >> 5) & 1);
    a.isActivity = ((data >> 6) & 1);
    a.isDataReady = ((data >> 0) & 1);

    data = readRegister8(mpuAddress,MPU6050_REG_MOT_DETECT_STATUS);

    a.isNegActivityOnX = ((data >> 7) & 1);
    a.isPosActivityOnX = ((data >> 6) & 1);

    a.isNegActivityOnY = ((data >> 5) & 1);
    a.isPosActivityOnY = ((data >> 4) & 1);

    a.isNegActivityOnZ = ((data >> 3) & 1);
    a.isPosActivityOnZ = ((data >> 2) & 1);

    return a;
}

Vector GY87::readRawAccel(void)
{

    #if SOFTWARE_I2C >= 1
      i2c_start(ADDR(mpuAddress) | I2C_WRITE);
      i2c_write(MPU6050_REG_ACCEL_XOUT_H);
      i2c_rep_start(ADDR(mpuAddress) | I2C_READ);
      uint8_t xha = i2c_read(false);
      uint8_t xla = i2c_read(false);
      uint8_t yha = i2c_read(false);
      uint8_t yla = i2c_read(false);
      uint8_t zha = i2c_read(false);
      uint8_t zla = i2c_read(true);
      i2c_stop();
    #else
      Wire.beginTransmission(mpuAddress);
      #if ARDUINO >= 100
  	   Wire.write(MPU6050_REG_ACCEL_XOUT_H);
      #else
  	   Wire.send(MPU6050_REG_ACCEL_XOUT_H);
      #endif
      Wire.endTransmission();
  
      Wire.beginTransmission(mpuAddress);
      Wire.requestFrom(mpuAddress, 6);
  
      while (Wire.available() < 6);
  
      #if ARDUINO >= 100
      	uint8_t xha = Wire.read();
      	uint8_t xla = Wire.read();
        uint8_t yha = Wire.read();
      	uint8_t yla = Wire.read();
      	uint8_t zha = Wire.read();
      	uint8_t zla = Wire.read();
      #else
      	uint8_t xha = Wire.receive();
      	uint8_t xla = Wire.receive();
      	uint8_t yha = Wire.receive();
      	uint8_t yla = Wire.receive();
      	uint8_t zha = Wire.receive();
      	uint8_t zla = Wire.receive();
      #endif
    #endif
    
    ra.XAxis = xha << 8 | xla;
    ra.YAxis = yha << 8 | yla;
    ra.ZAxis = zha << 8 | zla;

    return ra;
}

Vector GY87::readNormalizeAccel(void)
{
    readRawAccel();

    na.XAxis = ra.XAxis * rangePerDigit * 9.80665f;
    na.YAxis = ra.YAxis * rangePerDigit * 9.80665f;
    na.ZAxis = ra.ZAxis * rangePerDigit * 9.80665f;

    return na;
}

Vector GY87::readScaledAccel(void)
{
    readRawAccel();

    na.XAxis = ra.XAxis * rangePerDigit;
    na.YAxis = ra.YAxis * rangePerDigit;
    na.ZAxis = ra.ZAxis * rangePerDigit;

    return na;
}


Vector GY87::readRawGyro(void)
{

    #if SOFTWARE_I2C >= 1
      i2c_start(ADDR(mpuAddress) | I2C_WRITE);
      i2c_write(MPU6050_REG_GYRO_XOUT_H);
      i2c_rep_start(ADDR(mpuAddress) | I2C_READ);
      uint8_t xha = i2c_read(false);
      uint8_t xla = i2c_read(false);
      uint8_t yha = i2c_read(false);
      uint8_t yla = i2c_read(false);
      uint8_t zha = i2c_read(false);
      uint8_t zla = i2c_read(true);
      i2c_stop();
    #else
      Wire.beginTransmission(mpuAddress);
      #if ARDUINO >= 100
  	   Wire.write(MPU6050_REG_GYRO_XOUT_H);
      #else
  	   Wire.send(MPU6050_REG_GYRO_XOUT_H);
      #endif
      Wire.endTransmission();
  
      Wire.beginTransmission(mpuAddress);
      Wire.requestFrom(mpuAddress, 6);
  
      while (Wire.available() < 6);
  
      #if ARDUINO >= 100
      	uint8_t xha = Wire.read();
      	uint8_t xla = Wire.read();
        uint8_t yha = Wire.read();
      	uint8_t yla = Wire.read();
      	uint8_t zha = Wire.read();
      	uint8_t zla = Wire.read();
      #else
      	uint8_t xha = Wire.receive();
      	uint8_t xla = Wire.receive();
      	uint8_t yha = Wire.receive();
      	uint8_t yla = Wire.receive();
      	uint8_t zha = Wire.receive();
      	uint8_t zla = Wire.receive();
      #endif
    #endif

    rg.XAxis = xha << 8 | xla;
    rg.YAxis = yha << 8 | yla;
    rg.ZAxis = zha << 8 | zla;

    return rg;
}

Vector GY87::readNormalizeGyro(void)
{
    readRawGyro();

    if (useCalibrate)
    {
	ng.XAxis = (rg.XAxis - dg.XAxis) * dpsPerDigit;
	ng.YAxis = (rg.YAxis - dg.YAxis) * dpsPerDigit;
	ng.ZAxis = (rg.ZAxis - dg.ZAxis) * dpsPerDigit;
    } else
    {
	ng.XAxis = rg.XAxis * dpsPerDigit;
	ng.YAxis = rg.YAxis * dpsPerDigit;
	ng.ZAxis = rg.ZAxis * dpsPerDigit;
    }

    if (actualThreshold)
    {
	if (abs(ng.XAxis) < tg.XAxis) ng.XAxis = 0;
	if (abs(ng.YAxis) < tg.YAxis) ng.YAxis = 0;
	if (abs(ng.ZAxis) < tg.ZAxis) ng.ZAxis = 0;
    }

    return ng;
}

float GY87::readTemperature(void)
{
    int16_t T;
    T = readRegister16(mpuAddress,MPU6050_REG_TEMP_OUT_H);
    return (float)T/340 + 36.53;
}

int16_t GY87::getGyroOffsetX(void)
{
    return readRegister16(mpuAddress,MPU6050_REG_GYRO_XOFFS_H);
}

int16_t GY87::getGyroOffsetY(void)
{
    return readRegister16(mpuAddress,MPU6050_REG_GYRO_YOFFS_H);
}

int16_t GY87::getGyroOffsetZ(void)
{
    return readRegister16(mpuAddress,MPU6050_REG_GYRO_ZOFFS_H);
}

void GY87::setGyroOffsetX(int16_t offset)
{
    writeRegister16(mpuAddress,MPU6050_REG_GYRO_XOFFS_H, offset);
}

void GY87::setGyroOffsetY(int16_t offset)
{
    writeRegister16(mpuAddress,MPU6050_REG_GYRO_YOFFS_H, offset);
}

void GY87::setGyroOffsetZ(int16_t offset)
{
    writeRegister16(mpuAddress,MPU6050_REG_GYRO_ZOFFS_H, offset);
}

int16_t GY87::getAccelOffsetX(void)
{
    return readRegister16(mpuAddress,MPU6050_REG_ACCEL_XOFFS_H);
}

int16_t GY87::getAccelOffsetY(void)
{
    return readRegister16(mpuAddress,MPU6050_REG_ACCEL_YOFFS_H);
}

int16_t GY87::getAccelOffsetZ(void)
{
    return readRegister16(mpuAddress,MPU6050_REG_ACCEL_ZOFFS_H);
}

void GY87::setAccelOffsetX(int16_t offset)
{
    writeRegister16(mpuAddress,MPU6050_REG_ACCEL_XOFFS_H, offset);
}

void GY87::setAccelOffsetY(int16_t offset)
{
    writeRegister16(mpuAddress,MPU6050_REG_ACCEL_YOFFS_H, offset);
}

void GY87::setAccelOffsetZ(int16_t offset)
{
    writeRegister16(mpuAddress,MPU6050_REG_ACCEL_ZOFFS_H, offset);
}

// Calibrate algorithm
void GY87::calibrateGyro(uint8_t samples)
{
    // Set calibrate
    useCalibrate = true;

    // Reset values
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;
    float sigmaX = 0;
    float sigmaY = 0;
    float sigmaZ = 0;

    // Read n-samples
    for (uint8_t i = 0; i < samples; ++i)
    {
	readRawGyro();
	sumX += rg.XAxis;
	sumY += rg.YAxis;
	sumZ += rg.ZAxis;

	sigmaX += rg.XAxis * rg.XAxis;
	sigmaY += rg.YAxis * rg.YAxis;
	sigmaZ += rg.ZAxis * rg.ZAxis;

	delay(5);
    }

    // Calculate delta vectors
    dg.XAxis = sumX / samples;
    dg.YAxis = sumY / samples;
    dg.ZAxis = sumZ / samples;

    // Calculate threshold vectors
    th.XAxis = sqrt((sigmaX / 50) - (dg.XAxis * dg.XAxis));
    th.YAxis = sqrt((sigmaY / 50) - (dg.YAxis * dg.YAxis));
    th.ZAxis = sqrt((sigmaZ / 50) - (dg.ZAxis * dg.ZAxis));

    // If already set threshold, recalculate threshold vectors
    if (actualThreshold > 0)
    {
	setThreshold(actualThreshold);
    }
}

// Get current threshold value
uint8_t GY87::getThreshold(void)
{
    return actualThreshold;
}

// Set treshold value
void GY87::setThreshold(uint8_t multiple)
{
    if (multiple > 0)
    {
	// If not calibrated, need calibrate
	if (!useCalibrate)
	{
	    calibrateGyro();
	}

	// Calculate threshold vectors
	tg.XAxis = th.XAxis * multiple;
	tg.YAxis = th.YAxis * multiple;
	tg.ZAxis = th.ZAxis * multiple;
    } else
    {
	// No threshold
	tg.XAxis = 0;
	tg.YAxis = 0;
	tg.ZAxis = 0;
    }

    // Remember old threshold value
    actualThreshold = multiple;
}


Vector GY87::readRawCompass(void)
{
    v.XAxis = readRegister16(hmclAddress,HMC5883L_REG_OUT_X_M) - xOffset;
    v.YAxis = readRegister16(hmclAddress,HMC5883L_REG_OUT_Y_M) - yOffset;
    v.ZAxis = readRegister16(hmclAddress,HMC5883L_REG_OUT_Z_M);

    return v;
}

Vector GY87::readNormalizeCompass(void)
{
    v.XAxis = ((float)readRegister16(hmclAddress,HMC5883L_REG_OUT_X_M) - xOffset) * mgPerDigit;
    v.YAxis = ((float)readRegister16(hmclAddress,HMC5883L_REG_OUT_Y_M) - yOffset) * mgPerDigit;
    v.ZAxis = ((float)readRegister16(hmclAddress,HMC5883L_REG_OUT_Z_M) - zOffset) * mgPerDigit;

    return v;
}

void GY87::setOffset(int xo, int yo, int zo)
{
    xOffset = xo;
    yOffset = yo;           
    zOffset = zo;
    }

void GY87::setRangeCompass(hmc5883l_range_t range)
{
    switch(range)
    {
	case HMC5883L_RANGE_0_88GA:
	    mgPerDigit = 0.073f;
	    break;

	case HMC5883L_RANGE_1_3GA:
	    mgPerDigit = 0.92f;
	    break;

	case HMC5883L_RANGE_1_9GA:
	    mgPerDigit = 1.22f;
	    break;

	case HMC5883L_RANGE_2_5GA:
	    mgPerDigit = 1.52f;
	    break;

	case HMC5883L_RANGE_4GA:
	    mgPerDigit = 2.27f;
	    break;

	case HMC5883L_RANGE_4_7GA:
	    mgPerDigit = 2.56f;
	    break;

	case HMC5883L_RANGE_5_6GA:
	    mgPerDigit = 3.03f;
	    break;

	case HMC5883L_RANGE_8_1GA:
	    mgPerDigit = 4.35f;
	    break;

	default:
	    break;
    }

    writeRegister8(hmclAddress,HMC5883L_REG_CONFIG_B, range << 5);
}

hmc5883l_range_t GY87::getRangeCompass(void)
{
    return (hmc5883l_range_t)((readRegister8(hmclAddress,HMC5883L_REG_CONFIG_B) >> 5));
}

void GY87::setMeasurementMode(hmc5883l_mode_t mode)
{
    uint8_t value;

    value = readRegister8(hmclAddress,HMC5883L_REG_MODE);
    value &= 0b11111100;
    value |= mode;

    writeRegister8(hmclAddress,HMC5883L_REG_MODE, value);
}

hmc5883l_mode_t GY87::getMeasurementMode(void)
{
    uint8_t value;

    value = readRegister8(hmclAddress,HMC5883L_REG_MODE);
    value &= 0b00000011;

    return (hmc5883l_mode_t)value;
}

void GY87::setDataRate(hmc5883l_dataRate_t dataRate)
{
    uint8_t value;

    value = readRegister8(hmclAddress,HMC5883L_REG_CONFIG_A);
    value &= 0b11100011;
    value |= (dataRate << 2);

    writeRegister8(hmclAddress,HMC5883L_REG_CONFIG_A, value);
}

hmc5883l_dataRate_t GY87::getDataRate(void)
{
    uint8_t value;

    value = readRegister8(hmclAddress,HMC5883L_REG_CONFIG_A);
    value &= 0b00011100;
    value >>= 2;

    return (hmc5883l_dataRate_t)value;
}

void GY87::setSamples(hmc5883l_samples_t samples)
{
    uint8_t value;

    value = readRegister8(hmclAddress,HMC5883L_REG_CONFIG_A);
    value &= 0b10011111;
    value |= (samples << 5);

    writeRegister8(hmclAddress,HMC5883L_REG_CONFIG_A, value);
}

hmc5883l_samples_t GY87::getSamples(void)
{
    uint8_t value;

    value = readRegister8(hmclAddress,HMC5883L_REG_CONFIG_A);
    value &= 0b01100000;
    value >>= 5;

    return (hmc5883l_samples_t)value;
}

// Fast read 8-bit from register
uint8_t GY87::fastRegister8(uint8_t addr,uint8_t reg)
{
    uint8_t value;

    #if SOFTWARE_I2C >= 1
      i2c_start(ADDR(addr) | I2C_WRITE);
      i2c_write(reg);     
      i2c_rep_start(ADDR(addr) | I2C_READ);
      value = i2c_read(true);
      i2c_stop();
    #else
      Wire.beginTransmission(addr);
      #if ARDUINO >= 100
	    Wire.write(reg);
      #else
	     Wire.send(reg);
      #endif
      Wire.endTransmission();
      Wire.beginTransmission(addr);
      Wire.requestFrom(addr, 1);
      #if ARDUINO >= 100
	     value = Wire.read();
      #else
	     value = Wire.receive();
      #endif;
      Wire.endTransmission();
    #endif
    
    return value;
}

// Read 8-bit from register
uint8_t GY87::readRegister8(uint8_t addr,uint8_t reg)
{
    uint8_t value;

    #if SOFTWARE_I2C >= 1
      i2c_start(ADDR(addr) | I2C_WRITE);
      i2c_write(reg);
      i2c_rep_start(ADDR(addr) | I2C_READ);
      value = i2c_read(true);
      i2c_stop();
    #else
      Wire.beginTransmission(addr);
      #if ARDUINO >= 100
	     Wire.write(reg);
      #else
	     Wire.send(reg);
      #endif
      Wire.endTransmission();

      Wire.beginTransmission(addr);
      Wire.requestFrom(addr, 1);
      while(!Wire.available()) {};
      #if ARDUINO >= 100
	     value = Wire.read();
      #else
	     value = Wire.receive();
      #endif;
      Wire.endTransmission();
    #endif

    return value;
}

// Write 8-bit to register
void GY87::writeRegister8(uint8_t addr,uint8_t reg, uint8_t value)
{
    #if SOFTWARE_I2C >= 1
      i2c_start(ADDR(addr) | I2C_WRITE);
      i2c_write(reg);
      i2c_write(value);
      i2c_stop();
    #else
      Wire.beginTransmission(addr);
      #if ARDUINO >= 100
  	   Wire.write(reg);
  	   Wire.write(value);
      #else
  	   Wire.send(reg);
  	   Wire.send(value);
      #endif
      Wire.endTransmission();
    #endif
}

int16_t GY87::readRegister16(uint8_t addr,uint8_t reg)
{
    int16_t value;
    
    #if SOFTWARE_I2C >= 1
      i2c_start(ADDR(addr) | I2C_WRITE);
      i2c_write(reg);
      i2c_rep_start(ADDR(addr) | I2C_READ);
      uint8_t vha = i2c_read(false);
      uint8_t vla = i2c_read(true);
      i2c_stop();
    #else
      Wire.beginTransmission(addr);
      #if ARDUINO >= 100
          Wire.write(reg);
      #else
          Wire.send(reg);
      #endif
      Wire.endTransmission();
  
      Wire.beginTransmission(addr);
      Wire.requestFrom(addr, 2);
      while(!Wire.available()) {};
      #if ARDUINO >= 100
          uint8_t vha = Wire.read();
          uint8_t vla = Wire.read();
      #else
          uint8_t vha = Wire.receive();
          uint8_t vla = Wire.receive();
      #endif;
      Wire.endTransmission();
    #endif
    
    value = vha << 8 | vla;

    return value;
}

void GY87::writeRegister16(uint8_t addr,uint8_t reg, int16_t value)
{
    #if SOFTWARE_I2C >= 1
      i2c_start(ADDR(addr) | I2C_WRITE);
      i2c_write(reg);
      i2c_write((uint8_t)(value >> 8));
      i2c_write((uint8_t)value);
      i2c_stop();
    #else
      Wire.beginTransmission(addr);
      #if ARDUINO >= 100
      	Wire.write(reg);
      	Wire.write((uint8_t)(value >> 8));
      	Wire.write((uint8_t)value);
      #else
      	Wire.send(reg);
      	Wire.send((uint8_t)(value >> 8));
      	Wire.send((uint8_t)value);
      #endif
      Wire.endTransmission();
    #endif
}

// Read register bit
bool GY87::readRegisterBit(uint8_t addr,uint8_t reg, uint8_t pos)
{
    uint8_t value;
    value = readRegister8(addr,reg);
    return ((value >> pos) & 1);
}

// Write register bit
void GY87::writeRegisterBit(uint8_t addr,uint8_t reg, uint8_t pos, bool state)
{
    uint8_t value;
    value = readRegister8(addr,reg);

    if (state)
    {
        value |= (1 << pos);
    } else 
    {
        value &= ~(1 << pos);
    }

    writeRegister8(addr,reg, value);
}
