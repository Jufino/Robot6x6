/*
 * GY87.h
 *
 *  Created on: 26. 1. 2017
 *      Author: Juraj
 */

#ifndef GY87_H_
#define GY87_H_

extern "C" {
#include "stm32l1xx.h"
#include <I2CMasterLib/I2CMasterLib.h>
#include <Math.h>
}

class GY87 {
#define Q_angle  0.01
#define Q_gyro   0.0003
#define R_angle  0.01

// zavisle na zemepisnej sirke: http://magnetic-declination.com/
#define HMC5883L_DEGREE 4.0f
#define HMC5883L_MINUTES 30.0f

//HMC5883L
//https://github.com/jarzebski/Arduino-HMC5883L/
#define HMC5883L_REG_CONFIG_A         (0x00)
#define HMC5883L_REG_CONFIG_B         (0x01)
#define HMC5883L_REG_MODE             (0x02)
#define HMC5883L_REG_OUT_X_M          (0x03)
#define HMC5883L_REG_OUT_X_L          (0x04)
#define HMC5883L_REG_OUT_Z_M          (0x05)
#define HMC5883L_REG_OUT_Z_L          (0x06)
#define HMC5883L_REG_OUT_Y_M          (0x07)
#define HMC5883L_REG_OUT_Y_L          (0x08)
#define HMC5883L_REG_STATUS           (0x09)
#define HMC5883L_REG_IDENT_A          (0x0A)
#define HMC5883L_REG_IDENT_B          (0x0B)
#define HMC5883L_REG_IDENT_C          (0x0C)

//MPU6050
//https://github.com/jarzebski/Arduino-MPU6050/
#define MPU6050_REG_ACCEL_XOFFS_H     (0x06)
#define MPU6050_REG_ACCEL_XOFFS_L     (0x07)
#define MPU6050_REG_ACCEL_YOFFS_H     (0x08)
#define MPU6050_REG_ACCEL_YOFFS_L     (0x09)
#define MPU6050_REG_ACCEL_ZOFFS_H     (0x0A)
#define MPU6050_REG_ACCEL_ZOFFS_L     (0x0B)
#define MPU6050_REG_GYRO_XOFFS_H      (0x13)
#define MPU6050_REG_GYRO_XOFFS_L      (0x14)
#define MPU6050_REG_GYRO_YOFFS_H      (0x15)
#define MPU6050_REG_GYRO_YOFFS_L      (0x16)
#define MPU6050_REG_GYRO_ZOFFS_H      (0x17)
#define MPU6050_REG_GYRO_ZOFFS_L      (0x18)
#define MPU6050_REG_CONFIG            (0x1A)
#define MPU6050_REG_GYRO_CONFIG       (0x1B) // Gyroscope Configuration
#define MPU6050_REG_ACCEL_CONFIG      (0x1C) // Accelerometer Configuration
#define MPU6050_REG_FF_THRESHOLD      (0x1D)
#define MPU6050_REG_FF_DURATION       (0x1E)
#define MPU6050_REG_MOT_THRESHOLD     (0x1F)
#define MPU6050_REG_MOT_DURATION      (0x20)
#define MPU6050_REG_ZMOT_THRESHOLD    (0x21)
#define MPU6050_REG_ZMOT_DURATION     (0x22)
#define MPU6050_REG_INT_PIN_CFG       (0x37) // INT Pin. Bypass Enable Configuration
#define MPU6050_REG_INT_ENABLE        (0x38) // INT Enable
#define MPU6050_REG_INT_STATUS        (0x3A)
#define MPU6050_REG_ACCEL_XOUT_H      (0x3B)
#define MPU6050_REG_ACCEL_XOUT_L      (0x3C)
#define MPU6050_REG_ACCEL_YOUT_H      (0x3D)
#define MPU6050_REG_ACCEL_YOUT_L      (0x3E)
#define MPU6050_REG_ACCEL_ZOUT_H      (0x3F)
#define MPU6050_REG_ACCEL_ZOUT_L      (0x40)
#define MPU6050_REG_TEMP_OUT_H        (0x41)
#define MPU6050_REG_TEMP_OUT_L        (0x42)
#define MPU6050_REG_GYRO_XOUT_H       (0x43)
#define MPU6050_REG_GYRO_XOUT_L       (0x44)
#define MPU6050_REG_GYRO_YOUT_H       (0x45)
#define MPU6050_REG_GYRO_YOUT_L       (0x46)
#define MPU6050_REG_GYRO_ZOUT_H       (0x47)
#define MPU6050_REG_GYRO_ZOUT_L       (0x48)
#define MPU6050_REG_MOT_DETECT_STATUS (0x61)
#define MPU6050_REG_MOT_DETECT_CTRL   (0x69)
#define MPU6050_REG_USER_CTRL         (0x6A) // User Control
#define MPU6050_REG_PWR_MGMT_1        (0x6B) // Power Management 1
#define MPU6050_REG_WHO_AM_I          (0x75) // Who Am I

//BMP 180
#define BMP180_REG_CONTROL 0xF4
#define BMP180_REG_RESULT 0xF6
#define BMP180_COMMAND_TEMPERATURE 0x2E
#define BMP180_COMMAND_PRESSURE0 0x34
#define BMP180_COMMAND_PRESSURE1 0x74
#define BMP180_COMMAND_PRESSURE2 0xB4
#define BMP180_COMMAND_PRESSURE3 0xF4

private:
	struct Kalman_struct {
		double x_angle;
		double x_bias;
		double P_00;
		double P_01;
		double P_10;
		double P_11;
		double y;
		double S;
		double K_0;
		double K_1;
	};
	Kalman_struct kalman[2];
	double mgPerDigit = 0.92f;
	double rangePerDigit = 0.0f;
	double dpsPerDigit = 0.0f;
	unsigned char I2CAddressMPU6050;
	unsigned char I2CAddressHMC5883L;
	unsigned char I2CAddressBMP180;

	typedef enum {
		MPU6050_CLOCK_KEEP_RESET = 0b111,
		MPU6050_CLOCK_EXTERNAL_19MHZ = 0b101,
		MPU6050_CLOCK_EXTERNAL_32KHZ = 0b100,
		MPU6050_CLOCK_PLL_ZGYRO = 0b011,
		MPU6050_CLOCK_PLL_YGYRO = 0b010,
		MPU6050_CLOCK_PLL_XGYRO = 0b001,
		MPU6050_CLOCK_INTERNAL_8MHZ = 0b000
	} mpu6050_clockSource_t;

	typedef enum {
		MPU6050_SCALE_2000DPS = 0b11,
		MPU6050_SCALE_1000DPS = 0b10,
		MPU6050_SCALE_500DPS = 0b01,
		MPU6050_SCALE_250DPS = 0b00
	} mpu6050_dps_t;
	typedef enum {
		MPU6050_RANGE_16G = 0b11,
		MPU6050_RANGE_8G = 0b10,
		MPU6050_RANGE_4G = 0b01,
		MPU6050_RANGE_2G = 0b00,
	} mpu6050_range_t;
	typedef enum {
		MPU6050_DELAY_3MS = 0b11,
		MPU6050_DELAY_2MS = 0b10,
		MPU6050_DELAY_1MS = 0b01,
		MPU6050_NO_DELAY = 0b00,
	} mpu6050_onDelay_t;
	typedef enum {
		MPU6050_DHPF_HOLD = 0b111,
		MPU6050_DHPF_0_63HZ = 0b100,
		MPU6050_DHPF_1_25HZ = 0b011,
		MPU6050_DHPF_2_5HZ = 0b010,
		MPU6050_DHPF_5HZ = 0b001,
		MPU6050_DHPF_RESET = 0b000,
	} mpu6050_dhpf_t;
	typedef enum {
		MPU6050_DLPF_6 = 0b110,
		MPU6050_DLPF_5 = 0b101,
		MPU6050_DLPF_4 = 0b100,
		MPU6050_DLPF_3 = 0b011,
		MPU6050_DLPF_2 = 0b010,
		MPU6050_DLPF_1 = 0b001,
		MPU6050_DLPF_0 = 0b000,
	} mpu6050_dlpf_t;
	//--------------------------------------------
	//HMC5883L
	typedef enum {
		HMC5883L_SAMPLES_8 = 0b11,
		HMC5883L_SAMPLES_4 = 0b10,
		HMC5883L_SAMPLES_2 = 0b01,
		HMC5883L_SAMPLES_1 = 0b00
	} hmc5883l_samples_t;
	typedef enum {
		HMC5883L_DATARATE_75HZ = 0b110,
		HMC5883L_DATARATE_30HZ = 0b101,
		HMC5883L_DATARATE_15HZ = 0b100,
		HMC5883L_DATARATE_7_5HZ = 0b011,
		HMC5883L_DATARATE_3HZ = 0b010,
		HMC5883L_DATARATE_1_5HZ = 0b001,
		HMC5883L_DATARATE_0_75_HZ = 0b000
	} hmc5883l_dataRate_t;
	typedef enum {
		HMC5883L_RANGE_8_1GA = 0b111,
		HMC5883L_RANGE_5_6GA = 0b110,
		HMC5883L_RANGE_4_7GA = 0b101,
		HMC5883L_RANGE_4GA = 0b100,
		HMC5883L_RANGE_2_5GA = 0b011,
		HMC5883L_RANGE_1_9GA = 0b010,
		HMC5883L_RANGE_1_3GA = 0b001,
		HMC5883L_RANGE_0_88GA = 0b000
	} hmc5883l_range_t;
	typedef enum {
		HMC5883L_IDLE = 0b10, HMC5883L_SINGLE = 0b01, HMC5883L_CONTINOUS = 0b00
	} hmc5883l_mode_t;
	typedef enum {
		HMC5883L_NORMAL = 0b00,
		HMC5883L_POSITIVE_BIAS = 0b01,
		HMC5883L_NEGATIVE_BIAS = 0b10
	} hmc5883l_measurement_t;


	double kalmanCalculate(int indexKalman, double acc, double gy, double dt);
public:

	GY87(unsigned char I2CAddressMPU6050, unsigned char I2CAddressHMC5883L,
			unsigned char I2CAddressBMP180);

	bool HMC5883LTestConnection(void);
	hmc5883l_measurement_t getHMC5883LMeasurementSetting(void);
	void setHMC5883LMeasurementSetting(hmc5883l_measurement_t measurement);
	hmc5883l_dataRate_t getHMC5883LSampleSetting(void);
	void setHMC5883LSampleSetting(hmc5883l_samples_t sample);
	hmc5883l_dataRate_t getHMC5883LRateSetting(void);
	void setHMC5883LRateSetting(hmc5883l_dataRate_t datarate);
	hmc5883l_range_t getHMC5883LRangeSetting(void);
	void setHMC5883LRangeSetting(hmc5883l_range_t range);
	hmc5883l_mode_t getHMC5883LReadModeSetting();
	void setHMC5883LReadModeSetting(hmc5883l_mode_t mode);
	bool getHMC5883LHighI2CSpeedSetting(bool status);
	void setHMC5883LHighI2CSpeedSetting(bool status);

	void callibrateMPU6050Gyroscope(int samples);
	void callibrateMPU6050Accelerometer(int samples);

	bool MPU6050TestConnection(void);
	void setMPU6050ScaleSetting(mpu6050_dps_t scale);
	mpu6050_dps_t getMPU6050ScaleSetting(void);
	void setMPU6050RangeSetting(mpu6050_range_t range);
	mpu6050_range_t getMPU6050RangeSetting(void);
	void setMPU6050DHPFModeSetting(mpu6050_dhpf_t dhpf);
	void setMPU6050DLPFModeSetting(mpu6050_dlpf_t dlpf);
	void setMPU6050ClockSourceSetting(mpu6050_clockSource_t source);
	mpu6050_clockSource_t getMPU6050ClockSourceSetting(void);
	bool getMPU6050SleepEnabledSetting(void);
	void setMPU6050SleepEnabledSetting(bool state);
	bool getMPU6050I2CMasterModeEnabledSetting(void);
	void setMPU6050I2CMasterModeEnabledSetting(bool state);
	void setMPU6050I2CBypassEnabledSetting(bool state);
	bool getMPU6050I2CBypassEnabledSetting(void);

	double getMPU6050TempRaw(void);
	double getMPU6050TempNorm(void);

	double tiltCompensate(double magX, double magY, double magZ, double roll,
			double pitch);

	virtual ~GY87();

};

#endif /* GY87_H_ */
