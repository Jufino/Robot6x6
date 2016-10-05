#ifndef _LIBROBOTTERENNY_H
#define _LIBROBOTTERENNY_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <inttypes.h>
#include <unistd.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <signal.h>
#include <time.h>
#include <sys/wait.h>
#include <sys/ipc.h>
#include <sys/sem.h>

using namespace std;
using namespace cv;

#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <serial.h>
extern "C" {
  #include <gpio.h>
}

#define PORT_I2C          "/dev/i2c-1"
#define PORT_GPS          "/dev/ttyAMA0"
#define MPU6050_ADDRESS   (0x68)
#define HMC5883L_ADDRESS  (0x1E)
#define BMP180_ADDRESS    (0x77)
#define BLUE_ADDRESS      (8)
#define YELLOW_ADDRESS    (9)
#define ORANGE_ADDRESS    (10)

#define BATTERY_LED_INDICATING 0

#define SENSORS_WIFI 0
#define SENSORS_PORT 1213

#define REFRESH1_STATUS     1 //zap alebo vyp autorefresh
#define REFRESH1_MODULE     20.0f //v ms
#define REFRESH1_MOTORS     REFRESH1_MODULE*5.0f
#define REFRESH1_HMC5883L   REFRESH1_MODULE*2.0f
#define REFRESH1_BMP180     REFRESH1_MODULE*5.0f
#define REFRESH1_ACC        REFRESH1_MODULE*5.0f
#define REFRESH1_GY         REFRESH1_MODULE*1.0f
#define REFRESH1_TEMP       REFRESH1_MODULE*5.0f
#define REFRESH1_LEDS       REFRESH1_MODULE*5.0f
#define REFRESH1_POSITION   REFRESH1_MODULE*5.0f
#define REFRESH1_ULTRASONIC REFRESH1_MODULE*5.0f
#define REFRESH1_VOLTAGE    REFRESH1_MODULE*100.0f
#define REFRESH1_AMP        REFRESH1_MODULE*100.0f
#define REFRESH1_BATTERY    REFRESH1_MODULE*100.0f
#define REFRESH1_BUTTON     REFRESH1_MODULE*10.0f

#define REFRESH2_STATUS     0 //zap alebo vyp autorefresh
#define REFRESH2_MODULE     33.0f

#define REFRESH_GPS_STATUS 0 

#define R2 5.3f
#define R1 31.4f
#define ADC_MAXIMUM_VOLTAGE 5.0f
#define ADC_RESOLUTION 1023.0f
#define MAX_BATTERY_VOLTAGE 25.2f
#define MIN_BATTERY_VOLTAGE 22.8f

#define CONST_ULTRASONIC 58.0f
#define CONST_AMP 0.185f     // 185mV/A
#define CONST_ENCODER 200.0f
#define DIAMERER_WHEEL 0.085f
#define LENGTH_BETWEEN_LEFT_AND_RIGHT_WHEEL 0.24f   //vzdialenost kolies
#define MAX_DELTA_TICKS_ENCODER 200    //maximalna zmena pozicie - v pocte tikov za cas refreshPosition
#define I2C_WRITE_TIMEOUT 10        //pocet kolkokrat ma opakovat zapis pri zlyhani
#define CALLIBRATE_DATA_CALCULATE 1

#define HMC5883L_OFFSET_X -345.0f
#define HMC5883L_OFFSET_Y -540.0f
#define HMC5883L_OFFSET_Z    0.0f

// zavisle na zemepisnej sirke: http://magnetic-declination.com/  
#define HMC5883L_DEGREE 4.0f
#define HMC5883L_MINUTES 30.0f
#define CAMERA_WIFI  0
#define CAMERA_PORT  1212
#define CAMERA_HEIGHT 240
#define CAMERA_WIDTH 320
#define NUMBER_OF_CAMERA 0
#define INDEX_CAMERA_LEFT 0 //ak je iba jedna kamera pouziva sa lava
#define INDEX_CAMERA_RIGHT 1

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
#define	BMP180_REG_CONTROL 0xF4
#define	BMP180_REG_RESULT 0xF6
#define	BMP180_COMMAND_TEMPERATURE 0x2E
#define	BMP180_COMMAND_PRESSURE0 0x34
#define	BMP180_COMMAND_PRESSURE1 0x74
#define	BMP180_COMMAND_PRESSURE2 0xB4
#define	BMP180_COMMAND_PRESSURE3 0xF4

#define Q_angle  0.01
#define Q_gyro   0.0003
#define R_angle  0.01

typedef enum{
  CAMERA_VARIABLE_L,
  CAMERA_IMAGE_L1,
  CAMERA_IMAGE_L2,
  CAMERA_VARIABLE_R,
  CAMERA_IMAGE_R1,
  CAMERA_IMAGE_R2,
  ROBOTSENSORS,
  ROBOTACCULATORS,
  CALLIBRATE,
  REFRESH1_LOCK,
  REFRESH2_LOCK
} semafor_name_t;

struct Kalman_struct{
  float x_angle;
  float x_bias;
  float P_00;
  float P_01;
  float P_10;
  float P_11;
  float y;
  float S;
  float K_0;
  float K_1;
};

//MPU6050
typedef enum
{
    MPU6050_CLOCK_KEEP_RESET      = 0b111,
    MPU6050_CLOCK_EXTERNAL_19MHZ  = 0b101,
    MPU6050_CLOCK_EXTERNAL_32KHZ  = 0b100,
    MPU6050_CLOCK_PLL_ZGYRO       = 0b011,
    MPU6050_CLOCK_PLL_YGYRO       = 0b010,
    MPU6050_CLOCK_PLL_XGYRO       = 0b001,
    MPU6050_CLOCK_INTERNAL_8MHZ   = 0b000
} mpu6050_clockSource_t;

typedef enum
{
    MPU6050_SCALE_2000DPS         = 0b11,
    MPU6050_SCALE_1000DPS         = 0b10,
    MPU6050_SCALE_500DPS          = 0b01,
    MPU6050_SCALE_250DPS          = 0b00
} mpu6050_dps_t;

typedef enum
{
    MPU6050_RANGE_16G             = 0b11,
    MPU6050_RANGE_8G              = 0b10,
    MPU6050_RANGE_4G              = 0b01,
    MPU6050_RANGE_2G              = 0b00,
} mpu6050_range_t;

typedef enum
{
    MPU6050_DELAY_3MS             = 0b11,
    MPU6050_DELAY_2MS             = 0b10,
    MPU6050_DELAY_1MS             = 0b01,
    MPU6050_NO_DELAY              = 0b00,
} mpu6050_onDelay_t;

typedef enum
{
    MPU6050_DHPF_HOLD             = 0b111,
    MPU6050_DHPF_0_63HZ           = 0b100,
    MPU6050_DHPF_1_25HZ           = 0b011,
    MPU6050_DHPF_2_5HZ            = 0b010,
    MPU6050_DHPF_5HZ              = 0b001,
    MPU6050_DHPF_RESET            = 0b000,
} mpu6050_dhpf_t;

typedef enum
{
    MPU6050_DLPF_6                = 0b110,
    MPU6050_DLPF_5                = 0b101,
    MPU6050_DLPF_4                = 0b100,
    MPU6050_DLPF_3                = 0b011,
    MPU6050_DLPF_2                = 0b010,
    MPU6050_DLPF_1                = 0b001,
    MPU6050_DLPF_0                = 0b000,
} mpu6050_dlpf_t;
//--------------------------------------------
//HMC5883L
typedef enum{
    HMC5883L_SAMPLES_8     = 0b11,
    HMC5883L_SAMPLES_4     = 0b10,
    HMC5883L_SAMPLES_2     = 0b01,
    HMC5883L_SAMPLES_1     = 0b00
} hmc5883l_samples_t;

typedef enum{
    HMC5883L_DATARATE_75HZ       = 0b110,
    HMC5883L_DATARATE_30HZ       = 0b101,
    HMC5883L_DATARATE_15HZ       = 0b100,
    HMC5883L_DATARATE_7_5HZ      = 0b011,
    HMC5883L_DATARATE_3HZ        = 0b010,
    HMC5883L_DATARATE_1_5HZ      = 0b001,
    HMC5883L_DATARATE_0_75_HZ    = 0b000
} hmc5883l_dataRate_t;

typedef enum{
    HMC5883L_RANGE_8_1GA     = 0b111,
    HMC5883L_RANGE_5_6GA     = 0b110,
    HMC5883L_RANGE_4_7GA     = 0b101,
    HMC5883L_RANGE_4GA       = 0b100,
    HMC5883L_RANGE_2_5GA     = 0b011,
    HMC5883L_RANGE_1_9GA     = 0b010,
    HMC5883L_RANGE_1_3GA     = 0b001,
    HMC5883L_RANGE_0_88GA    = 0b000
} hmc5883l_range_t;

typedef enum{
    HMC5883L_IDLE          = 0b10,
    HMC5883L_SINGLE        = 0b01,
    HMC5883L_CONTINOUS     = 0b00
} hmc5883l_mode_t;

typedef enum{
    HMC5883L_NORMAL          = 0b00,
    HMC5883L_POSITIVE_BIAS   = 0b01,
    HMC5883L_NEGATIVE_BIAS   = 0b10
} hmc5883l_measurement_t;
//--------------------------------------------
//ostatne
typedef enum{
    COLOR_OFF,
    COLOR_GREEN,
    COLOR_RED,
    COLOR_ORANGE
} color_t;

typedef enum{
    SIDE_LEFT,
    SIDE_RIGHT
} side_t;

typedef enum{
    ROTATE_CLOCKWISE,
    ROTATE_ANTICLOCKWISE,
    ROTATE_STOP
} rotate_t;

typedef enum{
    DIRECTION_FRONT,
    DIRECTION_BACK,
    DIRECTION_LEFT,
    DIRECTION_RIGHT,
    DIRECTION_STOP
} direction_t;

typedef enum{
    POSITION_DOWN,
    POSITION_MIDDLE,
    POSITION_UP
} position3_t;

typedef enum{
    POSITION_DOWN_LEFT,
    POSITION_DOWN_RIGHT,
    POSITION_MIDDLE_LEFT,
    POSITION_MIDDLE_RIGHT,
    POSITION_UP_LEFT,
    POSITION_UP_RIGHT,
} position6_t;
//--------------------------------------------

struct Axis_struct{
  float x;
  float y;
  float z;
};

struct Angle3d_struct{
  float roll;
  float pitch;
  float yaw;
};


//MPU6050
struct MPU6050_struct {
  Axis_struct accAxis;
  Axis_struct gyAxis;
  Angle3d_struct accAngle;
  Angle3d_struct gyAngle;
  float temperature;
};

struct HMC5883L_struct {
  Axis_struct compassAxis;
  float yaw;
};
//-----------------------

struct BMP180_struct {
  float temperature;
  float preasure;
};

//------------------------
struct GPGGA_struct {
  char UTCTime[9];
  float Latitude;
  char NSIndicator;
  float Longitude;
  char EWindicator;
  int PositionFixIndictor;
  int SatellitesUsed;
  float HDOP;
  float MSLAltitude;
  char Units1;
  int GeoidSeparation;
  char Units2;
  int AgeofDifferentialCorrections;
  int DiffRefereceCorrections;
  char Checksum[3];
};

struct GPGLL_struct {
  float Latitude;
  char NSIndicator;
  float Longitude;
  char EWIndicator;
  char UTCTime[9];
  char Status;
  char ModeIndicator;
  char Checksum[3];
};

struct GPGSA_struct {
  char ModeChar;
  int ModeInt;
  int SatellitesUsedCH1;
  int SatellitesUsedCH2;
  int SatellitesUsedCH3;
  int SatellitesUsedCH4;
  int SatellitesUsedCH5;
  int SatellitesUsedCH6;
  int SatellitesUsedCH7;
  int SatellitesUsedCH8;
  int SatellitesUsedCH9;
  int SatellitesUsedCH10;
  int SatellitesUsedCH11;
  int SatellitesUsedCH12;
  float PDOP;
  float HDOP;
  float VDOP;
  char Checksum[3];
};

struct GPGSV_struct {
  int NumberOfMessages;
  int MessageNumber;
  int SatellitesInView;
  int SatelliteId1;
  int Elevation1;
  int Azimuth1;
  int SNR1;
  int SatelliteId2;
  int Elevation2;
  int Azimuth2;
  int SNR2;
  int SatelliteId3;
  int Elevation3;
  int Azimuth3;
  int SNR3;
  int SatelliteId4;
  int Elevation4;
  int Azimuth4;
  int SNR4;
  char Checksum[3];
};

struct GPRMC_struct {
  char UTCTime[9];
  char Status;
  float Latitude;
  char NSIndicator;
  float Longitude;
  char EWIndicator;
  float SpeedOverGround;
  float CourseOverGround;
  char Date[6];
  char Mode;
  char Checksum[3];
};

struct GPVTG_struct {
  float CourseTrue;
  char ReferenceTrue;
  float CourseMagnetic;
  char ReferenceMagnetic;
  float SpeedKnots;
  float SpeedKmh;
  char UnitsKnots;
  char UnitsKmh;
  char Mode;
  char Checksum[3];
};

struct GPS_struct {
  GPGGA_struct GPGGA;
  GPGLL_struct GPGLL;
  GPGSA_struct GPGSA;
  GPGSV_struct GPGSV;
  GPRMC_struct GPRMC;
  GPVTG_struct GPVTG;
};

//------------------------------

struct MotorAcculator_struct {
  rotate_t direction;
  unsigned char speed;
  bool onRegulator;
};

struct MotorsAcculator_struct {
  MotorAcculator_struct motorDownLeft;
  MotorAcculator_struct motorDownRight;
  MotorAcculator_struct motorMiddleLeft;
  MotorAcculator_struct motorMiddleRight;
  MotorAcculator_struct motorUpLeft;
  MotorAcculator_struct motorUpRight;
};

struct MotorSensor_struct {
  int distanceRaw;
  float distance;
  int speedRaw;
  float speed;
};

struct MotorsSensor_struct {
  MotorSensor_struct motorDownLeft;
  MotorSensor_struct motorDownRight;
  MotorSensor_struct motorMiddleLeft;
  MotorSensor_struct motorMiddleRight;
  MotorSensor_struct motorUpLeft;
  MotorSensor_struct motorUpRight;
};

struct Buttons_struct {
  bool buttonDown;
  bool buttonMiddle;
  bool buttonUp;
};

struct Callibrate {
  Axis_struct HMC5883LOffsetAxis;
  Axis_struct MPU6050AccOffsetAxis;
  Axis_struct MPU6050GyThresholdAxis;
  Axis_struct MPU6050GyOffsetAxis;
};

struct Leds_struct {
  color_t LedDown;
  color_t LedMiddle;
  color_t LedUp;
};

struct RobotPosition_struct {
  float x;
  float y;
  
  float distanceL;
  float distanceR;
  float distance;
  
  float speedL;
  float speedR;
  float speed;
  float angleEncoder;
  Angle3d_struct imuAngle;
};

struct Voltage_struct{
  float volts;
  float capacityPercent;
}; 

struct RobotSensors {                 //struktura pre snimace aktualizovane s casom refresh hodnot pre jednotlive snimace
  GPS_struct gps;                     //gps
  MPU6050_struct MPU6050;             //akcelerometer, gyroskop a teplomer
  BMP180_struct BMP180;               //barometer, teplomer - zatial nie je implementovane
  HMC5883L_struct HMC5883L;           //kompas
  MotorsSensor_struct motors;         //meranie s otackomerov
  Buttons_struct buttons;             //tlacidla
  RobotPosition_struct robotPosition; //prepocitana pozicia
  Voltage_struct voltage;
  float ultrasonic;                   //udaje z ultrazvuku
  float amper;                        //prud odoberany z baterii
};

struct RobotAcculators {              //struktura pre riadiace veliciny s casom refresh podla jednotlivych hodnot pre riadenie
  MotorsAcculator_struct motors;
  Leds_struct leds;
  int servoAngle;
  bool motorPowerSupply;
};

int semInit(int sem_id, int sem_num, int val);
int semCreate(key_t key, int poc);
int semWait(int sem_id, semafor_name_t sem_num);
int semPost(int sem_id, semafor_name_t sem_num);
void semRem(int sem_id);

void sigctrl(int param);
void sigpipe(int param);

void *cameraNetworkConnection(void *arg);
void *sensorsNetworkConnection(void *arg);

void initRobot(void);
void closeRobot(void);
void initI2C(void);
void closeI2C(void);
char setDevice(unsigned char addr);
char writeRegisterValue(unsigned char addr, unsigned char reg, unsigned char value);
char writeRegister(unsigned char addr, unsigned char reg);
unsigned int readRegister16(unsigned char addr, unsigned char reg);
signed int readRegister16s(unsigned char addr, unsigned char reg);
unsigned char readRegister8(unsigned char addr, unsigned char reg);
void sendMatImage(Mat img, int quality);
void *getImgL(void *arg);
void *getImgR(void *arg);
void initButton(position3_t pos);
void closeButton(position3_t pos);
unsigned char getButton(position3_t pos);
RobotAcculators getRobotAcculators();
void setRobotAcculators(RobotAcculators temp);
RobotSensors getRobotSensors(void);
Callibrate getCallibrate(void);
int getCameraClientsock(void);
int getSensorsClientsock(void);
bool blueTestConnection(void);
bool yellowTestConnection(void);
bool orangeTestConnection(void);
int getDistanceRaw(position6_t pos);
int getDeltaDistanceRaw(position6_t pos);
void resetDistance(position6_t pos);
void resetDistanceAll(void);
float prepocetTikovOtackomeraDoVzdialenosti(int pocetTikov);
float getDistance(position6_t pos);
float getDeltaDistance(position6_t pos);
void setServo(int angle);
unsigned int getUltrasonicRaw(void);
float getUltrasonic(void);
int getVoltageRaw(void);
float getVoltage(void);
float calcVoltagePercent(float volt);
int getAmpRaw(void);
float getAmpVolt(void);
float getAmp(void);
void setLed(position3_t pos, color_t color);
void initMotorPowerSupply(void);
void setMotorPowerSupply(bool state);
void closeMotorPowerSupply(void);
void setMotor(position6_t pos, rotate_t rotate, unsigned char speed, bool onReg);
void stopAllMotors(void);
void setMotors(side_t side,rotate_t rotate,unsigned char speed,bool onReg);
void setMove(direction_t direction,unsigned char speed,bool onReg);
int getKbhit(void);
GPS_struct getGPS(void);

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
HMC5883L_struct getHMC5883LRaw(void);
HMC5883L_struct getHMC5883LNorm(void);

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

Axis_struct getMPU6050GyRaw(void);
Axis_struct getMPU6050AccRaw(void);
Axis_struct getMPU6050GyNorm(void);
Axis_struct getMPU6050AccNorm(void);
float getMPU6050TempRaw(void);
float getMPU6050TempNorm(void);

float dist(float a, float b);
float getSpeedFromDistance(float distance,float dt);
float rad2Deg(float angle);
float deg2Rad(float angle);

Mat getImageLeft(void);
Mat getImageRight(void);
Mat getImage(void);

//http://rossum.sourceforge.net/papers/DiffSteer/DiffSteer.html
//http://users.isr.ist.utl.pt/~mir/cadeiras/robmovel/Kinematics.pdf
void calcRobotPosition(float deltaSpeedL,float deltaSpeedR,float dt);
bool compareMotors(MotorAcculator_struct motor, MotorAcculator_struct lastMotor);
void initRefresh1(void);
void stopRefresh1(void);
void initRefresh2(void);
void stopRefresh2(void);
void initRefreshSignalAction(void);
void *syncGPS(void *arg);
void syncModules(int signal , siginfo_t * siginfo, void * ptr);
#endif