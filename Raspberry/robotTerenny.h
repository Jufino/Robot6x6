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

using namespace std;
using namespace cv;

#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <serial.h>
extern "C" {
  #include "semafor.h"
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

#define BATTERY_LED_INDICATING 1

#define SENSORS_WIFI 0
#define SENSORS_PORT 1213

#define REFRESH_STATUS     1 //zap alebo vyp autorefresh
#define REFRESH_MODULE     10 //v ms
#define REFRESH_BATTERY    REFRESH_MODULE*200
#define REFRESH_MOTORS     REFRESH_MODULE*10
#define REFRESH_HMC5883L   REFRESH_MODULE*3
#define REFRESH_BMP180     REFRESH_MODULE*10
#define REFRESH_MPU6050    REFRESH_MODULE*10
#define REFRESH_LEDS       REFRESH_MODULE*10
#define REFRESH_POSITION   REFRESH_MODULE*10
#define REFRESH_AMP        REFRESH_MODULE*200
#define REFRESH_ULTRASONIC REFRESH_MODULE*10   
#define REFRESH_CAMERA     REFRESH_MODULE*33  

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

typedef enum{
    COLOR_GREEN,
    COLOR_RED,
    COLOR_ORANGE,
    COLOR_OFF
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

//MPU6050
struct MPU6050_struct {
  float AccX;
  float AccY;
  float AccZ;
  float Temp;
  float GyX;
  float GyY;
  float GyZ;
  float Roll;
  float Pitch;
  float Yaw;
};
//https://github.com/jarzebski/Arduino-HMC5883L/blob/master/HMC5883L_compass_MPU6050/HMC5883L_compass_MPU6050.ino

//HMC5883L
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

struct HMC5883L_struct {
  float X;
  float Y;
  float Z;
  float angleRad;
  float angleDeg;
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
  char buttonDown;
  char buttonMiddle;
  char buttonUp;
};

struct Callibrate {
  float HMC5883LOffsetX;
  float HMC5883LOffsetY;
};

struct Leds_struct {
  color_t LedDown;
  color_t LedMiddle;
  color_t LedUp;
};

struct RobotPosition_struct {
  float x;
  float y;
  float angleRad;
  float angleDeg;
  
  float distanceL;
  float distanceR;
  float distance;
  
  float speedL;
  float speedR;
  float speed;
};

struct Camera_struct {
  Mat imgLeft;
  Mat imgRight;
};

struct RobotSensors {                 //struktura pre snimace aktualizovane s casom refresh hodnot pre jednotlive snimace
  GPS_struct gps;                     //gps
  MPU6050_struct MPU6050;             //akcelerometer, gyroskop a teplomer
  BMP180_struct BMP180;               //barometer, teplomer - zatial nie je implementovane
  HMC5883L_struct HMC5883L;           //kompas
  MotorsSensor_struct motors;         //meranie s otackomerov
  Buttons_struct buttons;             //tlacidla
  RobotPosition_struct robotPosition; //prepocitana pozicia
  float ultrasonic;                   //udaje z ultrazvuku
  float voltage;                      //napatie na bateriach
  float voltagePercent;               //napetie na bateriach(%)
  float amper;                        //prud odoberany z baterii
};

struct RobotAcculators {              //struktura pre riadiace veliciny s casom refresh podla jednotlivych hodnot pre riadenie
  MotorsAcculator_struct motors;
  Buttons_struct buttons;
  Leds_struct leds;
  int servoAngle;
};

void initRobot();
void closeRobot();

void initI2C();
void closeI2C();
void errorLedBlink();
void sendMatImage(Mat img, int quality);
void *getImgL(void *arg);
void *getImgR(void *arg);
void wifiCamera();
void initButton(position3_t pos);
void closeButton(position3_t pos);
unsigned char getButton(position3_t pos);
RobotAcculators getRobotAcculators();
void setRobotAcculators(RobotAcculators temp);
RobotSensors getRobotSensors();
Callibrate getCallibrate();

int getCameraClientsock();
int getSensorsClientsock();
bool blueTestConnection();
bool yellowTestConnection();
bool orangeTestConnection();
int getDistanceRaw(position6_t pos);
int getDeltaDistanceRaw(position6_t pos);
void resetDistance(position6_t pos);
void resetDistanceAll();
float prepocetTikovOtackomeraDoVzdialenosti(int pocetTikov);
float getDistance(position6_t pos);
float getDeltaDistance(position6_t pos);
void setServo(int angle);
unsigned int getUltrasonicRaw();
float getUltrasonic();
int getVoltageRaw();
float getVoltage();
float getVoltagePercent();
int getAmpRaw();
float getAmpVolt();
float getAmp();
void setLed(position3_t pos, color_t color);
void initMotorPowerSupply();
void setMotorPowerSupply(bool state);
void closeMotorPowerSupply();
void setMotor(position6_t pos, rotate_t rotate, unsigned char speed, bool onReg);
void stopAllMotors();
void setMotors(side_t side,rotate_t rotate,unsigned char speed,bool onReg);
void setMove(direction_t direction,unsigned char speed,bool onReg);
int getKbhit(void);
GPS_struct getGPS();

bool HMC5883LTestConnection();
void HMC5883LMeasurementSetting(hmc5883l_measurement_t measurement);
void HMC5883LSampleSetting(hmc5883l_samples_t sample);
void HMC5883LRateSetting(hmc5883l_dataRate_t datarate);
void HMC5883LRangeSetting(hmc5883l_range_t range);
void HMC5883LReadModeSetting(hmc5883l_mode_t mode);
void HMC5883LHighI2CSpeedSetting(bool status);
HMC5883L_struct getHMC5883LRaw();
void calibrateOffsetHMC5883L(HMC5883L_struct HMC5883L);
HMC5883L_struct normHMC5883L(HMC5883L_struct HMC5883L);    

void MPU6050ResetPRY();
void MPU6050ResetOffset();
void MPU6050WakeUp();
void MPU6050DisableAsMaster();
MPU6050_struct getMPU6050Raw();
MPU6050_struct getMPU6050();
void MPU6050CalibrateOffset(int pocet);
float dist(float a, float b);
void setMPU6050Sensitivity(unsigned char acc_sens, unsigned char gy_sens);
void setMPU6050DLPF(unsigned char acc_dlpf, unsigned char gy_dlpf);
float getSpeedFromDistance(float distance,float dt);
//http://rossum.sourceforge.net/papers/DiffSteer/DiffSteer.html
//http://users.isr.ist.utl.pt/~mir/cadeiras/robmovel/Kinematics.pdf
void calcRobotPosition(float deltaSpeedL,float deltaSpeedR,float dt);
bool compareMotors(MotorAcculator_struct motor, MotorAcculator_struct lastMotor);
void initRefresh();
void stopRefresh();
void syncModules(int signal , siginfo_t * siginfo, void * ptr);

#endif

