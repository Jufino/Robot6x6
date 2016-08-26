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

#define PORT_I2C "/dev/i2c-1"
#define PORT_GPS "/dev/ttyAMA0"
#define MPU6050ADDR 0x68
#define HMC5883LADDR 0x1E
#define BMP180ADDR 0x77
#define MODRYADDR 8
#define ZLTYADDR 9
#define ORANZOVYADDR 10

#define BatteryLed3Indicate 1
#define Wifi_camera  0
#define Wifi_snimace 0
#define PORT_snimace 1213
#define PORT_camera  1212

#define refreshModule 10 //v ms
#define refreshBattery refreshModule*200
#define refreshMotors refreshModule*10
#define refreshHMC5883L refreshModule*3
#define refreshBMP180 refreshModule*10
#define refreshMPU6050 refreshModule*10
#define refreshLeds refreshModule*10
#define refreshPosition refreshModule*10
#define refreshAmp refreshModule*200
#define refreshUltrasonic refreshModule*10    

#define R2 5.3f
#define R1 31.4f
#define maxVoltADC 5.0f
#define rozlisenieADC 1023.0f
#define maxNapetie 25.2f
#define minNapetie 22.8f
#define UltrasonicConstant 58.0f
#define rozliseniePrud 0.185f     // 185mV/A
#define OtackomerConstant 200.0f
#define OtackomerPriemer 0.085f
#define vzdialenostKolies 0.24f   //vzdialenost kolies
#define maxZmenaOtackomera 200    //maximalna zmena pozicie - v pocte tikov za cas refreshPosition
#define i2cWriteTimeout 10        //pocet kolkokrat ma opakovat zapis pri zlyhani

#define defaultMinXHMC5883L -892.0f
#define defaultMinYHMC5883L -781.0f

#define defaultMaxXHMC5883L 363.3f
#define defaultMaxYHMC5883L 88.9f

#define degHMC5883L 4.0f
#define minHMC5883L 30.0f
//---------------------

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
struct HMC5883L_struct {
  float X;
  float Y;
  float Z;
  float angleRad;
  float angleDeg;
};

struct BMP180_struct {
  float temperature;
  float preasure;
};

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

struct MotorAcculator_struct {
  signed char direction;
  unsigned char speed;
  bool onRegulator;
};

struct MotorsAcculator_struct {
  MotorAcculator_struct motor1;
  MotorAcculator_struct motor2;
  MotorAcculator_struct motor3;
  MotorAcculator_struct motor4;
  MotorAcculator_struct motor5;
  MotorAcculator_struct motor6;
};

struct MotorSensor_struct {
  int distanceRaw;
  float distance;
  int speedRaw;
  float speed;
};

struct MotorsSensor_struct {
  MotorSensor_struct motor1;
  MotorSensor_struct motor2;
  MotorSensor_struct motor3;
  MotorSensor_struct motor4;
  MotorSensor_struct motor5;
  MotorSensor_struct motor6;
};

struct Buttons_struct {
  char button1;
  char button2;
  char button3;
};

struct Leds_struct {
  char Led1;
  char Led2;
  char Led3;
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

void setDevice(unsigned char addr);
void writeRegister(unsigned char addr, unsigned char reg, unsigned char value);
unsigned int readRegister16(unsigned char addr, unsigned char reg);
signed int readRegister16s(unsigned char addr, unsigned char reg);
unsigned char readRegister8(unsigned char addr, unsigned char reg);

void sendMatImage(Mat img, int quality);
unsigned char getButton(char pos);

RobotAcculators getRobotAcculators();
void setRobotAcculators(RobotAcculators temp);
RobotSensors getRobotSensors();
int getSocketCamera();
int getSocketSnimace();


int testModry();
int testZlty();
int testOranzovy();
int test();

int getDistanceRaw(int pos);
float prepocetTikovOtackomeraDoVzdialenosti(int pocetTikov);
int getDistance(int pos);
int getDeltaDistanceRaw(int pos);
float getDeltaDistance(int pos);
void resetDistance(int pos);
void setServo(int angle);
unsigned int getUltrasonicRaw();
float getUltrasonic();
int getVoltageRaw();
float getVoltage();
float getVoltagePercent();
int getAmpRaw();
float getAmpVolt();
float getAmp();
void setLed(int pos, char color);
void setMotorPowerSupply(bool state);
void setMotor(int pos, signed char dir, unsigned char speed, bool onReg);
void setMove(char direction,unsigned char speed,bool onReg);
int getKbhit(void);
GPS_struct getGPS();
void calibrateHMC5883L();
void HMC5883LSampleRateAndModeSetting(int sample,int datarate,int mode);
void HMC5883LGainSetting(int gain);
void HMC5883LReadModeSetting(int highI2cSpeed,int mode);
HMC5883L_struct getHMC5883LRaw();
HMC5883L_struct getHMC5883L();
void MPU6050ResetPRY();
void MPU6050ResetOffset();
void MPU6050WakeUp();
MPU6050_struct getMPU6050Raw() ;
MPU6050_struct getMPU6050();
void MPU6050CalibrateOffset(int pocet);
float dist(float a, float b);
void setMPU6050Sensitivity(unsigned char acc_sens, unsigned char gy_sens);
void setMPU6050DLPF(unsigned char acc_dlpf, unsigned char gy_dlpf);
float getSpeedFromDistance(float distance,float dt);
void calcRobotPosition(float deltaSpeedL,float deltaSpeedR,float dt);
bool compareMotors(MotorAcculator_struct motor, MotorAcculator_struct lastMotor);
void syncModules(int signal , siginfo_t * siginfo, void * ptr);
void MPU6050DisableAsMaster();
#endif

