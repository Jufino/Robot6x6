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
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <signal.h>
#include <time.h>
#include <sys/wait.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include "libfreenect.h"
#include "libfreenect_sync.h"

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
#define STM32_ADDRESS   (0x11)
#define I2C_WRITE_TIMEOUT 10        //pocet kolkokrat ma opakovat zapis pri zlyhani

#define SENSORS_WIFI 0
#define SENSORS_PORT 1213

#define SYNC_MIN_TIME 1000 // 1 ms
#define SYNC_POSSITION_TIME         SYNC_MIN_TIME*50
#define SYNC_MOTORS_TIME            SYNC_MIN_TIME*200
#define SYNC_ULTRASONIC_TIME        SYNC_MIN_TIME*50
#define SYNC_LEDS_TIME              SYNC_MIN_TIME*50
#define SYNC_BUTTONS_TIME           SYNC_MIN_TIME*50

#define ENABLE_I2C 1        //ok
#define ENABLE_MOTORS 1    //ok
#define ENABLE_ULTRASONIC 1 //ok
#define ENABLE_LEDS 1      //ok
#define ENABLE_BUTTONS 1    //ok
#define ENABLE_POSSITION 1  //ok

#define SYNC_KINECTACCULATORS_TIME SYNC_MIN_TIME*10
#define SYNC_KINECTSENSORS_TIME    SYNC_MIN_TIME*20

#define ENABLE_KINECTACCULATORS 0
#define ENABLE_KINECTSENSORS 0
#define ENABLE_KINECTCAMERA 0

#define CAMERA_WIFI  0
#define CAMERA_PORT  1212
#define CAMERA_HEIGHT 240
#define CAMERA_WIDTH 320
#define NUMBER_OF_CAMERA 0
#define INDEX_CAMERA_LEFT 0 //ak je iba jedna kamera pouziva sa lava
#define INDEX_CAMERA_RIGHT 1

#define CONST_ULTRASONIC 58.0f

#define ENABLE_LOG_ERROR 1
#define ENABLE_LOG_INFO 1
#define ENABLE_LOG_INFO_DETAIL 0

typedef enum {
  CAMERA_VARIABLE_L,
  CAMERA_IMAGE_L1,
  CAMERA_IMAGE_L2,
  CAMERA_VARIABLE_R,
  CAMERA_IMAGE_R1,
  CAMERA_IMAGE_R2,
  CAMERA_VARIABLE_KINECTDEPTH,
  CAMERA_DEPTH_KINECT1,
  CAMERA_DEPTH_KINECT2,
  CAMERA_VARIABLE_KINECTIMAGE,
  CAMERA_IMAGE_KINECT1,
  CAMERA_IMAGE_KINECT2,
  ROBOTSENSORS,
  ROBOTACCULATORS,
  I2C
} semafor_name_t;

typedef enum {
  SEMAFOR_TAG,
  I2C_TAG,
  KINECT_TAG,
  SENSOR_CONN_TAG,
  CAMERA_CONN_TAG,
  MOTOR_TAG,
  NUCLEO_TAG,
  ROBOT_TAG
} log_tag_t;

//ostatne
typedef enum {
  COLOR_OFF,
  COLOR_GREEN,
  COLOR_RED,
  COLOR_ORANGE
} color_t;


typedef enum {
  ROTATE_CLOCKWISE,
  ROTATE_ANTICLOCKWISE,
  ROTATE_STOP
} rotate_t;

typedef enum {
  FORWARD,
  BACKWARD,
  CLOCKWISE,
  ANTICLOCKWISE,
  STOP
} direction_t;

typedef enum {
  POSITION_DOWN,
  POSITION_MIDDLE,
  POSITION_UP
} position3_t;
//--------------------------------------------

struct Axis_struct {
  double x;
  double y;
  double z;
};

struct Angle3d_struct {
  double roll;
  double pitch;
  double yaw;
};

struct Buttons_struct {
  bool buttonDown;
  bool buttonMiddle;
  bool buttonUp;
};

struct Leds_struct {
  color_t LedDown;
  color_t LedMiddle;
  color_t LedUp;
  freenect_led_options LedKinect;
};

struct RobotPosition_struct {
  Axis_struct axisPossition;
  Angle3d_struct anglePossition;
};

struct Voltage_struct {
  double volts;
  double capacityPercent;
};

struct KinectSensor_struct {
  Angle3d_struct accAngle;
  Axis_struct accAxis;
};

struct RobotSensors {                 //struktura pre snimace aktualizovane s casom refresh hodnot pre jednotlive snimace
  KinectSensor_struct   kinect;         //kinect
  Buttons_struct        buttons;             //tlacidla
  RobotPosition_struct  robotPosition; //prepocitana pozicia
  double ultrasonic;

};

struct RobotAcculators {              //struktura pre riadiace veliciny s casom refresh podla jednotlivych hodnot pre riadenie
  direction_t     robotDirection;
  unsigned int    robotSpeed;  //rychlost v mm/s
  Leds_struct     leds;
  Angle3d_struct  kinect;
  bool            motorPowerSupply;
};

int semInit(int sem_id, int sem_num, int val);
int semCreate(key_t key, int poc);
int semWait(int sem_id, semafor_name_t sem_num);
int semPost(int sem_id, semafor_name_t sem_num);
void semRem(int sem_id);

void initRobot(void);
void closeRobot(void);

void initI2C(void);
void closeI2C(void);
char setDevice(unsigned char addr);
char writeRegisterAndValueU8(unsigned char addr, unsigned char reg, unsigned char value);
char writeRegisterAndValueU16(unsigned char addr, unsigned char reg, unsigned int value);
char writeRegisterAndValueS16(unsigned char addr, unsigned char reg, int value);
char writeRegister(unsigned char addr, unsigned char reg);
unsigned int readRegister16(unsigned char addr, unsigned char reg);
signed int readRegister16s(unsigned char addr, unsigned char reg);
unsigned char readRegister8(unsigned char addr, unsigned char reg);

bool initKinect();

Mat getImageKinect(void);
Mat getDepthKinect(void);
Mat getImageLeft(void);
Mat getImageRight(void);
Mat getImage(void);

void sendMatImage(Mat img, int quality);
int getCameraClientsock(void);
void closeCameraConnection(void);

int getSensorsClientsock(void);
void closeSensorConnection(void);

RobotAcculators getRobotAcculators(void);
void setRobotAcculators(RobotAcculators temp);
RobotSensors getRobotSensors(void);

void initMotorPowerSupply(void);
void setMotorPowerSupply(bool state);
void closeMotorPowerSupply(void);

bool nucleoTestConnection(void);
char motorsTestConnection(void);
void setServo(int angle);
void setMove(direction_t direction, unsigned int speed);
void setLeds(color_t ledUpColor, color_t ledMiddleColor, color_t ledDownColor);
unsigned int getUltrasonicRaw(void);
double getUltrasonic(void);
unsigned char getButtons(void);
double dist(double a, double b);
double rad2Deg(double angle);
double deg2Rad(double angle);

void *getImgR(void *arg);
void *getImgL(void *arg);
void *getImgAndDephKinect(void *arg);
void *cameraNetworkConnection(void *arg);
void *sensorsNetworkConnection(void *arg);
void *syncUltrasonic(void *arg);
void *syncLeds(void *arg);
void *syncMotors(void *arg);
void *syncPossition(void *arg);
void *syncButtons(void *arg);
void *syncKinectAcculators(void *arg);
void *syncKinectSensors(void *arg);

void *syncModules(void *arg);

void sigctrl(int param);
void sigpipe(int param);
#endif