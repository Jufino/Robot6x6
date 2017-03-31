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
#include <XnCppWrapper.h>
#include <XnUSB.h>

using namespace std;
using namespace cv;

#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
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
#define SYNC_MOTORS_TIME            SYNC_MIN_TIME*100
#define SYNC_ULTRASONIC_TIME        SYNC_MIN_TIME*50
#define SYNC_LEDS_TIME              SYNC_MIN_TIME*300
#define SYNC_BUTTONS_TIME           SYNC_MIN_TIME*100
#define SYNC_MAP_GENERATE_TIME      SYNC_MIN_TIME*100
#define SYNC_OPERATOR_DETECT_TIME      SYNC_MIN_TIME*100

#define ENABLE_I2C 1        //ok
#define ENABLE_MOTORS 0    //ok
#define ENABLE_ULTRASONIC 0 //ok
#define ENABLE_LEDS 0      //ok
#define ENABLE_BUTTONS 0    //ok
#define ENABLE_POSSITION 1  //ok
#define ENABLE_MAP_GENERATE 1
#define ENABLE_OPERATOR_DETECT 1

#define ENABLE_MAP_OBLIVION 1

#define SPEED_OF_MAP_OBLIVION 2
#define SPEED_OF_MAP_CREATION 10

#define MAP_WIDTH 640
#define MAP_HEIGHT 480

#define SYNC_KINECTMOTOR_TIME     SYNC_MIN_TIME*20
#define SYNC_KINECTLED_TIME       SYNC_MIN_TIME*1000
#define SYNC_KINECTSENSORS_TIME   SYNC_MIN_TIME*20

#define ENABLE_KINECTMOTOR 1
#define ENABLE_KINECTLED 1
#define ENABLE_KINECTSENSORS 1
#define ENABLE_KINECTCAMERA 1

#define CAMERA_WIFI  1
#define CAMERA_PORT  1212
#define CAMERA_HEIGHT 240
#define CAMERA_WIDTH 320
#define NUMBER_OF_CAMERA 0
#define INDEX_CAMERA_LEFT 0 //ak je iba jedna kamera pouziva sa lava
#define INDEX_CAMERA_RIGHT 1

#define CONST_ULTRASONIC 58.0f

#define ENABLE_LOG_ERROR 1
#define ENABLE_LOG_INFO 1

#define VID_MICROSOFT 0x45e
#define PID_NUI_MOTOR 0x02b0

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
  CAMERA_VARIABLE_KINECTPOINTCLOUDMAP,
  CAMERA_POINTCLOUDMAP_KINECT1,
  CAMERA_POINTCLOUDMAP_KINECT2,
  ROBOTSENSORS,
  ROBOTACCULATORS,
  MAP_VARIABLE,
  MAP_IMAGE1,
  MAP_IMAGE2,
  OPERATOR_IMAGES,
  I2C,
  DEPTH_OPERATOR_MASK1,
  DEPTH_OPERATOR_MASK2,
  DEPTH_OPERATOR_MASK_VARIABLE,
  RGB_OPERATOR1,
  RGB_OPERATOR2,
  RGB_OPERATOR_VARIABLE,
  HSV_OPERATOR1,
  HSV_OPERATOR2,
  HSV_OPERATOR_VARIABLE,
  ORANGE_OPERATOR_MASK1,
  ORANGE_OPERATOR_MASK2,
  ORANGE_OPERATOR_MASK_VARIABLE,
  GREEN_OPERATOR_MASK1,
  GREEN_OPERATOR_MASK2,
  GREEN_OPERATOR_MASK_VARIABLE,
  OPERATOR_POSSITION
} semafor_name_t; //nazabudnut pri pridani inicializovat v initRobot

typedef enum {
  SEMAFOR_TAG,
  I2C_TAG,
  KINECT_TAG,
  SENSOR_CONN_TAG,
  CAMERA_CONN_TAG,
  MOTOR_TAG,
  NUCLEO_TAG,
  ROBOT_TAG,
  MAP_TAG,
  OPERATOR_TAG
} log_tag_t;

//ostatne
typedef enum {
  COLOR_OFF,
  COLOR_GREEN,
  COLOR_RED,
  COLOR_ORANGE
} color_t;

typedef enum {
  LEDKINECT_OFF,
  LEDKINECT_GREEN,
  LEDKINECT_RED,
  LEDKINECT_ORANGE,
  LEDKINECT_BLINK_ORANGE,
  LEDKINECT_BLINK_GREEN,
  LEDKINECT_BLINK_RED_ORANGE
} ledKinect_t;

typedef enum {
  MOTORSTATUSKINECT_STOPPED = 0,
  MOTORSTATUSKINECT_REACHED_LIMITS = 1,
  MOTORSTATUSKINECT_MOVING = 4
} motorStatusKinect_t;


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
  motorStatusKinect_t motorStatus;
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
  ledKinect_t     ledKinect;
  bool            motorPowerSupply;
};

void charTag(log_tag_t tag, char *buffer);
void LOGError(log_tag_t tag, const char text[]);
void LOGInfo(log_tag_t tag, unsigned char priority, const char text[]);

int semInit(int sem_id, int sem_num, int val);
int semCreate(key_t key, int poc);
int semWait(int sem_id, semafor_name_t sem_num);
int semPost(int sem_id, semafor_name_t sem_num);
void semRem(int sem_id);

void initRobot(void);
void closeRobot(void);

bool initI2C(void);
void closeI2C(void);
char setDevice(unsigned char addr);
char writeRegisterAndValueU8(unsigned char addr, unsigned char reg, unsigned char value);
char writeRegisterAndValueU16(unsigned char addr, unsigned char reg, unsigned int value);
char writeRegisterAndValueS16(unsigned char addr, unsigned char reg, int value);
char writeRegister(unsigned char addr, unsigned char reg);
unsigned int readRegister16(unsigned char addr, unsigned char reg);
signed int readRegister16s(unsigned char addr, unsigned char reg);
signed long readRegister32s(unsigned char addr, unsigned char reg);
unsigned char readRegister8(unsigned char addr, unsigned char reg);

bool initKinect(unsigned char imageMode);

Mat getImageKinect(void);
Mat getDepthKinect(void);
Mat getImageLeft(void);

Mat getImageRight(void);

RobotAcculators getRobotAcculators(void);
void setRobotAcculators(RobotAcculators temp);
RobotSensors getRobotSensors(void);

void sendMatImage(Mat img, int quality);
void closeCameraConnection(void);
void closeSensorConnection(void);

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
Axis_struct getPossitionAxis(void);
Angle3d_struct getPossitionAngle3d(void);

void *waitForCameraConnection(void *arg);
void *waitForSensorConnection(void *arg);

void *syncImageLeft(void *arg);
void *syncImageRight(void *arg);
void *syncCameraNetworkConnection(void *arg);
void *syncSensorNetworkConnection(void *arg);
void *syncKinectFrames(void *arg);
void *syncUltrasonic(void *arg);
void *syncLeds(void *arg);
void *syncMotors(void *arg);
void *syncPossition(void *arg);
void *syncButtons(void *arg);
void *syncKinectMotor(void *arg);
void *syncKinectLed(void *arg);
void *syncKinectSensors(void *arg);
void *syncModules(void *arg);

void sigctrl(int param);
void sigpipe(int param);

double dist(double a, double b);
double rad2Deg(double angle);
double deg2Rad(double angle);

#endif