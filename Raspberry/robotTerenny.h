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
#include <limits.h>

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

#define SENSORS_WIFI 1
#define SENSORS_PORT 1213

#define SYNC_MIN_TIME 1000 // 1 ms
//sensors
#define SYNC_POSSITION_TIME         SYNC_MIN_TIME*50
#define SYNC_ULTRASONIC_TIME        SYNC_MIN_TIME*50
#define SYNC_KINECTSENSORS_TIME     SYNC_MIN_TIME*50
#define SYNC_BUTTONS_TIME           SYNC_MIN_TIME*50
#define SYNC_VOLTAGE_TIME           SYNC_MIN_TIME*1000
//acculators
#define SYNC_KINECTMOTOR_TIME       SYNC_MIN_TIME*100
#define SYNC_LEDS_TIME              SYNC_MIN_TIME*2000
#define SYNC_KINECTLED_TIME         SYNC_MIN_TIME*2000
#define SYNC_MOTORS_TIME            SYNC_MIN_TIME*100


#define SYNC_MAP_GENERATE_TIME      0.3 //v sekundach
#define SYNC_OPERATOR_DETECT_TIME   0.3

#define ENABLE_I2C 1        //ok
#define ENABLE_MOTORS 1    //ok
#define ENABLE_ULTRASONIC 0 //ok
#define ENABLE_LEDS 1     //ok
#define ENABLE_BUTTONS 1    //ok
#define ENABLE_POSSITION 1  //ok
#define ENABLE_JORNEY_GENERATE 1
#define ENABLE_MAP_GENERATE 1
#define ENABLE_MAP_OBLIVION 1
#define ENABLE_OPERATOR_DETECT 1
#define ENABLE_KINECTMOTOR 1
#define ENABLE_KINECTLED 1
#define ENABLE_KINECTSENSORS 1
#define ENABLE_KINECTCAMERA 1
#define ENABLE_VOLTAGE 1

#define NUMBER_OF_MODULES (ENABLE_I2C+ENABLE_MOTORS+ENABLE_ULTRASONIC+ENABLE_LEDS+ENABLE_BUTTONS+ENABLE_POSSITION+ENABLE_KINECTCAMERA+ENABLE_KINECTSENSORS+ENABLE_KINECTLED+ENABLE_VOLTAGE)

#define OPERATOR_STATUS_KINECT_LED 1
#define OPERATOR_MINCENTER_DISTANCE_ORANGE_AND_GREEN_MASK_X 0
#define OPERATOR_MAXCENTER_DISTANCE_ORANGE_AND_GREEN_MASK_X 20
#define OPERATOR_MINCENTER_DISTANCE_ORANGE_AND_GREEN_MASK_Y 0
#define OPERATOR_MAXCENTER_DISTANCE_ORANGE_AND_GREEN_MASK_Y 100
#define OPERATOR_MIN_AREA_ORANGE 300
#define OPERATOR_MIN_AREA_GREEN 300
#define OPERATOR_MIN_AREA_DEPTH 300

#define SPEED_OF_MAP_OBLIVION 5
#define SPEED_OF_MAP_CREATION 10
#define MIN_BARRIER_HEIGHT 0.03 //[m]
#define MAX_BARRIER_HEIGHT 1.2 //[m]
#define MAP_WIDTH 200
#define MAP_HEIGHT 200
#define MAP_SCALE 0.05 //default [mm]
#define MAX_DELTA_TRANSLATE 100 //[mm]
#define MAP_DISTANCE_FROM_OPERATOR 500


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

#define VOLTAGE_MIN 19.5
#define VOLTAGE_MAX 25.2
#define K_VOLTAGE (100/(VOLTAGE_MAX-VOLTAGE_MIN))
#define Q_VOLTAGE (-VOLTAGE_MIN*K_VOLTAGE)

const Point sipkaHore = Point(0, -1);
const Point sipkaDole = Point(0, +1);
const Point sipkaVlavo = Point(-1, 0);
const Point sipkaVpravo = Point(+1, 0);
const Point sipkaVpravoHore = Point(+1, -1);
const Point sipkaVpravoDole = Point(+1, +1);
const Point sipkaVlavoHore = Point(-1, -1);
const Point sipkaVlavoDole =  Point(-1, +1);

typedef enum {
  CAMERA_VARIABLE_L,
  CAMERA_IMAGE_L1,
  CAMERA_IMAGE_L2,
  CAMERA_VARIABLE_R,
  CAMERA_IMAGE_R1,
  CAMERA_IMAGE_R2,
  MAT_KINECT1,
  MAT_KINECT2,
  MAT_KINECT_VARIABLE,
  ROBOTSENSORS,
  ROBOTACCULATORS,
  ROBOTACCULATORS_LAST,
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

struct Axis3d_struct {
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
  Axis3d_struct axisPossition;
  Angle3d_struct anglePossition;
};

struct Voltage_struct {
  double volts;
  double capacityPercent;
};

struct KinectSensor_struct {
  Angle3d_struct accAngle;
  Axis3d_struct accAxis;
  motorStatusKinect_t motorStatus;
};

struct RobotSensors {                 //struktura pre snimace aktualizovane s casom refresh hodnot pre jednotlive snimace
  KinectSensor_struct   kinect;         //kinect
  Buttons_struct        buttons;             //tlacidla
  RobotPosition_struct  robotPosition; //prepocitana pozicia
  Voltage_struct voltage;
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

//loger function
void charTag(log_tag_t tag, char *buffer);
void LOGError(log_tag_t tag, const char text[]);
void LOGInfo(log_tag_t tag, unsigned char priority, const char text[]);

//semafor function
int semInit(int sem_id, int sem_num, int val);
int semCreate(key_t key, int poc);
int semWait(int sem_id, semafor_name_t sem_num);
int semPost(int sem_id, semafor_name_t sem_num);
void semRem(int sem_id);

//init and deinit function for full robot
void initRobot(void);
void closeRobot(void);

//i2c communication
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

//kinect and operator images
bool initKinect(unsigned char imageMode);
int pauseGrabKinect();
void continueGrabKinect();
Mat getRGBKinect(int index = -1);
Mat getDepthScaledKinect(int index = -1);
Mat getDepthKinect(int index = -1);
Mat getDepthValidKinect(int index = -1);
Mat getPointCloudMapKinect(int index = -1);

Mat getMapImage(void);
Mat getRGBOperator(void);
Mat getDepthOperatorMask(void);
Mat getOrangeOperatorMask(void);
Mat getGreenOperatorMask(void);
Mat getHSVOperator(void);
Mat getImageLeft(void);
Mat getImageRight(void);

//main struct with input and output data
RobotAcculators getRobotAcculators(void);
void setRobotAcculators(RobotAcculators temp);
RobotSensors getRobotSensors(void);
RobotPosition_struct getRobotPossition(void);

//setter and getter values by i2c and another communication
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
Axis3d_struct getPossitionAxis(void);
Angle3d_struct getPossitionAngle3d(void);

void sendDoubleByWifi(double value);
void sendIntByWifi(int value);

//synchronization information function
void *syncImageLeft(void *arg);
void *syncImageRight(void *arg);
void *syncCameraNetworkConnection(void *arg);
void *syncSensorNetworkConnection(void *arg);
void *syncKinectFrames(void *arg);
void *syncUltrasonic(void *arg);
void *syncLeds(bool checkChange);
void *syncMotors(bool checkChange);
void *syncPossition(void *arg);
void *syncButtons(void *arg);
void *syncGenerateMap(void *arg);
void *syncOperatorDetect(void *arg);
void *syncKinectMotor(bool checkChange);
void *syncKinectLed(bool checkChange);
void *syncKinectSensors(void *arg);
void *syncModules(void *arg);
void *syncGenerateJorney(void *arg);

//waiting function
void *readKey(void*);
void *waitForCameraConnection(void *arg);
void *waitForSensorConnection(void *arg);
void closeCameraConnection(void);
void closeSensorConnection(void);

//other function
void sendMatImageByWifi(Mat img, int quality);
Mat translateImg(Mat &img, int offsetx, int offsety);
void sigctrl(int param);
void sigpipe(int param);
double dist(double a, double b);
double rad2Deg(double angle);
double deg2Rad(double angle);

#endif