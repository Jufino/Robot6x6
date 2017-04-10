#include "robotTerenny.h"

int i2cHandle;
unsigned char lastAddr = 0x00;
int sensorsServersock, cameraServersock;
int sensorsClientsock, cameraClientsock;
int portHandle;
int sem_id;
bool onAllThreads = true;
bool onWifiCameraStill = true;
bool onWifiSensorStill = true;

#define NUMBER_OF_PRIORITIES 3
bool priorityVisible[NUMBER_OF_PRIORITIES];

XN_USB_DEV_HANDLE dev;
VideoCapture cameraKinect;

char mapImageChoose = 0;
Mat mapImage1;
Mat mapImage2;
long mapOffsetX = 0;
long mapOffsetY = 0;

char pointCloudMapChooseKinect = 0;
Mat pointCloudMap1Kinect;
Mat pointCloudMap2Kinect;
char depthChooseKinect = 0;
Mat depth1Kinect;
Mat depth2Kinect;
char imageChooseKinect = 0;
Mat img1Kinect;
Mat img2Kinect;

#if CAMERA_WIFI == 1
char depthOperatorMaskChoose = 0;
Mat depthOperatorMaskImage1;
Mat depthOperatorMaskImage2;
char rgbOperatorChoose = 0;
Mat rgbOperatorImage1;
Mat rgbOperatorImage2;
char hsvOperatorChoose = 0;
Mat hsvOperatorImage1;
Mat hsvOperatorImage2;
char greenOperatorMaskChoose = 0;
Mat greenOperatorMaskImage1;
Mat greenOperatorMaskImage2;
char orangeOperatorMaskChoose = 0;
Mat orangeOperatorMaskImage1;
Mat orangeOperatorMaskImage2;
#endif

Point3f operatorPossition =  Point3f(-1, -1, -1);;

const float scaleFactor = 0.05f;

CvCapture* cameraL;
char imageChooseL = 0;
IplImage *img1L;
IplImage *img2L;

CvCapture* cameraR;
char imageChooseR = 0;
IplImage *img1R;
IplImage *img2R;

RobotAcculators robotAcculators;
RobotAcculators robotAcculatorsLast;
RobotSensors robotSensors;

int iLowH_green = 20;
int iHighH_green = 45;
int iLowS_green = 50;
int iHighS_green  = 255;
int iLowV_green  = 100;
int iHighV_green  = 255;
int iLowH_orange = 0;
int iHighH_orange = 25;
int iLowS_orange = 100;
int iHighS_orange  = 255;
int iLowV_orange  = 100;
int iHighV_orange  = 255;

typedef union {
  int val; /* Value for SETVAL */
  struct semid_ds *buf; /* Buffer for IPC_STAT, IPC_SET */
  unsigned short *array; /* Array for GETALL, SETALL */
  struct seminfo *__buf; /* Buffer for IPC_INFO
                                           (Linux-specific) */
} semun;

void charTag(log_tag_t tag, char *buffer) {
  switch (tag) {
  case SEMAFOR_TAG:
    sprintf(buffer, "SEMAFOR");
    break;
  case I2C_TAG:
    sprintf(buffer, "I2C");
    break;
  case KINECT_TAG:
    sprintf(buffer, "KINECT");
    break;
  case SENSOR_CONN_TAG:
    sprintf(buffer, "SENSOR_CONN");
    break;
  case CAMERA_CONN_TAG:
    sprintf(buffer, "CAMERA_CONN");
    break;
  case NUCLEO_TAG:
    sprintf(buffer, "NUCLEO");
    break;
  case MOTOR_TAG:
    sprintf(buffer, "MOTOR");
    break;
  case ROBOT_TAG:
    sprintf(buffer, "ROBOT");
    break;
  case OPERATOR_TAG:
    sprintf(buffer, "OPERATOR");
    break;
  case MAP_TAG:
    sprintf(buffer, "MAP");
    break;
  }
}

void LOGError(log_tag_t tag, const char text[]) {
#if ENABLE_LOG_ERROR == 1
  char buffer[255];
  char timeStr[20];

  time_t t = time(NULL);
  struct tm *tm = localtime(&t);
  strftime(timeStr, sizeof(timeStr), "%D %T", tm);

  charTag(tag, buffer);
  printf("%s - LOGError:%s/%s\n", timeStr, buffer, text);
#endif
}

void LOGInfo(log_tag_t tag, unsigned char priority, const char text[]) {
#if ENABLE_LOG_INFO == 1
  if (priorityVisible[priority]) {
    char buffer[255];
    char timeStr[20];

    time_t t = time(NULL);
    struct tm *tm = localtime(&t);
    strftime(timeStr, sizeof(timeStr), "%D %T", tm);

    charTag(tag, buffer);
    printf("%s - LOGInfo(%d):%s/%s\n", timeStr, priority, buffer, text);
  }
#endif
}

int semInit(int sem_id, int sem_num, int val) {
  semun un;
  un.val = val;
  return semctl(sem_id, sem_num, SETVAL, un);
}

int semCreate(key_t key, int poc) {
  int sem_id = 0;
  if ((sem_id = semget(key, poc, 0666 | IPC_CREAT)) < 0) {
    LOGError(SEMAFOR_TAG, "Problem create.");
    exit(-2);
  }
  return sem_id;
}

int semWait(int sem_id, semafor_name_t sem_num) {
  struct sembuf my;
  my.sem_num = sem_num;
  my.sem_op = -1;
  my.sem_flg = 0;

  return semop(sem_id, &my, 1);
}

int semPost(int sem_id, semafor_name_t sem_num) {
  struct sembuf my;
  my.sem_num = sem_num;
  my.sem_op = 1;
  my.sem_flg = 0;

  return semop(sem_id, &my, 1);
}

void semRem(int sem_id) {
  semctl(sem_id, 0, IPC_RMID, NULL);
}

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}


void initRobot(void) {
  priorityVisible[0] = true;
  priorityVisible[1] = false;
  priorityVisible[2] = false;

  sem_id = semCreate(getpid(), 37);
  semInit(sem_id, CAMERA_VARIABLE_L, 1);
  semInit(sem_id, CAMERA_IMAGE_L1, 1);
  semInit(sem_id, CAMERA_IMAGE_L2, 1);
  semInit(sem_id, CAMERA_VARIABLE_R, 1);
  semInit(sem_id, CAMERA_IMAGE_R1, 1);
  semInit(sem_id, CAMERA_IMAGE_R2, 1);
  semInit(sem_id, ROBOTSENSORS, 1);
  semInit(sem_id, ROBOTACCULATORS, 1);
  semInit(sem_id, ROBOTACCULATORS_LAST, 1);
  semInit(sem_id, CAMERA_DEPTH_KINECT1, 1);
  semInit(sem_id, CAMERA_DEPTH_KINECT2, 1);
  semInit(sem_id, CAMERA_IMAGE_KINECT1, 1);
  semInit(sem_id, CAMERA_IMAGE_KINECT2, 1);
  semInit(sem_id, CAMERA_POINTCLOUDMAP_KINECT1, 1);
  semInit(sem_id, CAMERA_POINTCLOUDMAP_KINECT2, 1);
  semInit(sem_id, CAMERA_VARIABLE_KINECTIMAGE, 1);
  semInit(sem_id, CAMERA_VARIABLE_KINECTDEPTH, 1);
  semInit(sem_id, CAMERA_VARIABLE_KINECTPOINTCLOUDMAP, 1);
  semInit(sem_id, MAP_VARIABLE, 1);
  semInit(sem_id, MAP_IMAGE1, 1);
  semInit(sem_id, MAP_IMAGE2, 1);
  semInit(sem_id, I2C, 1);
  semInit(sem_id, DEPTH_OPERATOR_MASK1, 1);
  semInit(sem_id, DEPTH_OPERATOR_MASK2, 1);
  semInit(sem_id, RGB_OPERATOR1, 1);
  semInit(sem_id, RGB_OPERATOR2, 1);
  semInit(sem_id, HSV_OPERATOR1, 1);
  semInit(sem_id, HSV_OPERATOR2, 1);
  semInit(sem_id, ORANGE_OPERATOR_MASK1, 1);
  semInit(sem_id, ORANGE_OPERATOR_MASK2, 1);
  semInit(sem_id, GREEN_OPERATOR_MASK1, 1);
  semInit(sem_id, GREEN_OPERATOR_MASK2, 1);
  semInit(sem_id, DEPTH_OPERATOR_MASK_VARIABLE, 1);
  semInit(sem_id, RGB_OPERATOR_VARIABLE, 1);
  semInit(sem_id, HSV_OPERATOR_VARIABLE, 1);
  semInit(sem_id, ORANGE_OPERATOR_MASK_VARIABLE, 1);
  semInit(sem_id, GREEN_OPERATOR_MASK_VARIABLE, 1);
  semInit(sem_id, OPERATOR_POSSITION, 1);

  mapImage1 = Mat(MAP_HEIGHT, MAP_WIDTH,  CV_8UC3, Scalar(0, 0, 0));
  mapImageChoose = 1;

  initMotorPowerSupply();
  setMotorPowerSupply(true);

  if (ENABLE_I2C) {
    if (!initI2C()) {
      LOGError(I2C_TAG, "Connection failed.");
      exit(0);
    }
    else {
      LOGInfo(I2C_TAG, 0, "Connection ok.");
    }
  }

  if (ENABLE_I2C && (ENABLE_LEDS || ENABLE_MOTORS || ENABLE_ULTRASONIC || ENABLE_BUTTONS || ENABLE_POSSITION)) {
    if (!nucleoTestConnection()) {
      LOGError(NUCLEO_TAG, "Connection failed.");
      exit(0);
    }
    else {
      LOGInfo(NUCLEO_TAG, 0, "Connection ok.");
    }
  }

  if (ENABLE_I2C && ENABLE_LEDS)
    setLeds(COLOR_OFF, COLOR_OFF, COLOR_OFF);

  if (ENABLE_I2C && ENABLE_MOTORS) {
    char testValue = motorsTestConnection();
    for (int i = 0; i < 6; i++) {
      if (testValue & (1 << i) && (1 << i)) {
        char buffer [50];
        sprintf (buffer, "Connection %d ok.", i + 1);
        LOGInfo(MOTOR_TAG, 0, buffer);
      }
      else {
        char buffer [50];
        sprintf (buffer, "Connection %d failed.", i + 1);
        LOGInfo(MOTOR_TAG, 0, buffer);
      }
    }

    setMove(STOP, 0);
    setMotorPowerSupply(true);
  }

  if (ENABLE_KINECTCAMERA) {
    if (!initKinect(0)) {
      LOGError(KINECT_TAG, "Connection camera failed.");
      exit(0);
    }
    else {
      LOGInfo(KINECT_TAG, 0, "Connection camera ok.");
    }
  }

  if (NUMBER_OF_CAMERA == 1 || NUMBER_OF_CAMERA == 2) {
    cameraL = cvCaptureFromCAM(INDEX_CAMERA_LEFT);
    cvSetCaptureProperty( cameraL, CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    cvSetCaptureProperty( cameraL, CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
  }

  if (NUMBER_OF_CAMERA == 2) {
    cameraR = cvCaptureFromCAM(INDEX_CAMERA_RIGHT);
    cvSetCaptureProperty( cameraR, CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    cvSetCaptureProperty( cameraR, CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
  }

  if (NUMBER_OF_CAMERA == 1 || NUMBER_OF_CAMERA == 2) {
    pthread_t threadImgL;
    pthread_create(&threadImgL, NULL, &syncImageLeft, NULL);
  }

  if (NUMBER_OF_CAMERA == 2) {
    pthread_t threadImgR;
    pthread_create(&threadImgR, NULL, &syncImageRight, NULL);
  }

  if (ENABLE_KINECTCAMERA == 1) {
    pthread_t threadImgAndDepthKinect;
    pthread_create(&threadImgAndDepthKinect, NULL, &syncKinectFrames, NULL);
  }

  if (ENABLE_KINECTMOTOR == 1 || ENABLE_KINECTLED == 1) {
    XnStatus rc = XN_STATUS_OK;
    rc = xnUSBInit();
    if (rc != XN_STATUS_OK) {
      LOGError(KINECT_TAG, xnGetStatusString(rc) );
    }
    rc = xnUSBOpenDevice(VID_MICROSOFT, PID_NUI_MOTOR, NULL, NULL, &dev);
    if (rc != XN_STATUS_OK) {
      LOGError(KINECT_TAG, xnGetStatusString(rc) );
    }
    else {
      LOGInfo(KINECT_TAG, 0, "Connection acculators ok.");
    }
  }

#if ENABLE_OPERATOR_DETECT == 1
  pthread_t threadOperator;
  pthread_create(&threadOperator, NULL, &syncOperatorDetect, NULL);
#endif
#if ENABLE_MAP_GENERATE == 1
  pthread_t threadGenerateMap;
  pthread_create(&threadGenerateMap, NULL, &syncGenerateMap, NULL);
#endif

  pthread_t threadModules;
  pthread_create(&threadModules, NULL, &syncModules, NULL);

  if (SENSORS_WIFI) {
    pthread_t waitForSensorConnectionThread;
    pthread_create(&waitForSensorConnectionThread, NULL, &waitForSensorConnection, NULL);
  }

  if (CAMERA_WIFI) {
    pthread_t waitForCameraConnectionThread;
    pthread_create(&waitForCameraConnectionThread, NULL, &waitForCameraConnection, NULL);
  }
  if (SENSORS_WIFI == 1 || CAMERA_WIFI == 1)   signal(SIGPIPE, sigpipe);

  signal(SIGINT, sigctrl);

}

void closeRobot(void) {
  LOGInfo(ROBOT_TAG, 0, "Closing...");
  onAllThreads = false;

  if (ENABLE_KINECTLED == 1 || ENABLE_KINECTMOTOR == 1) {
    XnStatus rc = xnUSBCloseDevice(dev);
    if (rc != XN_STATUS_OK) {
      LOGError(KINECT_TAG, xnGetStatusString(rc) );
    }
  }

  if (ENABLE_MOTORS) {
    setMove(STOP, 0);
  }
  if (ENABLE_I2C) {
    closeI2C();
  }

  semRem(sem_id);

  closeMotorPowerSupply();

  if (CAMERA_WIFI == 1 && onWifiCameraStill) closeCameraConnection();
  if (SENSORS_WIFI == 1 && onWifiSensorStill) closeSensorConnection();
}

void *waitForCameraConnection(void *arg) {
  struct sockaddr_in server1;
  if ((cameraServersock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    LOGError(CAMERA_CONN_TAG, "Socket() failed.");
    return NULL;
  }
  memset(&server1, 0, sizeof(server1));
  server1.sin_family = AF_INET;
  server1.sin_port = htons(CAMERA_PORT);
  server1.sin_addr.s_addr = INADDR_ANY;
  if (bind(cameraServersock, (struct sockaddr *)&server1, sizeof(server1)) == -1) {
    LOGError(CAMERA_CONN_TAG, "Bind() failed.");
    return NULL;
  }
  if (listen(cameraServersock, 10) == -1) {
    LOGError(CAMERA_CONN_TAG, "Listen() failed.");
    return NULL;
  }
  char buffer [50];
  sprintf (buffer, "Waiting for camera connection on port %d.", CAMERA_PORT);
  LOGInfo(CAMERA_CONN_TAG, 0, buffer);
  if ((cameraClientsock = accept(cameraServersock, NULL, NULL)) == -1) {
    LOGError(CAMERA_CONN_TAG, "Accept() failed.");
    return NULL;
  }
  sprintf (buffer, "Connection on port %d ok.", CAMERA_PORT);
  LOGInfo(CAMERA_CONN_TAG, 0, buffer);

  pthread_t vlaknoCamera;
  pthread_create(&vlaknoCamera, NULL, &syncCameraNetworkConnection, NULL);
  return NULL;
}

void *waitForSensorConnection(void *arg) {
  struct sockaddr_in server;
  if ((sensorsServersock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    LOGError(SENSOR_CONN_TAG, "Socket() failed.");
    return NULL;
  }
  memset(&server, 0, sizeof(server));
  server.sin_family = AF_INET;
  server.sin_port = htons(SENSORS_PORT);
  server.sin_addr.s_addr = INADDR_ANY;
  if (bind(sensorsServersock, (struct sockaddr *)&server, sizeof(server)) == -1) {
    LOGError(SENSOR_CONN_TAG, "Bind() failed.");
    return NULL;
  }
  if (listen(sensorsServersock, 10) == -1) {
    LOGError(SENSOR_CONN_TAG, "Listen() failed.");
    return NULL;
  }
  char buffer [50];
  sprintf (buffer, "Waiting for sensor connection on port %d.", SENSORS_PORT);
  LOGInfo(SENSOR_CONN_TAG, 0, buffer);
  if ((sensorsClientsock = accept(sensorsServersock, NULL, NULL)) == -1) {
    LOGError(SENSOR_CONN_TAG, "Accept() failed.");
    return NULL;
  }
  sprintf (buffer, "Connection on port %d ok.", SENSORS_PORT);
  LOGInfo(SENSOR_CONN_TAG, 0, buffer);

  pthread_t vlaknoSensor;
  pthread_create(&vlaknoSensor, NULL, &syncSensorNetworkConnection, NULL);
  return NULL;
}

bool initI2C(void) {
  if ((i2cHandle = open(PORT_I2C, O_RDWR)) < 0)
    return false;
  else
    return true;
}

void closeI2C(void) {
  close(i2cHandle);
}

char setDevice(unsigned char addr) {
  if (addr != lastAddr) {
    if (ioctl(i2cHandle, I2C_SLAVE, addr) < 0) {
      char buffer [50];
      sprintf (buffer, "SetDevice on address:0x%02x failed.", addr);
      LOGError(I2C_TAG, buffer);
      return -1;
    }
    lastAddr = addr;
  }
  return 0;
}

char writeRegisterAndValueU8(unsigned char addr, unsigned char reg, unsigned char value) {
  char errorTimeout = 0;
  unsigned char data[2];
  data[0] = reg;
  data[1] = value;
  if (setDevice(addr) == 0) {
    while (write(i2cHandle, data, 2) != 2) {
      char buffer [50];
      sprintf (buffer, "addr:%i, write register %i, errorTimeout:%i", (int)addr, (int)reg, (int)errorTimeout);
      LOGError(I2C_TAG, buffer);
      if (errorTimeout++ >= I2C_WRITE_TIMEOUT) break;
    }
    if (errorTimeout < I2C_WRITE_TIMEOUT)
      return 0;
    else {
      char buffer [50];
      sprintf (buffer, "addr:%i, write register %i,val %i, errorTimeout:%i\n", (int)addr, (int)reg, (int)value, (int)errorTimeout);
      LOGError(I2C_TAG, buffer);
    }
  }
  return -1;
}

char writeRegisterAndValueU16(unsigned char addr, unsigned char reg, unsigned int value) {
  char errorTimeout = 0;
  unsigned char data[3];
  data[0] = reg;
  data[1] = (value >> 8) & 0xFF;
  data[2] = value & 0xFF;
  if (setDevice(addr) == 0) {
    while (write(i2cHandle, data, 3) != 3) {
      char buffer [50];
      sprintf (buffer, "addr:%i, write register %i, errorTimeout:%i", (int)addr, (int)reg, (int)errorTimeout);
      LOGError(I2C_TAG, buffer);
      if (errorTimeout++ >= I2C_WRITE_TIMEOUT) break;
    }
    if (errorTimeout < I2C_WRITE_TIMEOUT)
      return 0;
    else {
      char buffer [50];
      sprintf (buffer, "addr:%i, write register %i,val %i, errorTimeout:%i", (int)addr, (int)reg, (int)value, (int)errorTimeout);
      LOGError(I2C_TAG, buffer);
    }
  }
  return -1;
}

char writeRegisterAndValueS16(unsigned char addr, unsigned char reg, int value) {
  char errorTimeout = 0;
  unsigned char data[3];
  data[0] = reg;
  data[1] = (value >> 8) & 0xFF;
  data[2] = value & 0xFF;
  if (setDevice(addr) == 0) {
    while (write(i2cHandle, data, 3) != 3) {
      char buffer [50];
      sprintf (buffer, "addr:%i, write register %i, errorTimeout:%i", (int)addr, (int)reg, (int)errorTimeout);
      LOGError(I2C_TAG, buffer);
      if (errorTimeout++ >= I2C_WRITE_TIMEOUT) break;
    }
    if (errorTimeout < I2C_WRITE_TIMEOUT)
      return 0;
    else {
      char buffer [50];
      sprintf (buffer, "addr:%i, write register %i,val %i, errorTimeout:%i", (int)addr, (int)reg, (int)value, (int)errorTimeout);
      LOGError(I2C_TAG, buffer);
    }
  }
  return -1;
}

char writeRegister(unsigned char addr, unsigned char reg) {
  char errorTimeout = 0;
  unsigned char data[1];
  data[0] = reg;
  if (setDevice(addr) == 0) {
    while (write(i2cHandle, data, 1) != 1) {
      char buffer [50];
      sprintf (buffer, "addr:%i, write register %i, errorTimeout:%i", (int)addr, (int)reg, (int)errorTimeout);
      LOGError(I2C_TAG, buffer);
      if (errorTimeout++ >= I2C_WRITE_TIMEOUT) break;
    }
    if (errorTimeout < I2C_WRITE_TIMEOUT)
      return 0;
    else {
      char buffer [50];
      sprintf (buffer, "addr:%i, write register %i, errorTimeout:%i", (int)addr, (int)reg, (int)errorTimeout);
      LOGError(I2C_TAG, buffer);
    }
  }
  return -1;
}

unsigned int readRegister16(unsigned char addr, unsigned char reg) {
  if (writeRegister(addr, reg) == 0) {
    char data[2];

    if (read(i2cHandle, data, 2) != 2) {
      char buffer [50];
      sprintf (buffer, "addr:%i, read register %i", (int)addr, (int)reg);
      LOGError(I2C_TAG, buffer);
    }

    return (data[0] << 8) | (data[1] & 0xFF);
  }
  else return 0;
}

signed int readRegister16s(unsigned char addr, unsigned char reg) {
  if (writeRegister(addr, reg) == 0) {
    signed char data[2];

    if (read(i2cHandle, data, 2) != 2) {
      char buffer [50];
      sprintf (buffer, "addr:%i, read register %i", (int)addr, (int)reg);
      LOGError(I2C_TAG, buffer);
    }

    return (data[0] << 8) | (data[1] & 0xFF);
  }
  else return 0;
}

signed long readRegister32s(unsigned char addr, unsigned char reg) {
  if (writeRegister(addr, reg) == 0) {
    signed char data[4];

    if (read(i2cHandle, data, 4) != 4) {
      char buffer [50];
      sprintf (buffer, "addr:%i, read register %i", (int)addr, (int)reg);
      LOGError(I2C_TAG, buffer);
    }

    return (((data[0] & 0xFF) << 24) | ((data[1] & 0xFF) << 16) | ((data[2] & 0xFF) << 8) | (data[3] & 0xFF));
  }
  else return 0;
}

unsigned char readRegister8(unsigned char addr, unsigned char reg) {
  if (writeRegister(addr, reg) == 0) {
    signed char data[1];

    if (read(i2cHandle, data, 1) != 1) {
      char buffer [50];
      sprintf (buffer, "addr:%i, read register %i", (int)addr, (int)reg);
      LOGError(I2C_TAG, buffer);
    }

    return data[0];
  }
  else return 0;
}

bool initKinect(unsigned char imageMode) {
  cameraKinect.open( CAP_OPENNI2 );
  if ( !cameraKinect.isOpened() )
    cameraKinect.open( CAP_OPENNI );

  if ( !cameraKinect.isOpened() )
  {
    return false;
  }
  bool modeRes = false;
  switch ( imageMode )
  {
  case 0:
    modeRes = cameraKinect.set( CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CAP_OPENNI_VGA_30HZ );
    break;
  case 1:
    modeRes = cameraKinect.set( CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CAP_OPENNI_SXGA_15HZ );
    break;
  case 2:
    modeRes = cameraKinect.set( CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CAP_OPENNI_SXGA_30HZ );
    break;
  case 3:
    modeRes = cameraKinect.set( CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CAP_OPENNI_QVGA_30HZ );
    break;
  case 4:
    modeRes = cameraKinect.set( CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CAP_OPENNI_QVGA_60HZ );
    break;
  default:
    LOGError(KINECT_TAG, "Unsupported image mode property.");
  }
  if (!modeRes)
    LOGError(KINECT_TAG, "This image mode is not supported by the device, the default value (CV_CAP_OPENNI_SXGA_15HZ) will be used.");

  char buffer [150];
  sprintf (buffer, "\nDepth generator output mode:\nFRAME_WIDTH\t:%f\nFRAME_HEIGHT\t:%f\nFRAME_MAX_DEPTH\t:%f mm\nFPS\t\t:%f\nREGISTRATION\t:%f", cameraKinect.get( CAP_PROP_FRAME_WIDTH ), cameraKinect.get( CAP_PROP_FRAME_HEIGHT ), cameraKinect.get( CAP_PROP_OPENNI_FRAME_MAX_DEPTH ), cameraKinect.get( CAP_PROP_FPS ), cameraKinect.get( CAP_PROP_OPENNI_REGISTRATION ) );
  LOGInfo(KINECT_TAG, 0, buffer);

  if ( cameraKinect.get( CAP_OPENNI_IMAGE_GENERATOR_PRESENT ) )
  {
    sprintf (buffer, "\nImage generator output mode:\nFRAME_WIDTH\t:%f\nFRAME_HEIGHT\t:%f\nFPS\t:%f", cameraKinect.get( CAP_OPENNI_IMAGE_GENERATOR + CAP_PROP_FRAME_WIDTH ), cameraKinect.get( CAP_OPENNI_IMAGE_GENERATOR + CAP_PROP_FRAME_HEIGHT ), cameraKinect.get( CAP_OPENNI_IMAGE_GENERATOR + CAP_PROP_FPS ));
    LOGInfo(KINECT_TAG, 0, buffer);
  }

  return true;
}

Mat getMapImage(void) {
  Mat mapImageMat;
  char mapImageChooseMain;
  semWait(sem_id, MAP_VARIABLE);
  mapImageChooseMain = mapImageChoose;
  semPost(sem_id, MAP_VARIABLE);
  if (mapImageChooseMain == 1) {
    semWait(sem_id, MAP_IMAGE1);
    mapImageMat = mapImage1.clone();
    semPost(sem_id, MAP_IMAGE1);
  }
  else if (mapImageChooseMain == 2) {
    semWait(sem_id, MAP_IMAGE2);
    mapImageMat = mapImage2.clone();
    semPost(sem_id, MAP_IMAGE2);
  }
  return mapImageMat;
}

Mat getImageKinect(void) {
  Mat imgMatKinect;
  char imageChooseMainKinect;
  semWait(sem_id, CAMERA_VARIABLE_KINECTIMAGE);
  imageChooseMainKinect = imageChooseKinect;
  semPost(sem_id, CAMERA_VARIABLE_KINECTIMAGE);
  if (imageChooseMainKinect == 1) {
    semWait(sem_id, CAMERA_IMAGE_KINECT1);
    imgMatKinect = img1Kinect.clone();
    semPost(sem_id, CAMERA_IMAGE_KINECT1);
  }
  else if (imageChooseMainKinect == 2) {
    semWait(sem_id, CAMERA_IMAGE_KINECT2);
    imgMatKinect = img2Kinect.clone();
    semPost(sem_id, CAMERA_IMAGE_KINECT2);
  }
  return imgMatKinect;
}

Mat getRGBOperator(void) {
  Mat imgMat;
  char imageChooseMain;
  semWait(sem_id, RGB_OPERATOR_VARIABLE);
  imageChooseMain = rgbOperatorChoose;
  semPost(sem_id, RGB_OPERATOR_VARIABLE);
  if (imageChooseMain == 1) {
    semWait(sem_id, RGB_OPERATOR1);
    imgMat = rgbOperatorImage1.clone();
    semPost(sem_id, RGB_OPERATOR1);
  }
  else if (imageChooseMain == 2) {
    semWait(sem_id, RGB_OPERATOR2);
    imgMat = rgbOperatorImage2.clone();
    semPost(sem_id, RGB_OPERATOR2);
  }
  return imgMat;
}
Mat getDepthOperatorMask(void) {
  Mat imgMat;
  char imageChooseMain;
  semWait(sem_id, DEPTH_OPERATOR_MASK_VARIABLE);
  imageChooseMain = depthOperatorMaskChoose;
  semPost(sem_id, DEPTH_OPERATOR_MASK_VARIABLE);
  if (imageChooseMain == 1) {
    semWait(sem_id, DEPTH_OPERATOR_MASK1);
    imgMat = depthOperatorMaskImage1.clone();
    semPost(sem_id, DEPTH_OPERATOR_MASK1);
  }
  else if (imageChooseMain == 2) {
    semWait(sem_id, DEPTH_OPERATOR_MASK2);
    imgMat = depthOperatorMaskImage2.clone();
    semPost(sem_id, DEPTH_OPERATOR_MASK2);
  }
  return imgMat;
}

Mat getOrangeOperatorMask(void) {
  Mat imgMat;
  char imageChooseMain;
  semWait(sem_id, ORANGE_OPERATOR_MASK_VARIABLE);
  imageChooseMain = orangeOperatorMaskChoose;
  semPost(sem_id, ORANGE_OPERATOR_MASK_VARIABLE);
  if (imageChooseMain == 1) {
    semWait(sem_id, ORANGE_OPERATOR_MASK1);
    imgMat = orangeOperatorMaskImage1.clone();
    semPost(sem_id, ORANGE_OPERATOR_MASK1);
  }
  else if (imageChooseMain == 2) {
    semWait(sem_id, ORANGE_OPERATOR_MASK2);
    imgMat = orangeOperatorMaskImage2.clone();
    semPost(sem_id, ORANGE_OPERATOR_MASK2);
  }
  return imgMat;
}

Mat getGreenOperatorMask(void) {
  Mat imgMat;
  char imageChooseMain;
  semWait(sem_id, GREEN_OPERATOR_MASK_VARIABLE);
  imageChooseMain = greenOperatorMaskChoose;
  semPost(sem_id, GREEN_OPERATOR_MASK_VARIABLE);
  if (imageChooseMain == 1) {
    semWait(sem_id, GREEN_OPERATOR_MASK1);
    imgMat = greenOperatorMaskImage1.clone();
    semPost(sem_id, GREEN_OPERATOR_MASK1);
  }
  else if (imageChooseMain == 2) {
    semWait(sem_id, GREEN_OPERATOR_MASK2);
    imgMat = greenOperatorMaskImage2.clone();
    semPost(sem_id, GREEN_OPERATOR_MASK2);
  }
  return imgMat;
}

Mat getHSVOperator(void) {
  Mat imgMat;
  char imageChooseMain;
  semWait(sem_id, HSV_OPERATOR_VARIABLE);
  imageChooseMain = hsvOperatorChoose;
  semPost(sem_id, HSV_OPERATOR_VARIABLE);
  if (imageChooseMain == 1) {
    semWait(sem_id, HSV_OPERATOR1);
    imgMat = hsvOperatorImage1.clone();
    semPost(sem_id, HSV_OPERATOR1);
  }
  else if (imageChooseMain == 2) {
    semWait(sem_id, HSV_OPERATOR2);
    imgMat = hsvOperatorImage2.clone();
    semPost(sem_id, HSV_OPERATOR2);
  }
  return imgMat;
}

Mat getDepthKinect(void) {
  Mat depthMatKinect;
  char depthChooseMainKinect;
  semWait(sem_id, CAMERA_VARIABLE_KINECTDEPTH);
  depthChooseMainKinect = depthChooseKinect;
  semPost(sem_id, CAMERA_VARIABLE_KINECTDEPTH);
  if (depthChooseMainKinect == 1) {
    semWait(sem_id, CAMERA_DEPTH_KINECT1);
    depthMatKinect = depth1Kinect.clone();
    semPost(sem_id, CAMERA_DEPTH_KINECT1);
  }
  else if (depthChooseMainKinect == 2) {
    semWait(sem_id, CAMERA_DEPTH_KINECT2);
    depthMatKinect = depth2Kinect.clone();
    semPost(sem_id, CAMERA_DEPTH_KINECT2);
  }
  return depthMatKinect;
}

Mat getPointCloudMapKinect(void) {
  Mat pointCloudMapMatKinect;
  char pointCloudMapChooseMainKinect;
  semWait(sem_id, CAMERA_VARIABLE_KINECTPOINTCLOUDMAP);
  pointCloudMapChooseMainKinect = pointCloudMapChooseKinect;
  semPost(sem_id, CAMERA_VARIABLE_KINECTPOINTCLOUDMAP);
  if (pointCloudMapChooseMainKinect == 1) {
    semWait(sem_id, CAMERA_POINTCLOUDMAP_KINECT1);
    pointCloudMapMatKinect = pointCloudMap1Kinect.clone();
    semPost(sem_id, CAMERA_POINTCLOUDMAP_KINECT1);
  }
  else if (pointCloudMapChooseMainKinect == 2) {
    semWait(sem_id, CAMERA_POINTCLOUDMAP_KINECT2);
    pointCloudMapMatKinect = pointCloudMap2Kinect.clone();
    semPost(sem_id, CAMERA_POINTCLOUDMAP_KINECT2);
  }
  return pointCloudMapMatKinect;
}

Mat getImageLeft(void) {
  Mat imgMatL;
  char imageChooseMainL;
  semWait(sem_id, CAMERA_VARIABLE_L);
  imageChooseMainL = imageChooseL;
  semPost(sem_id, CAMERA_VARIABLE_L);
  if (imageChooseMainL == 1) {
    semWait(sem_id, CAMERA_IMAGE_L1);
    semWait(sem_id, ROBOTSENSORS);
    imgMatL = cvarrToMat(img1L);
    semPost(sem_id, ROBOTSENSORS);
    semPost(sem_id, CAMERA_IMAGE_L1);
  }
  else if (imageChooseMainL == 2) {
    semWait(sem_id, CAMERA_IMAGE_L2);
    semWait(sem_id, ROBOTSENSORS);
    imgMatL = cvarrToMat(img2L);
    semPost(sem_id, ROBOTSENSORS);
    semPost(sem_id, CAMERA_IMAGE_L2);
  }
  return imgMatL;
}

Mat getImageRight(void) {
  Mat imgMatR;
  char imageChooseMainR;
  semWait(sem_id, CAMERA_VARIABLE_R);
  imageChooseMainR = imageChooseR;
  semPost(sem_id, CAMERA_VARIABLE_R);
  if (imageChooseMainR == 1) {
    semWait(sem_id, CAMERA_IMAGE_R1);
    imgMatR = cvarrToMat(img1R);
    semPost(sem_id, CAMERA_IMAGE_R1);
  }
  else if (imageChooseMainR == 2) {
    semWait(sem_id, CAMERA_IMAGE_R2);
    imgMatR = cvarrToMat(img2R);
    semPost(sem_id, CAMERA_IMAGE_R2);
  }
  return imgMatR;
}

RobotAcculators getRobotAcculators(void) {
  RobotAcculators temp;
  semWait(sem_id, ROBOTACCULATORS);
  memcpy(&temp, &robotAcculators, sizeof(RobotAcculators));
  semPost(sem_id, ROBOTACCULATORS);
  return temp;
}

void setRobotAcculators(RobotAcculators temp) {
  semWait(sem_id, ROBOTACCULATORS);
  memcpy(&robotAcculators, &temp, sizeof(RobotAcculators));
  semPost(sem_id, ROBOTACCULATORS);
}

RobotSensors getRobotSensors(void) {
  RobotSensors temp;
  semWait(sem_id, ROBOTSENSORS);
  memcpy(&temp, &robotSensors, sizeof(RobotSensors));
  semPost(sem_id, ROBOTSENSORS);
  return temp;
}

RobotPosition_struct getRobotPossition(void) {
  RobotPosition_struct temp;
  semWait(sem_id, ROBOTSENSORS);
  memcpy(&temp, &robotSensors.robotPosition, sizeof(RobotPosition_struct));
  semPost(sem_id, ROBOTSENSORS);
  return temp;
}

void sendMatImageByWifi(Mat img, int quality) {
  vector<uchar> buff;
  vector<int> param = vector<int>(2);
  param[0] = 1;
  param[1] = quality;
  imencode(".jpg", img, buff, param);
  char len[21];
  sprintf(len, "%.20d", buff.size());
  send(cameraClientsock, len, strlen(len), 0);
  send(cameraClientsock, &buff[0], buff.size(), 0);
  buff.clear();
}

void sendDoubleByWifi(double value) {
  char buff[30];
  sprintf(buff, "%lf", value);
  int size = 30;
  for (int i = 0; i < 30; i++) {
    if (buff[i] == '\0') {
      size = i;
      break;
    }
  }
  char len[21];
  sprintf(len, "%.20d", size);
  send(sensorsClientsock, len, strlen(len), 0);
  send(sensorsClientsock, &buff[0], size, 0);
}

void sendIntByWifi(int value) {
  char buff[30];
  sprintf(buff, "%d", value);
  int size = 30;
  for (int i = 0; i < 30; i++) {
    if (buff[i] == '\0') {
      size = i;
      break;
    }
  }
  char len[21];
  sprintf(len, "%.20d", size);
  send(sensorsClientsock, len, strlen(len), 0);
  send(sensorsClientsock, &buff[0], size, 0);
}

void closeCameraConnection(void) {
  LOGInfo(CAMERA_CONN_TAG, 0, "Closing...");
  close(cameraServersock);
  close(cameraClientsock);
}

void closeSensorConnection(void) {
  LOGInfo(SENSOR_CONN_TAG, 0, "Closing...");
  close(sensorsServersock);
  close(sensorsClientsock);
}

void initMotorPowerSupply(void) {
  gpio_open(26, 1);
}

void setMotorPowerSupply(bool state) {
  if (state == false)  gpio_write(26, 1);
  else                 gpio_write(26, 0);
}

void closeMotorPowerSupply(void) {
  gpio_close(26);
}

bool nucleoTestConnection(void) {
  semWait(sem_id, I2C);
  bool value = (STM32_ADDRESS == readRegister8(STM32_ADDRESS, 255));
  semPost(sem_id, I2C);
  return value;
}

char motorsTestConnection(void) {
  return readRegister8(STM32_ADDRESS, 254);
}

void setServo(int angle) {
  semWait(sem_id, I2C);
  writeRegisterAndValueU8(STM32_ADDRESS, 13, angle);
  semPost(sem_id, I2C);
}

void setMove(direction_t direction, unsigned int speed) {
  semWait(sem_id, I2C);
  switch (direction) {
  case FORWARD:
    writeRegisterAndValueU16(STM32_ADDRESS, 7, speed);
    break;
  case BACKWARD:
    writeRegisterAndValueU16(STM32_ADDRESS, 8, speed);
    break;
  case CLOCKWISE:
    writeRegisterAndValueU16(STM32_ADDRESS, 9, speed);
    break;
  case ANTICLOCKWISE:
    writeRegisterAndValueU16(STM32_ADDRESS, 10, speed);
    break;
  default:
    writeRegister(STM32_ADDRESS, 11);
  }
  semPost(sem_id, I2C);
}

void setLeds(color_t ledUpColor, color_t ledMiddleColor, color_t ledDownColor) { //ok
  uint8_t dataLed = 0;
  switch (ledDownColor) {
  case COLOR_GREEN:
    dataLed |= (1 << 1);
    break;
  case COLOR_RED:
    dataLed |= (1 << 2);
    break;
  case COLOR_ORANGE:
    dataLed |= (1 << 1) | (1 << 2);
    break;
  case COLOR_OFF:
    break;
  }
  switch (ledMiddleColor) {
  case COLOR_GREEN:
    dataLed |= (1 << 3);
    break;
  case COLOR_RED:
    dataLed |= (1 << 4);
    break;
  case COLOR_ORANGE:
    dataLed |= (1 << 3) | (1 << 4);
    break;
  case COLOR_OFF:
    break;
  }
  switch (ledUpColor) {
  case COLOR_GREEN:
    dataLed |= (1 << 5);
    break;
  case COLOR_RED:
    dataLed |= (1 << 6);
    break;
  case COLOR_ORANGE:
    dataLed |= (1 << 5) | (1 << 6);
    break;
  case COLOR_OFF:
    break;
  }
  semWait(sem_id, I2C);
  writeRegisterAndValueU8(STM32_ADDRESS, 12, dataLed);
  semPost(sem_id, I2C);
}

unsigned int getUltrasonicRaw(void) {
  semWait(sem_id, I2C);
  unsigned int value = readRegister16(STM32_ADDRESS, 107);
  semPost(sem_id, I2C);
  return value;
}

double getUltrasonic(void) {
  return (double)getUltrasonicRaw() / CONST_ULTRASONIC;
}

unsigned char getButtons(void) {
  semWait(sem_id, I2C);
  unsigned char value = readRegister8(STM32_ADDRESS, 106);
  semPost(sem_id, I2C);
  return value;
}

Axis3d_struct getPossitionAxis(void) {
  Axis3d_struct axis;
  semWait(sem_id, I2C);
  axis.x = readRegister32s(STM32_ADDRESS, 100);
  axis.y = readRegister32s(STM32_ADDRESS, 101);
  axis.z = readRegister32s(STM32_ADDRESS, 102);
  semPost(sem_id, I2C);
  return axis;
}

Voltage_struct getVoltage(void) {
  Voltage_struct voltage;
  semWait(sem_id, I2C);
  voltage.volts = ((float)readRegister16s(STM32_ADDRESS, 108)) / 100;
  voltage.capacityPercent = voltage.volts * K_VOLTAGE + Q_VOLTAGE;
  if (voltage.capacityPercent < 0) voltage.capacityPercent = 0;
  else if (voltage.capacityPercent > 100) voltage.capacityPercent = 100;
  semPost(sem_id, I2C);
  return voltage;
}

Angle3d_struct getPossitionAngle3d(void) {
  Angle3d_struct angle3d;
  semWait(sem_id, I2C);
  angle3d.roll = ((double)readRegister16s(STM32_ADDRESS, 103)) / 10000;
  angle3d.pitch = ((double)readRegister16s(STM32_ADDRESS, 104)) / 10000;
  angle3d.yaw = ((double)readRegister16(STM32_ADDRESS, 105)) / 10000;
  semPost(sem_id, I2C);
  return angle3d;
}

void *syncImageLeft(void *arg) {
  while (onAllThreads && cvWaitKey(10) < 0) {
    //nacitanie obrazka z lavej kamery
    semWait(sem_id, CAMERA_IMAGE_L1);
    img1L = cvQueryFrame(cameraL);
    semWait(sem_id, CAMERA_VARIABLE_L);
    imageChooseL = 1;
    semPost(sem_id, CAMERA_VARIABLE_L);
    semPost(sem_id, CAMERA_IMAGE_L1);

    cvWaitKey(10);

    semWait(sem_id, CAMERA_IMAGE_L2);
    img2L = cvQueryFrame(cameraL);
    semWait(sem_id, CAMERA_VARIABLE_L);
    imageChooseL = 2;
    semPost(sem_id, CAMERA_VARIABLE_L);
    semPost(sem_id, CAMERA_IMAGE_L2);
  }
  return NULL;
}

void *syncImageRight(void *arg) {
  while (onAllThreads && cvWaitKey(10) < 0) {
    semWait(sem_id, CAMERA_IMAGE_R1);
    img1R = cvQueryFrame(cameraR);
    semWait(sem_id, CAMERA_VARIABLE_R);
    imageChooseR = 1;
    semPost(sem_id, CAMERA_VARIABLE_R);
    semPost(sem_id, CAMERA_IMAGE_R1);

    cvWaitKey(10);

    semWait(sem_id, CAMERA_IMAGE_R2);
    img2R = cvQueryFrame(cameraR);
    semWait(sem_id, CAMERA_VARIABLE_R);
    imageChooseR = 2;
    semPost(sem_id, CAMERA_VARIABLE_R);
    semPost(sem_id, CAMERA_IMAGE_R2);
  }
  return NULL;
}

void *syncCameraNetworkConnection(void *arg) {
  int bytes = 50;
  char recvdata[50];
  LOGInfo(CAMERA_CONN_TAG, 1, "Start:Camera sync by network.");
  while (bytes != 0 && onAllThreads && onWifiCameraStill) {
    bytes = recv(cameraClientsock, recvdata, 20, 0);
    if (bytes == 0) {
      onWifiCameraStill = false;
      closeCameraConnection();
      onWifiCameraStill = true;
      sleep(2);
      pthread_t waitForCameraConnectionThread;
      pthread_create(&waitForCameraConnectionThread, NULL, &waitForCameraConnection, NULL);

      break;
    }
    /*    char buffer [50];
        sprintf (buffer, "recv data : %s", recvdata);
        for (int i = 0; i < 50; i++) {
          if (buffer[i] == '\n') {
            buffer[i] = '\0';
            break;
          }
        }
        LOGInfo(CAMERA_CONN_TAG, 2, buffer);*/
    if (strcmp(recvdata, "imgL\n") == 0) {
      LOGInfo(CAMERA_CONN_TAG, 1, "RGB img left sync.");
      sendMatImageByWifi(getImageLeft(), 80);
    }
    else if (strcmp(recvdata, "imgR\n") == 0) {
      LOGInfo(CAMERA_CONN_TAG, 1, "RGB img right sync.");
      sendMatImageByWifi(getImageRight(), 80);
    }
    else if (strcmp(recvdata, "imgK\n") == 0) {
      LOGInfo(CAMERA_CONN_TAG, 1, "RGB img kinect sync.");
      sendMatImageByWifi(getImageKinect(), 80);
    }
    else if (strcmp(recvdata, "depK\n") == 0) {
      LOGInfo(CAMERA_CONN_TAG, 1, "Depth img kinect sync.");
      sendMatImageByWifi(getDepthKinect(), 80);
    }
    else if (strcmp(recvdata, "map\n") == 0) {
      LOGInfo(CAMERA_CONN_TAG, 1, "Map sync.");
      Mat map = getMapImage().clone();
      RobotPosition_struct robotPosition = getRobotPossition();
      int centerX = MAP_WIDTH / 2;
      int centerY = MAP_HEIGHT / 2;
      circle(map, Point(centerX, centerY), 40, Scalar( 0, 0, 255 ), 1, 8); // not visible zone
      circle(map, Point(centerX, centerY), 28, Scalar( 0, 255, 255 ), 1, 8); // robot zone
      semWait(sem_id, OPERATOR_POSSITION);
      double minPointX = operatorPossition.x * 100;
      double minPointZ = operatorPossition.z * 100;
      double r = sqrt((double)(minPointX * minPointX + minPointZ * minPointZ));
      double angle = acos((-(double)minPointX) / r) - 3.14 / 2;

      int x2 = r * cos(robotPosition.anglePossition.yaw + angle) + MAP_WIDTH / 2;
      int z2 = r * sin(robotPosition.anglePossition.yaw + angle) + MAP_HEIGHT / 2;
      if (operatorPossition.x != -1 && operatorPossition.y != -1)
        circle(map, Point(x2, z2), 5, Scalar( 0, 255, 255 ), -1, 8);
      semPost(sem_id, OPERATOR_POSSITION);
      line(map, Point(centerX, centerY), Point(centerX + 28 * cos(robotPosition.anglePossition.yaw), centerY + 28 * sin(robotPosition.anglePossition.yaw)), Scalar( 0, 255, 255 ), 1, 8);
      sendMatImageByWifi(map, 80);
    }
    else if (strcmp(recvdata, "rgbO\n") == 0) {
      LOGInfo(CAMERA_CONN_TAG, 1, "RGB operator sync.");
      sendMatImageByWifi(getRGBOperator(), 80);
    }
    else if (strcmp(recvdata, "hsvO\n") == 0) {
      LOGInfo(CAMERA_CONN_TAG, 1, "HSV operator sync.");
      sendMatImageByWifi(getHSVOperator(), 80);
    }
    else if (strcmp(recvdata, "greenO\n") == 0) {
      LOGInfo(CAMERA_CONN_TAG, 1, "Green mask sync.");
      sendMatImageByWifi(getGreenOperatorMask(), 80);
    }
    else if (strcmp(recvdata, "orangeO\n") == 0) {
      LOGInfo(CAMERA_CONN_TAG, 1, "Orange mask sync.");
      sendMatImageByWifi(getOrangeOperatorMask(), 80);
    }
    else if (strcmp(recvdata, "depthO\n") == 0) {
      LOGInfo(CAMERA_CONN_TAG, 1, "Depth mask sync.");
      sendMatImageByWifi(getDepthOperatorMask(), 80);
    }

    sleep(0.005);
  }
  LOGInfo(CAMERA_CONN_TAG, 1, "End:Camera sync by network.");

  return NULL;
}

#define maxBufferRecv 50
void *syncSensorNetworkConnection(void *arg) {
  LOGInfo(SENSOR_CONN_TAG, 1, "Start:Sensor sync by network.");
  char recvdata[maxBufferRecv];
  char command[maxBufferRecv / 2];
  char value[maxBufferRecv / 2];
  int bytes = 1;
  while (bytes != 0 && onAllThreads && onWifiSensorStill) {
    int indexValue = -1;
    bytes = recv(sensorsClientsock, recvdata, maxBufferRecv, 0);
    if (bytes == 0) {
      onWifiSensorStill = false;
      closeSensorConnection();
      onWifiSensorStill = true;
      sleep(2);
      pthread_t waitForSensorConnectionThread;
      pthread_create(&waitForSensorConnectionThread, NULL, &waitForSensorConnection, NULL);
      break;
    }

    for (int i = 0; i < maxBufferRecv; i++) {
      if ((recvdata[i] == '\n' || recvdata[i] == '\0')  && indexValue == -1) {
        command[i] = '\0';
        break;
      }
      else if (recvdata[i] != ';' && indexValue == -1) {
        command[i] = recvdata[i];
      }
      else if (recvdata[i] == ';' && indexValue == -1) {
        command[i] = '\0';
        indexValue = i + 1;
      }
      else if ((recvdata[i] == '\n' || recvdata[i] == '\0') && indexValue != -1) {
        value[i - indexValue] = '\0';
        break;
      }
      else if (recvdata[i] != '\n' && indexValue != 0) {
        value[i - indexValue] = recvdata[i];
      }
    }

    LOGInfo(SENSOR_CONN_TAG, 2, command);
    LOGInfo(SENSOR_CONN_TAG, 2, value);

    if (strcmp(command, "butt") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Button sync.");
      int valueButtons = 0;
      semWait(sem_id, ROBOTSENSORS);
      if (robotSensors.buttons.buttonUp) valueButtons += 1;
      if (robotSensors.buttons.buttonMiddle) valueButtons += 2;
      if (robotSensors.buttons.buttonDown) valueButtons += 4;
      semPost(sem_id, ROBOTSENSORS);
      sendIntByWifi(valueButtons);
    }
    else if (strcmp(command, "roll") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Roll sync.");
      semWait(sem_id, ROBOTSENSORS);
      double angle = robotSensors.robotPosition.anglePossition.roll;
      semPost(sem_id, ROBOTSENSORS);
      sendDoubleByWifi(angle);
    }
    else if (strcmp(command, "pitch") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Pitch sync.");
      semWait(sem_id, ROBOTSENSORS);
      double angle = robotSensors.robotPosition.anglePossition.pitch;
      semPost(sem_id, ROBOTSENSORS);
      sendDoubleByWifi(angle);
    }
    else if (strcmp(command, "yaw") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Yaw sync.");
      semWait(sem_id, ROBOTSENSORS);
      double angle = robotSensors.robotPosition.anglePossition.yaw;
      semPost(sem_id, ROBOTSENSORS);
      sendDoubleByWifi(angle);
    }
    else if (strcmp(command, "x") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "X sync.");
      semWait(sem_id, ROBOTSENSORS);
      double pos = robotSensors.robotPosition.axisPossition.x;
      semPost(sem_id, ROBOTSENSORS);
      sendDoubleByWifi(pos);
    }
    else if (strcmp(command, "y") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Y sync.");
      semWait(sem_id, ROBOTSENSORS);
      double pos = robotSensors.robotPosition.axisPossition.y;
      semPost(sem_id, ROBOTSENSORS);
      sendDoubleByWifi(pos);
    }
    else if (strcmp(command, "z") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Z sync.");
      semWait(sem_id, ROBOTSENSORS);
      double pos = robotSensors.robotPosition.axisPossition.z;
      semPost(sem_id, ROBOTSENSORS);
      sendDoubleByWifi(pos);
    }
    else if (strcmp(command, "voltPer") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Voltage percent sync.");

      semWait(sem_id, ROBOTSENSORS);
      double value = robotSensors.voltage.capacityPercent;
      semPost(sem_id, ROBOTSENSORS);
      sendDoubleByWifi(value);
    }
    else if (strcmp(command, "voltage") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Voltage sync.");
      semWait(sem_id, ROBOTSENSORS);
      double value = robotSensors.voltage.volts;
      semPost(sem_id, ROBOTSENSORS);
      sendDoubleByWifi(value);
    }
    else if (strcmp(command, "ult") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Ultrasonic sync.");
      semWait(sem_id, ROBOTSENSORS);
      double value = robotSensors.ultrasonic;
      semPost(sem_id, ROBOTSENSORS);
      sendDoubleByWifi(value);
    }
    else if (strcmp(command, "dir") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Direction sync.");

      semWait(sem_id, ROBOTACCULATORS);
      if (strcmp(value, "F") == 0)
        robotAcculators.robotDirection = FORWARD;
      else if (strcmp(value, "B") == 0)
        robotAcculators.robotDirection = BACKWARD;
      else if (strcmp(value, "C") == 0)
        robotAcculators.robotDirection = CLOCKWISE;
      else if (strcmp(value, "A") == 0)
        robotAcculators.robotDirection = ANTICLOCKWISE;
      else
        robotAcculators.robotDirection = STOP;
      semPost(sem_id, ROBOTACCULATORS);

      LOGInfo(SENSOR_CONN_TAG, 0, value);
    }
    else if (strcmp(command, "speed") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Speed sync.");
      semWait(sem_id, ROBOTACCULATORS);
      robotAcculators.robotSpeed = atoi(value);
      semPost(sem_id, ROBOTACCULATORS);
      LOGInfo(SENSOR_CONN_TAG, 0, value);
    }
    else if (strcmp(command, "leds") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Leds sync.");
      int valueLeds = 0;
      semWait(sem_id, ROBOTACCULATORS);
      if (robotAcculators.leds.LedUp == COLOR_RED) valueLeds += (1 << 1);
      else if (robotAcculators.leds.LedUp == COLOR_GREEN) valueLeds += (1 << 2);
      else if (robotAcculators.leds.LedUp == COLOR_ORANGE) valueLeds += (1 << 2) + (1 << 1);

      if (robotAcculators.leds.LedMiddle == COLOR_RED) valueLeds += (1 << 3);
      else if (robotAcculators.leds.LedMiddle == COLOR_GREEN) valueLeds += (1 << 4);
      else if (robotAcculators.leds.LedMiddle == COLOR_ORANGE) valueLeds += (1 << 3) + (1 << 4);

      if (robotAcculators.leds.LedDown == COLOR_RED) valueLeds += (1 << 5);
      else if (robotAcculators.leds.LedDown == COLOR_GREEN) valueLeds += (1 << 6);
      else if (robotAcculators.leds.LedDown == COLOR_ORANGE) valueLeds += (1 << 5) + (1 << 6);
      semPost(sem_id, ROBOTACCULATORS);
      sendIntByWifi(valueLeds);
    }
    else if (strcmp(command, "ledK") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Led kinect sync.");
      int valueLeds = 0;
      semWait(sem_id, ROBOTACCULATORS);
      if (robotAcculators.ledKinect == LEDKINECT_RED) valueLeds = (1 << 1);
      else if (robotAcculators.ledKinect == LEDKINECT_GREEN) valueLeds = (1 << 2);
      else if (robotAcculators.ledKinect == LEDKINECT_ORANGE) valueLeds = (1 << 3);
      else if (robotAcculators.ledKinect == LEDKINECT_BLINK_RED_ORANGE) valueLeds += (1 << 4);
      else if (robotAcculators.ledKinect == LEDKINECT_BLINK_GREEN) valueLeds = (1 << 5);
      else if (robotAcculators.ledKinect == LEDKINECT_BLINK_ORANGE) valueLeds = (1 << 6);
      semPost(sem_id, ROBOTACCULATORS);
      sendIntByWifi(valueLeds);
    }
    sleep(0.005);
  }
  LOGInfo(SENSOR_CONN_TAG, 1, "End:Sensor sync by network.");
  return NULL;
}

void *syncKinectFrames(void *arg) {
  Mat depthMap;
  LOGInfo(SENSOR_CONN_TAG, 1, "Start:Kinect sync frames.");
  while (onAllThreads && cvWaitKey(10) < 0) {
    if ( !cameraKinect.grab() )
    {
      LOGError(KINECT_TAG, "Problem grab frames.");
    }
    else
    {
      semWait(sem_id, CAMERA_DEPTH_KINECT1);
      if (cameraKinect.retrieve( depthMap, CAP_OPENNI_DEPTH_MAP   ) ) {
        depthMap.convertTo( depth1Kinect, CV_8UC1, scaleFactor );
        semWait(sem_id, CAMERA_VARIABLE_KINECTDEPTH);
        depthChooseKinect = 1;
        semPost(sem_id, CAMERA_VARIABLE_KINECTDEPTH);
      }
      semPost(sem_id, CAMERA_DEPTH_KINECT1);

      semWait(sem_id, CAMERA_IMAGE_KINECT1);
      if (cameraKinect.retrieve( img1Kinect, CAP_OPENNI_BGR_IMAGE )) {
        semWait(sem_id, CAMERA_VARIABLE_KINECTIMAGE);
        imageChooseKinect = 1;
        semPost(sem_id, CAMERA_VARIABLE_KINECTIMAGE);
      }
      semPost(sem_id, CAMERA_IMAGE_KINECT1);

      semWait(sem_id, CAMERA_POINTCLOUDMAP_KINECT1);
      if (cameraKinect.retrieve( pointCloudMap1Kinect, CAP_OPENNI_POINT_CLOUD_MAP )) {
        semWait(sem_id, CAMERA_VARIABLE_KINECTPOINTCLOUDMAP);
        pointCloudMapChooseKinect = 1;
        semPost(sem_id, CAMERA_VARIABLE_KINECTPOINTCLOUDMAP);
      }
      semPost(sem_id, CAMERA_POINTCLOUDMAP_KINECT1);
    }

    cvWaitKey(10);
    if ( !cameraKinect.grab() )
    {
      LOGError(KINECT_TAG, "Problem grab frames.");
    }
    else
    {
      semWait(sem_id, CAMERA_DEPTH_KINECT2);
      if (cameraKinect.retrieve( depthMap, CAP_OPENNI_DEPTH_MAP    ) ) {
        depthMap.convertTo( depth2Kinect, CV_8UC1, scaleFactor );
        semWait(sem_id, CAMERA_VARIABLE_KINECTDEPTH);
        depthChooseKinect = 2;
        semPost(sem_id, CAMERA_VARIABLE_KINECTDEPTH);
      }
      semPost(sem_id, CAMERA_DEPTH_KINECT2);

      semWait(sem_id, CAMERA_IMAGE_KINECT2);
      if (cameraKinect.retrieve( img2Kinect, CAP_OPENNI_BGR_IMAGE )) {
        semWait(sem_id, CAMERA_VARIABLE_KINECTIMAGE);
        imageChooseKinect = 2;
        semPost(sem_id, CAMERA_VARIABLE_KINECTIMAGE);
      }
      semPost(sem_id, CAMERA_IMAGE_KINECT2);

      semWait(sem_id, CAMERA_POINTCLOUDMAP_KINECT2);
      if (cameraKinect.retrieve( pointCloudMap2Kinect, CAP_OPENNI_POINT_CLOUD_MAP )) {
        semWait(sem_id, CAMERA_VARIABLE_KINECTPOINTCLOUDMAP);
        pointCloudMapChooseKinect = 2;
        semPost(sem_id, CAMERA_VARIABLE_KINECTPOINTCLOUDMAP);
      }
      semPost(sem_id, CAMERA_POINTCLOUDMAP_KINECT2);
    }
  }
  LOGInfo(SENSOR_CONN_TAG, 1, "End:Kinect sync frames.");
  return NULL;
}

void *syncUltrasonic(void *arg) {
  LOGInfo(NUCLEO_TAG, 1, "Start:Ultrasonic sync.");
  double value = getUltrasonic();
  semWait(sem_id, ROBOTSENSORS);
  robotSensors.ultrasonic = value;
  semPost(sem_id, ROBOTSENSORS);
  LOGInfo(NUCLEO_TAG, 1, "End:Ultrasonic sync.");
  return NULL;
}

void *syncLeds(bool checkChange) {
  LOGInfo(NUCLEO_TAG, 1, "Start:Leds sync.");
  semWait(sem_id, ROBOTACCULATORS);
  color_t LedUp = robotAcculators.leds.LedUp;
  color_t LedMiddle = robotAcculators.leds.LedMiddle;
  color_t LedDown = robotAcculators.leds.LedDown;
  semPost(sem_id, ROBOTACCULATORS);
  if (!checkChange) {
    setLeds(LedUp, LedMiddle, LedDown);
    semWait(sem_id, ROBOTACCULATORS_LAST);
    robotAcculatorsLast.leds.LedUp = LedUp;
    robotAcculatorsLast.leds.LedMiddle = LedMiddle;
    robotAcculatorsLast.leds.LedDown = LedDown;
    semPost(sem_id, ROBOTACCULATORS_LAST);
  }
  else {
    semWait(sem_id, ROBOTACCULATORS_LAST);
    color_t LedUpLast = robotAcculatorsLast.leds.LedUp;
    color_t LedMiddleLast = robotAcculatorsLast.leds.LedMiddle;
    color_t LedDownLast = robotAcculatorsLast.leds.LedDown;
    semPost(sem_id, ROBOTACCULATORS_LAST);
    if (LedUp != LedUpLast || LedMiddle != LedMiddleLast || LedDown != LedDownLast) {
      setLeds(LedUp, LedMiddle, LedDown);
      semWait(sem_id, ROBOTACCULATORS_LAST);
      robotAcculatorsLast.leds.LedUp = LedUp;
      robotAcculatorsLast.leds.LedMiddle = LedMiddle;
      robotAcculatorsLast.leds.LedDown = LedDown;
      semPost(sem_id, ROBOTACCULATORS_LAST);
      LOGInfo(NUCLEO_TAG, 1, "Change leds sync.");
    }
  }
  LOGInfo(NUCLEO_TAG, 1, "End:Leds sync.");
  return NULL;
}

void *syncMotors(bool checkChange) {
  LOGInfo(NUCLEO_TAG, 1, "Start:Motors sync.");
  semWait(sem_id, ROBOTACCULATORS);
  direction_t  robotDirection = robotAcculators.robotDirection;
  unsigned int robotSpeed = robotAcculators.robotSpeed;
  semPost(sem_id, ROBOTACCULATORS);
  if (!checkChange) {
    setMove(robotDirection, robotSpeed );
    semWait(sem_id, ROBOTACCULATORS_LAST);
    robotAcculatorsLast.robotDirection = robotDirection;
    robotAcculatorsLast.robotSpeed = robotSpeed;
    semPost(sem_id, ROBOTACCULATORS_LAST);
  }
  else {
    semWait(sem_id, ROBOTACCULATORS_LAST);
    direction_t  robotDirectionLast = robotAcculatorsLast.robotDirection;
    unsigned int robotSpeedLast = robotAcculatorsLast.robotSpeed;
    semPost(sem_id, ROBOTACCULATORS_LAST);
    if (robotDirection != robotDirectionLast || robotSpeed != robotSpeedLast) {
      setMove(robotDirection, robotSpeed );
      semWait(sem_id, ROBOTACCULATORS_LAST);
      robotAcculatorsLast.robotDirection = robotDirection;
      robotAcculatorsLast.robotSpeed = robotSpeed;
      semPost(sem_id, ROBOTACCULATORS_LAST);
      LOGInfo(NUCLEO_TAG, 0, "Change motors sync.");
    }
  }
  LOGInfo(NUCLEO_TAG, 1, "End:Motors sync.");
  return NULL;
}

void *syncPossition(void *arg) {
  LOGInfo(NUCLEO_TAG, 1, "Start:Possition sync.");
  Axis3d_struct axis = getPossitionAxis();
  Angle3d_struct angle = getPossitionAngle3d();
  semWait(sem_id, ROBOTSENSORS);
  robotSensors.robotPosition.axisPossition.x = axis.x;
  robotSensors.robotPosition.axisPossition.y = axis.y;
  robotSensors.robotPosition.axisPossition.z = axis.z;
  robotSensors.robotPosition.anglePossition.roll = angle.roll;
  robotSensors.robotPosition.anglePossition.pitch = angle.pitch;
  robotSensors.robotPosition.anglePossition.yaw = angle.yaw;
  semPost(sem_id, ROBOTSENSORS);
  LOGInfo(NUCLEO_TAG, 1, "End:Possition sync.");
  return NULL;
}

void *syncVoltage(void *arg) {
  LOGInfo(NUCLEO_TAG, 1, "Start:Voltage sync.");
  Voltage_struct voltage = getVoltage();
  semWait(sem_id, ROBOTSENSORS);
  robotSensors.voltage = voltage;
  semPost(sem_id, ROBOTSENSORS);
  LOGInfo(NUCLEO_TAG, 1, "End:Voltage sync.");
  return NULL;
}

void *syncButtons(void *arg) {
  LOGInfo(NUCLEO_TAG, 1, "Start:Buttons sync.");
  unsigned char buttons = getButtons();
  semWait(sem_id, ROBOTSENSORS);
  robotSensors.buttons.buttonDown = (buttons & 0x01) && 0x01;
  robotSensors.buttons.buttonMiddle = (buttons & 0x02) && 0x02;
  robotSensors.buttons.buttonUp = (buttons & 0x04) && 0x04;
  semPost(sem_id, ROBOTSENSORS);
  LOGInfo(NUCLEO_TAG, 1, "End:Buttons sync.");
  return NULL;
}

Mat translateImg(Mat &img, int offsetx, int offsety) {
  Mat trans_mat = (Mat_<double>(2, 3) << 1, 0, offsetx, 0, 1, offsety);
  warpAffine(img, img, trans_mat, img.size());
  return trans_mat;
}

void *syncGenerateMap(void *arg) {
  while (onAllThreads) {
    LOGInfo(MAP_TAG, 1, "Start:Map generate sync.");
    RobotPosition_struct robotPosition = getRobotPossition();
    Mat pointCloudBuffer = getPointCloudMapKinect();

    Mat map = getMapImage();
    if (map.empty())
      map = Mat(MAP_HEIGHT, MAP_WIDTH,  CV_8UC3, Scalar(0, 0, 0));

    long robotX = robotPosition.axisPossition.x / (MAP_SCALE / 10);
    long robotY = robotPosition.axisPossition.y / (MAP_SCALE / 10);

    int deltaX = robotX - mapOffsetX;
    int deltaY = robotY - mapOffsetY;
    if (deltaX < MAX_DELTA_TRANSLATE && deltaY < MAX_DELTA_TRANSLATE) {
      translateImg(map, -deltaX, -deltaY); // posun mapy aby sme nestratili predchadzajuce udaje, vzdy si pametame len okolie robota
      mapOffsetX = robotX;
      mapOffsetY = robotY;
    }

    for (int x = 0; x < map.cols; x++) {
      for (int y = 0; y < map.rows; y++) {
        Vec3b color = map.at<Vec3b>(Point(x, y));

        int varMapOblivion[3];
        varMapOblivion[0] = (int)color[0] - SPEED_OF_MAP_OBLIVION;
        varMapOblivion[1] = (int)color[1] - SPEED_OF_MAP_OBLIVION;
        varMapOblivion[2] = (int)color[2] - SPEED_OF_MAP_OBLIVION;

        if (varMapOblivion[0] <= 0)
          color[0] = 0;
        else
          color[0] = varMapOblivion[0];

        if (varMapOblivion[1] <= 0)
          color[1] = 0;
        else
          color[1] = varMapOblivion[1];

        if (varMapOblivion[2] <= 0)
          color[2] = 0;
        else
          color[2] = varMapOblivion[2];

        map.at<Vec3b>(Point(x, y)) = color;
      }
    }

    for (int x = 0; x < pointCloudBuffer.cols; x++) {
      int minPointX = 30000;
      int minPointZ = 30000;
      for (int y = 0; y < pointCloudBuffer.rows; y++) {
        if ( !pointCloudBuffer.empty() )   {
          Point3f p = pointCloudBuffer.at<Point3f>(y, x);
          float kinectPointX = p.x;         //pozicia vlavo/vpravo na kinecte
          float kinectpointY = p.y + 0.21;  //pozicia hore/dole na kinecte
          float kinectPointZ = p.z;         //hlbka

          if (kinectpointY >= MIN_BARRIER_HEIGHT && kinectpointY <= MAX_BARRIER_HEIGHT && kinectPointX != 0 && kinectpointY != 0 && kinectPointZ != 0) {
            int pointZ = kinectPointZ * 100;
            if (pointZ < minPointZ) minPointZ = pointZ;

            int pointX = kinectPointX * 100;
            minPointX = pointX;
          }
        }
      }
      if (minPointX != 30000 && minPointZ != 30000) {
        double r = sqrt((double)(minPointX * minPointX + minPointZ * minPointZ));
        double angle = acos((-(double)minPointX) / r) - 3.14 / 2;

        int x2 = r * cos(robotPosition.anglePossition.yaw + angle) + MAP_WIDTH / 2;
        int z2 = r * sin(robotPosition.anglePossition.yaw + angle) + MAP_HEIGHT / 2;

        if (x2 > 0 && x2 < MAP_WIDTH && z2 > 0 && z2 < MAP_HEIGHT) {

          Vec3b color = map.at<Vec3b>(Point(x2, z2));

          int varMapCreation[3];
          varMapCreation[0] = (int)color[0] + SPEED_OF_MAP_CREATION;
          varMapCreation[1] = (int)color[1] + SPEED_OF_MAP_CREATION;
          varMapCreation[2] = (int)color[2] + SPEED_OF_MAP_CREATION;

          if (varMapCreation[0] >= 255)
            color[0] = 255;
          else
            color[0] = varMapCreation[0];

          if (varMapCreation[1] >= 255)
            color[1] = 255;
          else
            color[1] = varMapCreation[1];

          if (varMapCreation[2] >= 255)
            color[2] = 255;
          else
            color[2] = varMapCreation[2];

          map.at<Vec3b>(Point(x2, z2)) = color;
        }
      }
    }

    semWait(sem_id, MAP_VARIABLE);
    char mapChooseLast = mapImageChoose;
    semPost(sem_id, MAP_VARIABLE);
    if (mapChooseLast == 1) {
      semWait(sem_id, MAP_IMAGE2);
      mapImage2 = map.clone();
      semWait(sem_id, MAP_VARIABLE);
      mapImageChoose = 2;
      semPost(sem_id, MAP_VARIABLE);
      semPost(sem_id, MAP_IMAGE2);
    }
    else {
      semWait(sem_id, MAP_IMAGE1);
      mapImage1 = map.clone();
      semWait(sem_id, MAP_VARIABLE);
      mapImageChoose = 1;
      semPost(sem_id, MAP_VARIABLE);
      semPost(sem_id, MAP_IMAGE1);
    }
    LOGInfo(MAP_TAG, 1, "End:Map generate sync.");
    sleep(SYNC_MAP_GENERATE_TIME);
  }
  return NULL;
}

int morph_elem = 0;
int morph_size = 8;
int morph_operator = 0;
int const max_operator = 4;
int const max_elem = 2;
int const max_kernel_size = 21;

void *syncOperatorDetect(void *arg) {
  while (onAllThreads) {
    LOGInfo(OPERATOR_TAG, 1, "Start:Operator sync.");

    Mat rgbOperatorImage = getImageKinect();
    Mat depthOperatorMap = getDepthKinect();
    Mat hsvOperatorImage;
    Mat orangeOperatorMaskImage;
    Mat greenOperatorMaskImage;
    Mat depthOperatorMaskImage = Mat(depthOperatorMap.size(), CV_8UC1, Scalar(0));
    if (!rgbOperatorImage.empty() && !depthOperatorMap.empty()) {
      cvtColor(rgbOperatorImage, hsvOperatorImage, COLOR_BGR2HSV); //konverzia na hsv model
      inRange(hsvOperatorImage, Scalar(iLowH_green, iLowS_green, iLowV_green), Scalar(iHighH_green, iHighS_green, iHighV_green), greenOperatorMaskImage); //vytvorenie masky zelenej farby
      inRange(hsvOperatorImage, Scalar(iLowH_orange, iLowS_orange, iLowV_orange), Scalar(iHighH_orange, iHighS_orange, iHighV_orange), orangeOperatorMaskImage); //vytvorenie maasky oranzovej farby
      /*
      int dilation_size = 3;
          Mat element = getStructuringElement( MORPH_CROSS,
                                               Size( 2 * dilation_size + 1, 2 * dilation_size + 1 ),
                                               Point( dilation_size, dilation_size ) );
          /// Apply the dilation operation
          dilate( greenOperatorMaskImage, greenOperatorMaskImage, element );
          dilate( orangeOperatorMaskImage, orangeOperatorMaskImage, element );
      */
#if OPERATOR_WITH_DEPTH == 1
      int dist = 5;
      int size = 4;
      int checkSize = 4;

      for (int y = size; y < depthOperatorMap.rows - size; y += size)
      {
        for (int x = size; x < depthOperatorMap.cols - size; x += size)
        {
          bool find = false;
          for (int p = -checkSize; p <= checkSize && !find; p++) {
            if (abs(depthOperatorMap.at<uchar>(Point(x + p, y)) - depthOperatorMap.at<uchar>(Point(x, y))) > dist) {
              find = true;
            }
          }
          for (int q = -checkSize; q <= checkSize && !find; q++) {
            if (abs(depthOperatorMap.at<uchar>(Point(x, y + q)) - depthOperatorMap.at<uchar>(Point(x, y))) > dist) {
              find = true;
            }
          }
          if (find) {
            circle(depthOperatorMaskImage, Point(x, y), size + 1, Scalar(255), 1, 8);
            x += size;
          }
        }
      }

      int choose_depth = -1;
      vector< vector<Point> > contours_depth;
      findContours(depthOperatorMaskImage, contours_depth, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
      long areasDepth[contours_depth.size()];
      Rect r_depth[contours_depth.size()];
      for (int i = 0; i < contours_depth.size(); i++) {
        areasDepth[i] = 0;
        r_depth[i] = boundingRect(contours_depth[i]);
      }
#endif

      int choose_people = -1;
      int choose_orange = -1;
      int choose_green = -1;
      long maxArea = -1;
      Point point1(-1, -1);
      Point point2(-1, -1);

      vector< vector<Point> > contours_green;
      findContours(greenOperatorMaskImage, contours_green, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
      Rect r_green[contours_green.size()];
      long areasGreen[contours_green.size()];
      for (unsigned int i = 0; i < contours_green.size(); i++) {
        areasGreen[i] = 0;
        r_green[i] = boundingRect(contours_green[i]);
      }

      vector< vector<Point> > contours_orange;
      findContours(orangeOperatorMaskImage, contours_orange, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
      Rect r_orange[contours_orange.size()];
      long areasOrange[contours_orange.size()];
      for (unsigned int i = 0; i < contours_orange.size(); i++) {
        areasOrange[i] = 0;
        r_orange[i] = boundingRect(contours_orange[i]);
      }

      for (unsigned int index_orange = 0; index_orange < contours_orange.size(); index_orange++) {
        for (unsigned int index_green = 0; index_green < contours_green.size(); index_green++) {

          int centerGreenX = r_green[index_green].x + r_green[index_green].width / 2;
          int centerOrangeX = r_orange[index_orange].x + r_orange[index_orange].width / 2;

          int centerGreenY = r_green[index_green].y + r_green[index_green].height / 2;
          int centerOrangeY = r_orange[index_orange].y + r_orange[index_orange].height / 2;

          int absDistanceX = abs(centerGreenX - centerOrangeX);
          int absDistanceY = abs(centerGreenY - centerOrangeY);

          if (absDistanceX  <= OPERATOR_MAXCENTER_DISTANCE_ORANGE_AND_GREEN_MASK_X && absDistanceY <= OPERATOR_MAXCENTER_DISTANCE_ORANGE_AND_GREEN_MASK_Y &&
              absDistanceX  >= OPERATOR_MINCENTER_DISTANCE_ORANGE_AND_GREEN_MASK_X && absDistanceY >= OPERATOR_MINCENTER_DISTANCE_ORANGE_AND_GREEN_MASK_Y) {

            if (areasGreen[index_green] == 0) areasGreen[index_green] = contourArea(Mat(contours_green[index_green]));
            if (areasOrange[index_orange] == 0) areasOrange[index_orange] = contourArea(Mat(contours_orange[index_orange]));

            if (areasGreen[index_green] > OPERATOR_MIN_AREA_GREEN && areasOrange[index_orange] > OPERATOR_MIN_AREA_ORANGE) {
#if OPERATOR_WITH_DEPTH == 1

              for (unsigned int index_depth = 0; index_depth < contours_depth.size(); index_depth++) {
                if (areasDepth[index_depth] == 0) areasDepth[index_depth] = contourArea(Mat(contours_depth[index_depth]));
                if (areasDepth[index_depth] > OPERATOR_MIN_AREA_DEPTH && areasDepth[index_depth] < 320 * 480) {

                  if (abs(r_depth[index_depth].x - (r_green[index_green].x + r_orange[index_orange].x) / 2) < 60 &&
                      abs(r_depth[index_depth].x + r_depth[index_depth].width - (r_green[index_green].x + r_green[index_green].width + r_orange[index_orange].x + r_orange[index_orange].width) / 2) < 60 &&
                      abs(r_depth[index_depth].y - (r_green[index_green].y + r_orange[index_orange].y) / 2) < 60 &&
                      abs(r_depth[index_depth].y + r_depth[index_depth].height - (r_green[index_green].y + r_green[index_green].height + r_orange[index_orange].y + r_orange[index_orange].height) / 2) < 60 &&
                      maxArea < (areasGreen[index_green] + areasOrange[index_orange] + areasDepth[index_depth]))
                  {
                    maxArea = (areasGreen[index_green] + areasOrange[index_orange] + areasDepth[index_depth]);
                    choose_green = index_green;
                    choose_orange = index_orange;
                    choose_depth = index_depth;
                  }
                }
              }
#else
              if (maxArea < (areasGreen[index_green] + areasOrange[index_orange]))
              {
                maxArea = (areasGreen[index_green] + areasOrange[index_orange]);
                choose_green = index_green;
                choose_orange = index_orange;
              }
#endif
            }
          }
        }
      }
      if (maxArea != -1) {

        if (r_orange[choose_orange].y < r_green[choose_green].y)
          choose_people = 1;
        else
          choose_people = 2;

        int minX;
        int minY;
        int maxX;
        int maxY;
        if (r_orange[choose_orange].x > r_green[choose_green].x)
          minX = r_green[choose_green].x;
        else
          minX = r_orange[choose_orange].x;

        if (r_orange[choose_orange].x + r_orange[choose_orange].width < r_green[choose_green].x + r_green[choose_green].width)
          maxX = r_green[choose_green].x + r_green[choose_green].width;
        else
          maxX = r_orange[choose_orange].x + r_orange[choose_orange].width ;

        if (r_orange[choose_orange].y > r_green[choose_green].y)
          minY = r_green[choose_green].y;
        else
          minY = r_orange[choose_orange].y;

        if (r_orange[choose_orange].y + r_orange[choose_orange].height < r_green[choose_green].y + r_green[choose_green].height)
          maxY = r_green[choose_green].y + r_green[choose_green].height;
        else
          maxY = r_orange[choose_orange].y + r_orange[choose_orange].height;

        point1 = Point(minX, minY);
        point2 = Point(maxX, maxY);
        Point center = Point(point2.x - (point2.x - point1.x) / 2, point2.y - (point2.y - point1.y) / 2);

#if CAMERA_WIFI == 1
#if OPERATOR_WITH_DEPTH == 1
        if (choose_depth != -1)
          drawContours(depthOperatorMaskImage, contours_depth, choose_depth, Scalar(255), CV_FILLED);
#endif
        if (choose_orange != -1)
          drawContours(orangeOperatorMaskImage, contours_orange, choose_orange, Scalar(255), CV_FILLED);
        if (choose_green != -1)
          drawContours(greenOperatorMaskImage, contours_green, choose_green, Scalar(255), CV_FILLED);
        rectangle(rgbOperatorImage, point1, point2, CV_RGB(0, 255, 0), 3, 8, 0);
        circle(rgbOperatorImage, center, 5, Scalar( 0, 255, 255 ), -1, 8);
        char text[20];
        sprintf(text, "%d,%d,P%d", center.x, center.y, choose_people);
        putText(rgbOperatorImage, text, Point(10, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 0, 255), 1, CV_AA);
#endif

        semWait(sem_id, OPERATOR_POSSITION);
        Mat pointCloudBuffer = getPointCloudMapKinect();
        operatorPossition = pointCloudBuffer.at<Point3f>(center.y, center.x);
        semPost(sem_id, OPERATOR_POSSITION);
        if (OPERATOR_STATUS_KINECT_LED) {
          semWait(sem_id, ROBOTACCULATORS);
          robotAcculators.ledKinect = LEDKINECT_GREEN;
          semPost(sem_id, ROBOTACCULATORS);
        }
      }
      else {
        semWait(sem_id, OPERATOR_POSSITION);
        operatorPossition = Point3f(-1, -1, -1);
        semPost(sem_id, OPERATOR_POSSITION);
        if (OPERATOR_STATUS_KINECT_LED) {
          semWait(sem_id, ROBOTACCULATORS);
          robotAcculators.ledKinect = LEDKINECT_RED;
          semPost(sem_id, ROBOTACCULATORS);
        }
      }
    }
    else {
      semWait(sem_id, OPERATOR_POSSITION);
      operatorPossition = Point3f(-1, -1, -1);
      semPost(sem_id, OPERATOR_POSSITION);
      if (OPERATOR_STATUS_KINECT_LED) {
        semWait(sem_id, ROBOTACCULATORS);
        robotAcculators.ledKinect = LEDKINECT_RED;
        semPost(sem_id, ROBOTACCULATORS);
      }
    }

#if CAMERA_WIFI == 1
//copy mat
    if (rgbOperatorChoose == 1) {
      semWait(sem_id, RGB_OPERATOR2);
      rgbOperatorImage2 = rgbOperatorImage;
      semWait(sem_id, RGB_OPERATOR_VARIABLE);
      rgbOperatorChoose = 2;
      semPost(sem_id, RGB_OPERATOR_VARIABLE);
      semPost(sem_id, RGB_OPERATOR2);

    }
    else {
      semWait(sem_id, RGB_OPERATOR1);
      rgbOperatorImage1 = rgbOperatorImage;
      semWait(sem_id, RGB_OPERATOR_VARIABLE);
      rgbOperatorChoose = 1;
      semPost(sem_id, RGB_OPERATOR_VARIABLE);
      semPost(sem_id, RGB_OPERATOR1);
    }

    if (hsvOperatorChoose == 1) {
      semWait(sem_id, HSV_OPERATOR2);
      hsvOperatorImage2 = hsvOperatorImage;
      semWait(sem_id, HSV_OPERATOR_VARIABLE);
      hsvOperatorChoose = 2;
      semPost(sem_id, HSV_OPERATOR_VARIABLE);
      semPost(sem_id, HSV_OPERATOR2);

    }
    else {
      semWait(sem_id, HSV_OPERATOR1);
      hsvOperatorImage1 = hsvOperatorImage;
      semWait(sem_id, HSV_OPERATOR_VARIABLE);
      hsvOperatorChoose = 1;
      semPost(sem_id, HSV_OPERATOR_VARIABLE);
      semPost(sem_id, HSV_OPERATOR1);
    }

    if (depthOperatorMaskChoose == 1) {
      semWait(sem_id, DEPTH_OPERATOR_MASK2);
      depthOperatorMaskImage2 = depthOperatorMaskImage;
      semWait(sem_id, DEPTH_OPERATOR_MASK_VARIABLE);
      depthOperatorMaskChoose = 2;
      semPost(sem_id, DEPTH_OPERATOR_MASK_VARIABLE);
      semPost(sem_id, DEPTH_OPERATOR_MASK2);

    }
    else {
      semWait(sem_id, DEPTH_OPERATOR_MASK1);
      depthOperatorMaskImage1 = depthOperatorMaskImage;
      semWait(sem_id, DEPTH_OPERATOR_MASK_VARIABLE);
      depthOperatorMaskChoose = 1;
      semPost(sem_id, DEPTH_OPERATOR_MASK_VARIABLE);
      semPost(sem_id, DEPTH_OPERATOR_MASK1);
    }

    if (greenOperatorMaskChoose == 1) {
      semWait(sem_id, GREEN_OPERATOR_MASK2);
      greenOperatorMaskImage2 = greenOperatorMaskImage;
      semWait(sem_id, GREEN_OPERATOR_MASK_VARIABLE);
      greenOperatorMaskChoose = 2;
      semPost(sem_id, GREEN_OPERATOR_MASK_VARIABLE);
      semPost(sem_id, GREEN_OPERATOR_MASK2);

    }
    else {
      semWait(sem_id, GREEN_OPERATOR_MASK1);
      greenOperatorMaskImage1 = greenOperatorMaskImage;
      semWait(sem_id, GREEN_OPERATOR_MASK_VARIABLE);
      greenOperatorMaskChoose = 1;
      semPost(sem_id, GREEN_OPERATOR_MASK_VARIABLE);
      semPost(sem_id, GREEN_OPERATOR_MASK1);
    }

    if (orangeOperatorMaskChoose == 1) {
      semWait(sem_id, ORANGE_OPERATOR_MASK2);
      orangeOperatorMaskImage2 = orangeOperatorMaskImage;
      semWait(sem_id, ORANGE_OPERATOR_MASK_VARIABLE);
      orangeOperatorMaskChoose = 2;
      semPost(sem_id, ORANGE_OPERATOR_MASK_VARIABLE);
      semPost(sem_id, ORANGE_OPERATOR_MASK2);
    }
    else {
      semWait(sem_id, ORANGE_OPERATOR_MASK1);
      orangeOperatorMaskImage1 = orangeOperatorMaskImage;
      semWait(sem_id, ORANGE_OPERATOR_MASK_VARIABLE);
      orangeOperatorMaskChoose = 1;
      semPost(sem_id, ORANGE_OPERATOR_MASK_VARIABLE);
      semPost(sem_id, ORANGE_OPERATOR_MASK1);
    }
#endif
    LOGInfo(OPERATOR_TAG, 1, "End:Operator sync.");
    sleep(SYNC_OPERATOR_DETECT_TIME);
  }
  return NULL;
}

void *syncKinectMotor(bool checkChange) {
  //https://openkinect.org/wiki/Protocol_Documentation#Control_Packet_Structure
  unsigned char empty[1];
  LOGInfo(KINECT_TAG, 1, "Start:Motor sync.");
  semWait(sem_id, ROBOTACCULATORS);
  int angle = robotAcculators.kinect.roll * 2;
  semPost(sem_id, ROBOTACCULATORS);

  if (!checkChange) {
    XnStatus rc = XN_STATUS_OK;
    rc = xnUSBSendControl(dev,
                          XN_USB_CONTROL_TYPE_VENDOR,
                          0x31,
                          (XnUInt16)angle,
                          0x0,
                          empty,
                          0x0, 0);
    if (rc != XN_STATUS_OK) {
      LOGError(KINECT_TAG, xnGetStatusString(rc) );
    }
    semWait(sem_id, ROBOTACCULATORS_LAST);
    robotAcculatorsLast.kinect.roll = angle;
    semPost(sem_id, ROBOTACCULATORS_LAST);
  }
  else {
    semWait(sem_id, ROBOTACCULATORS_LAST);
    int angleLast = robotAcculatorsLast.kinect.roll * 2;
    semPost(sem_id, ROBOTACCULATORS_LAST);
    if (angle != angleLast) {
      XnStatus rc = XN_STATUS_OK;
      rc = xnUSBSendControl(dev,
                            XN_USB_CONTROL_TYPE_VENDOR,
                            0x31,
                            (XnUInt16)angle,
                            0x0,
                            empty,
                            0x0, 0);
      if (rc != XN_STATUS_OK) {
        LOGError(KINECT_TAG, xnGetStatusString(rc) );
      }

      semWait(sem_id, ROBOTACCULATORS_LAST);
      robotAcculatorsLast.kinect.roll = angle;
      semPost(sem_id, ROBOTACCULATORS_LAST);

      LOGInfo(KINECT_TAG, 0, "Change motor sync.");
    }
  }

  LOGInfo(KINECT_TAG, 1, "End:Motor sync.");
  return NULL;
}

void *syncKinectLed(bool checkChange) {
  //https://openkinect.org/wiki/Protocol_Documentation#Control_Packet_Structure
  unsigned char empty[1];
  LOGInfo(KINECT_TAG, 1, "Start:Led sync.");
  semWait(sem_id, ROBOTACCULATORS);
  ledKinect_t led = robotAcculators.ledKinect;
  semPost(sem_id, ROBOTACCULATORS);

  if (!checkChange) {
    XnStatus rc = XN_STATUS_OK;
    rc = xnUSBSendControl(dev,
                          XN_USB_CONTROL_TYPE_VENDOR,
                          0x06,
                          (XnUInt16)led,
                          0x0,
                          empty,
                          0x0, 0);
    if (rc != XN_STATUS_OK) {
      LOGError(KINECT_TAG, xnGetStatusString(rc) );
    }
    semWait(sem_id, ROBOTACCULATORS_LAST);
    robotAcculatorsLast.ledKinect = led;
    semPost(sem_id, ROBOTACCULATORS_LAST);
  }
  else {
    semWait(sem_id, ROBOTACCULATORS_LAST);
    ledKinect_t ledLast = robotAcculatorsLast.ledKinect;
    semPost(sem_id, ROBOTACCULATORS_LAST);
    if (led != ledLast) {
      XnStatus rc = XN_STATUS_OK;
      rc = xnUSBSendControl(dev,
                            XN_USB_CONTROL_TYPE_VENDOR,
                            0x06,
                            (XnUInt16)led,
                            0x0,
                            empty,
                            0x0, 0);
      if (rc != XN_STATUS_OK) {
        LOGError(KINECT_TAG, xnGetStatusString(rc) );
      }
      semWait(sem_id, ROBOTACCULATORS_LAST);
      robotAcculatorsLast.ledKinect = led;
      semPost(sem_id, ROBOTACCULATORS_LAST);
      LOGInfo(KINECT_TAG, 0, "Change led sync.");
    }
  }

  LOGInfo(KINECT_TAG, 1, "End:Led sync.");
  return NULL;
}

void *syncKinectSensors(void *arg) {
  LOGInfo(KINECT_TAG, 1, "Start:Sensor sync.");
  XnUInt32 nBufferSize = 10;
  XnUChar * pBuffer = new XnUChar[nBufferSize];
  XnUInt32 pnBytesReceived;
  XnStatus rc = XN_STATUS_OK;
  rc = xnUSBReceiveControl(dev,
                           XN_USB_CONTROL_TYPE_VENDOR,
                           0x32,
                           0x0,
                           0x0,
                           pBuffer,
                           nBufferSize,
                           &pnBytesReceived,
                           0);
  if (rc != XN_STATUS_OK) {
    LOGError(KINECT_TAG, xnGetStatusString(rc) );
  }

  //prepocitane na zrychlenie
  double x = ((double)(((uint16_t)pBuffer[2] << 8) | pBuffer[3])) / 819;
  double y = ((double)(((uint16_t)pBuffer[4] << 8) | pBuffer[5])) / 819;
  double z = ((double)(((uint16_t)pBuffer[6] << 8) | pBuffer[7])) / 819;

  semWait(sem_id, ROBOTSENSORS);

  robotSensors.kinect.accAxis.x = x;
  robotSensors.kinect.accAxis.y = y;
  robotSensors.kinect.accAxis.z = z;
  robotSensors.kinect.accAngle.roll = pBuffer[8] / 2;

  robotSensors.kinect.motorStatus = (motorStatusKinect_t)pBuffer[9];

  semPost(sem_id, ROBOTSENSORS);
  LOGInfo(KINECT_TAG, 1, "End:Sensor sync.");
  return NULL;
}


void *syncModules(void *arg) {
  unsigned long timeRunThread[NUMBER_OF_MODULES];
  for (int i = 0; i < NUMBER_OF_MODULES; i++) {
    timeRunThread[i] = 0;
  }
  int threadIndex = 0;
  struct timespec tstart = {0, 0}, tend = {0, 0};
  while (onAllThreads) {

    clock_gettime(CLOCK_MONOTONIC, &tstart);
    threadIndex = 0;

//acculators
#if ENABLE_MOTORS == 1
    if (timeRunThread[threadIndex] > SYNC_MOTORS_TIME) {
      syncMotors(false);
      timeRunThread[threadIndex] = 0;
    }
    else
    {
      syncMotors(true);
    }
    threadIndex++;
    usleep(SYNC_MIN_TIME / NUMBER_OF_MODULES);
#endif

#if ENABLE_KINECTMOTOR == 1
    if (timeRunThread[threadIndex] > SYNC_KINECTMOTOR_TIME) {
      syncKinectMotor(false);
      timeRunThread[threadIndex] = 0;
    }
    else {
      syncKinectMotor(true);
    }
    threadIndex++;
    usleep(SYNC_MIN_TIME / NUMBER_OF_MODULES);
#endif

#if ENABLE_LEDS == 1
    if (timeRunThread[threadIndex] > SYNC_LEDS_TIME) {
      syncLeds(false);
      timeRunThread[threadIndex] = 0;
    }
    else {
      syncLeds(true);
    }
    threadIndex++;
    usleep(SYNC_MIN_TIME / NUMBER_OF_MODULES);
#endif

#if ENABLE_KINECTLED == 1
    if (timeRunThread[threadIndex] > SYNC_KINECTLED_TIME) {
      syncKinectLed(false);
      timeRunThread[threadIndex] = 0;
    }
    else {
      syncKinectLed(true);
    }
    threadIndex++;
    usleep(SYNC_MIN_TIME / NUMBER_OF_MODULES);
#endif

//sensors
#if ENABLE_ULTRASONIC == 1
    if (timeRunThread[threadIndex] > SYNC_ULTRASONIC_TIME) {
      syncUltrasonic(NULL);
      timeRunThread[threadIndex] = 0;
    }
    threadIndex++;
    usleep(SYNC_MIN_TIME / NUMBER_OF_MODULES);
#endif

#if ENABLE_BUTTONS == 1
    if (timeRunThread[threadIndex] > SYNC_BUTTONS_TIME) {
      syncButtons(NULL);
      timeRunThread[threadIndex] = 0;
    }
    threadIndex++;
    usleep(SYNC_MIN_TIME / NUMBER_OF_MODULES);
#endif

#if ENABLE_POSSITION == 1
    if (timeRunThread[threadIndex] > SYNC_POSSITION_TIME) {
      syncPossition(NULL);
      timeRunThread[threadIndex] = 0;
    }
    threadIndex++;
    usleep(SYNC_MIN_TIME / NUMBER_OF_MODULES);
#endif

#if ENABLE_KINECTSENSORS == 1
    if (timeRunThread[threadIndex] > SYNC_KINECTSENSORS_TIME) {
      syncKinectSensors(NULL);
      timeRunThread[threadIndex] = 0;
    }
    threadIndex++;
    usleep(SYNC_MIN_TIME / NUMBER_OF_MODULES);
#endif

#if ENABLE_VOLTAGE == 1
    if (timeRunThread[threadIndex] > SYNC_VOLTAGE_TIME) {
      syncVoltage(NULL);
      timeRunThread[threadIndex] = 0;
    }
    threadIndex++;
    usleep(SYNC_MIN_TIME / NUMBER_OF_MODULES);
#endif

    clock_gettime(CLOCK_MONOTONIC, &tend);

    unsigned long deltaTimeRun = ((tend.tv_sec - tstart.tv_sec) * 1000) + ((tend.tv_nsec - tstart.tv_nsec) / 1000);

    for (int i = 0; i < NUMBER_OF_MODULES; i++) {
      if (timeRunThread[i] <= ULONG_MAX) {
        timeRunThread[i] += deltaTimeRun;
      }
      else {
        timeRunThread[i] = 0;
      }
    }
  }
  return 0;
}

void sigctrl(int param) {
  closeRobot();
  exit(param);
}

void sigpipe(int param) {
  closeRobot();
  exit(param);
}

double dist(double a, double b) {
  return sqrt(a * a + b * b);
}

double rad2Deg(double angle) {
  return angle * (180 / M_PI);
}

double deg2Rad(double angle) {
  return angle * (M_PI / 180);
}