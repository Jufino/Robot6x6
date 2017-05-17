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

Mat pointCloudMap1Kinect;
Mat pointCloudMap2Kinect;
Mat depth1Kinect;
Mat depthValidMask1Kinect;
Mat depth2Kinect;
Mat depthValidMask2Kinect;
Mat img1Kinect;
Mat img2Kinect;
int matChooseKinect = 0;

#if CAMERA_WIFI == 1
char rgbOperatorChoose = 0;
Mat rgbOperatorImage1;
Mat rgbOperatorImage2;
char greenOperatorMaskChoose = 0;
Mat greenOperatorMaskImage1;
Mat greenOperatorMaskImage2;
char orangeOperatorMaskChoose = 0;
Mat orangeOperatorMaskImage1;
Mat orangeOperatorMaskImage2;
#endif

const float scaleFactor = 0.05f;

vector<Point> journeyPoint;

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
int erode_green = 1;
int dilate_green = 3;

int iLowH_orange = 0;
int iHighH_orange = 20;
int iLowS_orange = 110;
int iHighS_orange  = 255;
int iLowV_orange  = 110;
int iHighV_orange  = 255;
int erode_orange = 1;
int dilate_orange = 3;

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
  semInit(sem_id, MAT_KINECT1, 1);
  semInit(sem_id, MAT_KINECT2, 1);
  semInit(sem_id, MAT_KINECT_VARIABLE, 1);
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
  semInit(sem_id, JOURNEY_POINTS, 1);

  mapImage1 = Mat(MAP_HEIGHT, MAP_WIDTH,  CV_8UC3, Scalar(0, 0, 0));
  mapImageChoose = 1;

  robotAcculators.isManual = true;
  robotSensors.isFollowOperatorOn = true;

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
#if ENABLE_JORNEY_GENERATE == 1
  pthread_t threadGenerateJorney;
  pthread_create(&threadGenerateJorney, NULL, &syncGenerateJorney, NULL);
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

  sleep(2); //cakame kym sa ukoncia thready

  semWait(sem_id, ROBOTACCULATORS);
  robotAcculators.ledKinect = LEDKINECT_BLINK_RED_ORANGE;
  semPost(sem_id, ROBOTACCULATORS);
  syncKinectLed(true);

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

char writeRegisterAndValueS32(unsigned char addr, unsigned char reg, long value) {
  char errorTimeout = 0;
  unsigned char data[5];
  data[0] = reg;
  data[1] = (value >> 24) & 0xFF;
  data[2] = (value >> 16) & 0xFF;
  data[3] = (value >> 8) & 0xFF;
  data[4] = value & 0xFF;
  if (setDevice(addr) == 0) {
    while (write(i2cHandle, data, 5) != 5) {
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

int  pauseGrabKinect() {
  semWait(sem_id, MAT_KINECT_VARIABLE);
  int  index = matChooseKinect;
  semPost(sem_id, MAT_KINECT_VARIABLE);
  if (index == 1)
    semWait(sem_id, MAT_KINECT1);
  else
    semWait(sem_id, MAT_KINECT2);
  return index;
}

void continueGrabKinect(int index) {
  if (index == 1)
    semPost(sem_id, MAT_KINECT1);
  else
    semPost(sem_id, MAT_KINECT2);
}

Mat getRGBKinect(int  index) {
  Mat img;
  int  var = index;
  if (var == -1)
    index = pauseGrabKinect();
  if (index == 1)
    img = img1Kinect.clone();
  else if (index == 2)
    img = img2Kinect.clone();
  if (var == -1)
    continueGrabKinect(index);
  return img;
}

Mat getDepthValidKinect(int index) {
  Mat img;
  int var = index;
  if (var == -1)
    index = pauseGrabKinect();
  if (index == 1)
    img = depthValidMask1Kinect.clone();
  else if (index == 2)
    img = depthValidMask2Kinect.clone();
  if (var == -1)
    continueGrabKinect(index);
  return img;
}

Mat getDepthScaledKinect(int index) {
  Mat img;
  Point minLoc;
  double minval, maxval;
  int var = index;
  if (var == -1)
    index = pauseGrabKinect();
  if (index == 1) {
    Mat(depth1Kinect - 400.0).convertTo(img, CV_64FC1);
    minMaxLoc(img, &minval, &maxval, NULL, NULL);
    img.convertTo( img, CV_8UC1, 255 / maxval );
  }
  else if (index == 2) {
    Mat(depth2Kinect - 400.0).convertTo(img, CV_64FC1);
    minMaxLoc(img, &minval, &maxval, NULL, NULL);
    img.convertTo( img, CV_8UC1, 255 / maxval );
  }
  if (var == -1)
    continueGrabKinect(index);
  return img;
}


Mat getDepthKinect(int index) {
  Mat img;
  int var = index;
  if (var == -1)
    index = pauseGrabKinect();
  if (index == 1)
    img = depth1Kinect.clone();
  else if (index == 2)
    img = depth2Kinect.clone();
  if (var == -1)
    continueGrabKinect(index);
  return img;
}

Mat getPointCloudMapKinect(int index) {
  Mat img;
  int var = index;
  if (var == -1)
    index = pauseGrabKinect();
  if (index == 1)
    img = pointCloudMap1Kinect.clone();
  else if (index == 2)
    img = pointCloudMap2Kinect.clone();
  if (var == -1)
    continueGrabKinect(index);
  return img;
}

Mat getImageLeft(void) {
  Mat imgMatL;
  char imageChooseMainL;
  semWait(sem_id, CAMERA_VARIABLE_L);
  imageChooseMainL = imageChooseL;
  semPost(sem_id, CAMERA_VARIABLE_L);
  if (imageChooseMainL == 1) {
    semWait(sem_id, CAMERA_IMAGE_L1);
    imgMatL = cvarrToMat(img1L);
    semPost(sem_id, CAMERA_IMAGE_L1);
  }
  else if (imageChooseMainL == 2) {
    semWait(sem_id, CAMERA_IMAGE_L2);
    imgMatL = cvarrToMat(img2L);
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

vector<Point> getJorneyPoints() {
  semWait(sem_id, JOURNEY_POINTS);
  vector<Point> bodyCesty = journeyPoint;
  semPost(sem_id, JOURNEY_POINTS);
  return bodyCesty;
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
  angle = (angle * -1) + 90;
  if (angle > 180) angle = 180;
  else if (angle < 0) angle = 0;
  semWait(sem_id, I2C);
  writeRegisterAndValueS16(STM32_ADDRESS, 13, angle);
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

void setAngleRobot(double angle) {
  semWait(sem_id, I2C);
  if (angle < 0) angle += 2 * M_PI;
  else if (angle > 2 * M_PI) angle -= 2 * M_PI;
  writeRegisterAndValueU16(STM32_ADDRESS, 14, (int)(angle * 10000));
  //printf("%f,%d\n",angle,(int)(angle * 10000));
  semPost(sem_id, I2C);
}

void setDistanceRobot(long distance) {
  semWait(sem_id, I2C);
  writeRegisterAndValueS32(STM32_ADDRESS, 15, distance);
  semPost(sem_id, I2C);
}

void setGoToAngleAndDistance() {
  semWait(sem_id, I2C);
  writeRegister(STM32_ADDRESS, 16);
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

#define maxBufferRecv 50

void *syncCameraNetworkConnection(void *arg) {
  char recvdata[maxBufferRecv];
  char command[maxBufferRecv / 2];
  char value[maxBufferRecv / 2];
  int bytes = 1;
  while (bytes != 0 && onAllThreads && onWifiCameraStill) {
    int indexValue = -1;
    bytes = recv(cameraClientsock, recvdata, maxBufferRecv, 0);
    if (bytes == 0) {
      onWifiCameraStill = false;
      closeCameraConnection();
      onWifiCameraStill = true;
      sleep(2);
      pthread_t waitForCameraConnectionThread;
      pthread_create(&waitForCameraConnectionThread, NULL, &waitForCameraConnection, NULL);
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

    LOGInfo(CAMERA_CONN_TAG, 2, command);
    LOGInfo(CAMERA_CONN_TAG, 2, value);

    if (strcmp(command, "imgL") == 0) {
      LOGInfo(CAMERA_CONN_TAG, 1, "RGB img left sync.");
      sendMatImageByWifi(getImageLeft(), 80);
    }
    else if (strcmp(command, "imgR") == 0) {
      LOGInfo(CAMERA_CONN_TAG, 1, "RGB img right sync.");
      sendMatImageByWifi(getImageRight(), 80);
    }
    else if (strcmp(command, "imgK") == 0) {
      LOGInfo(CAMERA_CONN_TAG, 1, "RGB img kinect sync.");
      sendMatImageByWifi(getRGBKinect(), 80);
    }
    else if (strcmp(command, "depK") == 0) {
      LOGInfo(CAMERA_CONN_TAG, 1, "Depth img kinect sync.");
      sendMatImageByWifi(getDepthScaledKinect(), 80);
    }
    else if (strcmp(command, "depVK") == 0) {
      LOGInfo(CAMERA_CONN_TAG, 1, "Depth valid img kinect sync.");
      sendMatImageByWifi(getDepthValidKinect(), 80);
    }
    else if (strcmp(command, "map") == 0) {
      LOGInfo(CAMERA_CONN_TAG, 1, "Map sync.");
      Mat map = getMapImage().clone();
      RobotPosition_struct robotPosition = getRobotPossition();

      semWait(sem_id, JOURNEY_POINTS);
      for (unsigned int i = 1; i < journeyPoint.size(); i++) {
        line(map, journeyPoint.at(i - 1), journeyPoint.at(i), Scalar( 255, 0, 0 ), 2, 8);
      }
      if (journeyPoint.size() > 1)
        circle(map, journeyPoint.at(1), 4, Scalar( 0, 0, 255 ), -1, 8);
      semPost(sem_id, JOURNEY_POINTS);

      semWait(sem_id, ROBOTACCULATORS);
      double kinectYaw = robotAcculators.kinect.yaw;
      semPost(sem_id, ROBOTACCULATORS);



      circle(map, Point(MAP_WIDTH / 2, MAP_HEIGHT / 2), 200 * MAP_SCALE, Scalar( 0, 0, 255 ), 1, 8); // not visible zone
      if (140 * MAP_SCALE > 0) {
        circle(map, Point(MAP_WIDTH / 2, MAP_HEIGHT / 2), 140 * MAP_SCALE, Scalar( 0, 255, 255 ), 1, 8); // robot zone
        line(map, Point(MAP_WIDTH / 2, MAP_HEIGHT / 2), Point(MAP_WIDTH / 2 + (140 * MAP_SCALE) * cos(robotPosition.anglePossition.yaw), MAP_HEIGHT / 2 + (140 * MAP_SCALE) * sin(robotPosition.anglePossition.yaw)), Scalar( 0, 255, 255 ), 1, 8);
        line(map, Point(MAP_WIDTH / 2, MAP_HEIGHT / 2), Point(MAP_WIDTH / 2 + (140 * MAP_SCALE) * cos(robotPosition.anglePossition.yaw + kinectYaw), MAP_HEIGHT / 2 + (140 * MAP_SCALE) * sin(robotPosition.anglePossition.yaw + kinectYaw)), Scalar( 0, 0, 255 ), 1, 8);
      }
      else {
        circle(map, Point(MAP_WIDTH / 2, MAP_HEIGHT / 2), 1, Scalar( 0, 255, 255 ), 1, 8); // robot zone
      }

      semWait(sem_id, ROBOTSENSORS);
      double ultValue = robotSensors.ultrasonic;
      semPost(sem_id, ROBOTSENSORS);
      char text[20];
      sprintf(text, "yaw:%d deg+%d deg", (int)(robotPosition.anglePossition.yaw * (180 / M_PI)), (int)(kinectYaw * (180 / M_PI)));
      putText(map, text, Point(10, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 0, 255), 1, CV_AA);
      sprintf(text, "ult:%f", ultValue);
      putText(map, text, Point(10, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 0, 255), 1, CV_AA);

      semWait(sem_id, ROBOTSENSORS);
      Point3f operatorPossitionLocal = robotSensors.operatorPossition;
      semPost(sem_id, ROBOTSENSORS);

      double minPointX = operatorPossitionLocal.x * MAP_SCALE;
      double minPointZ = operatorPossitionLocal.z * MAP_SCALE;
      double r = sqrt((double)(minPointX * minPointX + minPointZ * minPointZ));
      double angle = acos((-(double)minPointX) / r) - 3.14 / 2;

      int x2 = r * cos(robotPosition.anglePossition.yaw + angle + kinectYaw) + MAP_WIDTH / 2;
      int z2 = r * sin(robotPosition.anglePossition.yaw + angle + kinectYaw) + MAP_HEIGHT / 2;
      if (!(operatorPossitionLocal.x == -1 && operatorPossitionLocal.z == -1 && operatorPossitionLocal.y == -1)) {
        circle(map, Point(x2, z2), 2, Scalar( 0, 0, 255 ), -1, 8);
        circle(map, Point(x2, z2), MAP_DISTANCE_FROM_OPERATOR * MAP_SCALE, Scalar( 0, 0, 255 ), 1, 8);
      }

      double vzd = MAP_NOT_JORNEY_CALCULATE_BEHIND * MAP_SCALE;

      double angle1 = robotPosition.anglePossition.yaw + kinectYaw + M_PI;
      double x1 = vzd * cos(angle1) + MAP_WIDTH / 2;
      double y1 = vzd * sin(angle1) + MAP_HEIGHT / 2;

      angle1 = angle1 - M_PI / 2;
      double x3 = vzd * cos(angle1) + x1;
      double y3 = vzd * sin(angle1) + y1;

      double k = (y3 - y1) / (x3 - x1);
      double q = y1 - k * x1;
      double yend = k * (MAP_WIDTH - 1) + q;
      if (yend > MAP_HEIGHT - 1) {
        double xend = (MAP_HEIGHT - q) / k;
        line(map, Point(0, (int)q), Point(xend, MAP_HEIGHT - 1), Scalar( 0, 255, 255 ), 1, 8);
      }
      else {
        line(map, Point(0, (int)q), Point(MAP_WIDTH - 1, yend), Scalar( 0, 255, 255 ), 1, 8);
      }

      sendMatImageByWifi(map, 100);
    }
    else if (strcmp(command, "rgbO") == 0) {
      LOGInfo(CAMERA_CONN_TAG, 1, "RGB operator sync.");
      sendMatImageByWifi(getRGBOperator(), 80);
    }
    else if (strcmp(command, "greenO") == 0) {
      LOGInfo(CAMERA_CONN_TAG, 1, "Green mask sync.");
      sendMatImageByWifi(getGreenOperatorMask(), 80);
    }
    else if (strcmp(command, "orangeO") == 0) {
      LOGInfo(CAMERA_CONN_TAG, 1, "Orange mask sync.");
      sendMatImageByWifi(getOrangeOperatorMask(), 80);
    }
    else if (strcmp(command, "iLowHG") == 0) {
      iLowH_green = atoi(value);
    }
    else if (strcmp(command, "iLowSG") == 0) {
      iLowS_green = atoi(value);
    }
    else if (strcmp(command, "iLowVG") == 0) {
      iLowV_green = atoi(value);
    }
    else if (strcmp(command, "iLowHO") == 0) {
      iLowH_orange = atoi(value);
    }
    else if (strcmp(command, "iLowSO") == 0) {
      iLowS_orange = atoi(value);
    }
    else if (strcmp(command, "iLowVO") == 0) {
      iLowV_orange = atoi(value);
    }
    else if (strcmp(command, "iHighHG") == 0) {
      iHighH_green = atoi(value);
    }
    else if (strcmp(command, "iHighSG") == 0) {
      iHighS_green = atoi(value);
    }
    else if (strcmp(command, "iHighVG") == 0) {
      iHighV_green = atoi(value);
    }
    else if (strcmp(command, "iHighHO") == 0) {
      iHighH_orange = atoi(value);
    }
    else if (strcmp(command, "iHighSO") == 0) {
      iHighS_orange = atoi(value);
    }
    else if (strcmp(command, "iHighVO") == 0) {
      iHighV_orange = atoi(value);
    }
    else if (strcmp(command, "erodeO") == 0) {
      erode_orange = atoi(value);
    }
    else if (strcmp(command, "erodeG") == 0) {
      erode_green = atoi(value);
    }
    else if (strcmp(command, "dilateO") == 0) {
      dilate_orange = atoi(value);
    }
    else if (strcmp(command, "dilateG") == 0) {
      dilate_green = atoi(value);
    }

  }
  LOGInfo(CAMERA_CONN_TAG, 1, "End:Camera sync by network.");

  return NULL;
}

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
        robotAcculators.direction = FORWARD;
      else if (strcmp(value, "B") == 0)
        robotAcculators.direction = BACKWARD;
      else if (strcmp(value, "C") == 0)
        robotAcculators.direction = CLOCKWISE;
      else if (strcmp(value, "A") == 0)
        robotAcculators.direction = ANTICLOCKWISE;
      else
        robotAcculators.direction = STOP;

      robotAcculators.isManual = true;

      semPost(sem_id, ROBOTACCULATORS);

      LOGInfo(SENSOR_CONN_TAG, 0, value);
    }
    else if (strcmp(command, "sfol") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Follow sync.");

      semWait(sem_id, ROBOTSENSORS);
      if (strcmp(value, "ON") == 0)
        robotSensors.isFollowOperatorOn = true;
      else if (strcmp(value, "OFF") == 0)
        robotSensors.isFollowOperatorOn = false;

      semPost(sem_id, ROBOTSENSORS);

      LOGInfo(SENSOR_CONN_TAG, 0, value);
    }
    else if (strcmp(command, "gfol") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Follow sync.");

      semWait(sem_id, ROBOTSENSORS);
      bool val = robotSensors.isFollowOperatorOn;
      semPost(sem_id, ROBOTSENSORS);

      if (val == true) sendIntByWifi(1);
      else            sendIntByWifi(0);
    }
    else if (strcmp(command, "sleg") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Leg sync.");
      semWait(sem_id, ROBOTSENSORS);
      //robotAcculators.speed = atoi(value);
      //robotAcculators.isManual = true;
      semPost(sem_id, ROBOTSENSORS);
      LOGInfo(SENSOR_CONN_TAG, 0, value);
    }

    else if (strcmp(command, "gleg") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Leg sync.");
      semWait(sem_id, ROBOTSENSORS);
      //robotAcculators.speed = atoi(value);
      //robotAcculators.isManual = true;
      semPost(sem_id, ROBOTSENSORS);
      LOGInfo(SENSOR_CONN_TAG, 0, value);
    }
    else if (strcmp(command, "sdbrao") == 0) { //distance between robot and operator
      LOGInfo(SENSOR_CONN_TAG, 1, "Distance between robot and operator sync.");
      semWait(sem_id, ROBOTSENSORS);
      //robotAcculators.speed = atoi(value);
      //robotAcculators.isManual = true;
      semPost(sem_id, ROBOTSENSORS);
      LOGInfo(SENSOR_CONN_TAG, 0, value);
    }

    else if (strcmp(command, "gdbrao") == 0) { //distance between robot and operator
      LOGInfo(SENSOR_CONN_TAG, 1, "Distance between robot and operator sync.");
      semWait(sem_id, ROBOTSENSORS);
      //robotAcculators.speed = atoi(value);
      //robotAcculators.isManual = true;
      semPost(sem_id, ROBOTSENSORS);
      LOGInfo(SENSOR_CONN_TAG, 0, value);
    }
    else if (strcmp(command, "speed") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Speed sync.");
      semWait(sem_id, ROBOTACCULATORS);
      robotAcculators.speed = atoi(value);
      robotAcculators.isManual = true;
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
  }
  LOGInfo(SENSOR_CONN_TAG, 1, "End:Sensor sync by network.");
  return NULL;
}

void *syncKinectFrames(void *arg) {
  LOGInfo(SENSOR_CONN_TAG, 1, "Start:Kinect sync frames.");
  while (onAllThreads) {

    if ( !cameraKinect.grab() )
    {
      LOGError(KINECT_TAG, "Problem grab frames.");
    }
    else
    {
      semWait(sem_id, MAT_KINECT1);
      bool ok = true;
      if (!cameraKinect.retrieve(depthValidMask1Kinect , CV_CAP_OPENNI_VALID_DEPTH_MASK)) {
        ok = false;
      }
      if (ok == true && !cameraKinect.retrieve( depth1Kinect, CAP_OPENNI_DEPTH_MAP    )) {
        ok = false;
      }
      if (ok == true && !cameraKinect.retrieve( img1Kinect, CAP_OPENNI_BGR_IMAGE )) {
        ok = false;
      }
      if (ok == true && !cameraKinect.retrieve( pointCloudMap1Kinect, CAP_OPENNI_POINT_CLOUD_MAP )) {
        ok = false;
      }

      semPost(sem_id, MAT_KINECT1);
      if (ok) {
        semWait(sem_id, MAT_KINECT_VARIABLE);
        matChooseKinect = 1;
        semPost(sem_id, MAT_KINECT_VARIABLE);
      }
      else {
        LOGError(KINECT_TAG, "Problem retreive frames.");
      }
    }
    cvWaitKey(10);

    if ( !cameraKinect.grab() )
    {
      LOGError(KINECT_TAG, "Problem grab frames.");
    }
    else
    {
      semWait(sem_id, MAT_KINECT2);
      bool ok = true;
      if (!cameraKinect.retrieve(depthValidMask2Kinect , CV_CAP_OPENNI_VALID_DEPTH_MASK)) {
        ok = false;
      }
      if (ok == true && !cameraKinect.retrieve( depth2Kinect, CAP_OPENNI_DEPTH_MAP    )) {
        ok = false;
      }
      if (ok == true && !cameraKinect.retrieve( img2Kinect, CAP_OPENNI_BGR_IMAGE )) {
        ok = false;
      }
      if (ok == true && !cameraKinect.retrieve( pointCloudMap2Kinect, CAP_OPENNI_POINT_CLOUD_MAP )) {
        ok = false;
      }

      semPost(sem_id, MAT_KINECT2);
      if (ok) {
        semWait(sem_id, MAT_KINECT_VARIABLE);
        matChooseKinect = 2;
        semPost(sem_id, MAT_KINECT_VARIABLE);
      }
      else {
        LOGError(KINECT_TAG, "Problem retreive frames.");
      }
    }
    cvWaitKey(10);
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

int pocKinecYawMotor = 0;
void *syncMotors(bool checkChange) {
  LOGInfo(NUCLEO_TAG, 1, "Start:Motors sync.");
  semWait(sem_id, ROBOTACCULATORS);
  bool isManual = robotAcculators.isManual;
  direction_t  direction = robotAcculators.direction;
  unsigned int speed = robotAcculators.speed;
  double angle = robotAcculators.angle;
  long distance = robotAcculators.distance;
  double kinectYaw = robotAcculators.kinect.yaw;
  semPost(sem_id, ROBOTACCULATORS);
  if (!checkChange) {
    if (isManual) {
      setMove(direction, speed );
    }
    else {
      setAngleRobot(angle);
      setDistanceRobot(distance);
      setGoToAngleAndDistance();
    }

    setServo((int)(kinectYaw * (180 / M_PI)));

    semWait(sem_id, ROBOTACCULATORS_LAST);
    double kinectYawLast = robotAcculatorsLast.kinect.yaw;
    semPost(sem_id, ROBOTACCULATORS_LAST);

    if (kinectYaw != kinectYawLast) {
      pocKinecYawMotor = 0;
    }

    semWait(sem_id, ROBOTACCULATORS_LAST);
    robotAcculatorsLast.direction = direction;
    robotAcculatorsLast.speed = speed;
    robotAcculatorsLast.kinect.yaw = kinectYaw;
    robotAcculatorsLast.isManual = isManual;
    semPost(sem_id, ROBOTACCULATORS_LAST);
  }
  else {
    semWait(sem_id, ROBOTACCULATORS_LAST);
    direction_t  directionLast = robotAcculatorsLast.direction;
    unsigned int speedLast = robotAcculatorsLast.speed;
    double kinectYawLast = robotAcculatorsLast.kinect.yaw;
    double angleLast = robotAcculatorsLast.angle;
    long distanceLast = robotAcculatorsLast.distance;
    bool isManualLast = robotAcculatorsLast.isManual;
    semPost(sem_id, ROBOTACCULATORS_LAST);

    if (isManual) {
      if (direction != directionLast || speed != speedLast || isManual != isManualLast) {
        setMove(direction, speed );
        semWait(sem_id, ROBOTACCULATORS_LAST);
        robotAcculatorsLast.direction = direction;
        robotAcculatorsLast.speed = speed;
        robotAcculatorsLast.isManual = isManual;
        semPost(sem_id, ROBOTACCULATORS_LAST);
        LOGInfo(NUCLEO_TAG, 0, "Change motors sync.");
      }
    }
    else {
      if (angle != angleLast || isManual != isManualLast) {
        setAngleRobot(angle);
        semWait(sem_id, ROBOTACCULATORS_LAST);
        robotAcculatorsLast.angle = angle;
        semPost(sem_id, ROBOTACCULATORS_LAST);
        LOGInfo(NUCLEO_TAG, 0, "Change angle sync.");
      }
      if (distance != distanceLast || isManual != isManualLast) {
        setDistanceRobot(distance);
        semWait(sem_id, ROBOTACCULATORS_LAST);
        robotAcculatorsLast.distance = distance;
        semPost(sem_id, ROBOTACCULATORS_LAST);
        LOGInfo(NUCLEO_TAG, 0, "Change distance sync.");
      }

      if (angle != angleLast || distance != distanceLast) {
        setGoToAngleAndDistance();
      }

      if (isManual != isManualLast) {
        semWait(sem_id, ROBOTACCULATORS_LAST);
        robotAcculatorsLast.isManual = isManual;
        semPost(sem_id, ROBOTACCULATORS_LAST);
      }
    }

    if (kinectYawLast != kinectYaw) {
      setServo((int)(kinectYaw * (180 / M_PI)));
      semWait(sem_id, ROBOTACCULATORS_LAST);
      robotAcculatorsLast.kinect.yaw = kinectYaw;
      semPost(sem_id, ROBOTACCULATORS_LAST);
      LOGInfo(NUCLEO_TAG, 0, "Change servo sync.");
      semWait(sem_id, ROBOTSENSORS);
      robotSensors.kinect.motorStatusYaw = MOTORSTATUSKINECT_MOVING;
      semPost(sem_id, ROBOTSENSORS);
      pocKinecYawMotor = 0;
    }
    else if (pocKinecYawMotor > 80) {
      semWait(sem_id, ROBOTSENSORS);
      robotSensors.kinect.motorStatusYaw = MOTORSTATUSKINECT_STOPPED;
      semPost(sem_id, ROBOTSENSORS);
      LOGInfo(NUCLEO_TAG, 0, "Change status motor.");
      pocKinecYawMotor = -1;
    }
    else if (pocKinecYawMotor != -1) {
      pocKinecYawMotor++;
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

    Mat map = getMapImage();
    if (map.empty())
      map = Mat(MAP_HEIGHT, MAP_WIDTH,  CV_8UC3, Scalar(0, 0, 0));

    RobotPosition_struct robotPosition = getRobotPossition();
    long robotX = robotPosition.axisPossition.x * MAP_SCALE;
    long robotY = robotPosition.axisPossition.y * MAP_SCALE;

    int deltaX = robotX - mapOffsetX;
    int deltaY = robotY - mapOffsetY;
    if (deltaX < (MAX_DELTA_TRANSLATE * MAP_SCALE) && deltaY < (MAX_DELTA_TRANSLATE * MAP_SCALE)) {
      translateImg(map, -deltaX, -deltaY); // posun mapy aby sme nestratili predchadzajuce udaje, vzdy si pametame len okolie robota
      mapOffsetX = robotX;
      mapOffsetY = robotY;
    }

    //zabudanie
    for (int x = 0; x < map.cols; x++) {
      for (int y = 0; y < map.rows; y++) {
        Vec3b color = map.at<Vec3b>(Point(x, y));

        int varMapOblivion[3];
        varMapOblivion[1] = (int)color[1] - SPEED_OF_MAP_OBLIVION;

        if (varMapOblivion[1] <= 0)
          color[1] = 0;
        else
          color[1] = varMapOblivion[1];

        map.at<Vec3b>(Point(x, y)) = color;
      }
    }

    /*

        //mapovanie na zaklade ultrazvuku
        semWait(sem_id, ROBOTSENSORS);
        double valueUlt = robotSensors.ultrasonic;
        semPost(sem_id, ROBOTSENSORS);
       for (double angle = -(M_PI / 180) * 15; angle <= (M_PI / 180) * 15; angle += (((M_PI / 180) * 15) / 100)) {

          int x = valueUlt * (10 * MAP_SCALE) * cos(robotPosition.anglePossition.yaw + angle) + MAP_WIDTH / 2;
          int y = valueUlt * (10 * MAP_SCALE) * sin(robotPosition.anglePossition.yaw + angle) + MAP_WIDTH / 2;

          if (x > 0 && x < MAP_WIDTH && y > 0 && y < MAP_HEIGHT) {
            Vec3b color = map.at<Vec3b>(Point(x, y));
            int varMapCreation[3];
            varMapCreation[1] = (int)color[1] + SPEED_OF_MAP_CREATION_ULT;

            if (varMapCreation[1] >= 255)
              color[1] = 255;
            else
              color[1] = varMapCreation[1];

            map.at<Vec3b>(Point(x, y)) = color;
          }
        }
    */
    semWait(sem_id, ROBOTSENSORS);
    motorStatusKinect_t motorYawStatus = robotSensors.kinect.motorStatusYaw;
    motorStatusKinect_t motorRollStatus = robotSensors.kinect.motorStatusRoll;
    semPost(sem_id, ROBOTSENSORS);

    if (motorYawStatus != MOTORSTATUSKINECT_MOVING && motorRollStatus != MOTORSTATUSKINECT_MOVING) {
      char index = pauseGrabKinect();
      Mat pointCloudBuffer = getPointCloudMapKinect(index);
      Mat depthValid = getDepthValidKinect(index);
      continueGrabKinect(index);

      semWait(sem_id, ROBOTACCULATORS);
      double kinectYaw = robotAcculators.kinect.yaw;
      semPost(sem_id, ROBOTACCULATORS);

      //mapovanie na zaklade kinectu
      for (int x = 0; x < pointCloudBuffer.cols; x++) {
        for (int y = 0; y < pointCloudBuffer.rows; y++) {
          if ( !pointCloudBuffer.empty() && depthValid.at<uchar>(y, x) > 0)   {
            Point3f p = pointCloudBuffer.at<Point3f>(y, x);
            float kinectPointX = p.x;         //pozicia vlavo/vpravo na kinecte
            float kinectpointY = p.y + 0.21;  //pozicia hore/dole na kinecte
            float kinectPointZ = p.z;         //hlbka

            if (kinectpointY >= MIN_BARRIER_HEIGHT && kinectpointY <= MAX_BARRIER_HEIGHT) {
              int pointZ = kinectPointZ * (1000 * MAP_SCALE);
              int pointX = kinectPointX * (1000 * MAP_SCALE);

              double r = sqrt((double)(pointX * pointX + pointZ * pointZ));
              double angle = acos((-(double)pointX) / r) - 3.14 / 2;

              int x2 = r * cos(robotPosition.anglePossition.yaw + angle + kinectYaw) + MAP_WIDTH / 2;
              int z2 = r * sin(robotPosition.anglePossition.yaw + angle + kinectYaw) + MAP_HEIGHT / 2;

              if (x2 >= 0 && x2 < MAP_WIDTH && z2 >= 0 && z2 < MAP_HEIGHT) {

                Vec3b color = map.at<Vec3b>(Point(x2, z2));

                int varMapCreation[3];
                varMapCreation[1] = (int)color[1] + SPEED_OF_MAP_CREATION_KINECT;

                if (varMapCreation[1] >= 255)
                  color[1] = 255;
                else
                  color[1] = varMapCreation[1];

                map.at<Vec3b>(Point(x2, z2)) = color;
              }
            }
          }
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
    usleep(SYNC_MAP_GENERATE_TIME);
  }
  return NULL;
}

void *syncOperatorDetect(void *arg) {
  signed char offsetXOperator = 0;
  char lastIndex = 0;
  while (onAllThreads) {
    LOGInfo(OPERATOR_TAG, 1, "Start:Operator sync.");

    char index = pauseGrabKinect();
    if (index != lastIndex) {
      lastIndex = index;
      Mat rgbOperatorImage = getRGBKinect(index);
      Mat depthValid = getDepthValidKinect(index);
      Mat pointCloudBuffer = getPointCloudMapKinect(index);
      continueGrabKinect(index);

      Mat hsvOperatorImage;
      Mat orangeOperatorMaskImage;
      Mat greenOperatorMaskImage;
      Mat depthOperatorMaskImage = Mat(Size(OPERATOR_ANALYSE_WIDTH, OPERATOR_ANALYSE_HEIGHT), CV_8UC1, Scalar(0));
      Point3f operatorPossitionLocal = Point3f(-1, -1, -1);

      if (!rgbOperatorImage.empty() && !depthValid.empty() && !pointCloudBuffer.empty()) {
        resize(rgbOperatorImage, rgbOperatorImage, Size(OPERATOR_ANALYSE_WIDTH, OPERATOR_ANALYSE_HEIGHT));

        cvtColor(rgbOperatorImage, hsvOperatorImage, COLOR_BGR2HSV); //konverzia na hsv model
        inRange(hsvOperatorImage, Scalar(iLowH_green, iLowS_green, iLowV_green), Scalar(iHighH_green, iHighS_green, iHighV_green), greenOperatorMaskImage); //vytvorenie masky zelenej farby
        inRange(hsvOperatorImage, Scalar(iLowH_orange, iLowS_orange, iLowV_orange), Scalar(iHighH_orange, iHighS_orange, iHighV_orange), orangeOperatorMaskImage); //vytvorenie maasky oranzovej farby

        Mat element = getStructuringElement( MORPH_ELLIPSE,
                                             Size( 2 * erode_green + 1, 2 * erode_green + 1 ),
                                             Point( erode_green, erode_green ) );
        erode( greenOperatorMaskImage, greenOperatorMaskImage, element );

        element = getStructuringElement( MORPH_ELLIPSE,
                                         Size( 2 * erode_orange + 1, 2 * erode_orange + 1 ),
                                         Point( erode_orange, erode_orange ) );
        erode( orangeOperatorMaskImage, orangeOperatorMaskImage, element );

        element = getStructuringElement( MORPH_ELLIPSE,
                                         Size( 2 * dilate_green + 1, 2 * dilate_green + 1 ),
                                         Point( dilate_green, dilate_green ) );
        dilate( greenOperatorMaskImage, greenOperatorMaskImage, element );

        element = getStructuringElement( MORPH_ELLIPSE,
                                         Size( 2 * dilate_orange + 1, 2 * dilate_orange + 1 ),
                                         Point( dilate_orange, dilate_orange ) );
        dilate( orangeOperatorMaskImage, orangeOperatorMaskImage, element );

        Point operatorCenter;
        int choose_people = -1;
        int choose_orange = -1;
        int choose_green = -1;
        long maxArea = -1;

        vector< vector<Point> > contours_green;
        findContours(greenOperatorMaskImage.clone(), contours_green, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
        Rect r_green[contours_green.size()];
        long areasGreen[contours_green.size()];
        for (unsigned int i = 0; i < contours_green.size(); i++) {
          areasGreen[i] = 0;
          r_green[i] = boundingRect(contours_green[i]);
        }

        vector< vector<Point> > contours_orange;
        findContours(orangeOperatorMaskImage.clone(), contours_orange, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
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

              if (areasGreen[index_green] <= 0) areasGreen[index_green] = contourArea(Mat(contours_green[index_green]));
              if (areasOrange[index_orange] <= 0) areasOrange[index_orange] = contourArea(Mat(contours_orange[index_orange]));

              if (areasGreen[index_green] > OPERATOR_MIN_AREA_GREEN && areasOrange[index_orange] > OPERATOR_MIN_AREA_ORANGE) {
                long area = areasGreen[index_green] + areasOrange[index_orange];
                if (maxArea < area)
                {
                  maxArea = area;
                  choose_green = index_green;
                  choose_orange = index_orange;
                  operatorCenter = Point((centerGreenX + centerOrangeX) / 2, (centerGreenY + centerOrangeY) / 2);
                }
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

          double operatorCameraPossitionX = (double)operatorCenter.x * ((double)pointCloudBuffer.cols / OPERATOR_ANALYSE_WIDTH);
          double operatorCameraPossitionY = (double)operatorCenter.y * ((double)pointCloudBuffer.rows / OPERATOR_ANALYSE_HEIGHT);

          if (operatorCenter.x > (OPERATOR_ANALYSE_WIDTH / 2))
            offsetXOperator = 1;
          else if (operatorCenter.x < (OPERATOR_ANALYSE_WIDTH / 2))
            offsetXOperator = -1;
          else
            offsetXOperator = 0;

          for (int okolie = 0; okolie < 10; okolie++) {
            if (depthValid.at<uchar>(operatorCameraPossitionY + okolie, operatorCameraPossitionX) > 0) {
              operatorPossitionLocal = pointCloudBuffer.at<Point3f>(operatorCameraPossitionY + okolie, operatorCameraPossitionX);
              break;
            }
            else if (okolie != 0 && depthValid.at<uchar>(operatorCameraPossitionY , operatorCameraPossitionX + okolie) > 0) {
              operatorPossitionLocal = pointCloudBuffer.at<Point3f>(operatorCameraPossitionY, operatorCameraPossitionX + 5);
              break;
            }
            else if (okolie != 0 && depthValid.at<uchar>(operatorCameraPossitionY , operatorCameraPossitionX - okolie) > 0) {
              operatorPossitionLocal = pointCloudBuffer.at<Point3f>(operatorCameraPossitionY, operatorCameraPossitionX - 5);
              break;
            }
            else if (okolie != 0 && depthValid.at<uchar>(operatorCameraPossitionY - okolie , operatorCameraPossitionX) > 0) {
              operatorPossitionLocal = pointCloudBuffer.at<Point3f>(operatorCameraPossitionY - okolie, operatorCameraPossitionX);
              break;
            }
          }

          if (operatorPossitionLocal.x == -1 && operatorPossitionLocal.y == -1 && operatorPossitionLocal.z == -1) {
            semWait(sem_id, ROBOTACCULATORS);
            double alfa0 = robotAcculators.kinect.roll;
            semPost(sem_id, ROBOTACCULATORS);
            double alfa1 = (OPERATOR_ANALYSE_HEIGHT - (double)maxY) * ((43 * (M_PI / 180)) / OPERATOR_ANALYSE_HEIGHT);
            double alfa2 = (operatorCenter.x - OPERATOR_ANALYSE_WIDTH / 2) * ((57 * (M_PI / 180)) / OPERATOR_ANALYSE_HEIGHT);
            operatorPossitionLocal.z = OPERATOR_LEG_HEIGHT / tan(alfa0 + alfa1);
            operatorPossitionLocal.x = tan(alfa2) * operatorPossitionLocal.z;
            operatorPossitionLocal.y = -1;
          }

          if (operatorPossitionLocal.x != -1 && operatorPossitionLocal.y != -1 && operatorPossitionLocal.z != -1) {
            operatorPossitionLocal.x = operatorPossitionLocal.x * 1000;
            operatorPossitionLocal.y = operatorPossitionLocal.y * 1000;
            operatorPossitionLocal.z = operatorPossitionLocal.z * 1000;
          }

#if CAMERA_WIFI == 1
          if (choose_orange != -1)
            drawContours(orangeOperatorMaskImage, contours_orange, choose_orange, Scalar(255), CV_FILLED);
          if (choose_green != -1)
            drawContours(greenOperatorMaskImage, contours_green, choose_green, Scalar(255), CV_FILLED);
          rectangle(rgbOperatorImage, Point(minX, minY), Point(maxX, maxY), CV_RGB(0, 255, 0), 3, 8, 0);
          circle(rgbOperatorImage, operatorCenter, 5, Scalar( 0, 255, 255 ), -1, 8);
          char text[20];
          sprintf(text, "%d,%d,O%d", (int)(operatorCenter.x * ((double)pointCloudBuffer.cols / OPERATOR_ANALYSE_WIDTH)), (int)(operatorCenter.y * ((double)pointCloudBuffer.rows / OPERATOR_ANALYSE_HEIGHT)), choose_people);
          putText(rgbOperatorImage, text, Point(10, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 0, 255), 1, CV_AA);
#endif
        }
      }

      semWait(sem_id, ROBOTSENSORS);
      robotSensors.operatorPossition = operatorPossitionLocal;
      robotSensors.lastOperatorDirection = offsetXOperator;
      semPost(sem_id, ROBOTSENSORS);

      if (OPERATOR_STATUS_KINECT_LED) {
        semWait(sem_id, ROBOTACCULATORS);
        if (operatorPossitionLocal.x != -1 && operatorPossitionLocal.y != -1 && operatorPossitionLocal.z != -1) {
          robotAcculators.ledKinect = LEDKINECT_GREEN;
        }
        else {
          robotAcculators.ledKinect = LEDKINECT_RED;

          //printf("%d,%f\n", offsetXOperator, robotAcculators.kinect.yaw * (180 / M_PI));
        }
        semPost(sem_id, ROBOTACCULATORS);
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
      usleep(SYNC_OPERATOR_DETECT_TIME);
    }
    else {
      continueGrabKinect(index);
    }
  }
  return NULL;
}

Point posunBoduPodlaPriority(int priority, char rightLeft, char upDown) {
  if (rightLeft == 'R' && upDown == 'U') { // rozlozenie 4
    switch (priority) {
    case 1:
      return sipkaHore;
    case 2:
      return sipkaVpravo;
    case 3:
      return sipkaVlavo;
    case 4:
      return sipkaDole;
    case 5:
      return sipkaVpravoHore;
    case 6:
      return sipkaVlavoHore;
    case 7:
      return sipkaVpravoDole;
    case 8:
      return sipkaVlavoDole;
    }
  } else if (rightLeft == 'R' && upDown == 'D') { //rozlozenie 2
    switch (priority) {
    case 1:
      return sipkaVpravo;
    case 2:
      return sipkaDole;
    case 3:
      return sipkaHore;
    case 4:
      return sipkaVlavo;
    case 5:
      return sipkaVpravoDole;
    case 6:
      return sipkaVpravoHore;
    case 7:
      return sipkaVlavoDole;
    case 8:
      return sipkaVlavoHore;
    }
  } else if (rightLeft == 'L' && upDown == 'U') { //rozlozenie 1
    switch (priority) {
    case 1:
      return sipkaHore;
    case 2:
      return sipkaVlavo;
    case 3:
      return sipkaVpravo;
    case 4:
      return sipkaDole;
    case 5:
      return sipkaVlavoHore;
    case 6:
      return sipkaVpravoHore;
    case 7:
      return sipkaVlavoDole;
    case 8:
      return sipkaVpravoDole;
    }
  } else if (rightLeft == 'L' && upDown == 'D') { //rozlozenie 3
    switch (priority) {
    case 1:
      return sipkaDole;
    case 2:
      return sipkaVlavo;
    case 3:
      return sipkaHore;
    case 4:
      return sipkaVpravo;
    case 5:
      return sipkaVlavoDole;
    case 6:
      return sipkaVlavoHore;
    case 7:
      return sipkaVpravoDole;
    case 8:
      return sipkaVpravoHore;
    }
  } else if (rightLeft == 'R' && upDown == 'X') { //rozlozenie 5
    switch (priority) {
    case 1:
      return sipkaVpravo;
    case 2:
      return sipkaHore;
    case 3:
      return sipkaDole;
    case 4:
      return sipkaVlavo;
    case 5:
      return sipkaVpravoHore;
    case 6:
      return sipkaVpravoDole;
    case 7:
      return sipkaVlavoHore;
    case 8:
      return sipkaVlavoDole;
    }
  } else if (rightLeft == 'L' && upDown == 'X') { //rozlozenie 6
    switch (priority) {
    case 1:
      return sipkaVlavo;
    case 2:
      return sipkaHore;
    case 3:
      return sipkaDole;
    case 4:
      return sipkaVpravo;
    case 5:
      return sipkaVlavoHore;
    case 6:
      return sipkaVlavoDole;
    case 7:
      return sipkaVpravoHore;
    case 8:
      return sipkaVpravoDole;
    }
  } else if (upDown == 'U' && rightLeft == 'Q') { //rozlozenie 8
    switch (priority) {
    case 1:
      return sipkaHore;
    case 2:
      return sipkaVpravo;
    case 3:
      return sipkaVlavo;
    case 4:
      return sipkaDole;
    case 5:
      return sipkaVpravoHore;
    case 6:
      return sipkaVlavoHore;
    case 7:
      return sipkaVpravoDole;
    case 8:
      return sipkaVlavoDole;
    }
  } else if (upDown == 'D' && rightLeft == 'Q') { //rozlozenie 7
    switch (priority) {
    case 1:
      return sipkaDole;
    case 2:
      return sipkaVpravo;
    case 3:
      return sipkaVlavo;
    case 4:
      return sipkaHore;
    case 5:
      return sipkaVpravoDole;
    case 6:
      return sipkaVlavoDole;
    case 7:
      return sipkaVpravoHore;
    case 8:
      return sipkaVlavoHore;
    }
  }
}


bool osemsusednost(Mat &map, vector<Point> *prejdiBody, int x, int y, int nasledujuceOhodnotenie, int xO, int yO, Point *realFinish, double k, double q) {

  Point bodNaPrejdenie;
  for (int i = 0; i < 8; i++) {
    switch (i) {
    case 0: bodNaPrejdenie = Point(x - 1, y); break;
    case 1: bodNaPrejdenie = Point(x, y - 1); break;
    case 2: bodNaPrejdenie = Point(x + 1, y); break;
    case 3: bodNaPrejdenie = Point(x, y + 1); break;
    case 4: bodNaPrejdenie = Point(x - 1, y - 1); break;
    case 5: bodNaPrejdenie = Point(x + 1, y + 1); break;
    case 6: bodNaPrejdenie = Point(x + 1, y - 1); break;
    case 7: bodNaPrejdenie = Point(x - 1, y + 1); break;
    }

    int priamkaY = (int)(k * bodNaPrejdenie.x * k + q);

    if (bodNaPrejdenie.x >= 0 && bodNaPrejdenie.x < MAP_WIDTH &&
        bodNaPrejdenie.y >= 0 && bodNaPrejdenie.y < MAP_HEIGHT && priamkaY != bodNaPrejdenie.y) {
      Vec3b color = map.at<Vec3b>(bodNaPrejdenie);
      if (color[1] < MIN_BARRIER_CREATION && color[0] <= 0 && color[2] <= 0) {
        bool isAtFinish = sqrt(pow(bodNaPrejdenie.x - xO, 2) + pow(bodNaPrejdenie.y - yO, 2)) <= MAP_DISTANCE_FROM_OPERATOR * MAP_SCALE;
        if (!isAtFinish) {
          color[0] = nasledujuceOhodnotenie;
          map.at<Vec3b>(bodNaPrejdenie) = color;
          (*prejdiBody).push_back(bodNaPrejdenie);
        }
        else {
          (*realFinish) = bodNaPrejdenie;
          return true;
        }
      }
    }
  }
  return false;
}

int getPointIndexWithOhodnotenie(Mat map, vector<Point> prejdiBody, int aktualneOhodnotenie) {
  for (unsigned int i = 0; i < prejdiBody.size(); i++) {
    Vec3b color = map.at<Vec3b>(prejdiBody.at(i));
    if (color[0] == aktualneOhodnotenie)
      return i;
  }
  return -1;
}

void *syncGenerateJorney(void *arg) {
  while (onAllThreads) {
    LOGInfo(MAP_TAG, 1, "Start:Jorney generate sync.");

    semWait(sem_id, ROBOTSENSORS);
    RobotPosition_struct robotPosition = robotSensors.robotPosition;
    double kinectYaw = robotAcculators.kinect.yaw;
    Point3f operatorPossitionLocal = robotSensors.operatorPossition;
    semPost(sem_id, ROBOTSENSORS);

    double minPointX = operatorPossitionLocal.x * MAP_SCALE;
    double minPointZ = operatorPossitionLocal.z * MAP_SCALE;
    double r = sqrt((double)(minPointX * minPointX + minPointZ * minPointZ));
    double angle = acos((-(double)minPointX) / r) - 3.14 / 2;

    int xO = r * cos(robotPosition.anglePossition.yaw + angle + kinectYaw) + MAP_WIDTH / 2;
    int zO = r * sin(robotPosition.anglePossition.yaw + angle + kinectYaw) + MAP_HEIGHT / 2;

    if (operatorPossitionLocal.z != -1 || operatorPossitionLocal.x != -1) {
      Point realFinish;
      Mat map = getMapImage();
      if (map.empty())
        map = Mat(MAP_HEIGHT, MAP_WIDTH,  CV_8UC3, Scalar(255, 255, 255));

      double vzd = MAP_NOT_JORNEY_CALCULATE_BEHIND * MAP_SCALE;
      double x1 = vzd * cos(robotPosition.anglePossition.yaw + kinectYaw + M_PI) + MAP_WIDTH / 2;
      double y1 = vzd * sin(robotPosition.anglePossition.yaw + kinectYaw + M_PI) + MAP_HEIGHT / 2;
      double x3 = vzd * cos(robotPosition.anglePossition.yaw + kinectYaw + M_PI / 2) + x1;
      double y3 = vzd * sin(robotPosition.anglePossition.yaw + kinectYaw + M_PI / 2) + y1;
      double k = (y3 - y1) / (x3 - x1);
      double q = y1 - k * x1;

      //rozsirenie prekazok----------------------------
      for (int x = 0; x < map.cols; x++) {
        for (int y = 0; y < map.rows; y++) {
          Vec3b color = map.at<Vec3b>(Point(x, y));
          if (color[1] >= MIN_BARRIER_CREATION) {
            for (int xRect = -(MAP_BARRIER_EXTENDED * MAP_SCALE); xRect <= (MAP_BARRIER_EXTENDED * MAP_SCALE); xRect++) {
              for (int yRect = -(MAP_BARRIER_EXTENDED * MAP_SCALE); yRect <= (MAP_BARRIER_EXTENDED * MAP_SCALE); yRect++) {
                int changePointX = x + xRect;
                int changePointY = y + yRect;
                if (changePointX >= 0 && changePointX < MAP_WIDTH && changePointY >= 0 && changePointY < MAP_HEIGHT) {
                  color = map.at<Vec3b>(Point(changePointX, changePointY));
                  color[2] = 255;
                  map.at<Vec3b>(Point(changePointX, changePointY)) = color;
                }
              }
            }
          }
        }
      }
      //-----------------------------------------------

      int aktualneOhodnotenie = 0;
      vector<Point> prejdiBody;

      //zaplavenie mapy---------------------------------------------------
      osemsusednost(map, &prejdiBody, MAP_WIDTH / 2, MAP_HEIGHT / 2, aktualneOhodnotenie + 1, xO, zO, &realFinish, k, q);

      bool isAtFinish = false;
      while (prejdiBody.size() > 0 && !isAtFinish) {

        int refBodIndex = getPointIndexWithOhodnotenie(map, prejdiBody, aktualneOhodnotenie);
        if (refBodIndex != -1) {

          //printf("%d,%d,%d\n", prejdiBody.size(), aktualneOhodnotenie, refBodIndex);
          isAtFinish = osemsusednost(map, &prejdiBody, prejdiBody.at(refBodIndex).x, prejdiBody.at(refBodIndex).y, aktualneOhodnotenie + 1, xO, zO, &realFinish, k, q);
          prejdiBody.erase (prejdiBody.begin() + refBodIndex);
        }
        else {
          //printf("%d,%d\n", prejdiBody.size(), aktualneOhodnotenie);
          aktualneOhodnotenie++;
        }
      }
      //------------------------------------------------------------------
      char rightLeft;
      char upDown;
      if (MAP_WIDTH / 2 < realFinish.x) {
        rightLeft = 'L';
      } else if (MAP_WIDTH / 2 == realFinish.x) {
        rightLeft = 'Q';
      } else {
        rightLeft = 'R';
      }
      if (MAP_HEIGHT / 2 < realFinish.y) {
        upDown = 'U';
      } else if (MAP_HEIGHT / 2 == realFinish.y) {
        upDown = 'X';
      } else {
        upDown = 'D';
      }

      vector<Point> journeyPointReverse;
      Point oznacenyBod = realFinish;
      int lastPriorita = -1;
      bool moznyPosun = true;
      while (aktualneOhodnotenie != 1 && moznyPosun) {
        moznyPosun = false;
        for (int priorita = 1; priorita <= 8; priorita++) {
          Point offsetBodu = posunBoduPodlaPriority(priorita, rightLeft, upDown);
          Point moznyNasledujuciBod = Point(oznacenyBod.x + offsetBodu.x, oznacenyBod.y + offsetBodu.y);
          if (moznyNasledujuciBod.x >= 0 && moznyNasledujuciBod.x < MAP_WIDTH &&
              moznyNasledujuciBod.y >= 0 && moznyNasledujuciBod.y < MAP_HEIGHT) {
            Vec3b color = map.at<Vec3b>(moznyNasledujuciBod);
            if (color[0] == aktualneOhodnotenie) {
              if (priorita != lastPriorita && lastPriorita != -1) { //zmenil sa smer zapiseme si ho
                journeyPointReverse.push_back(oznacenyBod);
              }
              lastPriorita = priorita;
              oznacenyBod = moznyNasledujuciBod;

              aktualneOhodnotenie--;
              moznyPosun = true;
              break;
            }
          }
        }
      }
      //zapis bodov----------------
      semWait(sem_id, JOURNEY_POINTS);
      journeyPoint.clear();
      if (moznyPosun) {
        journeyPoint.push_back(Point(MAP_WIDTH / 2, MAP_HEIGHT / 2));
        for (int i = journeyPointReverse.size() - 1; i >= 0; i--) {
          journeyPoint.push_back(journeyPointReverse.at(i));
        }
        journeyPoint.push_back(realFinish);
      }
      semPost(sem_id, JOURNEY_POINTS);
      //---------------------------
    }
    else {
      semWait(sem_id, JOURNEY_POINTS);
      journeyPoint.clear();
      semPost(sem_id, JOURNEY_POINTS);
    }
    LOGInfo(MAP_TAG, 1, "End:Jorney generate sync.");
    usleep(SYNC_MAP_GENERATE_TIME);
  }
  return NULL;
}

void *syncKinectMotor(bool checkChange) {
  //https://openkinect.org/wiki/Protocol_Documentation#Control_Packet_Structure
  unsigned char empty[1];
  LOGInfo(KINECT_TAG, 1, "Start:Motor sync.");
  semWait(sem_id, ROBOTACCULATORS);
  int angle = robotAcculators.kinect.roll * (180 / M_PI) * 2;
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

      LOGInfo(KINECT_TAG, 1, "Change motor sync.");
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

  robotSensors.kinect.motorStatusRoll = (motorStatusKinect_t)pBuffer[9];

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
    if (timeRunThread[threadIndex] > SYNC_MOTORS_TIME * 2)
      LOGError(I2C_TAG, "Twice time motor sync.");
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
    if (timeRunThread[threadIndex] > SYNC_KINECTMOTOR_TIME * 2)
      LOGError(I2C_TAG, "Twice time kinect motor sync.");
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
    if (timeRunThread[threadIndex] > SYNC_LEDS_TIME * 2)
      LOGError(I2C_TAG, "Twice time led sync.");
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

    if (timeRunThread[threadIndex] > SYNC_KINECTLED_TIME * 2)
      LOGError(I2C_TAG, "Twice time kinect led sync.");
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
    if (timeRunThread[threadIndex] > (SYNC_ULTRASONIC_TIME * 2))
      LOGError(I2C_TAG, "Twice time ultrasonic sync.");
    if (timeRunThread[threadIndex] > SYNC_ULTRASONIC_TIME) {
      syncUltrasonic(NULL);
//printf("%lu\n",timeRunThread[threadIndex]);
      timeRunThread[threadIndex] = 0;
    }
    threadIndex++;
    usleep(SYNC_MIN_TIME / NUMBER_OF_MODULES);
#endif

#if ENABLE_BUTTONS == 1
    if (timeRunThread[threadIndex] > SYNC_BUTTONS_TIME * 2)
      LOGError(I2C_TAG, "Twice time button sync.");
    if (timeRunThread[threadIndex] > SYNC_BUTTONS_TIME) {
      syncButtons(NULL);
      timeRunThread[threadIndex] = 0;
    }
    threadIndex++;
    usleep(SYNC_MIN_TIME / NUMBER_OF_MODULES);
#endif

#if ENABLE_POSSITION == 1
    if (timeRunThread[threadIndex] > SYNC_POSSITION_TIME * 2)
      LOGError(I2C_TAG, "Twice time possition sync.");
    if (timeRunThread[threadIndex] > SYNC_POSSITION_TIME) {
      syncPossition(NULL);
      timeRunThread[threadIndex] = 0;
    }
    threadIndex++;
    usleep(SYNC_MIN_TIME / NUMBER_OF_MODULES);
#endif

#if ENABLE_KINECTSENSORS == 1
    if (timeRunThread[threadIndex] > SYNC_KINECTSENSORS_TIME * 2)
      LOGError(I2C_TAG, "Twice time kinect sensor sync.");
    if (timeRunThread[threadIndex] > SYNC_KINECTSENSORS_TIME) {
      syncKinectSensors(NULL);
      timeRunThread[threadIndex] = 0;
    }
    threadIndex++;
    usleep(SYNC_MIN_TIME / NUMBER_OF_MODULES);
#endif

#if ENABLE_VOLTAGE == 1
    if (timeRunThread[threadIndex] > SYNC_VOLTAGE_TIME * 2)
      LOGError(I2C_TAG, "Twice time voltage sync.");
    if (timeRunThread[threadIndex] > SYNC_VOLTAGE_TIME) {
      syncVoltage(NULL);
      timeRunThread[threadIndex] = 0;
    }
    threadIndex++;
    usleep(SYNC_MIN_TIME / NUMBER_OF_MODULES);
#endif

    clock_gettime(CLOCK_MONOTONIC, &tend);

    //printf("%lu;%lu;%lu;%lu;%lu\n",tstart.tv_sec,tend.tv_sec,tstart.tv_nsec,tend.tv_nsec, timeRunThread[0]);

    unsigned long deltaTimeRun = (unsigned long)(((double)(tend.tv_sec - tstart.tv_sec) * 1000000) + ((double)(tend.tv_nsec - tstart.tv_nsec) / 1000));


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