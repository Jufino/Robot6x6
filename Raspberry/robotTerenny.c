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

#define numberOfPriorities 3
bool priorityVisible[numberOfPriorities];

XN_USB_DEV_HANDLE dev;
VideoCapture cameraKinect;
char imageChooseKinect = 0;
char depthChooseKinect = 0;
Mat depth1Kinect;
Mat depth2Kinect;
Mat img1Kinect;
Mat img2Kinect;
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
RobotAcculators lastRobotAcculators;
RobotSensors robotSensors;

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
  }
}

void LOGError(log_tag_t tag, const char text[]) {
  if (ENABLE_LOG_ERROR) {
    char buffer[255];
    char timeStr[20];

    time_t t = time(NULL);
    struct tm *tm = localtime(&t);
    strftime(timeStr, sizeof(timeStr), "%D %T", tm);

    charTag(tag, buffer);
    printf("%s - LOGError:%s/%s\n", timeStr, buffer, text);
  }
}

void LOGInfo(log_tag_t tag, unsigned char priority, const char text[]) {
  if (ENABLE_LOG_INFO && priorityVisible[priority]) {
    char buffer[255];
    char timeStr[20];

    time_t t = time(NULL);
    struct tm *tm = localtime(&t);
    strftime(timeStr, sizeof(timeStr), "%D %T", tm);

    charTag(tag, buffer);
    printf("%s - LOGInfo(%d):%s/%s\n", timeStr, priority, buffer, text);
  }
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

void initRobot(void) {
  priorityVisible[0] = true;
  priorityVisible[1] = false;
  priorityVisible[2] = false;

  sem_id = semCreate(getpid(), 16);
  semInit(sem_id, CAMERA_VARIABLE_L, 1);
  semInit(sem_id, CAMERA_IMAGE_L1, 1);
  semInit(sem_id, CAMERA_IMAGE_L2, 1);
  semInit(sem_id, CAMERA_VARIABLE_R, 1);
  semInit(sem_id, CAMERA_IMAGE_R1, 1);
  semInit(sem_id, CAMERA_IMAGE_R2, 1);
  semInit(sem_id, ROBOTSENSORS, 1);
  semInit(sem_id, ROBOTACCULATORS, 1);
  semInit(sem_id, CAMERA_DEPTH_KINECT1, 1);
  semInit(sem_id, CAMERA_DEPTH_KINECT2, 1);
  semInit(sem_id, CAMERA_IMAGE_KINECT1, 1);
  semInit(sem_id, CAMERA_IMAGE_KINECT2, 1);
  semInit(sem_id, CAMERA_VARIABLE_KINECTIMAGE, 1);
  semInit(sem_id, CAMERA_VARIABLE_KINECTDEPTH, 1);
  semInit(sem_id, I2C, 1);

  initMotorPowerSupply();

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

  if (SENSORS_WIFI) {
    pthread_t waitForSensorConnectionThread;
    pthread_create(&waitForSensorConnectionThread, NULL, &waitForSensorConnection, NULL);
  }

  if (CAMERA_WIFI) {
    pthread_t waitForCameraConnectionThread;
    pthread_create(&waitForCameraConnectionThread, NULL, &waitForCameraConnection, NULL);
  }
  if (SENSORS_WIFI == 1 || CAMERA_WIFI == 1)   signal(SIGPIPE, sigpipe);

  pthread_t threadModules;
  pthread_create(&threadModules, NULL, &syncModules, NULL);
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

Mat getImageKinect(void) {
  Mat imgMatKinect;
  double imageChooseMainKinect;
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

Mat getDepthKinect(void) {
  Mat depthMatKinect;
  double depthChooseMainKinect;
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

Mat getImageLeft(void) {
  Mat imgMatL;
  double imageChooseMainL;
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
  double imageChooseMainR;
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

void sendMatImage(Mat img, int quality) {
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

Axis_struct getPossitionAxis(void) {
  Axis_struct axis;
  semWait(sem_id, I2C);
  axis.x = readRegister32s(STM32_ADDRESS, 100);
  axis.y = readRegister32s(STM32_ADDRESS, 101);
  axis.z = readRegister32s(STM32_ADDRESS, 102);
  semPost(sem_id, I2C);
  return axis;
}

Angle3d_struct getPossitionAngle3d(void) {
  Angle3d_struct angle3d;
  semWait(sem_id, I2C);
  angle3d.roll = ((double)readRegister16(STM32_ADDRESS, 103)) / 10000;
  angle3d.pitch = ((double)readRegister16(STM32_ADDRESS, 104)) / 10000;
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
  int bytes = 10;
  char recvdata[10];
  LOGInfo(SENSOR_CONN_TAG, 1, "Start:Camera sync by network.");
  while (bytes != 0 && onAllThreads && onWifiCameraStill) {
    bytes = recv(cameraClientsock, recvdata, 10, 0);
    if (bytes == 0) {
      onWifiCameraStill = false;
      closeCameraConnection();
      onWifiCameraStill = true;
      sleep(2);
      pthread_t waitForCameraConnectionThread;
      pthread_create(&waitForCameraConnectionThread, NULL, &waitForCameraConnection, NULL);

      break;
    }
    char buffer [50];
    sprintf (buffer, "recv data : %s", recvdata);
    for (int i = 0; i < 50; i++) {
      if (buffer[i] == '\n') {
        buffer[i] = '\0';
        break;
      }
    }
    LOGInfo(CAMERA_CONN_TAG, 2, buffer);
    if (strcmp(recvdata, "imgL\n") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "RGB img left sync.");
      sendMatImage(getImageLeft(), 80);
    }
    else if (strcmp(recvdata, "imgR\n") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "RGB img right sync.");
      sendMatImage(getImageRight(), 80);
    }
    else if (strcmp(recvdata, "imgK\n") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "RGB img kinect sync.");
      sendMatImage(getImageKinect(), 80);
    }
    else if (strcmp(recvdata, "depK\n") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Depth img kinect sync.");
      sendMatImage(getDepthKinect(), 80);
    }
  }
  LOGInfo(SENSOR_CONN_TAG, 1, "End:Camera sync by network.");
  return NULL;
}

void *syncSensorNetworkConnection(void *arg) {
  LOGInfo(SENSOR_CONN_TAG, 1, "Start:Sensor sync by network.");
  char recvdata[10];
  int bytes = 10;
  while (bytes != 0 && onAllThreads && onWifiSensorStill) {
    bytes = recv(sensorsClientsock, recvdata, 10, 0);
    if (bytes == 0) {
      onWifiSensorStill = false;
      closeSensorConnection();
      onWifiSensorStill = true;
      sleep(2);
      pthread_t waitForSensorConnectionThread;
      pthread_create(&waitForSensorConnectionThread, NULL, &waitForSensorConnection, NULL);
      break;
    }

    char buffer [50];
    sprintf (buffer, "recv data : %s", recvdata);
    for (int i = 0; i < 50; i++) {
      if (buffer[i] == '\n') {
        buffer[i] = '\0';
        break;
      }
    }
    LOGInfo(SENSOR_CONN_TAG, 2, buffer);

    if (strcmp(recvdata, "sensor\n") == 0) {
      LOGInfo(SENSOR_CONN_TAG, 1, "Sensor wifi sync.");
    }
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
      if (cameraKinect.retrieve( depthMap, CAP_OPENNI_DEPTH_MAP ) ) {
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
    }

    cvWaitKey(10);
    if ( !cameraKinect.grab() )
    {
      LOGError(KINECT_TAG, "Problem grab frames.");
    }
    else
    {
      semWait(sem_id, CAMERA_DEPTH_KINECT2);
      if (cameraKinect.retrieve( depthMap, CAP_OPENNI_DEPTH_MAP ) ) {
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

void *syncLeds(void *arg) {
  LOGInfo(NUCLEO_TAG, 1, "Start:Leds sync.");
  semWait(sem_id, ROBOTACCULATORS);
  color_t LedUp = robotAcculators.leds.LedUp;
  color_t LedMiddle = robotAcculators.leds.LedMiddle;
  color_t LedDown = robotAcculators.leds.LedDown;
  semPost(sem_id, ROBOTACCULATORS);
  setLeds(LedUp, LedMiddle, LedDown);
  LOGInfo(NUCLEO_TAG, 1, "End:Leds sync.");
  return NULL;
}

void *syncMotors(void *arg) {
  LOGInfo(NUCLEO_TAG, 1, "Start:Motors sync.");
  direction_t  robotDirection = robotAcculators.robotDirection;
  unsigned int robotSpeed = robotAcculators.robotSpeed;
  semWait(sem_id, ROBOTACCULATORS);
  setMove(robotDirection, robotSpeed );
  semPost(sem_id, ROBOTACCULATORS);
  LOGInfo(NUCLEO_TAG, 1, "End:Motors sync.");
  return NULL;
}

void *syncPossition(void *arg) {
  LOGInfo(NUCLEO_TAG, 1, "Start:Possition sync.");
  Axis_struct axis = getPossitionAxis();
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

void *syncKinectMotor(void *arg) {
  //https://openkinect.org/wiki/Protocol_Documentation#Control_Packet_Structure
  unsigned char empty[1];
  LOGInfo(KINECT_TAG, 1, "Start:Motor sync.");
  semWait(sem_id, ROBOTACCULATORS);
  int angle = robotAcculators.kinect.roll * 2;
  semPost(sem_id, ROBOTACCULATORS);

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

  LOGInfo(KINECT_TAG, 1, "End:Motor sync.");
  return NULL;
}

void *syncKinectLed(void *arg) {
  //https://openkinect.org/wiki/Protocol_Documentation#Control_Packet_Structure
  unsigned char empty[1];
  LOGInfo(KINECT_TAG, 1, "Start:Led sync.");
  semWait(sem_id, ROBOTACCULATORS);
  ledKinect_t led = robotAcculators.ledKinect;
  semPost(sem_id, ROBOTACCULATORS);

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

#define numberOfModules 8
void *syncModules(void *arg) {
  unsigned long timeRunThread[numberOfModules];
  for (int i = 0; i < numberOfModules; i++) {
    timeRunThread[i] = 0;
  }
  int threadIndex = 0;
  struct timespec tstart = {0, 0}, tend = {0, 0};
  while (onAllThreads) {

    clock_gettime(CLOCK_MONOTONIC, &tstart);

    threadIndex = 0;
    if (ENABLE_MOTORS && (timeRunThread[threadIndex] > SYNC_MOTORS_TIME)) {
      syncMotors(NULL);
      timeRunThread[threadIndex] = 0;
    }
    threadIndex++;

    usleep(SYNC_MIN_TIME / numberOfModules);

    if (ENABLE_ULTRASONIC && (timeRunThread[threadIndex] > SYNC_ULTRASONIC_TIME)) {
      syncUltrasonic(NULL);
      timeRunThread[threadIndex] = 0;
    }
    threadIndex++;

    usleep(SYNC_MIN_TIME / numberOfModules);

    if (ENABLE_LEDS && (timeRunThread[threadIndex] > SYNC_LEDS_TIME)) {
      syncLeds(NULL);
      timeRunThread[threadIndex] = 0;
    }
    threadIndex++;

    usleep(SYNC_MIN_TIME / numberOfModules);

    if (ENABLE_BUTTONS && (timeRunThread[threadIndex] > SYNC_BUTTONS_TIME)) {
      syncButtons(NULL);
      timeRunThread[threadIndex] = 0;
    }
    threadIndex++;

    usleep(SYNC_MIN_TIME / numberOfModules);

    if (ENABLE_POSSITION && (timeRunThread[threadIndex] > SYNC_POSSITION_TIME)) {
      syncPossition(NULL);
      timeRunThread[threadIndex] = 0;
    }
    threadIndex++;

    usleep(SYNC_MIN_TIME / numberOfModules);

    if (ENABLE_KINECTMOTOR && (timeRunThread[threadIndex] > SYNC_KINECTMOTOR_TIME)) {
      syncKinectMotor(NULL);
      timeRunThread[threadIndex] = 0;
    }
    threadIndex++;

    if (ENABLE_KINECTLED && (timeRunThread[threadIndex] > SYNC_KINECTLED_TIME)) {
      syncKinectLed(NULL);
      timeRunThread[threadIndex] = 0;
    }
    threadIndex++;

    usleep(SYNC_MIN_TIME / numberOfModules);

    if (ENABLE_KINECTSENSORS && (timeRunThread[threadIndex] > SYNC_KINECTSENSORS_TIME)) {
      syncKinectSensors(NULL);
      timeRunThread[threadIndex] = 0;
    }

    usleep(SYNC_MIN_TIME / numberOfModules);

    clock_gettime(CLOCK_MONOTONIC, &tend);

    unsigned long deltaTimeRun = ((tend.tv_nsec - tstart.tv_nsec) / 1000);

    for (int i = 0; i < numberOfModules; i++) {
      if (timeRunThread[i] <= SYNC_MIN_TIME * 1000)
        timeRunThread[i] += deltaTimeRun;
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

static void colorizeDisparity( const Mat & gray, Mat & rgb, double maxDisp = -1.f, float S = 1.f, float V = 1.f ) {
  CV_Assert( !gray.empty() );
  CV_Assert( gray.type() == CV_8UC1 );

  if ( maxDisp <= 0 )
  {
    maxDisp = 0;
    minMaxLoc( gray, 0, &maxDisp );
  }

  rgb.create( gray.size(), CV_8UC3 );
  rgb = Scalar::all(0);
  if ( maxDisp < 1 )
    return;

  for ( int y = 0; y < gray.rows; y++ )
  {
    for ( int x = 0; x < gray.cols; x++ )
    {
      uchar d = gray.at<uchar>(y, x);
      unsigned int H = ((uchar)maxDisp - d) * 240 / (uchar)maxDisp;

      unsigned int hi = (H / 60) % 6;
      float f = H / 60.f - H / 60;
      float p = V * (1 - S);
      float q = V * (1 - f * S);
      float t = V * (1 - (1 - f) * S);

      Point3f res;

      if ( hi == 0 ) //R = V,  G = t,  B = p
        res = Point3f( p, t, V );
      if ( hi == 1 ) // R = q, G = V,  B = p
        res = Point3f( p, V, q );
      if ( hi == 2 ) // R = p, G = V,  B = t
        res = Point3f( t, V, p );
      if ( hi == 3 ) // R = p, G = q,  B = V
        res = Point3f( V, q, p );
      if ( hi == 4 ) // R = t, G = p,  B = V
        res = Point3f( V, p, t );
      if ( hi == 5 ) // R = V, G = p,  B = q
        res = Point3f( q, p, V );

      uchar b = (uchar)(std::max(0.f, std::min (res.x, 1.f)) * 255.f);
      uchar g = (uchar)(std::max(0.f, std::min (res.y, 1.f)) * 255.f);
      uchar r = (uchar)(std::max(0.f, std::min (res.z, 1.f)) * 255.f);

      rgb.at<Point3_<uchar> >(y, x) = Point3_<uchar>(b, g, r);
    }
  }
}

static float getMaxDisparity( VideoCapture & capture ) {
  const int minDistance = 400; // mm
  float b = (float)capture.get( CAP_OPENNI_DEPTH_GENERATOR_BASELINE ); // mm
  float F = (float)capture.get( CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH ); // pixels
  return b * F / minDistance;
}