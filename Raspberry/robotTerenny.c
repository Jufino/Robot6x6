#include "robotTerenny.h"

freenect_context *ctx;  // pointer to the freenect context
freenect_device *dev;  // pointer to the device

int i2cHandle;
unsigned char lastAddr = 0x00;
int sensorsServersock, cameraServersock;
int sensorsClientsock, cameraClientsock;
int portHandle;
int sem_id;
bool onAllThreads = true;
bool onWifiCameraStill = true;
bool onWifiSensorStill = true;

VideoCapture cameraKinect( CV_CAP_OPENNI );
char imageChooseKinect = 0;
char depthChooseKinect = 0;
Mat depth1Kinect;
Mat depth2Kinect;
Mat img1Kinect;
Mat img2Kinect;

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

void charTag(log_tag_t tag, char *buffer)
{
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
  }

}

void LOGError(log_tag_t tag, char text[]) {
  if (ENABLE_LOG_ERROR) {
    char buffer[50];
    charTag(tag, buffer);
    printf("LOGError:%s/%s\n", buffer, text);
  }
}

void LOGInfo(log_tag_t tag, char text[]) {
  if (ENABLE_LOG_INFO) {
    char buffer[50];
    charTag(tag, buffer);
    printf("LOGInfo:%s/%s\n", buffer, text);
  }
}
void LOGInfoDetail(log_tag_t tag, char text[]) {
  if (ENABLE_LOG_INFO_DETAIL) {
    char buffer[50];
    charTag(tag, buffer);
    printf("LOGInfoDetail:%s/%s\n", buffer, text);
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
  sem_id = semCreate(getpid(), 9);
  semInit(sem_id, CAMERA_VARIABLE_L, 1);
  semInit(sem_id, CAMERA_IMAGE_L1, 1);
  semInit(sem_id, CAMERA_IMAGE_L2, 1);
  semInit(sem_id, CAMERA_VARIABLE_R, 1);
  semInit(sem_id, CAMERA_IMAGE_R1, 1);
  semInit(sem_id, CAMERA_IMAGE_R2, 1);
  semInit(sem_id, ROBOTSENSORS, 1);
  semInit(sem_id, ROBOTACCULATORS, 1);
  semInit(sem_id, I2C, 1);

  initMotorPowerSupply();

  if (ENABLE_I2C)
    initI2C();

  if (!nucleoTestConnection()) {
    LOGError(NUCLEO_TAG, "Connection failed.");
    exit(0);
  }
  else {
    LOGInfo(NUCLEO_TAG, "Connection ok.");
  }

  if (ENABLE_LEDS)
    setLeds(COLOR_OFF, COLOR_OFF, COLOR_OFF);

  if (ENABLE_MOTORS) {
    setMove(STOP, 0);
    setMotorPowerSupply(true);
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
    pthread_create(&threadImgL, NULL, &getImgL, NULL);
  }

  if (NUMBER_OF_CAMERA == 2) {
    pthread_t threadImgR;
    pthread_create(&threadImgR, NULL, &getImgR, NULL);
  }

  if (ENABLE_KINECTCAMERA == 1) {
    cameraKinect.open( CAP_OPENNI );
    cameraKinect.set( CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CAP_OPENNI_VGA_30HZ );
    pthread_t threadImgAndDepthKinect;
    pthread_create(&threadImgAndDepthKinect, NULL, &getImgAndDephKinect, NULL);
  }

  if (SENSORS_WIFI) {
    struct sockaddr_in server;
    if ((sensorsServersock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
      LOGError(SENSOR_CONN_TAG, "socket() failed.");
      exit(1);
    }
    memset(&server, 0, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_port = htons(SENSORS_PORT);
    server.sin_addr.s_addr = INADDR_ANY;
    if (bind(sensorsServersock, (struct sockaddr *)&server, sizeof(server)) == -1) {
      LOGError(SENSOR_CONN_TAG, "bind() failed.");
      exit(1);
    }
    if (listen(sensorsServersock, 10) == -1) {
      LOGError(SENSOR_CONN_TAG, "listen() failed.");
      exit(1);
    }
    char buffer [50];
    sprintf (buffer, "Cakanie spojenia pre snimace na porte: %d\n", SENSORS_PORT);
    LOGInfo(SENSOR_CONN_TAG, buffer);
    if ((sensorsClientsock = accept(sensorsServersock, NULL, NULL)) == -1) {
      LOGError(SENSOR_CONN_TAG, "accept() failed.");
      exit(1);
    }
    sprintf (buffer, "Spojenie na porte %d ok.\n", SENSORS_PORT);
    LOGInfo(SENSOR_CONN_TAG, buffer);
    pthread_t vlaknoSensors;
    pthread_create(&vlaknoSensors, NULL, &sensorsNetworkConnection, NULL);
  }

  if (CAMERA_WIFI) {
    struct sockaddr_in server1;
    if ((cameraServersock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
      LOGError(CAMERA_CONN_TAG, "socket() failed.");
      exit(1);
    }
    memset(&server1, 0, sizeof(server1));
    server1.sin_family = AF_INET;
    server1.sin_port = htons(CAMERA_PORT);
    server1.sin_addr.s_addr = INADDR_ANY;
    if (bind(cameraServersock, (struct sockaddr *)&server1, sizeof(server1)) == -1) {
      LOGError(CAMERA_CONN_TAG, "bind() failed.");
      exit(1);
    }
    if (listen(cameraServersock, 10) == -1) {
      LOGError(CAMERA_CONN_TAG, "listen() failed.");
      exit(1);
    }
    char buffer [50];
    sprintf (buffer, "Cakanie spojenia pre kameru na porte: %d\n", CAMERA_PORT);
    LOGInfo(SENSOR_CONN_TAG, buffer);
    if ((cameraClientsock = accept(cameraServersock, NULL, NULL)) == -1) {
      LOGError(CAMERA_CONN_TAG, "accept() failed.\n");
      exit(1);
    }
    sprintf (buffer, "Spojenie na porte %d ok.", CAMERA_PORT);
    LOGInfo(CAMERA_CONN_TAG, buffer);

    pthread_t vlaknoCamera;
    pthread_create(&vlaknoCamera, NULL, &cameraNetworkConnection, NULL);
  }
  if (SENSORS_WIFI == 1 || CAMERA_WIFI == 1)   signal(SIGPIPE, sigpipe);

  if (ENABLE_KINECTACCULATORS || ENABLE_KINECTSENSORS) {
    if (!initKinect()) {
      LOGError(KINECT_TAG, "Init failed.");
      exit(0);
    }
    else {
      LOGInfo(KINECT_TAG, "Init ok.");
    }
  }

  pthread_t threadModules;
  pthread_create(&threadModules, NULL, &syncModules, NULL);
  signal(SIGINT, sigctrl);

}

void closeRobot(void) {
  onAllThreads = false;
  if (ENABLE_KINECTACCULATORS || ENABLE_KINECTSENSORS) {
    freenect_shutdown(ctx);
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

void initI2C(void) {
  if ((i2cHandle = open(PORT_I2C, O_RDWR)) < 0) {
    LOGError(I2C_TAG, "Init failed.");
    exit(1);
  }
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
      if (errorTimeout++ >= I2C_WRITE_TIMEOUT) break;
    }
    if (errorTimeout < I2C_WRITE_TIMEOUT)
      return 0;
    else {
      char buffer [50];
      sprintf (buffer, "addr:%i, write register %i,val %i, errorTimeout:%i\n", (int)addr, (int)reg, (int)value, (int)errorTimeout);
      LOGError(I2C_TAG, buffer);
      return -1;
    }
  }
  else {
    return -1;
  }
}

char writeRegisterAndValueU16(unsigned char addr, unsigned char reg, unsigned int value) {
  char errorTimeout = 0;
  unsigned char data[3];
  data[0] = reg;
  data[1] = (value >> 8) & 0xFF;
  data[2] = value & 0xFF;
  if (setDevice(addr) == 0) {
    while (write(i2cHandle, data, 2) != 2) {
      if (errorTimeout++ >= I2C_WRITE_TIMEOUT) break;
    }
    if (errorTimeout < I2C_WRITE_TIMEOUT)
      return 0;
    else {
      char buffer [50];
      sprintf (buffer, "addr:%i, write register %i,val %i, errorTimeout:%i", (int)addr, (int)reg, (int)value, (int)errorTimeout);
      LOGError(I2C_TAG, buffer);
      return -1;
    }
  }
  else {
    return -1;
  }
}

char writeRegisterAndValueS16(unsigned char addr, unsigned char reg, int value) {
  char errorTimeout = 0;
  unsigned char data[3];
  data[0] = reg;
  data[1] = (value >> 8) & 0xFF;
  data[2] = value & 0xFF;
  if (setDevice(addr) == 0) {
    while (write(i2cHandle, data, 2) != 2) {
      if (errorTimeout++ >= I2C_WRITE_TIMEOUT) break;
    }
    if (errorTimeout < I2C_WRITE_TIMEOUT)
      return 0;
    else {
      char buffer [50];
      sprintf (buffer, "addr:%i, write register %i,val %i, errorTimeout:%i", (int)addr, (int)reg, (int)value, (int)errorTimeout);
      LOGError(I2C_TAG, buffer);
      return -1;
    }
  }
  else {
    return -1;
  }
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
      return -1;
    }
  }
  else {
    return -1;
  }
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

bool initKinect() {
  if (freenect_init(&ctx, NULL) < 0)
  {
    LOGError(KINECT_TAG, "Init failed.");
    exit(EXIT_FAILURE);
  }
// set the highest log level so we can see what is going on
  freenect_set_log_level(ctx, FREENECT_LOG_SPEW);

  int nr_devices = freenect_num_devices (ctx);
  char buffer [50];
  sprintf (buffer, "Number of kinects detected is %d.", nr_devices);
  LOGInfo(KINECT_TAG, buffer);

  if (freenect_init(&ctx, NULL) < 0) {
    return false;
  }

  //freenect_set_log_level(ctx, FREENECT_LOG_SPEW);

  if (freenect_open_device(ctx, &dev, 0) < 0)
  {
    LOGError(KINECT_TAG, "Open failed.");
    freenect_shutdown(ctx);
    return false;
  }
  return true;
}

Mat getImageKinect(void) {
  Mat imgMatKinect;
  double imageChooseMainKinect;
  semWait(sem_id, CAMERA_VARIABLE_KINECTIMAGE);
  imageChooseMainKinect = imageChooseKinect;
  semPost(sem_id, CAMERA_VARIABLE_KINECTIMAGE);
  if (imageChooseMainKinect != 0) {
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
  }
  return imgMatKinect;
}

Mat getDepthKinect(void) {
  Mat depthMatKinect;
  double depthChooseMainKinect;
  semWait(sem_id, CAMERA_VARIABLE_KINECTDEPTH);
  depthChooseMainKinect = depthChooseKinect;
  semPost(sem_id, CAMERA_VARIABLE_KINECTDEPTH);
  if (depthChooseMainKinect != 0) {
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
  }
  return depthMatKinect;
}

Mat getImageLeft(void) {
  Mat imgMatL;
  double imageChooseMainL;
  semWait(sem_id, CAMERA_VARIABLE_L);
  imageChooseMainL = imageChooseL;
  semPost(sem_id, CAMERA_VARIABLE_L);
  if (imageChooseMainL != 0) {
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
  }
  return imgMatL;
}

Mat getImageRight(void) {
  Mat imgMatR;
  double imageChooseMainR;
  semWait(sem_id, CAMERA_VARIABLE_R);
  imageChooseMainR = imageChooseR;
  semPost(sem_id, CAMERA_VARIABLE_R);
  if (imageChooseMainR != 0) {
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
  }
  return imgMatR;
}

Mat getImage(void) {
  return getImageLeft();
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
  param[0] = CV_IMWRITE_JPEG_QUALITY;
  param[1] = quality;
  imencode(".jpg", img, buff, param);
  char len[10];
  sprintf(len, "%.8d", buff.size());
  send(cameraClientsock, len, strlen(len), 0);
  send(cameraClientsock, &buff[0], buff.size(), 0);
  buff.clear();
}

int getCameraClientsock(void) {
  return cameraClientsock;
}

void closeCameraConnection(void) {
  close(cameraServersock);
  close(cameraClientsock);
}

int getSensorsClientsock(void) {
  return sensorsClientsock;
}

void closeSensorConnection(void) {
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

void setServo(int angle) {
  semWait(sem_id, I2C);
  writeRegisterAndValueU8(STM32_ADDRESS, 13, angle);
  semPost(sem_id, I2C);
}

void setMove(direction_t direction, unsigned char speed) {
  semWait(sem_id, I2C);
  switch (direction) {
  case FORWARD:
    writeRegisterAndValueU8(STM32_ADDRESS, 7, speed);
    break;
  case BACKWARD:
    writeRegisterAndValueU8(STM32_ADDRESS, 8, speed);
    break;
  case CLOCKWISE:
    writeRegisterAndValueU8(STM32_ADDRESS, 9, speed);
    break;
  case ANTICLOCKWISE:
    writeRegisterAndValueU8(STM32_ADDRESS, 10, speed);
    break;
  default:
    writeRegisterAndValueU8(STM32_ADDRESS, 11, speed);
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

double dist(double a, double b) {
  return sqrt(a * a + b * b);
}

double rad2Deg(double angle) {
  return angle * (180 / M_PI);
}

double deg2Rad(double angle) {
  return angle * (M_PI / 180);
}

void *getImgAndDephKinect(void *arg) {
  while (onAllThreads) {
    bool imageReady = true;//cameraKinect.get( CV_CAP_PROP_OPENNI_IMAGE_GENERATOR_PRESENT ) != 0;
    bool depthReady = cameraKinect.get( CV_CAP_OPENNI_DEPTH_GENERATOR  ) != 0;
    if (imageReady || depthReady) {
      cameraKinect.grab();
      if (depthReady) {
        semWait(sem_id, CAMERA_DEPTH_KINECT1);
        cameraKinect.retrieve( depth1Kinect, CV_CAP_OPENNI_DEPTH_MAP );
        semWait(sem_id, CAMERA_VARIABLE_KINECTDEPTH);
        depthChooseKinect = 1;
        semPost(sem_id, CAMERA_VARIABLE_KINECTDEPTH);
        semPost(sem_id, CAMERA_DEPTH_KINECT1);
      }
      if (imageReady) {
        semWait(sem_id, CAMERA_IMAGE_KINECT1);
        cameraKinect.retrieve( img1Kinect, CV_CAP_OPENNI_BGR_IMAGE );
        semWait(sem_id, CAMERA_VARIABLE_KINECTIMAGE);
        imageChooseKinect = 1;
        semPost(sem_id, CAMERA_VARIABLE_KINECTIMAGE);
        semPost(sem_id, CAMERA_IMAGE_KINECT1);
      }

      cameraKinect.grab();
      if (depthReady) {
        semWait(sem_id, CAMERA_DEPTH_KINECT2);
        cameraKinect.retrieve( depth2Kinect, CV_CAP_OPENNI_DEPTH_MAP );
        semWait(sem_id, CAMERA_VARIABLE_KINECTDEPTH);
        depthChooseKinect = 2;
        semPost(sem_id, CAMERA_VARIABLE_KINECTDEPTH);
        semPost(sem_id, CAMERA_DEPTH_KINECT2);
      }
      if (imageReady) {
        semWait(sem_id, CAMERA_IMAGE_KINECT2);
        cameraKinect.retrieve( img2Kinect, CV_CAP_OPENNI_BGR_IMAGE );
        semWait(sem_id, CAMERA_VARIABLE_KINECTIMAGE);
        imageChooseKinect = 2;
        semPost(sem_id, CAMERA_VARIABLE_KINECTIMAGE);
        semPost(sem_id, CAMERA_IMAGE_KINECT2);
      }
    }
  }
  return 0;
}

void *getImgL(void *arg) {
  while (onAllThreads) {
    //nacitanie obrazka z lavej kamery
    semWait(sem_id, CAMERA_IMAGE_L1);
    img1L = cvQueryFrame(cameraL);
    semWait(sem_id, CAMERA_VARIABLE_L);
    imageChooseL = 1;
    semPost(sem_id, CAMERA_VARIABLE_L);
    semPost(sem_id, CAMERA_IMAGE_L1);

    semWait(sem_id, CAMERA_IMAGE_L2);
    img2L = cvQueryFrame(cameraL);
    semWait(sem_id, CAMERA_VARIABLE_L);
    imageChooseL = 2;
    semPost(sem_id, CAMERA_VARIABLE_L);
    semPost(sem_id, CAMERA_IMAGE_L2);
  }
  return 0;
}

void *getImgR(void *arg) {
  while (onAllThreads) {
    semWait(sem_id, CAMERA_IMAGE_R1);
    img1R = cvQueryFrame(cameraR);
    semWait(sem_id, CAMERA_VARIABLE_R);
    imageChooseR = 1;
    semPost(sem_id, CAMERA_VARIABLE_R);
    semPost(sem_id, CAMERA_IMAGE_R1);

    semWait(sem_id, CAMERA_IMAGE_R2);
    img2R = cvQueryFrame(cameraR);
    semWait(sem_id, CAMERA_VARIABLE_R);
    imageChooseR = 2;
    semPost(sem_id, CAMERA_VARIABLE_R);
    semPost(sem_id, CAMERA_IMAGE_R2);
  }
  return 0;
}

void *cameraNetworkConnection(void *arg) {
  while (onWifiCameraStill) {
    char recvdata[30];
    int bytes = recv(getCameraClientsock(), recvdata, 10, 0);
    if (bytes == 0) {
      onWifiCameraStill = false;
      closeCameraConnection();
      break;
    }
    char buffer [50];
    sprintf (buffer, "recv data : %s\n", recvdata);
    LOGInfo(CAMERA_CONN_TAG, buffer);
    if (strcmp(recvdata, "imgL\n") == 0)     sendMatImage(getImageLeft(), 80);
    else if (strcmp(recvdata, "imgR\n") == 0) sendMatImage(getImageRight(), 80);
    else if (strcmp(recvdata, "imgK\n") == 0) sendMatImage(getImageKinect(), 80);
    else if (strcmp(recvdata, "depK\n") == 0) sendMatImage(getDepthKinect(), 80);
  }
  return 0;
}

void *sensorsNetworkConnection(void *arg) {
  while (onWifiSensorStill) {
    char recvdata[30];
    int bytes = recv(getCameraClientsock(), recvdata, 4, 0);
    if (bytes == 0) {
      onWifiSensorStill = false;
      closeSensorConnection();
      break;
    }
    if (strcmp(recvdata, "sensor\n") == 0) {
      //semWait(sem_id, 1);
      //sendMatImage(robotSensors.camera.imgLeft,80);
      //semPost(sem_id, 1);
    }
  }
  return 0;
}

void *syncUltrasonic(void *arg) {
  double value = getUltrasonic();
  semWait(sem_id, ROBOTSENSORS);
  robotSensors.ultrasonic = value;
  semPost(sem_id, ROBOTSENSORS);
  return 0;
}

void *syncLeds(void *arg) {
  semWait(sem_id, ROBOTACCULATORS);
  color_t LedUp = robotAcculators.leds.LedUp;
  color_t LedMiddle = robotAcculators.leds.LedMiddle;
  color_t LedDown = robotAcculators.leds.LedDown;
  semPost(sem_id, ROBOTACCULATORS);
  setLeds(LedUp, LedMiddle, LedDown);
  return 0;
}

void *syncMotors(void *arg) {
  direction_t  robotDirection = robotAcculators.robotDirection;
  unsigned int robotSpeed = robotAcculators.robotSpeed;
  semWait(sem_id, ROBOTACCULATORS);
  setMove(robotDirection, robotSpeed );
  semPost(sem_id, ROBOTACCULATORS);
  return 0;
}

void *syncPossition(void *arg) {
  semWait(sem_id, ROBOTSENSORS);

//get possition

  semPost(sem_id, ROBOTSENSORS);
  return 0;
}

void *syncButtons(void *arg) {
  unsigned char buttons = getButtons();
  semWait(sem_id, ROBOTSENSORS);
  robotSensors.buttons.buttonDown = (buttons & 0x01) && 0x01;
  robotSensors.buttons.buttonMiddle = (buttons & 0x02) && 0x02;
  robotSensors.buttons.buttonUp = (buttons & 0x04) && 0x04;
  semPost(sem_id, ROBOTSENSORS);
  return 0;
}

void *syncKinectAcculators(void *arg) {
  semWait(sem_id, ROBOTACCULATORS);
  //freenect_set_tilt_degs(dev, robotAcculators.kinect.roll);
  freenect_set_led(dev, robotAcculators.leds.LedKinect);
  semPost(sem_id, ROBOTACCULATORS);
  return 0;
}

void *syncKinectSensors(void *arg) {
  freenect_raw_tilt_state *state = 0;
  // Get the raw accelerometer values and tilt data
  state = freenect_get_tilt_state(dev);
  semWait(sem_id, ROBOTSENSORS);
  // Get the processed accelerometer values (calibrated to gravity)
  freenect_get_mks_accel(state, &robotSensors.kinect.accAxis.x, &robotSensors.kinect.accAxis.y, &robotSensors.kinect.accAxis.z);
  robotSensors.kinect.accAngle.roll = freenect_get_tilt_degs(state);
  semPost(sem_id, ROBOTSENSORS);
  return 0;
}

#define numberOfModules 7
void *syncModules(void *arg) {
  pthread_t threads[numberOfModules];
  unsigned long timeRunThread[numberOfModules];
  for (int i = 0; i < numberOfModules; i++) {
    timeRunThread[i] = 0;
  }
  int threadIndex = 0;
  struct timespec tstart = {0, 0}, tend = {0, 0};
  while (onAllThreads) {
    threadIndex = 0;
    clock_gettime(CLOCK_MONOTONIC, &tstart);

    if (ENABLE_MOTORS && timeRunThread[threadIndex] > SYNC_MOTORS_TIME) {
      LOGInfoDetail(NUCLEO_TAG, "Motors sync.");
      pthread_create(&threads[threadIndex], NULL, &syncMotors, NULL);
      timeRunThread[threadIndex++] = 0;
    }

    if (ENABLE_ULTRASONIC && timeRunThread[threadIndex] > SYNC_ULTRASONIC_TIME) {
      LOGInfoDetail(NUCLEO_TAG, "Ultrasonic sync.");
      pthread_create(&threads[threadIndex], NULL, &syncUltrasonic, NULL);
      timeRunThread[threadIndex++] = 0;
    }

    if (ENABLE_LEDS && timeRunThread[threadIndex] > SYNC_LEDS_TIME) {
      LOGInfoDetail(NUCLEO_TAG, "Leds sync.");
      pthread_create(&threads[threadIndex], NULL, &syncLeds, NULL);
      timeRunThread[threadIndex++] = 0;
    }

    if (ENABLE_BUTTONS && timeRunThread[threadIndex] > SYNC_BUTTONS_TIME) {
      LOGInfoDetail(NUCLEO_TAG, "Buttons sync.");
      pthread_create(&threads[threadIndex], NULL, &syncButtons, NULL);
      timeRunThread[threadIndex++] = 0;
    }

    if (ENABLE_POSSITION && timeRunThread[threadIndex] > SYNC_POSSITION_TIME) {
      LOGInfoDetail(NUCLEO_TAG, "Possition sync.");
      pthread_create(&threads[threadIndex], NULL, &syncPossition, NULL);
      timeRunThread[threadIndex++] = 0;
    }

    if (ENABLE_KINECTACCULATORS && timeRunThread[threadIndex] > SYNC_KINECTACCULATORS_TIME) {
      LOGInfoDetail(KINECT_TAG, "Motor sync.");
      pthread_create(&threads[threadIndex], NULL, &syncKinectAcculators, NULL);
      timeRunThread[threadIndex++] = 0;
    }

    if (ENABLE_KINECTSENSORS && timeRunThread[threadIndex] > SYNC_KINECTSENSORS_TIME) {
      LOGInfoDetail(KINECT_TAG, "Sensor sync.");
      pthread_create(&threads[threadIndex], NULL, &syncKinectSensors, NULL);
      timeRunThread[threadIndex++] = 0;
    }
    usleep(SYNC_MIN_TIME);
    clock_gettime(CLOCK_MONOTONIC, &tend);

    unsigned long deltaTimeRun = (tend.tv_nsec - tstart.tv_nsec) / 1000000;
    for (int i = 0; i < numberOfModules; i++) {
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