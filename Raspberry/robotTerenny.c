#include "robotTerenny.h"

int i2cHandle;
unsigned char lastAddr = 0x00;
int serversock_snimace, serversock_camera;
int clientsock_snimace, clientsock_camera;
int portHandle;  
int sem_id;
bool onAllThreads = true;

CvCapture* cameraL;
char imageChooseL=0;
IplImage *img1L;
IplImage *img2L;

CvCapture* cameraR;
char imageChooseR=0;
IplImage *img1R;
IplImage *img2R;

RobotAcculators robotAcculators;
RobotAcculators lastRobotAcculators;
RobotSensors robotSensors;
timer_t casovac;
float mgPerDigit = 0.92f;

float AccX_offset = 0;
float AccY_offset = 0;
float AccZ_offset = 0;
float GyX_offset = 0;
float GyY_offset = 0;
float GyZ_offset = 0;
float AccScale = 16384;
float GyScale = 131;
float Pitch = 0;
float Roll = 0;
float Yaw = 0;

int pocetPosition = 100;
int pocetMotors = 100;
int pocetBattery = 100;
int pocetMPU6050 = 100;
int pocetBMP180 = 100;
int pocetHMC5883L = 100;
int pocetLeds = 100;
int pocetAmp = 100;
int pocetUltrasonic = 100;
int pocetCamera = 100;

float minX = HMC5883L_MIN_X;
float minY = HMC5883L_MIN_Y;
float maxX = HMC5883L_MAX_X;
float maxY = HMC5883L_MAX_Y;
float offX = (HMC5883L_MIN_X+HMC5883L_MAX_X)/2;
float offY = (HMC5883L_MIN_Y+HMC5883L_MAX_Y)/2;

void initRobot() {
  if ((i2cHandle = open(PORT_I2C, O_RDWR)) < 0) {
    perror("Problem s otvorenim portu.\n");
    exit(1);
  }
  if (test() == 1) {
    portHandle = SerialOpen(PORT_GPS, B9600);

    initButton(POSITION_DOWN);
    initButton(POSITION_MIDDLE);
    initButton(POSITION_UP);
    initMotorPowerSupply();

    stopAllMotors();

    sem_id = semCreate(getpid(), 8); //vytvor semafor
    semInit(sem_id, 0, 1);
    semInit(sem_id, 1, 1);
    semInit(sem_id, 2, 1);
    semInit(sem_id, 3, 1);
	  semInit(sem_id, 4, 1);
    semInit(sem_id, 5, 1);
	  semInit(sem_id, 6, 1);
    semInit(sem_id, 7, 1);
    
    if(NUMBER_OF_CAMERA == 1 || NUMBER_OF_CAMERA == 2){
      cameraL = cvCaptureFromCAM(INDEX_CAMERA_LEFT);
      cvSetCaptureProperty( cameraL, CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
      cvSetCaptureProperty( cameraL, CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
    }
    
    if(NUMBER_OF_CAMERA == 2){
      cameraR = cvCaptureFromCAM(INDEX_CAMERA_RIGHT);
      cvSetCaptureProperty( cameraR, CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
      cvSetCaptureProperty( cameraR, CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
    }

    setLed(POSITION_DOWN, COLOR_RED);
    setLed(POSITION_MIDDLE, COLOR_RED);
    setLed(POSITION_UP, COLOR_RED);
    usleep(500000);
    setLed(POSITION_DOWN, COLOR_GREEN);
    setLed(POSITION_MIDDLE, COLOR_GREEN);
    setLed(POSITION_UP, COLOR_GREEN);
    usleep(500000);
    setLed(POSITION_DOWN, COLOR_ORANGE);
    setLed(POSITION_MIDDLE, COLOR_ORANGE);
    setLed(POSITION_UP, COLOR_ORANGE);
    usleep(500000);
    setLed(POSITION_DOWN, COLOR_OFF);
    setLed(POSITION_MIDDLE, COLOR_OFF);
    setLed(POSITION_UP, COLOR_OFF);
    
    if (SENSORS_WIFI == 1) {
      struct sockaddr_in server;
      if ((serversock_snimace = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket() failed");
        exit(1);
      }
      memset(&server, 0, sizeof(server));
      server.sin_family = AF_INET;
      server.sin_port = htons(SENSORS_PORT);
      server.sin_addr.s_addr = INADDR_ANY;
      if (bind(serversock_snimace, (struct sockaddr *)&server, sizeof(server)) == -1) {
        perror("bind() failed");
        exit(1);
      }
      if (listen(serversock_snimace, 10) == -1) {
        perror("listen() failed.");
        exit(1);
      }
      printf("Cakanie spojenia pre snimace na porte: %d\n", SENSORS_PORT);
      if ((clientsock_snimace = accept(serversock_snimace, NULL, NULL)) == -1) {
        perror("accept() failed");
        exit(1);
      }
      printf("Spojenie na porte %d ok.\n", SENSORS_PORT);
    }
    if (CAMERA_WIFI == 1) {
      struct sockaddr_in server1;
      if ((serversock_camera = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket() failed");
        exit(1);
      }
      memset(&server1, 0, sizeof(server1));
      server1.sin_family = AF_INET;
      server1.sin_port = htons(CAMERA_PORT);
      server1.sin_addr.s_addr = INADDR_ANY;
      if (bind(serversock_camera, (struct sockaddr *)&server1, sizeof(server1)) == -1) {
        perror("bind() failed");
        exit(1);
      }
      if (listen(serversock_camera, 10) == -1) {
        perror("listen() failed.");
        exit(1);
      }
      printf("Cakanie spojenia pre kameru na porte: %d\n", CAMERA_PORT);
      if ((clientsock_camera = accept(serversock_camera, NULL, NULL)) == -1) {
        perror("accept() failed");
        exit(1);
      }
      printf("Spojenie na porte %d ok.\n", CAMERA_PORT);
    }
    
    resetDistanceAll();
    MPU6050WakeUp();
    setMPU6050Sensitivity(1,1);
    setMPU6050DLPF(6,6);
    MPU6050CalibrateOffset(20);
    MPU6050DisableAsMaster();
    MPU6050WakeUp();
    HMC5883LSampleRateAndModeSetting(4,5,3);
    HMC5883LGainSetting(1);
    HMC5883LReadModeSetting(0,0);
    setMotorPowerSupply(true);//potom zmenit na true
    
    if(REFRESH_STATUS == 1){
      initRefresh();
    }
    if(NUMBER_OF_CAMERA == 1 || NUMBER_OF_CAMERA == 2){
        pthread_t vlaknoImgL;
        pthread_create(&vlaknoImgL,NULL,&getImgL,NULL);
    }
    if(NUMBER_OF_CAMERA == 2){
        pthread_t vlaknoImgR;
        pthread_create(&vlaknoImgR,NULL,&getImgR,NULL);
    }
  }
  else {
    for(int i=0;i<10;i++){
      setLed(POSITION_DOWN, COLOR_RED);
      setLed(POSITION_MIDDLE, COLOR_RED);
      setLed(POSITION_UP, COLOR_RED);
      usleep(500000);
      setLed(POSITION_DOWN, COLOR_OFF);
      setLed(POSITION_MIDDLE, COLOR_OFF);
      setLed(POSITION_UP, COLOR_OFF);
      usleep(500000);
    }
    exit(0);
  }
}

void closeRobot() {
  onAllThreads = false;

  stopRefresh();
  
  SerialClose(portHandle);
  semRem(sem_id);
  
  closeButton(POSITION_DOWN);
  closeButton(POSITION_MIDDLE);
  closeButton(POSITION_UP);
  closeMotorPowerSupply();
  
  if (CAMERA_WIFI == 1) {
    close(serversock_camera);
    close(clientsock_camera);
  }
  if (SENSORS_WIFI == 1) {
    close(serversock_snimace);
    close(clientsock_snimace);
  }
  
  stopAllMotors();
}

void setDevice(unsigned char addr) {
  if (addr != lastAddr) {
    if (ioctl(i2cHandle, I2C_SLAVE, addr) < 0) {
      printf("Problem s vytvorenim spojenia so zariadenim s adresou:%d\n", addr);
      exit(1);
    }
    lastAddr = addr;
  }
}

void writeRegister(unsigned char addr, unsigned char reg, unsigned char value) {
  unsigned char data[3];
  data[0] = reg;
  data[1] = value;
  setDevice(addr);
  if (write(i2cHandle, data, 2) != 2)
	 printf("addr:%i, write register %i,val %i\n", (int)addr, (int)reg, (int)value);
}

unsigned int readRegister16(unsigned char addr, unsigned char reg) {
  char data[3];
  char errorTimeout = 0;
  data[0] = reg;
  setDevice(addr);
  while (write(i2cHandle, data, 1) != 1){ 
	printf("addr:%i, write register %i, errorTimeout:%i\n", (int)addr, (int)reg,(int)errorTimeout);
	if(errorTimeout++ >= I2C_WRITE_TIMEOUT) break;
  }
  if(errorTimeout < I2C_WRITE_TIMEOUT){
  	if (read(i2cHandle, data, 2) != 2){   
		printf("addr:%i, read register %i\n", (int)addr, (int)reg);
  	}
  	return (data[0] << 8) + data[1];
  }
  else{
	return 0;
  }
}

signed int readRegister16s(unsigned char addr, unsigned char reg) {
  char wdata[2];
  char errorTimeout = 0;
  wdata[0] = reg;
  setDevice(addr);
  while (write(i2cHandle, wdata, 1) != 1){  
	printf("addr:%i, write register %i, errorTimeout:%i\n", (int)addr, (int)reg,(int)errorTimeout);
  	if(errorTimeout++ >= I2C_WRITE_TIMEOUT) break;
  }
  signed char data[3];
  if(errorTimeout < I2C_WRITE_TIMEOUT){
  	if (read(i2cHandle, data, 2) != 2)   printf("addr:%i, read register %i\n", (int)addr, (int)reg);
  	return (data[0] << 8) + data[1];
  }
  else{
	return 0;
  }
}

unsigned char readRegister8(unsigned char addr, unsigned char reg) {
  char data[2];
  char errorTimeout = 0;
  data[0] = reg;
  setDevice(addr);
  while (write(i2cHandle, data, 1) != 1){  
	printf("addr:%i, write register %i, errorTimeout:%i\n", (int)addr, (int)reg,(int)errorTimeout);
	if(errorTimeout++ >= I2C_WRITE_TIMEOUT) break;
  }
  if(errorTimeout < I2C_WRITE_TIMEOUT){
  	if (read(i2cHandle, data, 1) != 1)   printf("addr:%i, read register %i\n", (int)addr, (int)reg);
  	return data[0];
  }
  else{
	return 0;
  }
}

void sendMatImage(Mat img, int quality) {
  vector<uchar> buff;
  vector<int> param = vector<int>(2);
  param[0] = CV_IMWRITE_JPEG_QUALITY;
  param[1] = quality;
  imencode(".jpg", img, buff, param);
  char len[10];
  sprintf(len, "%.8d", buff.size());
  send(clientsock_camera, len, strlen(len), 0);
  send(clientsock_camera, &buff[0], buff.size(), 0);
  buff.clear();
}

void *getImgL(void *arg){
  while(onAllThreads){
    semWait(sem_id,2);
    img1L = cvQueryFrame(cameraL);
		
    semWait(sem_id,3);
    imageChooseL = 1;
    semPost(sem_id,3);
		
    semPost(sem_id,2);
		
    semWait(sem_id,4);	
    img2L = cvQueryFrame(cameraL);
		
    semWait(sem_id,3);
    imageChooseL = 2;
    semPost(sem_id,3);
    
    semPost(sem_id,4);
  }
}

void *getImgR(void *arg){
  while(onAllThreads){
    semWait(sem_id,5);
    img1R = cvQueryFrame(cameraR);
		
    semWait(sem_id,6);
    imageChooseR = 1;
    semPost(sem_id,6);
    
		semPost(sem_id,5);
		
    semWait(sem_id,7);	
    img2R = cvQueryFrame(cameraR);
		
    semWait(sem_id,6);
    imageChooseR = 2;
    semPost(sem_id,6);
    semPost(sem_id,7);
  }
}

void wifiCamera(){    //premenovat
        char recvdata[30];
        int bytes = recv(getSocketCamera(), recvdata, 4, 0);
        if (bytes == 0){
		      sigctrl(0);
        }
        if (strcmp(recvdata, "img\n") == 0){ //&& imgSendL.empty() != true){
          semWait(sem_id, 1);
		      sendMatImage(robotSensors.camera.imgLeft,80);
          semPost(sem_id, 1);
        }
}

void initButton(position3_t pos){
  switch (pos) {
    case POSITION_DOWN:   return !gpio_open(27, 0); break;
    case POSITION_MIDDLE: return !gpio_open(17, 0); break;
    case POSITION_UP:     return !gpio_open(22, 0); break;
  }   
}

void closeButton(position3_t pos){
  switch (pos) {
    case POSITION_DOWN:   return gpio_close(27); break;
    case POSITION_MIDDLE: return gpio_close(17); break;
    case POSITION_UP:     return gpio_close(22); break;
  }   
}

unsigned char getButton(position3_t pos) {
  switch (pos) {
    case POSITION_DOWN:   return !gpio_read(27); break;
    case POSITION_MIDDLE: return !gpio_read(17); break;
    case POSITION_UP:     return !gpio_read(22); break;
    default: return 0;
  }
}

RobotAcculators getRobotAcculators() {
  RobotAcculators temp;
  semWait(sem_id, 1);
  memcpy(&temp, &robotAcculators, sizeof(RobotAcculators));
  semPost(sem_id, 1);
  return temp;
}

void setRobotAcculators(RobotAcculators temp) {
  semWait(sem_id, 1);
  memcpy(&robotAcculators, &temp, sizeof(RobotAcculators));
  semPost(sem_id, 1);
}

RobotSensors getRobotSensors() {
  RobotSensors temp;
  semWait(sem_id, 0);
  memcpy(&temp, &robotSensors, sizeof(RobotSensors));
  semPost(sem_id, 0);
  return temp;
}

int getSocketCamera() {
  return clientsock_camera;
}

int getSocketSnimace() {
  return clientsock_snimace;
}

int testModry() {
  if (300 == readRegister16(MODRYADDR, 127))  return 1;
  else                                        return 0;
}

int testZlty() {
  if (300 == readRegister16(ZLTYADDR, 127)) return 1;
  else                                      return 0;
}

int testOranzovy() {
  if (300 == readRegister16(ORANGE_ADDRESS, 127)) return 1;
  else                                          return 0;
}

int test() {
  if (testZlty() && testOranzovy() && testModry())  return 1;
  else                                              return 0;
}

int getDistanceRaw(position6_t pos) {
  switch (pos) {
    case POSITION_DOWN_RIGHT: return readRegister16s(BLUE_ADDRESS, 3); break;
    case POSITION_MIDDLE_RIGHT: return readRegister16s(ORANGE_ADDRESS, 3); break;
    case POSITION_UP_RIGHT: return readRegister16s(ORANGE_ADDRESS, 4); break;
    case POSITION_UP_LEFT: return readRegister16s(BLUE_ADDRESS, 4); break;
    case POSITION_MIDDLE_LEFT: return readRegister16s(ZLTYADDR, 4); break;
    case POSITION_DOWN_LEFT: return readRegister16s(ZLTYADDR, 3); break;
    default: return 0;
  }
}

float prepocetTikovOtackomeraDoVzdialenosti(position6_t pocetTikov){
  return ((float)pocetTikov * ((M_PI * DIAMERER_WHEEL) / CONST_ENCODER));
}

int getDistance(position6_t pos) {
  return (int)prepocetTikovOtackomeraDoVzdialenosti(getDistanceRaw(pos));
}

int getDeltaDistanceRaw(position6_t pos) {
  switch (pos) {
    case POSITION_DOWN_RIGHT: return readRegister16s(BLUE_ADDRESS, 7); break;
    case POSITION_MIDDLE_RIGHT: return readRegister16s(ORANGE_ADDRESS, 7); break;
    case POSITION_UP_RIGHT: return readRegister16s(ORANGE_ADDRESS, 8); break;
    case POSITION_UP_LEFT: return readRegister16s(BLUE_ADDRESS, 8); break;
    case POSITION_MIDDLE_LEFT: return readRegister16s(ZLTYADDR, 6); break;
    case POSITION_DOWN_LEFT: return readRegister16s(ZLTYADDR, 5); break;
    default: return 0;
  }
}

float getDeltaDistance(position6_t pos) {
  return prepocetTikovOtackomeraDoVzdialenosti(getDeltaDistanceRaw(pos));
}

void resetDistance(position6_t pos) {
  switch (pos) {
    case POSITION_DOWN_RIGHT: writeRegister(BLUE_ADDRESS, 100, 0);  break;
    case POSITION_MIDDLE_RIGHT: writeRegister(ORANGE_ADDRESS, 100, 0);  break;
    case POSITION_UP_RIGHT: writeRegister(ORANGE_ADDRESS, 99, 0);   break;
    case POSITION_UP_LEFT: writeRegister(BLUE_ADDRESS, 99, 0);   break;
    case POSITION_MIDDLE_LEFT: writeRegister(ZLTYADDR, 99, 0);   break;
    case POSITION_DOWN_LEFT: writeRegister(ZLTYADDR, 100, 0);  break;
  }
}

void resetDistanceAll(){
      resetDistance(POSITION_DOWN_RIGHT);
      resetDistance(POSITION_UP_LEFT);
      resetDistance(POSITION_MIDDLE_RIGHT);
      resetDistance(POSITION_UP_RIGHT);
      resetDistance(POSITION_MIDDLE_LEFT);
      resetDistance(POSITION_DOWN_LEFT);
}

void setServo(int angle) {
  if (angle + 91 < 1) writeRegister(ORANGE_ADDRESS, 84, 1);
  else if (angle + 91 > 181) writeRegister(ORANGE_ADDRESS, 84, 181);
  else writeRegister(0x0A, 84, angle + 91);
}

unsigned int getUltrasonicRaw() {
  return readRegister16(ORANGE_ADDRESS, 6);
}

float getUltrasonic() {
  return (float)readRegister16(ORANGE_ADDRESS, 6) / CONST_ULTRASONIC;
}

int getVoltageRaw() {
  return readRegister16(BLUE_ADDRESS, 5);
}

float getVoltage() {
  return (float)getVoltageRaw() * (ADC_MAXIMUM_VOLTAGE / ADC_RESOLUTION) * ((R1 + R2) / R2);
}

float getVoltagePercent() {
  return (float)getVoltage() * (100 / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE)) - (100 / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE)) * MIN_BATTERY_VOLTAGE;
}

int getAmpRaw() {
  return readRegister16(BLUE_ADDRESS, 6);
}

float getAmpVolt() {
  return (float)getAmpRaw() * (ADC_MAXIMUM_VOLTAGE / ADC_RESOLUTION);
}

float getAmp() {
  return ((float)getAmpVolt() - ADC_MAXIMUM_VOLTAGE / 2) / CONST_AMP;
}

void setLed(position3_t pos, color_t color) {
  if (pos == POSITION_DOWN) {
    if (color == COLOR_GREEN) {
      writeRegister(BLUE_ADDRESS, 96, 0);
      writeRegister(BLUE_ADDRESS, 97, 1);
    }
    else if (color == COLOR_RED) {
      writeRegister(BLUE_ADDRESS, 97, 0);
      writeRegister(BLUE_ADDRESS, 96, 1);
    }
    else if (color == COLOR_ORANGE) {
      writeRegister(BLUE_ADDRESS, 97, 1);
      writeRegister(BLUE_ADDRESS, 96, 1);
    }
    else if (color == COLOR_OFF) {
      writeRegister(BLUE_ADDRESS, 97, 0);
      writeRegister(BLUE_ADDRESS, 96, 0);
    }
  }
  else if (pos == POSITION_MIDDLE) {
    if (color == COLOR_GREEN) {
      writeRegister(ZLTYADDR, 96, 0);
      writeRegister(ZLTYADDR, 97, 1);
    }
    else if (color == COLOR_RED) {
      writeRegister(ZLTYADDR, 97, 0);
      writeRegister(ZLTYADDR, 96, 1);
    }
    else if (color == COLOR_ORANGE) {
      writeRegister(ZLTYADDR, 97, 1);
      writeRegister(ZLTYADDR, 96, 1);
    }
    else if (color == COLOR_OFF) {
      writeRegister(ZLTYADDR, 97, 0);
      writeRegister(ZLTYADDR, 96, 0);
    }
  }
  else if (pos == POSITION_UP) {
    if (color == COLOR_GREEN) {
      writeRegister(ORANGE_ADDRESS, 96, 0);
      writeRegister(ORANGE_ADDRESS, 97, 1);
    }
    else if (color == COLOR_RED) {
      writeRegister(ORANGE_ADDRESS, 97, 0);
      writeRegister(ORANGE_ADDRESS, 96, 1);
    }
    else if (color == COLOR_ORANGE) {
      writeRegister(ORANGE_ADDRESS, 97, 1);
      writeRegister(ORANGE_ADDRESS, 96, 1);
    }
    else if (color == COLOR_OFF) {
      writeRegister(ORANGE_ADDRESS, 97, 0);
      writeRegister(ORANGE_ADDRESS, 96, 0);
    }
  }
}

void initMotorPowerSupply(){
    gpio_open(26, 1);
}

void setMotorPowerSupply(bool state) {
  if (state == false)  gpio_write(26, 1);
  else                 gpio_write(26, 0);
}

void closeMotorPowerSupply(){
    gpio_close(26);
}

void setMotor(position6_t pos, rotate_t rotate, unsigned char speed, bool onReg) {
  if (onReg == true) {
    if (rotate == ROTATE_CLOCKWISE) {
      switch (pos) {
        case POSITION_DOWN_RIGHT: writeRegister(BLUE_ADDRESS, 94, speed); break;
        case POSITION_MIDDLE_RIGHT: writeRegister(ORANGE_ADDRESS, 89, speed); break;
        case POSITION_UP_RIGHT: writeRegister(ORANGE_ADDRESS, 94, speed); break;
        case POSITION_UP_LEFT: writeRegister(BLUE_ADDRESS, 89, speed); break;
        case POSITION_MIDDLE_LEFT: writeRegister(ZLTYADDR, 89, speed); break;
        case POSITION_DOWN_LEFT: writeRegister(ZLTYADDR, 94, speed); break;
      }
    }
    else if(rotate == ROTATE_ANTICLOCKWISE) {
      switch (pos) {
        case POSITION_DOWN_RIGHT: writeRegister(BLUE_ADDRESS, 93, speed); break;
        case POSITION_MIDDLE_RIGHT: writeRegister(ORANGE_ADDRESS, 88, speed); break;
        case POSITION_UP_RIGHT: writeRegister(ORANGE_ADDRESS, 93, speed); break;
        case POSITION_UP_LEFT: writeRegister(BLUE_ADDRESS, 88, speed); break;
        case POSITION_MIDDLE_LEFT: writeRegister(ZLTYADDR, 88, speed); break;
        case POSITION_DOWN_LEFT: writeRegister(ZLTYADDR, 93, speed); break;
      }
    }   
    else if(rotate == ROTATE_STOP){
      switch (pos) {
        case POSITION_DOWN_RIGHT: writeRegister(BLUE_ADDRESS, 93, 0); break;
        case POSITION_MIDDLE_RIGHT: writeRegister(ORANGE_ADDRESS, 88, 0); break;
        case POSITION_UP_RIGHT: writeRegister(ORANGE_ADDRESS, 93, 0); break;
        case POSITION_UP_LEFT: writeRegister(BLUE_ADDRESS, 88, 0); break;
        case POSITION_MIDDLE_LEFT: writeRegister(ZLTYADDR, 88, 0); break;
        case POSITION_DOWN_LEFT: writeRegister(ZLTYADDR, 93, 0); break;
      }    
    
    }
  }
  else {
    if (rotate == ROTATE_CLOCKWISE) {
      switch (pos) {
        case POSITION_DOWN_RIGHT: writeRegister(BLUE_ADDRESS, 92, speed); break;
        case POSITION_MIDDLE_RIGHT: writeRegister(ORANGE_ADDRESS, 87, speed); break;
        case POSITION_UP_RIGHT: writeRegister(ORANGE_ADDRESS, 92, speed); break;
        case POSITION_UP_LEFT: writeRegister(BLUE_ADDRESS, 87, speed); break;
        case POSITION_MIDDLE_LEFT: writeRegister(ZLTYADDR, 87, speed); break;
        case POSITION_DOWN_LEFT: writeRegister(ZLTYADDR, 92, speed); break;
      }
    }
    else if (ROTATE_STOP == 0) {
      switch (pos) {
        case POSITION_DOWN_RIGHT: writeRegister(BLUE_ADDRESS, 91, speed); break;
        case POSITION_MIDDLE_RIGHT: writeRegister(ORANGE_ADDRESS, 86, speed); break;
        case POSITION_UP_RIGHT: writeRegister(ORANGE_ADDRESS, 91, speed); break;
        case POSITION_UP_LEFT: writeRegister(BLUE_ADDRESS, 86, speed); break;
        case POSITION_MIDDLE_LEFT: writeRegister(ZLTYADDR, 86, speed); break;
        case POSITION_DOWN_LEFT: writeRegister(ZLTYADDR, 91, speed); break;
      }
    }
    else if(rotate ROTATE_ANTICLOCKWISE) {
      switch (pos) {
        case POSITION_DOWN_RIGHT: writeRegister(BLUE_ADDRESS, 90, speed); break;
        case POSITION_MIDDLE_RIGHT: writeRegister(ORANGE_ADDRESS, 85, speed); break;
        case POSITION_UP_RIGHT: writeRegister(ORANGE_ADDRESS, 90, speed); break;
        case POSITION_UP_LEFT: writeRegister(BLUE_ADDRESS, 85, speed); break;
        case POSITION_MIDDLE_LEFT: writeRegister(ZLTYADDR, 85, speed); break;
        case POSITION_DOWN_LEFT: writeRegister(YELLOW_ADDRESS, 90, speed); break;
      }
    }
  }
}

void stopAllMotors(){
  setMotor(POSITION_DOWN_RIGHT,ROTATE_STOP,0,false);
  setMotor(POSITION_UP_LEFT,ROTATE_STOP,0,false);
  setMotor(POSITION_MIDDLE_RIGHT,ROTATE_STOP,0,false);
  setMotor(POSITION_UP_RIGHT,ROTATE_STOP,0,false);
  setMotor(POSITION_MIDDLE_LEFT,ROTATE_STOP,0,false);
  setMotor(POSITION_DOWN_LEFT,ROTATE_STOP,0,false);
}

void setMotors(side_t side,rotate_t rotate,unsigned char speed,bool onReg){
  switch(side){
    case SIDE_LEFT:
      if(rotate == ROTATE_CLOCKWISE){
        robotAcculators.motors.motorUpLeft.direction = 1;
        robotAcculators.motors.motorMiddleLeft.direction = 1;
        robotAcculators.motors.motorDownLeft.direction = 1;
      }
      else if(rotate == ROTATE_ANTICLOCKWISE){
        robotAcculators.motors.motorUpLeft.direction = -1;
        robotAcculators.motors.motorMiddleLeft.direction = -1;
        robotAcculators.motors.motorDownLeft.direction = -1;      
      }
      else if(rotate == ROTATE_STOP){
        robotAcculators.motors.motorUpLeft.direction = 0;
        robotAcculators.motors.motorMiddleLeft.direction = 0;
        robotAcculators.motors.motorDownLeft.direction = 0;                  
      }
      robotAcculators.motors.motorUpLeft.speed = speed;
      robotAcculators.motors.motorMiddleLeft.speed = speed;
      robotAcculators.motors.motorDownLeft.speed = speed;
      robotAcculators.motors.motorUpLeft.onRegulator = onReg;
      robotAcculators.motors.motorMiddleLeft.onRegulator = onReg;
      robotAcculators.motors.motorDownLeft.onRegulator = onReg;    
      break;    
    case SIDE_RIGHT:
      if(rotate == ROTATE_CLOCKWISE){
        robotAcculators.motors.motorDownRight.direction = 1;
        robotAcculators.motors.motorMiddleRight.direction = 1;
        robotAcculators.motors.motormotorUpRight.direction = 1;
      }
      else if(rotate == ROTATE_ANTICLOCKWISE){
        robotAcculators.motors.motorDownRight.direction = -1;
        robotAcculators.motors.motorMiddleRight.direction = -1;
        robotAcculators.motors.motorUpRight.direction = -1;      
      }
      else if(rotate == ROTATE_STOP){
        robotAcculators.motors.motorDownRight.direction = 0;
        robotAcculators.motors.motorMiddleRight.direction = 0;
        robotAcculators.motors.motorUpRight.direction = 0;                  
      }
      robotAcculators.motors.motorDownRight.speed = speed;
      robotAcculators.motors.motorMiddleRight.speed = speed;
      robotAcculators.motors.motorUpRight.speed = speed;
      robotAcculators.motors.motorDownRight.onRegulator = onReg;
      robotAcculators.motors.motorMiddleRight.onRegulator = onReg;
      robotAcculators.motors.motorUpRight.onRegulator = onReg;
      break;
  }
}

void setMove(direction_t direction,unsigned char speed,bool onReg){
  semWait(sem_id, 1);
  if(direction == DIRECTION_FRONT){
    setMotors(SIDE_LEFT,ROTATE_CLOCKWISE,speed,onReg);
    setMotors(SIDE_RIGHT,ROTATE_CLOCKWISE,speed,onReg);
  }
  else if(direction == DIRECTION_BACK){
    setMotors(SIDE_LEFT,ROTATE_ANTICLOCKWISE,speed,onReg);
    setMotors(SIDE_RIGHT,ROTATE_ANTICLOCKWISE,speed,onReg);
  }
  else if(direction == DIRECTION_RIGHT){
    setMotors(SIDE_LEFT,ROTATE_CLOCKWISE,speed,onReg);
    setMotors(SIDE_RIGHT,ROTATE_ANTICLOCKWISE,speed,onReg);
  }
  else if(direction == DIRECTION_LEFT){
    setMotors(SIDE_LEFT,ROTATE_ANTICLOCKWISE,speed,onReg);
    setMotors(SIDE_RIGHT,ROTATE_CLOCKWISE,speed,onReg);
  }
  else if(direction == DIRECTION_STOP){
    setMotors(SIDE_LEFT,ROTATE_STOP,speed,onReg);
    setMotors(SIDE_RIGHT,ROTATE_STOP,speed,onReg);  
  }
  semPost(sem_id, 1);
}



int getKbhit(void) {
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

GPS_struct getGPS() {
  GPS_struct GPS;
  while (strcmp(SerialRead(portHandle, 1), "$") != 0);
  char *dataCMD = SerialRead(portHandle, 5);
  int i = 0;
  int x = 0;
  char buffer[20];
  if (!strcmp(dataCMD, "GPVTG")) {
    char data[] = {' '};
    while (data[0] != '\n') {
      char *datas = SerialRead(portHandle, 1);
      data[0] = datas[0];
      if (data[0] == ',') {
        buffer[x] = '\0';
        switch (i++) {
          case 1:  GPS.GPVTG.CourseTrue = atof(buffer);       break;
          case 3:  GPS.GPVTG.CourseMagnetic = atof(buffer);   break;
          case 5:  GPS.GPVTG.SpeedKnots = atoi(buffer);       break;
          case 7:  GPS.GPVTG.SpeedKmh = atof(buffer);          break;
        }
        x = 0;
      }
      else {
        switch (i) {
          case 1:
          case 3:
          case 5:
          case 7:  buffer[x++] = data[0];                 break;
          case 2:  GPS.GPVTG.ReferenceTrue = data[0];         break;
          case 4:  GPS.GPVTG.ReferenceMagnetic = data[0];     break;
          case 6:  GPS.GPVTG.UnitsKnots = data[0];            break;
          case 8:  GPS.GPVTG.UnitsKmh = data[0];            break;
          case 9:  GPS.GPVTG.Mode = data[0];      break;
          case 10: GPS.GPVTG.Checksum[x++] = data[0];         break;
        }
      }
    }
  }
  else if (!strcmp(dataCMD, "GPGGA")) {
    char data[] = {' '};
    while (data[0] != '\n') {
      char *datas = SerialRead(portHandle, 1);
      data[0] = datas[0];
      if (data[0] == ',') {
        buffer[x] = '\0';
        switch (i++) {
          case 2:  GPS.GPGGA.Latitude = atof(buffer);     break;
          case 4:  GPS.GPGGA.Longitude = atof(buffer);      break;
          case 7:  GPS.GPGGA.SatellitesUsed = atoi(buffer);   break;
          case 8:  GPS.GPGGA.HDOP = atof(buffer);       break;
          case 9:  GPS.GPGGA.MSLAltitude = atof(buffer);    break;
          case 10: GPS.GPGGA.GeoidSeparation = atoi(buffer);    break;
          case 12: GPS.GPGGA.AgeofDifferentialCorrections = atoi(buffer); break;
        }
        x = 0;
      }
      else {
        switch (i) {
          case 1:  GPS.GPGGA.UTCTime[x++] = data[0];         break;
          case 2:
          case 7:
          case 8:
          case 9:
          case 4:  buffer[x++] = data[0];                 break;
          case 10: GPS.GPGGA.Units1 = data[0];    break;
          case 12: GPS.GPGGA.Units2 = data[0];    break;
          case 3:  GPS.GPGGA.NSIndicator = data[0];           break;
          case 5:  GPS.GPGGA.EWindicator = data[0];           break;
          case 6:  GPS.GPGGA.PositionFixIndictor = data[0];   break;
          case 13: GPS.GPGGA.Checksum[x++] = data[0];   break;
        }
      }
    }
  }
  else if (!strcmp(dataCMD, "GPGSA")) {
    char data[] = {' '};
    while (data[0] != '\n') {
      char *datas = SerialRead(portHandle, 1);
      data[0] = datas[0];
      if (data[0] == ',') {
        buffer[x] = '\0';
        switch (i++) {
          case 3:  GPS.GPGSA.SatellitesUsedCH1 = atoi(buffer); break;
          case 4:  GPS.GPGSA.SatellitesUsedCH2 = atoi(buffer); break;
          case 5:  GPS.GPGSA.SatellitesUsedCH3 = atoi(buffer); break;
          case 6:  GPS.GPGSA.SatellitesUsedCH4 = atoi(buffer); break;
          case 7:  GPS.GPGSA.SatellitesUsedCH5 = atoi(buffer); break;
          case 8:  GPS.GPGSA.SatellitesUsedCH6 = atoi(buffer); break;
          case 9:  GPS.GPGSA.SatellitesUsedCH7 = atoi(buffer); break;
          case 10:  GPS.GPGSA.SatellitesUsedCH8 = atoi(buffer); break;
          case 11:  GPS.GPGSA.SatellitesUsedCH9 = atoi(buffer); break;
          case 12:  GPS.GPGSA.SatellitesUsedCH10 = atoi(buffer); break;
          case 13:  GPS.GPGSA.SatellitesUsedCH11 = atoi(buffer); break;
          case 14:  GPS.GPGSA.SatellitesUsedCH12 = atoi(buffer); break;
          case 15:  GPS.GPGSA.PDOP = atof(buffer); break;
          case 16:  GPS.GPGSA.HDOP = atof(buffer); break;
          case 17:  GPS.GPGSA.VDOP = atof(buffer); break;
        }
        x = 0;
      }
      else {
        switch (i) {
          case 1:  GPS.GPGSA.ModeChar = data[0];  break;
          case 2:  GPS.GPGSA.ModeInt = data[0];       break;
          case 3:
          case 4:
          case 5:
          case 6:
          case 7:
          case 8:
          case 9:
          case 10:
          case 11:
          case 12:
          case 13:
          case 14:
          case 15:
          case 16:
          case 17:  buffer[x++] = data[0];         break;
          case 18:  GPS.GPGSA.Checksum[x++] = data[0]; break;
        }
      }
    }
  }
  else if (!strcmp(dataCMD, "GPGLL")) {
    char data[] = {' '};
    while (data[0] != '\n') {
      char *datas = SerialRead(portHandle, 1);
      data[0] = datas[0];
      if (data[0] == ',') {
        buffer[x] = '\0';
        switch (i++) {
          case 1:  GPS.GPGLL.Latitude = atof(buffer);                 break;
          case 3:  GPS.GPGLL.Longitude = atof(buffer);                break;
        }
        x = 0;
      }
      else {
        switch (i) {
          case 1:
          case 3:  buffer[x++] = data[0];         break;
          case 2:  GPS.GPGLL.NSIndicator = data[0]; break;
          case 4:  GPS.GPGLL.EWIndicator = data[0];   break;
          case 5:  GPS.GPGLL.UTCTime[x++] = data[0];  break;
          case 6:  GPS.GPGLL.Status = data[0];    break;
          case 7:  GPS.GPGLL.ModeIndicator = data[0]; break;
          case 8:  GPS.GPGLL.Checksum[x++] = data[0]; break;
        }
      }
    }
  }
  else if (!strcmp(dataCMD, "GPRMC")) {
    char data[] = {' '};
    while (data[0] != '\n') {
      char *datas = SerialRead(portHandle, 1);
      data[0] = datas[0];
      if (data[0] == ',') {
        buffer[x] = '\0';
        switch (i++) {
          case 3:  GPS.GPRMC.Latitude = atof(buffer); break;
          case 5:  GPS.GPRMC.Longitude = atof(buffer); break;
          case 7:  GPS.GPRMC.SpeedOverGround = atof(buffer); break;
          case 8:  GPS.GPRMC.CourseOverGround = atof(buffer); break;
        }
        x = 0;
      }
      else {
        switch (i) {
          case 1:  GPS.GPRMC.UTCTime[x++] = data[0];  break;
          case 2:  GPS.GPRMC.Status = data[0];        break;
          case 9:  GPS.GPRMC.Date[x++] = data[0]; break;
          case 5:
          case 7:
          case 8:
          case 3:  buffer[x++] = data[0];         break;
          case 10: GPS.GPRMC.Mode = data[0];    break;
          case 4:  GPS.GPRMC.NSIndicator = data[0];   break;
          case 6:  GPS.GPRMC.EWIndicator = data[0];   break;
          case 11: GPS.GPRMC.Checksum[x++] = data[0]; break;
        }
      }
    }
  }
  else if (!strcmp(dataCMD, "GPGSV")) {
    char data[] = {' '};
    while (data[0] != '\n') {
      char *datas = SerialRead(portHandle, 1);
      data[0] = datas[0];
      if (data[0] == ',') {
        buffer[x] = '\0';
        switch (i++) {
          case 1:  GPS.GPGSV.NumberOfMessages = atoi(buffer); break;
          case 2:  GPS.GPGSV.MessageNumber = atoi(buffer);    break;
          case 3:  GPS.GPGSV.SatellitesInView = atoi(buffer); break;
          case 4:  GPS.GPGSV.SatelliteId1 = atoi(buffer);     break;
          case 5:  GPS.GPGSV.Elevation1 = atoi(buffer);     break;
          case 6:  GPS.GPGSV.Azimuth1 = atoi(buffer);     break;
          case 7:  GPS.GPGSV.SNR1 = atoi(buffer);       break;
          case 8:  GPS.GPGSV.SatelliteId2 = atoi(buffer);     break;
          case 9:  GPS.GPGSV.Elevation2 = atoi(buffer);     break;
          case 10: GPS.GPGSV.Azimuth2 = atoi(buffer);     break;
          case 11: GPS.GPGSV.SNR2 = atoi(buffer);       break;
          case 12: GPS.GPGSV.SatelliteId3 = atoi(buffer);     break;
          case 13: GPS.GPGSV.Elevation3 = atoi(buffer);     break;
          case 14: GPS.GPGSV.Azimuth3 = atoi(buffer);     break;
          case 15: GPS.GPGSV.SNR3 = atoi(buffer);       break;
          case 16: GPS.GPGSV.SatelliteId4 = atoi(buffer);     break;
          case 17: GPS.GPGSV.Elevation4 = atoi(buffer);     break;
          case 18: GPS.GPGSV.Azimuth4 = atoi(buffer);     break;
          case 19: GPS.GPGSV.SNR4 = atoi(buffer);       break;
        }
        x = 0;
      }
      else {
        switch (i) {
          case 1:
          case 2:
          case 3:
          case 4:
          case 5:
          case 6:
          case 7:
          case 8:
          case 9:
          case 10:
          case 11:
          case 12:
          case 13:
          case 14:
          case 15:
          case 16:
          case 17:
          case 18:
          case 19:  buffer[x++] = data[0];         break;
          case 20:  GPS.GPGSV.Checksum[x++] = data[0]; break;
        }
      }
    }
  }
  return GPS;
}

void HMC5883LMeasurementSetting(int mode){
  switch(mode){
    case 0: mode = 0; break;   // normal measurement configuration(Default)
    case 1: mode = 1; break;   // positive bias configuration (more resistive)
    case 2: mode = 2; break;   // negative bias configuration (more resistive)
    default:mode = 0; break;   // normal measurement configuration(Default)
  }
  writeRegister(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_A,((sample<<5)|(datarate<<2)|mode));
}

void HMC5883LSampleSetting(hmc5883l_samples_t sample){
  char oldRegister = readRegister8(HMC5883L_REG_CONFIG_B) && 0x80;
  writeRegister(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_A,((sample<<5)|(datarate<<2)|mode));
}

void HMC5883LRateSetting(hmc5883l_dataRate_t datarate){
  char oldRegister = readRegister8(HMC5883L_REG_CONFIG_B) && 0x80;
  writeRegister(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_A,((sample<<5)|(datarate<<2)|mode));
}

void HMC5883LRangeSetting(hmc5883l_range_t range){
  switch(range){
    case HMC5883L_RANGE_0_88GA:
            mgPerDigit = 0.73f;
            break; 
    case HMC5883L_RANGE_1_3GA: 
            mgPerDigit = 0.92f;
            break; 
    case HMC5883L_RANGE_1_9GA:  
            mgPerDigit = 1.22f;
            break; 
    case HMC5883L_RANGE_2_5GA:  
            mgPerDigit = 1.52f;
            break; 
    case HMC5883L_RANGE_4GA: 
            mgPerDigit = 2.27f;
            break; 
    case HMC5883L_RANGE_4_7GA: 
            mgPerDigit = 2.56f;
            break; 
    case HMC5883L_RANGE_5_6GA: 
            mgPerDigit = 3.03f;
            break; 
    case HMC5883L_RANGE_8_1GA: 
            mgPerDigit = 4.35f;
            break; 
  }
  writeRegister(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_A,range<<5);
}

void HMC5883LReadModeSetting(hmc5883l_mode_t mode){
  char oldRegister = readRegister8(HMC5883L_REG_CONFIG_B) && 0x80;
  writeRegister(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_B,oldRegister | mode);
}

void HMC5883LHighI2CSpeedSetting(bool status){
  char oldRegister = readRegister8(HMC5883L_REG_CONFIG_B) && 0x7F;
  if(status)
    oldRegister |= 0x80;
  writeRegister(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_B,oldRegister);
}

HMC5883L_struct getHMC5883LRaw() {
  HMC5883L_struct HMC5883L;
  HMC5883L.X = (float)readRegister16s(HMC5883L_ADDRESS, HMC5883L_REG_OUT_X_M);
  HMC5883L.Y = (float)readRegister16s(HMC5883L_ADDRESS, HMC5883L_REG_OUT_Y_M);
  HMC5883L.Z = (float)readRegister16s(HMC5883L_ADDRESS, HMC5883L_REG_OUT_Z_M);
  printf("X:%f,Y:%f,normX:%f,normY:%f,mgPerDigit:%f\n",HMC5883L.X,HMC5883L.Y,HMC5883L.X*mgPerDigit,HMC5883L.Y*mgPerDigit,mgPerDigit);
  return HMC5883L;
}

void calibrateOffsetHMC5883L(HMC5883L_struct HMC5883L){
 if (HMC5883L.X < minX) minX = HMC5883L.X;
 if (HMC5883L.X > maxX) maxX = HMC5883L.X;
 if (HMC5883L.Y < minY) minY = HMC5883L.Y;
 if (HMC5883L.Y > maxY) maxY = HMC5883L.Y;
 offX = (maxX + minX)/2;
 offY = (maxY + minY)/2;
// printf("minX:%f,minY:%f,maxX:%f,maxY:%f,offX:%f,offY:%f\n",minX,minY,maxX,maxY,offX,offY);
}

HMC5883L_struct normHMC5883L(HMC5883L_struct HMC5883L) {
  
  HMC5883L.X = (HMC5883L.X-offX) * mgPerDigit;
  HMC5883L.Y = (HMC5883L.Y-offY) * mgPerDigit;
  HMC5883L.Z = HMC5883L.Z * mgPerDigit;

//  float declinationAngle = (HMC5883L_DEGREE + (HMC5883L_MINUTES / 60.0)) / (180 / M_PI);   //posun podla zemepisnej sirky a dlzky
  HMC5883L.angleRad = atan2(HMC5883L.Y,HMC5883L.X); //+ declinationAngle; 
  
if(HMC5883L.angleRad < 0){
    HMC5883L.angleRad+= 2*M_PI;
  }
  else if(HMC5883L.angleRad > 2*M_PI){
    HMC5883L.angleRad-=2*M_PI;
  }

  HMC5883L.angleDeg = HMC5883L.angleRad*(180/M_PI);
  return HMC5883L;
}

void MPU6050ResetPRY() {
  Pitch = 0;
  Roll = 0;
  Yaw = 0;
}

void MPU6050ResetOffset() {
  AccX_offset = 0;
  AccY_offset = 0;
  AccZ_offset = 0;
  GyX_offset = 0;
  GyY_offset = 0;
  GyZ_offset = 0;
}

void MPU6050WakeUp() {
  writeRegister(MPU6050_ADDRESS, 0x6B, readRegister8(MPU6050_ADDRESS,0x6B)&(!(1<<6)));
}

void MPU6050DisableAsMaster(){
   writeRegister(MPU6050_ADDRESS, 0x6A, readRegister8(MPU6050_ADDRESS,0x6A)&(!(1<<5|1<<4)));
   writeRegister(MPU6050_ADDRESS, 0x37, readRegister8(MPU6050_ADDRESS,0x37)|(1<<1));
}

MPU6050_struct getMPU6050Raw() {
  MPU6050_struct MPU6050;
  MPU6050.AccX = (float)readRegister16s(MPU6050_ADDRESS, 0x3B);
  MPU6050.AccY = (float)readRegister16s(MPU6050_ADDRESS, 0x3D);
  MPU6050.AccZ = (float)readRegister16s(MPU6050_ADDRESS, 0x3F);
  MPU6050.Temp = (float)readRegister16s(MPU6050_ADDRESS, 0x41);
  MPU6050.GyX  = (float)readRegister16s(MPU6050_ADDRESS, 0x43);
  MPU6050.GyY  = (float)readRegister16s(MPU6050_ADDRESS, 0x45);
  MPU6050.GyZ  = (float)readRegister16s(MPU6050_ADDRESS, 0x47);
  return MPU6050;
}

MPU6050_struct getMPU6050() {
  MPU6050_struct MPU6050;
  MPU6050.AccX = (float)readRegister16s(MPU6050_ADDRESS, 0x3B) / AccScale - AccX_offset;
  MPU6050.AccY = (float)readRegister16s(MPU6050_ADDRESS, 0x3D) / AccScale - AccY_offset;
  MPU6050.AccZ = (float)readRegister16s(MPU6050_ADDRESS, 0x3F) / AccScale - AccZ_offset;
  MPU6050.Temp = (float)readRegister16s(MPU6050_ADDRESS, 0x41) / 340 + 36.53;
  MPU6050.GyX  = (float)readRegister16s(MPU6050_ADDRESS, 0x43) / GyScale - GyX_offset;
  MPU6050.GyY  = (float)readRegister16s(MPU6050_ADDRESS, 0x45) / GyScale - GyY_offset;
  MPU6050.GyZ  = (float)readRegister16s(MPU6050_ADDRESS, 0x47) / GyScale - GyZ_offset;
  return MPU6050;
}

void MPU6050CalibrateOffset(int pocet) {
  float acc_x = 0, acc_y = 0, acc_z = 0;
  float gy_x = 0, gy_y = 0, gy_z = 0;
  MPU6050_struct MPU6050;
  for (int i = 0; i < pocet; i++) {
    MPU6050 = getMPU6050();
    acc_x += MPU6050.AccX;
    acc_y += MPU6050.AccY;
    acc_z += MPU6050.AccZ;
    gy_x += MPU6050.GyX;
    gy_y += MPU6050.GyY;
    gy_z += MPU6050.GyZ;
  }
  AccX_offset = acc_x / pocet;
  AccY_offset = acc_y / pocet;
  AccZ_offset = acc_z / pocet + 1;

  GyX_offset = gy_x / pocet;
  GyY_offset = gy_y / pocet;
  GyZ_offset = gy_z / pocet;
}

float dist(float a, float b) {
  return sqrt(a * a + b * b);
}

void setMPU6050Sensitivity(unsigned char acc_sens, unsigned char gy_sens) {
  switch (acc_sens) {
    case 0: AccScale = 16384; break;    //2g
    case 1: AccScale = 8192;  break;    //4g
    case 2: AccScale = 4096;  break;    //8g
    case 3: AccScale = 2048;  break;    //16g
  }

  switch (gy_sens) {
    case 0: GyScale = 131;        break;          //250 stup/s
    case 1: GyScale = 65.5;       break;          //500 stup/s
    case 2: GyScale = 32.75;      break;          //1000 stup/s
    case 3: GyScale = 16.375;     break;          //2000 stup/s
  }
  writeRegister(MPU6050_ADDRESS, 0x1B, (gy_sens << 3) | 0xE0);
  writeRegister(MPU6050_ADDRESS, 0x1C, (acc_sens << 3) | 0xE0);
}

void setMPU6050DLPF(unsigned char acc_dlpf, unsigned char gy_dlpf) {
  writeRegister(MPU6050_ADDRESS, 0x1A, acc_dlpf | (5 << 3));
  writeRegister(MPU6050_ADDRESS, 0x1A, acc_dlpf | (6 << 3));
  writeRegister(MPU6050_ADDRESS, 0x1A, acc_dlpf | (7 << 3));
  writeRegister(MPU6050_ADDRESS, 0x1A, gy_dlpf | (2 << 3));
  writeRegister(MPU6050_ADDRESS, 0x1A, gy_dlpf | (3 << 3));
  writeRegister(MPU6050_ADDRESS, 0x1A, gy_dlpf | (4 << 3));
}

float getSpeedFromDistance(float distance,float dt) {
  return distance / dt;
}

//http://rossum.sourceforge.net/papers/DiffSteer/DiffSteer.html
//http://users.isr.ist.utl.pt/~mir/cadeiras/robmovel/Kinematics.pdf
void calcRobotPosition(float deltaSpeedL,float deltaSpeedR,float dt) {
  float v = (deltaSpeedL+deltaSpeedR)/2;
  robotSensors.robotPosition.angleRad+= ((deltaSpeedR-deltaSpeedL) / LENGTH_BETWEEN_LEFT_AND_RIGHT_WHEEL)*dt;
  robotSensors.robotPosition.angleDeg = robotSensors.robotPosition.angleRad*(360/(2*M_PI));
  robotSensors.robotPosition.x +=  v*cos(robotSensors.robotPosition.angleRad)*dt;
  robotSensors.robotPosition.y +=  v*sin(robotSensors.robotPosition.angleRad)*dt;
}

bool compareMotors(MotorAcculator_struct motor, MotorAcculator_struct lastMotor) {
  return motor.direction != lastMotor.direction || motor.speed != lastMotor.speed || motor.onRegulator != lastMotor.onRegulator;
}

void initRefresh(){
      struct sigevent CasovacSignalEvent;
      CasovacSignalEvent.sigev_notify = SIGEV_SIGNAL;
      CasovacSignalEvent.sigev_signo = SIGUSR1;
  
      timer_create(CLOCK_REALTIME, &CasovacSignalEvent, &casovac);
      struct itimerspec cas;
      cas.it_value.tv_sec = 0;
      cas.it_value.tv_nsec = 100 * 1000 * 1000;
      cas.it_interval.tv_sec = 0;
      cas.it_interval.tv_nsec = 100 * 1000 * 1000;
      timer_settime(casovac, CLOCK_REALTIME, &cas, NULL);
      sigset_t signalSet;
      struct sigaction CasovacSignalAction;
      sigemptyset(&signalSet);
      CasovacSignalAction.sa_sigaction = syncModules;
      CasovacSignalAction.sa_flags = SA_SIGINFO;
      CasovacSignalAction.sa_mask = signalSet;
      sigaction(CasovacSignalEvent.sigev_signo, &CasovacSignalAction, NULL);
}

void stopRefresh(){
  struct itimerspec cas;
  cas.it_value.tv_sec = 0;
  cas.it_value.tv_nsec = 0;
  cas.it_interval.tv_sec = 0;
  cas.it_interval.tv_nsec = 0;
  timer_settime(casovac, CLOCK_REALTIME, &cas, NULL);
}

void syncModules(int signal , siginfo_t * siginfo, void * ptr) {
  switch (signal)
  {
    case SIGUSR1:
    
      bool refreshPositionCheck = (REFRESH_POSITION / REFRESH_MODULE) <= pocetPosition;
      bool refreshMotorsCheck = (REFRESH_MOTORS / REFRESH_MODULE) <= pocetMotors;
      bool refreshBatteryCheck = (REFRESH_BATTERY / REFRESH_MODULE) <= pocetBattery;
      bool refreshMPU6050Check = (REFRESH_MPU6050 / REFRESH_MODULE) <= pocetMPU6050;
      bool refreshBMP180Check = (REFRESH_BMP180 / REFRESH_MODULE) <= pocetBMP180;
      bool refreshHMC5883LCheck = (REFRESH_HMC5883L / REFRESH_MODULE) <= pocetHMC5883L;
      bool refreshLedsCheck = (REFRESH_Leds / REFRESH_MODULE) <= pocetLeds;
      bool refreshAmpCheck = (REFRESH_AMP / REFRESH_MODULE) <= pocetAmp;
      bool refreshUltrasonicCheck = (REFRESH_ULTRASONIC / REFRESH_MODULE) <= pocetUltrasonic;   
      bool refreshUltrasonicCheck = (REFRESH_CAMERA / REFRESH_MODULE) <= pocetCamera;
      
      float pomocnaZmenaOtackomera = 0;
      float deltaDistanceL;
      float deltaDistanceR;
      float deltaDistanceDownRight;
      float deltaDistanceMiddleRight;
      float deltaDistanceUpRight;
      float deltaDistanceUpLeft;
      float deltaDistanceMiddleLeft;
      float deltaDistanceDownLeft;
      float imageChooseMainL;
      float imageChooseMainR;
      
      if(refreshCamera && numberOfCamera > 0){
        if(NUMBER_OF_CAMERA == 1 || NUMBER_OF_CAMERA == 2){
          semWait(sem_id,3);
          imageChooseMainL = imageChooseL;
          semPost(sem_id,3);
    		  if(imageChooseMainL !=0){
            if(imageChooseMainL == 1){        
    				  semWait(sem_id,2);
                semWait(sem_id, 0);
      				  cvarrToMat(img1L).copyTo(robotSensors.camera.imgLeft);
                semPost(sem_id, 0);
    				  semPost(sem_id,2);
    			  }
            else if(imageChooseMainL == 2){   
    				  semWait(sem_id1,4);
              semWait(sem_id, 0);
    				  cvarrToMat(img2L).copyTo(robotSensors.camera.imgLeft);
              semPost(sem_id, 0);
    				  semPost(sem_id1,4);
    			  }
          }
        }
        if(NUMBER_OF_CAMERA == 2){
          semWait(sem_id,6);
          imageChooseMainR = imageChooseR;
          semPost(sem_id,6);
    		  if(imageChooseMainR !=0){
            if(imageChooseMainR == 1){        
    				  semWait(sem_id,5);
                semWait(sem_id, 0);
      				  cvarrToMat(img1R).copyTo(robotSensors.camera.imgRight);
                semPost(sem_id, 0);
    				  semPost(sem_id,5);
    			  }
            else if(imageChooseMainR == 2){   
    				  semWait(sem_id,7);
                semWait(sem_id, 0);
      				  cvarrToMat(img2R).copyTo(robotSensors.camera.imgRight);
                semPost(sem_id, 0);
    				  semPost(sem_id,7);
    			  }
          }
        }
      }
        
      if (refreshBMP180Check) {
        semWait(sem_id, 0);
        //sem pojde BMO180
        semPost(sem_id, 0);
        pocetBMP180 = 0;
      }

      if (refreshHMC5883LCheck) {
        semWait(sem_id, 0);
        robotSensors.HMC5883L = getHMC5883LRaw();
//	calibrateOffsetHMC5883L(robotSensors.HMC5883L);
	      robotSensors.HMC5883L = normHMC5883L(robotSensors.HMC5883L); 
        semPost(sem_id, 0);
        pocetHMC5883L = 0;
      }
      
      if (refreshMPU6050Check) {
        semWait(sem_id, 0);
        robotSensors.MPU6050 = getMPU6050();
        semPost(sem_id, 0);
        pocetMPU6050 = 0;
      }

      if (refreshPositionCheck) {
        semWait(sem_id, 0);
	      pomocnaZmenaOtackomera = getDeltaDistanceRaw(POSITION_DOWN_RIGHT);
        if(pomocnaZmenaOtackomera > MAX_DELTA_TICKS_ENCODER) pomocnaZmenaOtackomera = MAX_DELTA_TICKS_ENCODER;
        else if(pomocnaZmenaOtackomera < -MAX_DELTA_TICKS_ENCODER) pomocnaZmenaOtackomera = -MAX_DELTA_TICKS_ENCODER;

        robotSensors.motors.motorDownRight.distanceRaw+= pomocnaZmenaOtackomera;
        deltaDistanceDownRight = prepocetTikovOtackomeraDoVzdialenosti(pomocnaZmenaOtackomera);
 	      robotSensors.motors.motorDownRight.distance +=  deltaDistanceDownRight;
        robotSensors.motors.motorDownRight.speed = getSpeedFromDistance(deltaDistanceDownRight,(float)refreshPosition / 1000);
        
        semPost(sem_id, 0);
        pocetPosition = 0;
      }
      if (refreshMotorsCheck || compareMotors(robotAcculators.motors.motorDownRight, lastRobotAcculators.motors.motorDownRight)) {
        semWait(sem_id, 1);
        setMotor(POSITION_DOWN_RIGHT, robotAcculators.motors.motorDownRight.direction, robotAcculators.motors.motorDownRight.speed, robotAcculators.motors.motorDownRight.onRegulator);
        semPost(sem_id, 1);
        pocetMotors = 0;
      }
      if (refreshPositionCheck) {
        semWait(sem_id, 0);
	      pomocnaZmenaOtackomera = getDeltaDistanceRaw(POSITION_UP_LEFT);
	      if(pomocnaZmenaOtackomera > MAX_DELTA_TICKS_ENCODER) pomocnaZmenaOtackomera = MAX_DELTA_TICKS_ENCODER;
	      else if(pomocnaZmenaOtackomera < -MAX_DELTA_TICKS_ENCODER) pomocnaZmenaOtackomera = -MAX_DELTA_TICKS_ENCODER;
        
        robotSensors.motors.motorUpLeft.distanceRaw+= pomocnaZmenaOtackomera;
        deltaDistanceUpLeft = prepocetTikovOtackomeraDoVzdialenosti(pomocnaZmenaOtackomera);
 	      robotSensors.motors.motorUpLeft.distance +=  deltaDistanceUpLeft;
        robotSensors.motors.motorUpLeft.speed = getSpeedFromDistance(deltaDistanceUpLeft,(float)refreshPosition / 1000);
        
        semPost(sem_id, 0);
        pocetPosition = 0;
      }
      if (refreshMotorsCheck || compareMotors(robotAcculators.motors.motorUpLeft, lastRobotAcculators.motors.motorUpLeft)) {
        semWait(sem_id, 1);
        setMotor(POSITION_UP_LEFT, robotAcculators.motors.motorUpLeft.direction, robotAcculators.motors.motorUpLeft.speed, robotAcculators.motors.motorUpLeft.onRegulator);
        semPost(sem_id, 1);
        pocetMotors = 0;
      }
      if (refreshPositionCheck) {
        semWait(sem_id, 0);
	      pomocnaZmenaOtackomera = getDeltaDistanceRaw(POSITION_MIDDLE_RIGHT);
        if(pomocnaZmenaOtackomera > MAX_DELTA_TICKS_ENCODER) pomocnaZmenaOtackomera = MAX_DELTA_TICKS_ENCODER;
        else if(pomocnaZmenaOtackomera < -MAX_DELTA_TICKS_ENCODER) pomocnaZmenaOtackomera = -MAX_DELTA_TICKS_ENCODER;

        robotSensors.motors.motorMiddleRight.distanceRaw+= pomocnaZmenaOtackomera;
        deltaDistanceMiddleRight = prepocetTikovOtackomeraDoVzdialenosti(pomocnaZmenaOtackomera);
 	      robotSensors.motors.motorMiddleRight.distance +=  deltaDistanceMiddleRight;
        robotSensors.motors.motorMiddleRight.speed = getSpeedFromDistance(deltaDistanceMiddleRight,(float)refreshPosition / 1000);
        
        semPost(sem_id, 0);
        pocetPosition = 0;
      }
      if (refreshMotorsCheck || compareMotors(robotAcculators.motors.motorMiddleRight, lastRobotAcculators.motors.motorMiddleRight)) {
        semWait(sem_id, 1);
        setMotor(POSITION_MIDDLE_RIGHT, robotAcculators.motors.motorMiddleRight.direction, robotAcculators.motors.motorMiddleRight.speed, robotAcculators.motors.motorMiddleRight.onRegulator);
        semPost(sem_id, 1);
        pocetMotors = 0;
      }
      if (refreshMotorsCheck || robotAcculators.servoAngle != lastRobotAcculators.servoAngle) {
        semWait(sem_id, 1);
        setServo(robotAcculators.servoAngle);
        semPost(sem_id, 1);
        pocetMotors = 0;
      }
      if (refreshPositionCheck) {
        semWait(sem_id, 0);
	      pomocnaZmenaOtackomera = getDeltaDistanceRaw(POSITION_UP_RIGHT);
        if(pomocnaZmenaOtackomera > MAX_DELTA_TICKS_ENCODER) pomocnaZmenaOtackomera = MAX_DELTA_TICKS_ENCODER;
        else if(pomocnaZmenaOtackomera < -MAX_DELTA_TICKS_ENCODER) pomocnaZmenaOtackomera = -MAX_DELTA_TICKS_ENCODER;

        robotSensors.motors.motorUpRight.distanceRaw+= pomocnaZmenaOtackomera;
        deltaDistanceUpRight = prepocetTikovOtackomeraDoVzdialenosti(pomocnaZmenaOtackomera);
 	      robotSensors.motors.motorUpRight.distance +=  deltaDistanceUpRight;
        robotSensors.motors.motorUpRight.speed = getSpeedFromDistance(deltaDistanceUpRight,(float)refreshPosition / 1000);

        semPost(sem_id, 0);
        pocetPosition = 0;
      }
      if (refreshMotorsCheck || compareMotors(robotAcculators.motors.motorUpRight, lastRobotAcculators.motors.motorUpRight)) {
        semWait(sem_id, 1);
        setMotor(POSITION_UP_RIGHT, robotAcculators.motors.motorUpRight.direction, robotAcculators.motors.motorUpRight.speed, robotAcculators.motors.motorUpRight.onRegulator);
        semPost(sem_id, 1);
        pocetMotors = 0;
      }
      if (refreshPositionCheck) {
        semWait(sem_id, 0);
	       pomocnaZmenaOtackomera = getDeltaDistanceRaw(POSITION_MIDDLE_LEFT);
        if(pomocnaZmenaOtackomera > MAX_DELTA_TICKS_ENCODER) pomocnaZmenaOtackomera = MAX_DELTA_TICKS_ENCODER;
        else if(pomocnaZmenaOtackomera < -MAX_DELTA_TICKS_ENCODER) pomocnaZmenaOtackomera = -MAX_DELTA_TICKS_ENCODER;   

        robotSensors.motors.motorMiddleLeft.distanceRaw+= pomocnaZmenaOtackomera;
        deltaDistanceMiddleLeft = prepocetTikovOtackomeraDoVzdialenosti(pomocnaZmenaOtackomera);
 	      robotSensors.motors.motorMiddleLeft.distance +=  deltaDistanceMiddleLeft;
        robotSensors.motors.motorMiddleLeft.speed = getSpeedFromDistance(deltaDistanceMiddleLeft,(float)refreshPosition / 1000);

	      semPost(sem_id, 0);
        pocetPosition = 0;
      }
      if (refreshMotorsCheck || compareMotors(robotAcculators.motors.motorMiddleLeft, lastRobotAcculators.motors.motorMiddleLeft)) {
        semWait(sem_id, 1);
        setMotor(POSITION_MIDDLE_LEFT, robotAcculators.motors.motorMiddleLeft.direction, robotAcculators.motors.motorMiddleLeft.speed, robotAcculators.motors.motorMiddleLeft.onRegulator);
        semPost(sem_id, 1);
        pocetMotors = 0;
      }
      if (refreshPositionCheck) {
        semWait(sem_id, 0);
	       pomocnaZmenaOtackomera = getDeltaDistanceRaw(POSITION_DOWN_LEFT);
        if(pomocnaZmenaOtackomera > MAX_DELTA_TICKS_ENCODER) pomocnaZmenaOtackomera = MAX_DELTA_TICKS_ENCODER;
        else if(pomocnaZmenaOtackomera < -MAX_DELTA_TICKS_ENCODER) pomocnaZmenaOtackomera = -MAX_DELTA_TICKS_ENCODER;

        robotSensors.motors.motorDownLeft.distanceRaw+= pomocnaZmenaOtackomera;
        deltaDistanceDownLeft = prepocetTikovOtackomeraDoVzdialenosti(pomocnaZmenaOtackomera);
 	      robotSensors.motors.motorDownLeft.distance +=  deltaDistanceDownLeft;
        robotSensors.motors.motorDownLeft.speed = getSpeedFromDistance(deltaDistanceDownLeft,(float)refreshPosition / 1000);

        semPost(sem_id, 0);
        pocetPosition = 0;
      }
      if (refreshMotorsCheck || compareMotors(robotAcculators.motors.motorDownLeft, lastRobotAcculators.motors.motorDownLeft)) {
        semWait(sem_id, 1);
        setMotor(POSITION_DOWN_LEFT, robotAcculators.motors.motorDownLeft.direction, robotAcculators.motors.motorDownLeft.speed, robotAcculators.motors.motorDownLeft.onRegulator);
        semPost(sem_id, 1);
        pocetMotors = 0;
      }
      if (refreshLedsCheck) {
        semWait(sem_id, 1);
        setLed(POSITION_UP, robotAcculators.leds.LedUp);
        setLed(POSITION_MIDDLE, robotAcculators.leds.LedMiddle);
        if (!BATTERY_LED_INDICATING) setLed(POSITION_DOWN, robotAcculators.leds.LedDown);
        semPost(sem_id, 1);
      }
      if(refreshPositionCheck){
      	float diff1 = abs(deltaDistanceUpLeft-deltaDistanceMiddleLeft);
      	float diff2 = abs(deltaDistanceUpLeft-deltaDistanceDownLeft);
      	float diff3 = abs(deltaDistanceMiddleLeft-deltaDistanceDownLeft);
      	if(diff1 < diff2 && diff1 < diff3)
      		deltaDistanceL = (deltaDistanceUpLeft+deltaDistanceMiddleLeft)/2;
      	else if(diff2 < diff1 && diff2 < diff3)
      		deltaDistanceL = (deltaDistanceUpLeft+deltaDistanceDownLeft)/2;
      	else
      		deltaDistanceL = (deltaDistanceMiddleLeft+deltaDistanceDownLeft)/2;
      	robotSensors.robotPosition.distanceL+= deltaDistanceL;
        robotSensors.robotPosition.speedL = getSpeedFromDistance(deltaDistanceL,(float)refreshPosition / 1000);
                                                             
      	float diff4 = abs(deltaDistanceDownRight-deltaDistanceMiddleRight);
        float diff5 = abs(deltaDistanceMiddleRight-deltaDistanceUpRight);
        float diff6 = abs(deltaDistanceUpRight-deltaDistanceDownRight);
        if(diff4 < diff5 && diff4 < diff6)
          deltaDistanceR = (deltaDistanceDownRight+deltaDistanceMiddleRight)/2;
        else if(diff5 < diff4 && diff5 < diff6)
          deltaDistanceR = (deltaDistanceMiddleRight+deltaDistanceUpRight)/2;
        else
          deltaDistanceR = (deltaDistanceUpRight+deltaDistanceDownRight)/2;
      	robotSensors.robotPosition.distanceR+= deltaDistanceR;      
        robotSensors.robotPosition.speedR = getSpeedFromDistance(deltaDistanceR,(float)refreshPosition / 1000);
        
        calcRobotPosition(robotSensors.robotPosition.speedL,robotSensors.robotPosition.speedR,(float)refreshPosition / 1000);
        
      }
      semWait(sem_id, 0);

      robotSensors.buttons.buttonDown =   getButton(POSITION_DOWN);
      robotSensors.buttons.buttonMiddle = getButton(POSITION_MIDDLE);
      robotSensors.buttons.buttonUp =     getButton(POSITION_UP);
      if (refreshBatteryCheck) {
        robotSensors.voltage = getVoltage();
        robotSensors.voltagePercent = getVoltagePercent();
        if (BATTERY_LED_INDICATING == 1) {
          if (robotSensors.voltagePercent > 60)           setLed(3, 'G');
          else if (robotSensors.voltagePercent > 20)      setLed(3, 'O');
          else                                            setLed(3, 'R');
        }
      }
      if (refreshAmpCheck)    robotSensors.amper = getAmp();
      if (refreshUltrasonicCheck) robotSensors.ultrasonic = getUltrasonic();
      semPost(sem_id, 0);

      semWait(sem_id, 1);
      memcpy(&lastRobotAcculators, &robotAcculators, sizeof(RobotAcculators));
      semPost(sem_id, 1);

      pocetBMP180++;
      pocetHMC5883L++;      
      pocetPosition++;
      pocetMotors++;
      pocetBattery++;
      pocetMPU6050++;
      pocetLeds++;
      pocetAmp++;
      pocetUltrasonic++;
      break;
  }
}
