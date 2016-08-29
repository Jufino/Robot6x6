#include "robotTerenny.h"

int i2cHandle;
unsigned char lastAddr = 0x00;
int sensorsServersock, cameraServersock;
int sensorsClientsock, cameraClientsock;
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
Callibrate callibrate;

timer_t casovac;
float mgPerDigit = 0.92f;    
float rangePerDigit = 0.0f;
float dpsPerDigit = 0.0f;

Axis_struct gyAxisOffset;
Axis_struct gyAxisthreshold;

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

Axis_struct minAxis;
Axis_struct maxAxis;

void initRobot() {
  initI2C();

  setMPU6050ScaleSetting(MPU6050_SCALE_2000DPS);
  setMPU6050RangeSetting(MPU6050_RANGE_2G);
  setMPU6050I2CMasterModeEnabledSetting(false);
  setMPU6050I2CBypassEnabledSetting(true) ;
  setMPU6050SleepEnabledSetting(false);

  callibrateMPU6050Gyroscope(50);
 
  if(!HMC5883LTestConnection()){
    errorLedBlink();
    closeI2C();
    printf("HMC5883L problem s konektivitou");
    exit(0);
  }

  setHMC5883LMeasurementSetting(HMC5883L_NORMAL);
  setHMC5883LSampleSetting(HMC5883L_SAMPLES_8);
  setHMC5883LRateSetting(HMC5883L_DATARATE_30HZ);
  setHMC5883LRangeSetting(HMC5883L_RANGE_1_3GA);
  setHMC5883LReadModeSetting(HMC5883L_CONTINOUS);
  setHMC5883LHighI2CSpeedSetting(false);

  if(!blueTestConnection()){
    errorLedBlink();
    closeI2C();
    printf("Modry problem s konektivitou");
    exit(0);
  } 
  if(!yellowTestConnection()){
    errorLedBlink();
    closeI2C();
    printf("Zlty problem s konektivitou");
    exit(0);
  } 
  if(!orangeTestConnection()){
    errorLedBlink();
    closeI2C();
    printf("Oranzovy problem s konektivitou");
    exit(0);
  } 

  portHandle = SerialOpen(PORT_GPS, B9600);

  initButton(POSITION_DOWN);
  initButton(POSITION_MIDDLE);
  initButton(POSITION_UP);
  initMotorPowerSupply();

  stopAllMotors();

  sem_id = semCreate(getpid(), 8);
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
    if ((sensorsServersock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
      perror("socket() failed");
      exit(1);
    }
    memset(&server, 0, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_port = htons(SENSORS_PORT);
    server.sin_addr.s_addr = INADDR_ANY;
    if (bind(sensorsServersock, (struct sockaddr *)&server, sizeof(server)) == -1) {
      perror("bind() failed");
      exit(1);
    }
    if (listen(sensorsServersock, 10) == -1) {
      perror("listen() failed.");
      exit(1);
    }
    printf("Cakanie spojenia pre snimace na porte: %d\n", SENSORS_PORT);
    if ((sensorsClientsock = accept(sensorsServersock, NULL, NULL)) == -1) {
      perror("accept() failed");
      exit(1);
    }
    printf("Spojenie na porte %d ok.\n", SENSORS_PORT);
  }
  if (CAMERA_WIFI == 1) {
    struct sockaddr_in server1;
    if ((cameraServersock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
      perror("socket() failed");
      exit(1);
    }
    memset(&server1, 0, sizeof(server1));
    server1.sin_family = AF_INET;
    server1.sin_port = htons(CAMERA_PORT);
    server1.sin_addr.s_addr = INADDR_ANY;
    if (bind(cameraServersock, (struct sockaddr *)&server1, sizeof(server1)) == -1) {
      perror("bind() failed");
      exit(1);
    }
    if (listen(cameraServersock, 10) == -1) {
      perror("listen() failed.");
      exit(1);
    }
    printf("Cakanie spojenia pre kameru na porte: %d\n", CAMERA_PORT);
    if ((cameraClientsock = accept(cameraServersock, NULL, NULL)) == -1) {
      perror("accept() failed");
      exit(1);
    }
    printf("Spojenie na porte %d ok.\n", CAMERA_PORT);
  }
    
  resetDistanceAll();
    
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

void closeRobot() {
  onAllThreads = false;

  stopRefresh();
  closeI2C();
  SerialClose(portHandle);
  semRem(sem_id);
  
  closeButton(POSITION_DOWN);
  closeButton(POSITION_MIDDLE);
  closeButton(POSITION_UP);
  closeMotorPowerSupply();
  
  if (CAMERA_WIFI == 1) {
    close(cameraServersock);
    close(cameraClientsock);
  }
  if (SENSORS_WIFI == 1) {
    close(sensorsServersock);
    close(sensorsClientsock);
  }
  
  stopAllMotors();
}

void initI2C(){
  if ((i2cHandle = open(PORT_I2C, O_RDWR)) < 0) {
    perror("Problem s otvorenim portu.\n");
    exit(1);
  }
}

void closeI2C(){
  close(i2cHandle);
}

void errorLedBlink(){
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
  	return (data[0] << 8) | (data[1]&0xFF);
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
  	return (data[0] << 8) | (data[1]&0xFF);
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

bool readRegisterBit(unsigned char addr, unsigned char reg, char pin){
  return (readRegister8(addr,reg) & (1<<pin)) && (1<<pin);
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
  return 0;
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
  return 0;
}

void wifiCamera(){    //premenovat
        char recvdata[30];
        int bytes = recv(getCameraClientsock(), recvdata, 4, 0);
        if (bytes == 0){
		//closeRobot();
		//exit(0);
        }
        if (strcmp(recvdata, "img\n") == 0){ //&& imgSendL.empty() != true){
          semWait(sem_id, 1);
	  //sendMatImage(robotSensors.camera.imgLeft,80);
          semPost(sem_id, 1);
        }
}

void initButton(position3_t pos){
  switch (pos) {
    case POSITION_DOWN:   gpio_open(27, 0); break;
    case POSITION_MIDDLE: gpio_open(17, 0); break;
    case POSITION_UP:     gpio_open(22, 0); break;
  }   
}

void closeButton(position3_t pos){
  switch (pos) {
    case POSITION_DOWN:   gpio_close(27); break;
    case POSITION_MIDDLE: gpio_close(17); break;
    case POSITION_UP:     gpio_close(22); break;
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

Callibrate getCallibrate(){
  Callibrate temp;
  semWait(sem_id, 0);
  memcpy(&temp, &callibrate, sizeof(Callibrate));
  semPost(sem_id, 0);
  return temp;}

int getCameraClientsock() {
  return cameraClientsock;
}

int getSensorsClientsock() {
  return sensorsClientsock;
}

bool blueTestConnection() {
  return (300 == readRegister16(BLUE_ADDRESS, 127));
}

bool yellowTestConnection() {
  return (300 == readRegister16(YELLOW_ADDRESS, 127));
}

bool orangeTestConnection() {
  return (300 == readRegister16(ORANGE_ADDRESS, 127));
}

int getDistanceRaw(position6_t pos) {
  switch (pos) {
    case POSITION_DOWN_RIGHT:   return readRegister16s(BLUE_ADDRESS, 3); break;
    case POSITION_MIDDLE_RIGHT: return readRegister16s(ORANGE_ADDRESS, 3); break;
    case POSITION_UP_RIGHT:     return readRegister16s(ORANGE_ADDRESS, 4); break;
    case POSITION_UP_LEFT:      return readRegister16s(BLUE_ADDRESS, 4); break;
    case POSITION_MIDDLE_LEFT:  return readRegister16s(YELLOW_ADDRESS, 4); break;
    case POSITION_DOWN_LEFT:    return readRegister16s(YELLOW_ADDRESS, 3); break;
    default: return 0;
  }
}

int getDeltaDistanceRaw(position6_t pos) {
  switch (pos) {
    case POSITION_DOWN_RIGHT:   return readRegister16s(BLUE_ADDRESS, 7); break;
    case POSITION_MIDDLE_RIGHT: return readRegister16s(ORANGE_ADDRESS, 7); break;
    case POSITION_UP_RIGHT:     return readRegister16s(ORANGE_ADDRESS, 8); break;
    case POSITION_UP_LEFT:      return readRegister16s(BLUE_ADDRESS, 8); break;
    case POSITION_MIDDLE_LEFT:  return readRegister16s(YELLOW_ADDRESS, 6); break;
    case POSITION_DOWN_LEFT:    return readRegister16s(YELLOW_ADDRESS, 5); break;
    default: return 0;
  }
}

void resetDistance(position6_t pos) {
  switch (pos) {
    case POSITION_DOWN_RIGHT: writeRegister(BLUE_ADDRESS, 100, 0);  break;
    case POSITION_MIDDLE_RIGHT: writeRegister(ORANGE_ADDRESS, 100, 0);  break;
    case POSITION_UP_RIGHT: writeRegister(ORANGE_ADDRESS, 99, 0);   break;
    case POSITION_UP_LEFT: writeRegister(BLUE_ADDRESS, 99, 0);   break;
    case POSITION_MIDDLE_LEFT: writeRegister(YELLOW_ADDRESS, 99, 0);   break;
    case POSITION_DOWN_LEFT: writeRegister(YELLOW_ADDRESS, 100, 0);  break;
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

float prepocetTikovOtackomeraDoVzdialenosti(int pocetTikov){
  return ((float)pocetTikov * ((M_PI * DIAMERER_WHEEL) / CONST_ENCODER));
}

float getDistance(position6_t pos) {
  return prepocetTikovOtackomeraDoVzdialenosti(getDistanceRaw(pos));
}

float getDeltaDistance(position6_t pos) {
  return prepocetTikovOtackomeraDoVzdialenosti(getDeltaDistanceRaw(pos));
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

float calcVoltagePercent(float volt) {
  return (float)volt * (100 / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE)) - (100 / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE)) * MIN_BATTERY_VOLTAGE;
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
      writeRegister(YELLOW_ADDRESS, 96, 0);
      writeRegister(YELLOW_ADDRESS, 97, 1);
    }
    else if (color == COLOR_RED) {
      writeRegister(YELLOW_ADDRESS, 97, 0);
      writeRegister(YELLOW_ADDRESS, 96, 1);
    }
    else if (color == COLOR_ORANGE) {
      writeRegister(YELLOW_ADDRESS, 97, 1);
      writeRegister(YELLOW_ADDRESS, 96, 1);
    }
    else if (color == COLOR_OFF) {
      writeRegister(YELLOW_ADDRESS, 97, 0);
      writeRegister(YELLOW_ADDRESS, 96, 0);
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
        case POSITION_MIDDLE_LEFT: writeRegister(YELLOW_ADDRESS, 89, speed); break;
        case POSITION_DOWN_LEFT: writeRegister(YELLOW_ADDRESS, 94, speed); break;
      }
    }
    else if(rotate == ROTATE_ANTICLOCKWISE) {
      switch (pos) {
        case POSITION_DOWN_RIGHT: writeRegister(BLUE_ADDRESS, 93, speed); break;
        case POSITION_MIDDLE_RIGHT: writeRegister(ORANGE_ADDRESS, 88, speed); break;
        case POSITION_UP_RIGHT: writeRegister(ORANGE_ADDRESS, 93, speed); break;
        case POSITION_UP_LEFT: writeRegister(BLUE_ADDRESS, 88, speed); break;
        case POSITION_MIDDLE_LEFT: writeRegister(YELLOW_ADDRESS, 88, speed); break;
        case POSITION_DOWN_LEFT: writeRegister(YELLOW_ADDRESS, 93, speed); break;
      }
    }   
    else if(rotate == ROTATE_STOP){
      switch (pos) {
        case POSITION_DOWN_RIGHT: writeRegister(BLUE_ADDRESS, 93, 0); break;
        case POSITION_MIDDLE_RIGHT: writeRegister(ORANGE_ADDRESS, 88, 0); break;
        case POSITION_UP_RIGHT: writeRegister(ORANGE_ADDRESS, 93, 0); break;
        case POSITION_UP_LEFT: writeRegister(BLUE_ADDRESS, 88, 0); break;
        case POSITION_MIDDLE_LEFT: writeRegister(YELLOW_ADDRESS, 88, 0); break;
        case POSITION_DOWN_LEFT: writeRegister(YELLOW_ADDRESS, 93, 0); break;
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
        case POSITION_MIDDLE_LEFT: writeRegister(YELLOW_ADDRESS, 87, speed); break;
        case POSITION_DOWN_LEFT: writeRegister(YELLOW_ADDRESS, 92, speed); break;
      }
    }
    else if (ROTATE_STOP == 0) {
      switch (pos) {
        case POSITION_DOWN_RIGHT: writeRegister(BLUE_ADDRESS, 91, speed); break;
        case POSITION_MIDDLE_RIGHT: writeRegister(ORANGE_ADDRESS, 86, speed); break;
        case POSITION_UP_RIGHT: writeRegister(ORANGE_ADDRESS, 91, speed); break;
        case POSITION_UP_LEFT: writeRegister(BLUE_ADDRESS, 86, speed); break;
        case POSITION_MIDDLE_LEFT: writeRegister(YELLOW_ADDRESS, 86, speed); break;
        case POSITION_DOWN_LEFT: writeRegister(YELLOW_ADDRESS, 91, speed); break;
      }
    }
    else if(rotate == ROTATE_ANTICLOCKWISE) {
      switch (pos) {
        case POSITION_DOWN_RIGHT: writeRegister(BLUE_ADDRESS, 90, speed); break;
        case POSITION_MIDDLE_RIGHT: writeRegister(ORANGE_ADDRESS, 85, speed); break;
        case POSITION_UP_RIGHT: writeRegister(ORANGE_ADDRESS, 90, speed); break;
        case POSITION_UP_LEFT: writeRegister(BLUE_ADDRESS, 85, speed); break;
        case POSITION_MIDDLE_LEFT: writeRegister(YELLOW_ADDRESS, 85, speed); break;
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
      robotAcculators.motors.motorUpLeft.direction = rotate;
      robotAcculators.motors.motorMiddleLeft.direction = rotate;
      robotAcculators.motors.motorDownLeft.direction = rotate;
      robotAcculators.motors.motorUpLeft.speed = speed;
      robotAcculators.motors.motorMiddleLeft.speed = speed;
      robotAcculators.motors.motorDownLeft.speed = speed;
      robotAcculators.motors.motorUpLeft.onRegulator = onReg;
      robotAcculators.motors.motorMiddleLeft.onRegulator = onReg;
      robotAcculators.motors.motorDownLeft.onRegulator = onReg;    
      break;    
    case SIDE_RIGHT:
      robotAcculators.motors.motorDownRight.direction = rotate;
      robotAcculators.motors.motorMiddleRight.direction = rotate;
      robotAcculators.motors.motorUpRight.direction = rotate;
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

bool HMC5883LTestConnection(){
   char identA = readRegister8(HMC5883L_ADDRESS,HMC5883L_REG_IDENT_A);  
   char identB = readRegister8(HMC5883L_ADDRESS,HMC5883L_REG_IDENT_B);  
   char identC = readRegister8(HMC5883L_ADDRESS,HMC5883L_REG_IDENT_C);  
   return identA=='H' && identB=='4' && identC=='3';
}

hmc5883l_measurement_t getHMC5883LMeasurementSetting(){  
  return (hmc5883l_measurement_t)((readRegister8(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_A) & 0b00000011));
}

void setHMC5883LMeasurementSetting(hmc5883l_measurement_t measurement){
  char oldRegister = readRegister8(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_A) & 0b00000011;
  writeRegister(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_A,oldRegister|(measurement));
}

hmc5883l_dataRate_t getHMC5883LSampleSetting(){  
  return (hmc5883l_dataRate_t)((readRegister8(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_A) & 0b11100011)>>2);
}

void setHMC5883LSampleSetting(hmc5883l_samples_t sample){
  char oldRegister = readRegister8(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_A) & 0b00011111;
  writeRegister(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_A,oldRegister|(sample<<5)|0b10000000);
}

hmc5883l_dataRate_t getHMC5883LRateSetting(){  
  return (hmc5883l_dataRate_t)((readRegister8(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_A) & 0b11100011)>>2);
}

void setHMC5883LRateSetting(hmc5883l_dataRate_t datarate){
  char oldRegister = readRegister8(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_A) & 0b11100011;
  writeRegister(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_A,oldRegister|(datarate<<2));
}

hmc5883l_range_t getHMC5883LRangeSetting(){  
  return (hmc5883l_range_t)((readRegister8(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_B) & 0b11100000)>>5);
}

void setHMC5883LRangeSetting(hmc5883l_range_t range){
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
  writeRegister(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_B,range<<5);
}

hmc5883l_mode_t getHMC5883LReadModeSetting(){  
  return (hmc5883l_mode_t)((readRegister8(HMC5883L_ADDRESS,HMC5883L_REG_MODE) & 0b00011000)>>3);
}

void setHMC5883LReadModeSetting(hmc5883l_mode_t mode){
  char oldRegister = readRegister8(HMC5883L_ADDRESS,HMC5883L_REG_MODE) & 0b10000000;
  writeRegister(HMC5883L_ADDRESS,HMC5883L_REG_MODE,oldRegister | mode);
}

bool getHMC5883LHighI2CSpeedSetting(bool status){
  return readRegisterBit(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_B,7);
}

void setHMC5883LHighI2CSpeedSetting(bool status){
  char oldRegister = readRegister8(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_B) & 0b00000011;
  if(status)
    oldRegister |= 0x80;
  writeRegister(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_B,oldRegister);
}

HMC5883L_struct getHMC5883LRaw() {
  HMC5883L_struct HMC5883L;
  
  HMC5883L.compassAxis.x = (float)readRegister16s(HMC5883L_ADDRESS, HMC5883L_REG_OUT_X_M);
  HMC5883L.compassAxis.y = (float)readRegister16s(HMC5883L_ADDRESS, HMC5883L_REG_OUT_Y_M);
  HMC5883L.compassAxis.z = (float)readRegister16s(HMC5883L_ADDRESS, HMC5883L_REG_OUT_Z_M);
  
  return HMC5883L;
}

HMC5883L_struct getHMC5883LNorm() {
  HMC5883L_struct HMC5883L = getHMC5883LRaw();
  
  if(CALLIBRATE_DATA_CALCULATE){
    if (HMC5883L.compassAxis.x < minAxis.x) minAxis.x = HMC5883L.compassAxis.x;
    if (HMC5883L.compassAxis.x > maxAxis.x) maxAxis.x = HMC5883L.compassAxis.x;
    if (HMC5883L.compassAxis.y < maxAxis.y) maxAxis.y = HMC5883L.compassAxis.y;
    if (HMC5883L.compassAxis.y > maxAxis.y) maxAxis.y = HMC5883L.compassAxis.y;
    if (HMC5883L.compassAxis.z < maxAxis.z) maxAxis.z = HMC5883L.compassAxis.z;
    if (HMC5883L.compassAxis.z > maxAxis.z) maxAxis.z = HMC5883L.compassAxis.z;
    callibrate.HMC5883LOffsetAxis.x = (maxAxis.x + minAxis.x)/2;
    callibrate.HMC5883LOffsetAxis.y = (maxAxis.y + minAxis.y)/2; 
    callibrate.HMC5883LOffsetAxis.z = (maxAxis.z + minAxis.z)/2; 
  }
  
  HMC5883L.compassAxis.x = (HMC5883L.compassAxis.x-HMC5883L_OFFSET_X) * mgPerDigit;
  HMC5883L.compassAxis.y = (HMC5883L.compassAxis.y-HMC5883L_OFFSET_Y) * mgPerDigit;
  HMC5883L.compassAxis.z = (HMC5883L.compassAxis.z-HMC5883L_OFFSET_Z) * mgPerDigit;

  float declinationAngle = (HMC5883L_DEGREE + (HMC5883L_MINUTES / 60.0)) / (180 / M_PI);   //posun magnetickeho pola podla zemepisnej sirky a dlzky
  HMC5883L.angle.radian = atan2(HMC5883L.compassAxis.y,HMC5883L.compassAxis.x) + declinationAngle; 
  
  if(HMC5883L.angle.radian < 0){
    HMC5883L.angle.radian+= 2*M_PI;
  }
  else if(HMC5883L.angle.radian > 2*M_PI){
    HMC5883L.angle.radian-=2*M_PI;
  }

  HMC5883L.angle.degree = HMC5883L.angle.radian*(180/M_PI);
  return HMC5883L;
}


void callibrateMPU6050Gyroscope(int samples){
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;

    float sigmaX = 0;
    float sigmaY = 0;
    float sigmaZ = 0;

    for (unsigned char i = 0; i < samples; ++i)
    {
	MPU6050_struct MPU6050 = getMPU6050Raw();
	sumX += MPU6050.gyAxis.x;
	sumY += MPU6050.gyAxis.y;
	sumZ += MPU6050.gyAxis.z;

	sigmaX += MPU6050.gyAxis.x * MPU6050.gyAxis.x;
	sigmaY += MPU6050.gyAxis.y * MPU6050.gyAxis.y;
	sigmaZ += MPU6050.gyAxis.z * MPU6050.gyAxis.z;	

	usleep(5);
    }

    gyAxisOffset.x = sumX / samples;
    gyAxisOffset.y = sumY / samples;
    gyAxisOffset.z = sumZ / samples;

    gyAxisthreshold.x = sqrt((sigmaX / samples) - (gyAxisOffset.x * gyAxisOffset.x));
    gyAxisthreshold.y = sqrt((sigmaY / samples) - (gyAxisOffset.y * gyAxisOffset.y));
    gyAxisthreshold.z = sqrt((sigmaZ / samples) - (gyAxisOffset.z * gyAxisOffset.z));
}

void setMPU6050ScaleSetting(mpu6050_dps_t scale)
{
  switch (scale){
    case MPU6050_SCALE_250DPS:
      dpsPerDigit = .007633f;
      break;
    case MPU6050_SCALE_500DPS:
	    dpsPerDigit = .015267f;
	    break;
    case MPU6050_SCALE_1000DPS:
      dpsPerDigit = .030487f;
      break;
    case MPU6050_SCALE_2000DPS:
      dpsPerDigit = .060975f;
      break;
	  default:
      break;
  }
    
  char oldRegister = readRegister8(MPU6050_ADDRESS,MPU6050_REG_GYRO_CONFIG) & 0b11100111;
  oldRegister |= (scale << 3);
  writeRegister(MPU6050_ADDRESS,MPU6050_REG_GYRO_CONFIG, oldRegister);  
}

mpu6050_dps_t getMPU6050ScaleSetting(){  
  return (mpu6050_dps_t)(readRegister8(MPU6050_ADDRESS,MPU6050_REG_GYRO_CONFIG) & 0b00011000);
}

void setMPU6050RangeSetting(mpu6050_range_t range){
  switch (range){
    case MPU6050_RANGE_2G:
	   rangePerDigit = .000061f;
	   break;
    case MPU6050_RANGE_4G:
	   rangePerDigit = .000122f;
	   break;
    case MPU6050_RANGE_8G:
	   rangePerDigit = .000244f;
	   break;
    case MPU6050_RANGE_16G:
	   rangePerDigit = .0004882f;
	   break;
    default:
    break;
  }

  char oldRegister = readRegister8(MPU6050_ADDRESS,MPU6050_REG_ACCEL_CONFIG) & 0b11100111;
  oldRegister |= (range << 3);
  writeRegister(MPU6050_ADDRESS,MPU6050_REG_ACCEL_CONFIG, oldRegister);  
}

mpu6050_range_t getMPU6050RangeSetting(void)
{
  return (mpu6050_range_t)((readRegister8(MPU6050_ADDRESS,MPU6050_REG_ACCEL_CONFIG) & 0b00011000)>>3);
}

void setMPU6050DHPFModeSetting(mpu6050_dhpf_t dhpf)
{
  char oldRegister = readRegister8(MPU6050_ADDRESS,MPU6050_REG_ACCEL_CONFIG) & 0b11111000;
  oldRegister |= dhpf;
  writeRegister(MPU6050_ADDRESS,MPU6050_REG_ACCEL_CONFIG, oldRegister);  
}

void setMPU6050DLPFModeSetting(mpu6050_dlpf_t dlpf)
{
  char oldRegister = readRegister8(MPU6050_ADDRESS,MPU6050_REG_CONFIG) & 0b11111000;
  oldRegister |= dlpf;
  writeRegister(MPU6050_ADDRESS,MPU6050_REG_CONFIG, oldRegister);  
}

void setMPU6050ClockSourceSetting(mpu6050_clockSource_t source)
{
  char oldRegister = readRegister8(MPU6050_ADDRESS,MPU6050_REG_PWR_MGMT_1) & 0b11111000;
  oldRegister |= source;
  writeRegister(MPU6050_ADDRESS,MPU6050_REG_PWR_MGMT_1, oldRegister);
}

mpu6050_clockSource_t getMPU6050ClockSourceSetting(void)
{
  return (mpu6050_clockSource_t)(readRegister8(MPU6050_ADDRESS,MPU6050_REG_PWR_MGMT_1) & 0b00000111);
}

bool getMPU6050SleepEnabledSetting(void)
{
  return readRegisterBit(MPU6050_ADDRESS,MPU6050_REG_PWR_MGMT_1, 6);
}

void setMPU6050SleepEnabledSetting(bool state)
{
  char oldRegister = readRegister8(MPU6050_ADDRESS,MPU6050_REG_PWR_MGMT_1) & !(1<<6);
  if(state)
    oldRegister |= (1<<6);
  writeRegister(MPU6050_ADDRESS,MPU6050_REG_PWR_MGMT_1, oldRegister);
}

bool getMPU6050I2CMasterModeEnabledSetting(void)
{                            
  return readRegisterBit(MPU6050_ADDRESS,MPU6050_REG_USER_CTRL, 5);
}

void setMPU6050I2CMasterModeEnabledSetting(bool state)
{
  char oldRegister = readRegister8(MPU6050_ADDRESS,MPU6050_REG_USER_CTRL) & !(1<<5);
  if(state)
    oldRegister |= (1<<5);
  writeRegister(MPU6050_ADDRESS,MPU6050_REG_USER_CTRL, oldRegister);
}

void setMPU6050I2CBypassEnabledSetting(bool state)
{
  char oldRegister = readRegister8(MPU6050_ADDRESS,MPU6050_REG_INT_PIN_CFG) & !(1<<1);
  if(state)
    oldRegister |= (1<<1);
  writeRegister(MPU6050_ADDRESS,MPU6050_REG_INT_PIN_CFG, oldRegister);
}

bool getMPU6050I2CBypassEnabledSetting(void)
{
  return readRegisterBit(MPU6050_ADDRESS,MPU6050_REG_INT_PIN_CFG,1);
}

MPU6050_struct getMPU6050Raw() {
  MPU6050_struct MPU6050;
  
  MPU6050.accAxis.x = (float)readRegister16s(MPU6050_ADDRESS, MPU6050_REG_ACCEL_XOUT_H);
  MPU6050.accAxis.y = (float)readRegister16s(MPU6050_ADDRESS, MPU6050_REG_ACCEL_YOUT_H);
  MPU6050.accAxis.z = (float)readRegister16s(MPU6050_ADDRESS, MPU6050_REG_ACCEL_ZOUT_H);
  MPU6050.gyAxis.x  = (float)readRegister16s(MPU6050_ADDRESS, MPU6050_REG_GYRO_XOUT_H);
  MPU6050.gyAxis.y  = (float)readRegister16s(MPU6050_ADDRESS, MPU6050_REG_GYRO_YOUT_H);
  MPU6050.gyAxis.z  = (float)readRegister16s(MPU6050_ADDRESS, MPU6050_REG_GYRO_ZOUT_H);
  MPU6050.Temp =      (float)readRegister16s(MPU6050_ADDRESS, MPU6050_REG_TEMP_OUT_H);
  
  return MPU6050;
}

MPU6050_struct getMPU6050Norm(){
  MPU6050_struct MPU6050 = getMPU6050Raw();
  MPU6050.accAxis.x = MPU6050.accAxis.x * rangePerDigit * 9.80665f; 
  MPU6050.accAxis.y = MPU6050.accAxis.y * rangePerDigit * 9.80665f; 
  MPU6050.accAxis.z = MPU6050.accAxis.z * rangePerDigit * 9.80665f; 
  
  MPU6050.gyAxis.x = (MPU6050.gyAxis.x-gyAxisOffset.x) * dpsPerDigit; 
  MPU6050.gyAxis.y = (MPU6050.gyAxis.y-gyAxisOffset.y) * dpsPerDigit; 
  MPU6050.gyAxis.z = (MPU6050.gyAxis.z-gyAxisOffset.z) * dpsPerDigit;
  
  if(abs(MPU6050.gyAxis.x) < gyAxisthreshold.x) MPU6050.gyAxis.x = 0; 
  if(abs(MPU6050.gyAxis.y) < gyAxisthreshold.y) MPU6050.gyAxis.y = 0; 
  if(abs(MPU6050.gyAxis.z) < gyAxisthreshold.z) MPU6050.gyAxis.z = 0; 
  
  MPU6050.Temp = MPU6050.Temp/340 + 36.53;
  return MPU6050;   
}

float dist(float a, float b) {
  return sqrt(a * a + b * b);
}

float getSpeedFromDistance(float distance,float dt) {
  return distance / dt;
}

//http://rossum.sourceforge.net/papers/DiffSteer/DiffSteer.html
//http://users.isr.ist.utl.pt/~mir/cadeiras/robmovel/Kinematics.pdf
void calcRobotPosition(float deltaSpeedL,float deltaSpeedR,float dt) {
  float v = (deltaSpeedL+deltaSpeedR)/2;
  robotSensors.robotPosition.angle.radian+= ((deltaSpeedR-deltaSpeedL) / LENGTH_BETWEEN_LEFT_AND_RIGHT_WHEEL)*dt;
  robotSensors.robotPosition.angle.degree = robotSensors.robotPosition.angle.radian*(360/(2*M_PI));
  robotSensors.robotPosition.x +=  v*cos(robotSensors.robotPosition.angle.radian)*dt;
  robotSensors.robotPosition.y +=  v*sin(robotSensors.robotPosition.angle.radian)*dt;
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
      bool refreshLedsCheck = (REFRESH_LEDS / REFRESH_MODULE) <= pocetLeds;
      bool refreshAmpCheck = (REFRESH_AMP / REFRESH_MODULE) <= pocetAmp;
      bool refreshUltrasonicCheck = (REFRESH_ULTRASONIC / REFRESH_MODULE) <= pocetUltrasonic;   
      bool refreshCameraCheck = (REFRESH_CAMERA / REFRESH_MODULE) <= pocetCamera;
      
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
      
      if(refreshCameraCheck){
        if(NUMBER_OF_CAMERA == 1 || NUMBER_OF_CAMERA == 2){
          semWait(sem_id,3);
          imageChooseMainL = imageChooseL;
          semPost(sem_id,3);
    		  if(imageChooseMainL !=0){
            if(imageChooseMainL == 1){        
    				  semWait(sem_id,2);
                semWait(sem_id, 0);
      				  //cvarrToMat(img1L).copyTo(robotSensors.camera.imgLeft);
                semPost(sem_id, 0);
    				  semPost(sem_id,2);
    			  }
            else if(imageChooseMainL == 2){   
    				  semWait(sem_id,4);
              semWait(sem_id, 0);
    				  //cvarrToMat(img2L).copyTo(robotSensors.camera.imgLeft);
              semPost(sem_id, 0);
    				  semPost(sem_id,4);
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
      				  //cvarrToMat(img1R).copyTo(robotSensors.camera.imgRight);
                semPost(sem_id, 0);
    				  semPost(sem_id,5);
    			  }
            else if(imageChooseMainR == 2){   
    				  semWait(sem_id,7);
                semWait(sem_id, 0);
      				  //cvarrToMat(img2R).copyTo(robotSensors.camera.imgRight);
                semPost(sem_id, 0);
    				  semPost(sem_id,7);
    			  }
          }
        }
        pocetCamera = 0;
      }
        
      if (refreshBMP180Check) {
        semWait(sem_id, 0);
        //sem pojde BMO180
        semPost(sem_id, 0);
        pocetBMP180 = 0;
      }

      if (refreshHMC5883LCheck) {
        semWait(sem_id, 0);
	robotSensors.HMC5883L = getHMC5883LNorm(); 
        semPost(sem_id, 0);
        pocetHMC5883L = 0;
      }
      
      if (refreshMPU6050Check) {
        semWait(sem_id, 0);
        robotSensors.MPU6050 = getMPU6050Norm();
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
        robotSensors.motors.motorDownRight.speed = getSpeedFromDistance(deltaDistanceDownRight,(float)REFRESH_POSITION / 1000);
        
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
        robotSensors.motors.motorUpLeft.speed = getSpeedFromDistance(deltaDistanceUpLeft,(float)REFRESH_POSITION / 1000);
        
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
        robotSensors.motors.motorMiddleRight.speed = getSpeedFromDistance(deltaDistanceMiddleRight,(float)REFRESH_POSITION / 1000);
        
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
        robotSensors.motors.motorUpRight.speed = getSpeedFromDistance(deltaDistanceUpRight,(float)REFRESH_POSITION / 1000);

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
        robotSensors.motors.motorMiddleLeft.speed = getSpeedFromDistance(deltaDistanceMiddleLeft,(float)REFRESH_POSITION / 1000);

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
        robotSensors.motors.motorDownLeft.speed = getSpeedFromDistance(deltaDistanceDownLeft,(float)REFRESH_POSITION / 1000);

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
        robotSensors.robotPosition.speedL = getSpeedFromDistance(deltaDistanceL,(float)REFRESH_POSITION / 1000);
                                                             
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
        robotSensors.robotPosition.speedR = getSpeedFromDistance(deltaDistanceR,(float)REFRESH_POSITION / 1000);
        
        calcRobotPosition(robotSensors.robotPosition.speedL,robotSensors.robotPosition.speedR,(float)REFRESH_POSITION / 1000);
        
      }
      semWait(sem_id, 0);

      robotSensors.buttons.buttonDown =   getButton(POSITION_DOWN);
      robotSensors.buttons.buttonMiddle = getButton(POSITION_MIDDLE);
      robotSensors.buttons.buttonUp =     getButton(POSITION_UP);
      
      if (refreshBatteryCheck) {
        robotSensors.voltage.volts = getVoltage();
        robotSensors.voltage.capacityPercent = calcVoltagePercent(robotSensors.voltage.volts);
        if (BATTERY_LED_INDICATING == 1) {
          if (robotSensors.voltage.capacityPercent > 60)           setLed(POSITION_DOWN, COLOR_GREEN);
          else if (robotSensors.voltage.capacityPercent > 20)      setLed(POSITION_DOWN, COLOR_ORANGE);
          else                                            	setLed(POSITION_DOWN, COLOR_RED);
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
      pocetCamera++;
      break;
  }
}
