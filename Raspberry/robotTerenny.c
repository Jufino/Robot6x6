#include "robotTerenny.h"

int file;
unsigned char lastAddr = 0x00;
int serversock_snimace, serversock_camera;
int clientsock_snimace, clientsock_camera;
int portHandle;
int sem_id;
RobotAcculators robotAcculators;
RobotAcculators lastRobotAcculators;
RobotSensors robotSensors;
timer_t casovac;

void initRobot() {
  if ((file = open(PORT_I2C, O_RDWR)) < 0) {
    perror("Problem s otvorenim portu.\n");
    exit(1);
  }
  if (test() == 1) {
    portHandle = SerialOpen(PORT_GPS, B9600);

    gpio_open(17, 0);
    gpio_open(27, 0);
    gpio_open(22, 0);

    sem_id = semCreate(getpid() + 1, 2); //vytvor semafor
    semInit(sem_id, 0, 1);
    semInit(sem_id, 1, 1);

    resetDistance(1);
    resetDistance(2);
    resetDistance(3);
    resetDistance(4);
    resetDistance(5);
    resetDistance(6);

    gpio_open(26, 1);
    setMotorPowerSupply(true);//potom zmenit na true

    setLed(1, 'R');
    setLed(2, 'R');
    setLed(3, 'R');
    usleep(500000);
    setLed(1, 'G');
    setLed(2, 'G');
    setLed(3, 'G');
    usleep(500000);
    setLed(1, 'O');
    setLed(2, 'O');
    setLed(3, 'O');
    usleep(500000);
    setLed(1, 'V');
    setLed(2, 'V');
    setLed(3, 'V');
    
    if (Wifi_snimace == 1) {
      struct sockaddr_in server;
      if ((serversock_snimace = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket() failed");
        exit(1);
      }
      memset(&server, 0, sizeof(server));
      server.sin_family = AF_INET;
      server.sin_port = htons(PORT_snimace);
      server.sin_addr.s_addr = INADDR_ANY;
      if (bind(serversock_snimace, (struct sockaddr *)&server, sizeof(server)) == -1) {
        perror("bind() failed");
        exit(1);
      }
      if (listen(serversock_snimace, 10) == -1) {
        perror("listen() failed.");
        exit(1);
      }
      printf("Cakanie spojenia pre snimace na porte: %d\n", PORT_snimace);
      if ((clientsock_snimace = accept(serversock_snimace, NULL, NULL)) == -1) {
        perror("accept() failed");
        exit(1);
      }
      printf("Spojenie na porte %d ok.\n", PORT_snimace);
    }
    if (Wifi_camera == 1) {
      struct sockaddr_in server1;
      if ((serversock_camera = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket() failed");
        exit(1);
      }
      memset(&server1, 0, sizeof(server1));
      server1.sin_family = AF_INET;
      server1.sin_port = htons(PORT_camera);
      server1.sin_addr.s_addr = INADDR_ANY;
      if (bind(serversock_camera, (struct sockaddr *)&server1, sizeof(server1)) == -1) {
        perror("bind() failed");
        exit(1);
      }
      if (listen(serversock_camera, 10) == -1) {
        perror("listen() failed.");
        exit(1);
      }
      printf("Cakanie spojenia pre kameru na porte: %d\n", PORT_camera);
      if ((clientsock_camera = accept(serversock_camera, NULL, NULL)) == -1) {
        perror("accept() failed");
        exit(1);
      }
      printf("Spojenie na porte %d ok.\n", PORT_camera);
    }
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

    MPU6050WakeUp();
    setMPU6050Sensitivity(1,1);
    setMPU6050DLPF(6,6);
    MPU6050CalibrateOffset(20);
    MPU6050DisableAsMaster();
    MPU6050WakeUp();
    HMC5883LSampleRateAndModeSetting(4,5,3);
    HMC5883LGainSetting(1);
    HMC5883LReadModeSetting(0,0);
  }
  else {
    for(int i=0;i<10;i++){
	setLed(1, 'R');
        setLed(2, 'R');
        setLed(3, 'R');
        usleep(500000);
	setLed(1, 'V');
    	setLed(2, 'V');
    	setLed(3, 'V');
    	usleep(500000);
    }
    exit(0);
  }
}

void closeRobot() {
  SerialClose(portHandle);
  semRem(sem_id);
  gpio_close(17);
  gpio_close(27);
  gpio_close(22);
  gpio_close(26);
  if (Wifi_camera == 1) {
    close(serversock_camera);
    close(clientsock_camera);
  }
  if (Wifi_snimace == 1) {
    close(serversock_snimace);
    close(clientsock_snimace);
  }
  struct itimerspec cas;
  cas.it_value.tv_sec = 0;
  cas.it_value.tv_nsec = 0;
  cas.it_interval.tv_sec = 0;
  cas.it_interval.tv_nsec = 0;
  timer_settime(casovac, CLOCK_REALTIME, &cas, NULL);
  setMotor(1, 0, 255, false);
  setMotor(2, 0, 255, false);
  setMotor(3, 0, 255, false);
  setMotor(4, 0, 255, false);
  setMotor(5, 0, 255, false);
  setMotor(6, 0, 255, false);

}

void setDevice(unsigned char addr) {
  if (addr != lastAddr) {
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
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
  if (write(file, data, 2) != 2)
	printf("addr:%i, write register %i,val %i\n", (int)addr, (int)reg, (int)value);
}

unsigned int readRegister16(unsigned char addr, unsigned char reg) {
  char data[3];
  char errorTimeout = 0;
  data[0] = reg;
  setDevice(addr);
  while (write(file, data, 1) != 1){ 
	printf("addr:%i, write register %i, errorTimeout:%i\n", (int)addr, (int)reg,(int)errorTimeout);
	if(errorTimeout++ >= i2cWriteTimeout) break;
  }
  if(errorTimeout < i2cWriteTimeout){
  	if (read(file, data, 2) != 2){   
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
  while (write(file, wdata, 1) != 1){  
	printf("addr:%i, write register %i, errorTimeout:%i\n", (int)addr, (int)reg,(int)errorTimeout);
  	if(errorTimeout++ >= i2cWriteTimeout) break;
  }
  signed char data[3];
  if(errorTimeout < i2cWriteTimeout){
  	if (read(file, data, 2) != 2)   printf("addr:%i, read register %i\n", (int)addr, (int)reg);
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
  while (write(file, data, 1) != 1){  
	printf("addr:%i, write register %i, errorTimeout:%i\n", (int)addr, (int)reg,(int)errorTimeout);
	if(errorTimeout++ >= i2cWriteTimeout) break;
  }
  if(errorTimeout < i2cWriteTimeout){
  	if (read(file, data, 1) != 1)   printf("addr:%i, read register %i\n", (int)addr, (int)reg);
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

unsigned char getButton(char pos) {
  switch (pos) {
    case 1: return !gpio_read(27); break;
    case 2: return !gpio_read(17); break;
    case 3: return !gpio_read(22); break;
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
  //  return robotSensors;
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
  if (300 == readRegister16(ORANZOVYADDR, 127)) return 1;
  else                                          return 0;
}

int test() {
  if (testZlty() && testOranzovy() && testModry())  return 1;
  else                                              return 0;
}

int getDistanceRaw(int pos) {
  switch (pos) {
    case 1: return readRegister16s(MODRYADDR, 3); break;
    case 2: return readRegister16s(ORANZOVYADDR, 3); break;
    case 3: return readRegister16s(ORANZOVYADDR, 4); break;
    case 4: return readRegister16s(MODRYADDR, 4); break;
    case 5: return readRegister16s(ZLTYADDR, 4); break;
    case 6: return readRegister16s(ZLTYADDR, 3); break;
    default: return 0;
  }
}

float prepocetTikovOtackomeraDoVzdialenosti(int pocetTikov){
  return ((float)pocetTikov * ((M_PI * OtackomerPriemer) / OtackomerConstant));
}

int getDistance(int pos) {
  return (int)prepocetTikovOtackomeraDoVzdialenosti(getDistanceRaw(pos));
}

int getDeltaDistanceRaw(int pos) {
  switch (pos) {
    case 1: return readRegister16s(MODRYADDR, 7); break;
    case 2: return readRegister16s(ORANZOVYADDR, 7); break;
    case 3: return readRegister16s(ORANZOVYADDR, 8); break;
    case 4: return readRegister16s(MODRYADDR, 8); break;
    case 5: return readRegister16s(ZLTYADDR, 6); break;
    case 6: return readRegister16s(ZLTYADDR, 5); break;
    default: return 0;
  }
}

float getDeltaDistance(int pos) {
  return prepocetTikovOtackomeraDoVzdialenosti(getDeltaDistanceRaw(pos));
}

void resetDistance(int pos) {
  switch (pos) {
    case 1: writeRegister(MODRYADDR, 100, 0);  break;
    case 2: writeRegister(ORANZOVYADDR, 100, 0);  break;
    case 3: writeRegister(ORANZOVYADDR, 99, 0);   break;
    case 4: writeRegister(MODRYADDR, 99, 0);   break;
    case 5: writeRegister(ZLTYADDR, 99, 0);   break;
    case 6: writeRegister(ZLTYADDR, 100, 0);  break;
  }
}

void setServo(int angle) {
  if (angle + 91 < 1) writeRegister(ORANZOVYADDR, 84, 1);
  else if (angle + 91 > 181) writeRegister(ORANZOVYADDR, 84, 181);
  else writeRegister(0x0A, 84, angle + 91);
}

unsigned int getUltrasonicRaw() {
  return readRegister16(ORANZOVYADDR, 6);
}

float getUltrasonic() {
  return (float)readRegister16(ORANZOVYADDR, 6) / UltrasonicConstant;
}

int getVoltageRaw() {
  return readRegister16(MODRYADDR, 5);
}

float getVoltage() {
  return (float)getVoltageRaw() * (maxVoltADC / rozlisenieADC) * ((R1 + R2) / R2);
}

float getVoltagePercent() {
  return (float)getVoltage() * (100 / (maxNapetie - minNapetie)) - (100 / (maxNapetie - minNapetie)) * minNapetie;
}

int getAmpRaw() {
  return readRegister16(MODRYADDR, 6);
}

float getAmpVolt() {
  return (float)getAmpRaw() * (maxVoltADC / rozlisenieADC);
}

float getAmp() {
  return ((float)getAmpVolt() - maxVoltADC / 2) / rozliseniePrud;
}

void setLed(int pos, char color) {
  if (pos == 3) {
    if (color == 'G') {
      writeRegister(MODRYADDR, 96, 0);
      writeRegister(MODRYADDR, 97, 1);
    }
    else if (color == 'R') {
      writeRegister(MODRYADDR, 97, 0);
      writeRegister(MODRYADDR, 96, 1);
    }
    else if (color == 'O') {
      writeRegister(MODRYADDR, 97, 1);
      writeRegister(MODRYADDR, 96, 1);
    }
    else {
      writeRegister(MODRYADDR, 97, 0);
      writeRegister(MODRYADDR, 96, 0);
    }
  }
  else if (pos == 1) {
    if (color == 'G') {
      writeRegister(ZLTYADDR, 96, 0);
      writeRegister(ZLTYADDR, 97, 1);
    }
    else if (color == 'R') {
      writeRegister(ZLTYADDR, 97, 0);
      writeRegister(ZLTYADDR, 96, 1);
    }
    else if (color == 'O') {
      writeRegister(ZLTYADDR, 97, 1);
      writeRegister(ZLTYADDR, 96, 1);
    }
    else {
      writeRegister(ZLTYADDR, 97, 0);
      writeRegister(ZLTYADDR, 96, 0);
    }
  }
  else {
    if (color == 'G') {
      writeRegister(ORANZOVYADDR, 96, 0);
      writeRegister(ORANZOVYADDR, 97, 1);
    }
    else if (color == 'R') {
      writeRegister(ORANZOVYADDR, 97, 0);
      writeRegister(ORANZOVYADDR, 96, 1);
    }
    else if (color == 'O') {
      writeRegister(ORANZOVYADDR, 97, 1);
      writeRegister(ORANZOVYADDR, 96, 1);
    }
    else {
      writeRegister(ORANZOVYADDR, 97, 0);
      writeRegister(ORANZOVYADDR, 96, 0);
    }
  }
}

void setMotorPowerSupply(bool state) {
  if (state == false)   gpio_write(26, 1);
  else                 gpio_write(26, 0);
}

void setMotor(int pos, signed char dir, unsigned char speed, bool onReg) {
  if (onReg == true) {
    if (dir >= 0) {
      switch (pos) {
        case 1: writeRegister(MODRYADDR, 94, speed); break;
        case 2: writeRegister(ORANZOVYADDR, 89, speed); break;
        case 3: writeRegister(ORANZOVYADDR, 94, speed); break;
        case 4: writeRegister(MODRYADDR, 89, speed); break;
        case 5: writeRegister(ZLTYADDR, 89, speed); break;
        case 6: writeRegister(ZLTYADDR, 94, speed); break;
      }
    }
    else {
      switch (pos) {
        case 1: writeRegister(MODRYADDR, 93, speed); break;
        case 2: writeRegister(ORANZOVYADDR, 88, speed); break;
        case 3: writeRegister(ORANZOVYADDR, 93, speed); break;
        case 4: writeRegister(MODRYADDR, 88, speed); break;
        case 5: writeRegister(ZLTYADDR, 88, speed); break;
        case 6: writeRegister(ZLTYADDR, 93, speed); break;
      }
    }
  }
  else {
    if (dir > 0) {
      switch (pos) {
        case 1: writeRegister(MODRYADDR, 92, speed); break;
        case 2: writeRegister(ORANZOVYADDR, 87, speed); break;
        case 3: writeRegister(ORANZOVYADDR, 92, speed); break;
        case 4: writeRegister(MODRYADDR, 87, speed); break;
        case 5: writeRegister(ZLTYADDR, 87, speed); break;
        case 6: writeRegister(ZLTYADDR, 92, speed); break;
      }
    }
    else if (dir == 0) {
      switch (pos) {
        case 1: writeRegister(MODRYADDR, 91, speed); break;
        case 2: writeRegister(ORANZOVYADDR, 86, speed); break;
        case 3: writeRegister(ORANZOVYADDR, 91, speed); break;
        case 4: writeRegister(MODRYADDR, 86, speed); break;
        case 5: writeRegister(ZLTYADDR, 86, speed); break;
        case 6: writeRegister(ZLTYADDR, 91, speed); break;
      }
    }
    else {
      switch (pos) {
        case 1: writeRegister(MODRYADDR, 90, speed); break;
        case 2: writeRegister(ORANZOVYADDR, 85, speed); break;
        case 3: writeRegister(ORANZOVYADDR, 90, speed); break;
        case 4: writeRegister(MODRYADDR, 85, speed); break;
        case 5: writeRegister(ZLTYADDR, 85, speed); break;
        case 6: writeRegister(ZLTYADDR, 90, speed); break;
      }
    }
  }
}

void setMove(char direction,unsigned char speed,bool onReg){
  semWait(sem_id, 1);
  if(direction == 'F'){
    robotAcculators.motors.motor1.direction = 1;
    robotAcculators.motors.motor2.direction = 1;
    robotAcculators.motors.motor3.direction = 1;
    robotAcculators.motors.motor4.direction = 1;
    robotAcculators.motors.motor5.direction = 1;
    robotAcculators.motors.motor6.direction = 1;
    robotAcculators.motors.motor1.speed = speed;
    robotAcculators.motors.motor2.speed = speed;
    robotAcculators.motors.motor3.speed = speed;
    robotAcculators.motors.motor4.speed = speed;
    robotAcculators.motors.motor5.speed = speed;
    robotAcculators.motors.motor6.speed = speed;
    robotAcculators.motors.motor1.onRegulator = onReg;
    robotAcculators.motors.motor2.onRegulator = onReg;
    robotAcculators.motors.motor3.onRegulator = onReg;
    robotAcculators.motors.motor4.onRegulator = onReg;
    robotAcculators.motors.motor5.onRegulator = onReg;
    robotAcculators.motors.motor6.onRegulator = onReg;
  }
  else if(direction == 'B'){
    robotAcculators.motors.motor1.direction = -1;
    robotAcculators.motors.motor2.direction = -1;
    robotAcculators.motors.motor3.direction = -1;
    robotAcculators.motors.motor4.direction = -1;
    robotAcculators.motors.motor5.direction = -1;
    robotAcculators.motors.motor6.direction = -1;
    robotAcculators.motors.motor1.speed = speed;
    robotAcculators.motors.motor2.speed = speed;
    robotAcculators.motors.motor3.speed = speed;
    robotAcculators.motors.motor4.speed = speed;
    robotAcculators.motors.motor5.speed = speed;
    robotAcculators.motors.motor6.speed = speed;
    robotAcculators.motors.motor1.onRegulator = onReg;
    robotAcculators.motors.motor2.onRegulator = onReg;
    robotAcculators.motors.motor3.onRegulator = onReg;
    robotAcculators.motors.motor4.onRegulator = onReg;
    robotAcculators.motors.motor5.onRegulator = onReg;
    robotAcculators.motors.motor6.onRegulator = onReg;
  }
  else if(direction == 'R'){
    robotAcculators.motors.motor1.direction = -1;
    robotAcculators.motors.motor2.direction = -1;
    robotAcculators.motors.motor3.direction = -1;
    robotAcculators.motors.motor4.direction = 1;
    robotAcculators.motors.motor5.direction = 1;
    robotAcculators.motors.motor6.direction = 1;
    robotAcculators.motors.motor1.speed = speed;
    robotAcculators.motors.motor2.speed = speed;
    robotAcculators.motors.motor3.speed = speed;
    robotAcculators.motors.motor4.speed = speed;
    robotAcculators.motors.motor5.speed = speed;
    robotAcculators.motors.motor6.speed = speed;
    robotAcculators.motors.motor1.onRegulator = onReg;
    robotAcculators.motors.motor2.onRegulator = onReg;
    robotAcculators.motors.motor3.onRegulator = onReg;
    robotAcculators.motors.motor4.onRegulator = onReg;
    robotAcculators.motors.motor5.onRegulator = onReg;
    robotAcculators.motors.motor6.onRegulator = onReg;
  }
  else if(direction == 'L'){
    robotAcculators.motors.motor1.direction = 1;
    robotAcculators.motors.motor2.direction = 1;
    robotAcculators.motors.motor3.direction = 1;
    robotAcculators.motors.motor4.direction = -1;
    robotAcculators.motors.motor5.direction = -1;
    robotAcculators.motors.motor6.direction = -1;
    robotAcculators.motors.motor1.speed = speed;
    robotAcculators.motors.motor2.speed = speed;
    robotAcculators.motors.motor3.speed = speed;
    robotAcculators.motors.motor4.speed = speed;
    robotAcculators.motors.motor5.speed = speed;
    robotAcculators.motors.motor6.speed = speed;
    robotAcculators.motors.motor1.onRegulator = onReg;
    robotAcculators.motors.motor2.onRegulator = onReg;
    robotAcculators.motors.motor3.onRegulator = onReg;
    robotAcculators.motors.motor4.onRegulator = onReg;
    robotAcculators.motors.motor5.onRegulator = onReg;
    robotAcculators.motors.motor6.onRegulator = onReg;
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

void HMC5883LSampleRateAndModeSetting(int sample, int datarate,int mode){
  switch(mode){
    case 0: mode = 0; break;   // normal measurement configuration(Default)
    case 1: mode = 1; break;   // positive bias configuration (more resistive)
    case 2: mode = 2; break;   // negative bias configuration (more resistive)
    default:mode = 0; break;   // normal measurement configuration(Default)
  }
  switch(datarate){
    case 0: datarate = 0;  break; //0.75 Hz
    case 1: datarate = 1;  break; //1.5 Hz
    case 2: datarate = 2;  break; //3 Hz
    case 3: datarate = 3;  break; //7.5 Hz
    case 4: datarate = 4;  break; //15 Hz (default)
    case 5: datarate = 5;  break; //30 Hz
    case 6: datarate = 6;  break; //75 Hz
    default: datarate = 4; break; //15 Hz
  }
  switch(sample){
    case 0: sample = 0; break; //1
    case 1: sample = 1; break; //2
    case 2: sample = 2; break; //4
    case 3: sample = 3; break; //8
  }
  writeRegister(HMC5883LADDR,0x00,((sample<<5)|(datarate<<2)|mode));
}

float mgPerDigit = 0.92f;
void HMC5883LGainSetting(int gain){
  switch(gain){
    case 0: gain = 0; 
            mgPerDigit = 0.73f;
            break; // 0.73
    case 1: gain = 1; 
            mgPerDigit = 0.92f;
            break; // 0.92(default)
    case 2: gain = 2; 
            mgPerDigit = 1.22f;
            break; // 1.22
    case 3: gain = 3; 
            mgPerDigit = 1.52f;
            break; // 1.52
    case 4: gain = 4; 
            mgPerDigit = 2.27f;
            break; // 2.27
    case 5: gain = 5; 
            mgPerDigit = 2.56f;
            break; // 2.56
    case 6: gain = 6; 
            mgPerDigit = 3.03f;
            break; // 3.03
    case 7: gain = 7; 
            mgPerDigit = 4.35f;
            break; // 4.35
    default:gain = 1; 
            mgPerDigit = 0.92f;
            break; // 0.92(default)
  }
  writeRegister(HMC5883LADDR,0x01,gain<<5);
}

void HMC5883LReadModeSetting(int highI2cSpeed,int mode){
  switch(mode){
    case 0: mode = 0; break;   // continous-measurement mode
    case 1: mode = 1; break;   // single measurement mode
    case 2: mode = 2; break;   // idle mode
    default:mode = 0; break;   // continous-measurement mode
  }
  writeRegister(HMC5883LADDR,0x02,((highI2cSpeed<<7)|mode));
}

float minX = defaultMinXHMC5883L;
float minY = defaultMinYHMC5883L;
float maxX = defaultMaxXHMC5883L;
float maxY = defaultMaxYHMC5883L;
float offX = -326.0f;
float offY = -174.0f;

HMC5883L_struct getHMC5883LRaw() {
  HMC5883L_struct HMC5883L;
  HMC5883L.X = (float)readRegister16s(HMC5883LADDR, 0x03);
  HMC5883L.Y = (float)readRegister16s(HMC5883LADDR, 0x07);
  HMC5883L.Z = (float)readRegister16s(HMC5883LADDR, 0x05);
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

//  float declinationAngle = (degHMC5883L + (minHMC5883L / 60.0)) / (180 / M_PI);   //posun podla zemepisnej sirky a dlzky
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
  writeRegister(MPU6050ADDR, 0x6B, readRegister8(MPU6050ADDR,0x6B)&(!(1<<6)));
}

void MPU6050DisableAsMaster(){
   writeRegister(MPU6050ADDR, 0x6A, readRegister8(MPU6050ADDR,0x6A)&(!(1<<5|1<<4)));
   writeRegister(MPU6050ADDR, 0x37, readRegister8(MPU6050ADDR,0x37)|(1<<1));
}

MPU6050_struct getMPU6050Raw() {
  MPU6050_struct MPU6050;
  MPU6050.AccX = (float)readRegister16s(MPU6050ADDR, 0x3B);
  MPU6050.AccY = (float)readRegister16s(MPU6050ADDR, 0x3D);
  MPU6050.AccZ = (float)readRegister16s(MPU6050ADDR, 0x3F);
  MPU6050.Temp = (float)readRegister16s(MPU6050ADDR, 0x41);
  MPU6050.GyX  = (float)readRegister16s(MPU6050ADDR, 0x43);
  MPU6050.GyY  = (float)readRegister16s(MPU6050ADDR, 0x45);
  MPU6050.GyZ  = (float)readRegister16s(MPU6050ADDR, 0x47);
  return MPU6050;
}

MPU6050_struct getMPU6050() {
  MPU6050_struct MPU6050;
  MPU6050.AccX = (float)readRegister16s(MPU6050ADDR, 0x3B) / AccScale - AccX_offset;
  MPU6050.AccY = (float)readRegister16s(MPU6050ADDR, 0x3D) / AccScale - AccY_offset;
  MPU6050.AccZ = (float)readRegister16s(MPU6050ADDR, 0x3F) / AccScale - AccZ_offset;
  MPU6050.Temp = (float)readRegister16s(MPU6050ADDR, 0x41) / 340 + 36.53;
  MPU6050.GyX  = (float)readRegister16s(MPU6050ADDR, 0x43) / GyScale - GyX_offset;
  MPU6050.GyY  = (float)readRegister16s(MPU6050ADDR, 0x45) / GyScale - GyY_offset;
  MPU6050.GyZ  = (float)readRegister16s(MPU6050ADDR, 0x47) / GyScale - GyZ_offset;
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
  writeRegister(MPU6050ADDR, 0x1B, (gy_sens << 3) | 0xE0);
  writeRegister(MPU6050ADDR, 0x1C, (acc_sens << 3) | 0xE0);
}

void setMPU6050DLPF(unsigned char acc_dlpf, unsigned char gy_dlpf) {
  writeRegister(MPU6050ADDR, 0x1A, acc_dlpf | (5 << 3));
  writeRegister(MPU6050ADDR, 0x1A, acc_dlpf | (6 << 3));
  writeRegister(MPU6050ADDR, 0x1A, acc_dlpf | (7 << 3));
  writeRegister(MPU6050ADDR, 0x1A, gy_dlpf | (2 << 3));
  writeRegister(MPU6050ADDR, 0x1A, gy_dlpf | (3 << 3));
  writeRegister(MPU6050ADDR, 0x1A, gy_dlpf | (4 << 3));
}

float getSpeedFromDistance(float distance,float dt) {
  return distance / dt;
}

//http://rossum.sourceforge.net/papers/DiffSteer/DiffSteer.html
//http://users.isr.ist.utl.pt/~mir/cadeiras/robmovel/Kinematics.pdf
void calcRobotPosition(float deltaSpeedL,float deltaSpeedR,float dt) {
  float v = (deltaSpeedL+deltaSpeedR)/2;
  robotSensors.robotPosition.angleRad+= ((deltaSpeedR-deltaSpeedL) / vzdialenostKolies)*dt;
  robotSensors.robotPosition.angleDeg = robotSensors.robotPosition.angleRad*(360/(2*M_PI));
  robotSensors.robotPosition.x +=  v*cos(robotSensors.robotPosition.angleRad)*dt;
  robotSensors.robotPosition.y +=  v*sin(robotSensors.robotPosition.angleRad)*dt;
}

bool compareMotors(MotorAcculator_struct motor, MotorAcculator_struct lastMotor) {
  return motor.direction != lastMotor.direction || motor.speed != lastMotor.speed || motor.onRegulator != lastMotor.onRegulator;
}

int pocetPosition = 100;
int pocetMotors = 100;
int pocetBattery = 100;
int pocetMPU6050 = 100;
int pocetBMP180 = 100;
int pocetHMC5883L = 100;
int pocetLeds = 100;
int pocetAmp = 100;
int pocetUltrasonic = 100;

void syncModules(int signal , siginfo_t * siginfo, void * ptr) {
  switch (signal)
  {
    case SIGUSR1:
    
      bool refreshPositionCheck = (refreshPosition / refreshModule) <= pocetPosition;
      bool refreshMotorsCheck = (refreshMotors / refreshModule) <= pocetMotors;
      bool refreshBatteryCheck = (refreshBattery / refreshModule) <= pocetBattery;
      bool refreshMPU6050Check = (refreshMPU6050 / refreshModule) <= pocetMPU6050;
      bool refreshBMP180Check = (refreshBMP180 / refreshModule) <= pocetBMP180;
      bool refreshHMC5883LCheck = (refreshHMC5883L / refreshModule) <= pocetHMC5883L;
      bool refreshLedsCheck = (refreshLeds / refreshModule) <= pocetLeds;
      bool refreshAmpCheck = (refreshAmp / refreshModule) <= pocetAmp;
      bool refreshUltrasonicCheck = (refreshUltrasonic / refreshModule) <= pocetUltrasonic;
      
      float pomocnaZmenaOtackomera = 0;
      float deltaDistanceL;
      float deltaDistanceR;
      float deltaDistance1;
      float deltaDistance2;
      float deltaDistance3;
      float deltaDistance4;
      float deltaDistance5;
      float deltaDistance6;
      
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
	      pomocnaZmenaOtackomera = getDeltaDistanceRaw(1);
        if(pomocnaZmenaOtackomera > maxZmenaOtackomera) pomocnaZmenaOtackomera = maxZmenaOtackomera;
        else if(pomocnaZmenaOtackomera < -maxZmenaOtackomera) pomocnaZmenaOtackomera = -maxZmenaOtackomera;

        robotSensors.motors.motor1.distanceRaw+= pomocnaZmenaOtackomera;
        deltaDistance1 = prepocetTikovOtackomeraDoVzdialenosti(pomocnaZmenaOtackomera);
 	      robotSensors.motors.motor1.distance +=  deltaDistance1;
        robotSensors.motors.motor1.speed = getSpeedFromDistance(deltaDistance1,(float)refreshPosition / 1000);
        
        semPost(sem_id, 0);
        pocetPosition = 0;
      }
      if (refreshMotorsCheck || compareMotors(robotAcculators.motors.motor1, lastRobotAcculators.motors.motor1)) {
        semWait(sem_id, 1);
        setMotor(1, robotAcculators.motors.motor1.direction, robotAcculators.motors.motor1.speed, robotAcculators.motors.motor1.onRegulator);
        semPost(sem_id, 1);
        pocetMotors = 0;
      }
      if (refreshPositionCheck) {
        semWait(sem_id, 0);
	      pomocnaZmenaOtackomera = getDeltaDistanceRaw(4);
	      if(pomocnaZmenaOtackomera > maxZmenaOtackomera) pomocnaZmenaOtackomera = maxZmenaOtackomera;
	      else if(pomocnaZmenaOtackomera < -maxZmenaOtackomera) pomocnaZmenaOtackomera = -maxZmenaOtackomera;
        
        robotSensors.motors.motor4.distanceRaw+= pomocnaZmenaOtackomera;
        deltaDistance4 = prepocetTikovOtackomeraDoVzdialenosti(pomocnaZmenaOtackomera);
 	      robotSensors.motors.motor4.distance +=  deltaDistance4;
        robotSensors.motors.motor4.speed = getSpeedFromDistance(deltaDistance4,(float)refreshPosition / 1000);
        
        semPost(sem_id, 0);
        pocetPosition = 0;
      }
      if (refreshMotorsCheck || compareMotors(robotAcculators.motors.motor4, lastRobotAcculators.motors.motor4)) {
        semWait(sem_id, 1);
        setMotor(4, robotAcculators.motors.motor4.direction, robotAcculators.motors.motor4.speed, robotAcculators.motors.motor4.onRegulator);
        semPost(sem_id, 1);
        pocetMotors = 0;
      }
      if (refreshPositionCheck) {
        semWait(sem_id, 0);
	      pomocnaZmenaOtackomera = getDeltaDistanceRaw(2);
        if(pomocnaZmenaOtackomera > maxZmenaOtackomera) pomocnaZmenaOtackomera = maxZmenaOtackomera;
        else if(pomocnaZmenaOtackomera < -maxZmenaOtackomera) pomocnaZmenaOtackomera = -maxZmenaOtackomera;

        robotSensors.motors.motor2.distanceRaw+= pomocnaZmenaOtackomera;
        deltaDistance2 = prepocetTikovOtackomeraDoVzdialenosti(pomocnaZmenaOtackomera);
 	      robotSensors.motors.motor2.distance +=  deltaDistance2;
        robotSensors.motors.motor2.speed = getSpeedFromDistance(deltaDistance2,(float)refreshPosition / 1000);
        
        semPost(sem_id, 0);
        pocetPosition = 0;
      }
      if (refreshMotorsCheck || compareMotors(robotAcculators.motors.motor2, lastRobotAcculators.motors.motor2)) {
        semWait(sem_id, 1);
        setMotor(2, robotAcculators.motors.motor2.direction, robotAcculators.motors.motor2.speed, robotAcculators.motors.motor2.onRegulator);
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
	      pomocnaZmenaOtackomera = getDeltaDistanceRaw(3);
        if(pomocnaZmenaOtackomera > maxZmenaOtackomera) pomocnaZmenaOtackomera = maxZmenaOtackomera;
        else if(pomocnaZmenaOtackomera < -maxZmenaOtackomera) pomocnaZmenaOtackomera = -maxZmenaOtackomera;

        robotSensors.motors.motor3.distanceRaw+= pomocnaZmenaOtackomera;
        deltaDistance3 = prepocetTikovOtackomeraDoVzdialenosti(pomocnaZmenaOtackomera);
 	      robotSensors.motors.motor3.distance +=  deltaDistance3;
        robotSensors.motors.motor3.speed = getSpeedFromDistance(deltaDistance3,(float)refreshPosition / 1000);

        semPost(sem_id, 0);
        pocetPosition = 0;
      }
      if (refreshMotorsCheck || compareMotors(robotAcculators.motors.motor3, lastRobotAcculators.motors.motor3)) {
        semWait(sem_id, 1);
        setMotor(3, robotAcculators.motors.motor3.direction, robotAcculators.motors.motor3.speed, robotAcculators.motors.motor3.onRegulator);
        semPost(sem_id, 1);
        pocetMotors = 0;
      }
      if (refreshPositionCheck) {
        semWait(sem_id, 0);
	       pomocnaZmenaOtackomera = getDeltaDistanceRaw(5);
        if(pomocnaZmenaOtackomera > maxZmenaOtackomera) pomocnaZmenaOtackomera = maxZmenaOtackomera;
        else if(pomocnaZmenaOtackomera < -maxZmenaOtackomera) pomocnaZmenaOtackomera = -maxZmenaOtackomera;   

        robotSensors.motors.motor5.distanceRaw+= pomocnaZmenaOtackomera;
        deltaDistance5 = prepocetTikovOtackomeraDoVzdialenosti(pomocnaZmenaOtackomera);
 	      robotSensors.motors.motor5.distance +=  deltaDistance5;
        robotSensors.motors.motor5.speed = getSpeedFromDistance(deltaDistance5,(float)refreshPosition / 1000);

	      semPost(sem_id, 0);
        pocetPosition = 0;
      }
      if (refreshMotorsCheck || compareMotors(robotAcculators.motors.motor5, lastRobotAcculators.motors.motor5)) {
        semWait(sem_id, 1);
        setMotor(5, robotAcculators.motors.motor5.direction, robotAcculators.motors.motor5.speed, robotAcculators.motors.motor5.onRegulator);
        semPost(sem_id, 1);
        pocetMotors = 0;
      }
      if (refreshPositionCheck) {
        semWait(sem_id, 0);
	       pomocnaZmenaOtackomera = getDeltaDistanceRaw(6);
        if(pomocnaZmenaOtackomera > maxZmenaOtackomera) pomocnaZmenaOtackomera = maxZmenaOtackomera;
        else if(pomocnaZmenaOtackomera < -maxZmenaOtackomera) pomocnaZmenaOtackomera = -maxZmenaOtackomera;

        robotSensors.motors.motor6.distanceRaw+= pomocnaZmenaOtackomera;
        deltaDistance6 = prepocetTikovOtackomeraDoVzdialenosti(pomocnaZmenaOtackomera);
 	      robotSensors.motors.motor6.distance +=  deltaDistance6;
        robotSensors.motors.motor6.speed = getSpeedFromDistance(deltaDistance6,(float)refreshPosition / 1000);

        semPost(sem_id, 0);
        pocetPosition = 0;
      }
      if (refreshMotorsCheck || compareMotors(robotAcculators.motors.motor6, lastRobotAcculators.motors.motor6)) {
        semWait(sem_id, 1);
        setMotor(6, robotAcculators.motors.motor6.direction, robotAcculators.motors.motor6.speed, robotAcculators.motors.motor6.onRegulator);
        semPost(sem_id, 1);
        pocetMotors = 0;
      }
      if (refreshLedsCheck) {
        semWait(sem_id, 1);
        setLed(1, robotAcculators.leds.Led1);
        setLed(2, robotAcculators.leds.Led2);
        if (!BatteryLed3Indicate) setLed(3, robotAcculators.leds.Led3);
        semPost(sem_id, 1);
      }
      if(refreshPositionCheck){
      	float diff1 = abs(deltaDistance4-deltaDistance5);
      	float diff2 = abs(deltaDistance4-deltaDistance6);
      	float diff3 = abs(deltaDistance5-deltaDistance6);
      	if(diff1 < diff2 && diff1 < diff3)
      		deltaDistanceL = (deltaDistance4+deltaDistance5)/2;
      	else if(diff2 < diff1 && diff2 < diff3)
      		deltaDistanceL = (deltaDistance4+deltaDistance6)/2;
      	else
      		deltaDistanceL = (deltaDistance5+deltaDistance6)/2;
      	robotSensors.robotPosition.distanceL+= deltaDistanceL;
        robotSensors.robotPosition.speedL = getSpeedFromDistance(deltaDistanceL,(float)refreshPosition / 1000);
                                                             
      	float diff4 = abs(deltaDistance1-deltaDistance2);
        float diff5 = abs(deltaDistance2-deltaDistance3);
        float diff6 = abs(deltaDistance3-deltaDistance1);
        if(diff4 < diff5 && diff4 < diff6)
          deltaDistanceR = (deltaDistance1+deltaDistance2)/2;
        else if(diff5 < diff4 && diff5 < diff6)
          deltaDistanceR = (deltaDistance2+deltaDistance3)/2;
        else
          deltaDistanceR = (deltaDistance3+deltaDistance1)/2;
      	robotSensors.robotPosition.distanceR+= deltaDistanceR;      
        robotSensors.robotPosition.speedR = getSpeedFromDistance(deltaDistanceR,(float)refreshPosition / 1000);
        
        calcRobotPosition(robotSensors.robotPosition.speedL,robotSensors.robotPosition.speedR,(float)refreshPosition / 1000);
        
      }
      semWait(sem_id, 0);

      robotSensors.buttons.button1 = getButton(1);
      robotSensors.buttons.button2 = getButton(2);
      robotSensors.buttons.button3 = getButton(3);
      if (refreshBatteryCheck) {
        robotSensors.voltage = getVoltage();
        robotSensors.voltagePercent = getVoltagePercent();
        if (BatteryLed3Indicate == 1) {
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
