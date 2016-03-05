#include "robotTerenny.h"

int file;
unsigned char lastAddr = 0x00;
int serversock_snimace,serversock_camera;
int clientsock_snimace,clientsock_camera;
int portHandle;
int sem_id;
RobotVariables robotVariables;
RobotVariables lastRobotVariables;
timer_t casovac;

RobotVariables getRobotVariables(){
	RobotVariables temp;
	semWait(sem_id,0);
	memcpy(&robotVariables, &temp, sizeof(robotVariables));
	semPost(sem_id,0);
	return temp;
}
void setRobotVariables(RobotVariables temp){
	semWait(sem_id,0);
	memcpy(&temp, &robotVariables, sizeof(temp));
	semPost(sem_id,0);
}
int getSocketCamera(){
	return clientsock_camera;
}
int getSocketSnimace(){
	return clientsock_snimace;
}
int testModry(){
	if(300 == readRegister16(MODRYADDR,127)) return 1;
	else					 return 0;
}
int testZlty(){
	if(300 == readRegister16(ZLTYADDR,127)) return 1;
	else					return 0;
}
int testOranzovy(){
	if(300 == readRegister16(ORANZOVYADDR,127)) return 1;
	else					    return 0;
}
int test(){
	if(testZlty() && testOranzovy() && testModry()) return 1;
	else						return 0;
}
void initRobot(){
    if ((file = open(PORT_I2C, O_RDWR)) < 0) {
        perror("Problem s otvorenim portu.\n");
        exit(1);
    }
    portHandle = SerialOpen(PORT_GPS,B9600);
    gpio_open(17,0);
    gpio_open(27,0);
    gpio_open(22,0);
    if(test() == 1){
	sem_id = semCreate(getpid()+1,1);   //vytvor semafor
        semInit(sem_id,0,1);

        gpio_open(26,1);
        setMotorPowerSupply(false);//potom zmenit na true

	setLed(1,'R');
        setLed(2,'R');
        setLed(3,'R');
        usleep(500000);
        setLed(1,'G');
        setLed(2,'G');
        setLed(3,'G');
        usleep(500000);
        setLed(1,'O');
        setLed(2,'O');
        setLed(3,'O');
        usleep(500000);
        setLed(1,'V');
        setLed(2,'V');
        setLed(3,'V');
	struct sigevent CasovacSignalEvent;
    CasovacSignalEvent.sigev_notify=SIGEV_SIGNAL;
    CasovacSignalEvent.sigev_signo=SIGUSR1;

    timer_create(CLOCK_REALTIME, &CasovacSignalEvent, &casovac);
    struct itimerspec cas;
    cas.it_value.tv_sec=0;
    cas.it_value.tv_nsec=refreshModule*1000*1000;
    cas.it_interval.tv_sec=0;
    cas.it_interval.tv_nsec=refreshModule*1000*1000;
    timer_settime(casovac,CLOCK_REALTIME,&cas,NULL);
    sigset_t signalSet;
    struct sigaction CasovacSignalAction;
    sigemptyset(&signalSet);
    CasovacSignalAction.sa_sigaction=syncModules;
    CasovacSignalAction.sa_flags=SA_SIGINFO;
    CasovacSignalAction.sa_mask=signalSet;
    sigaction(CasovacSignalEvent.sigev_signo,&CasovacSignalAction,NULL);
    
    if(Wifi_snimace == 1){
        struct sockaddr_in server;
        if ((serversock_snimace = socket(AF_INET, SOCK_STREAM, 0)) == -1){
            perror("socket() failed");
            exit(1);
        }
        memset(&server, 0, sizeof(server));
        server.sin_family = AF_INET;
        server.sin_port = htons(PORT_snimace);
        server.sin_addr.s_addr = INADDR_ANY;
        if (bind(serversock_snimace, (struct sockaddr *)&server, sizeof(server)) == -1){
            perror("bind() failed");
            exit(1);
        }
        if (listen(serversock_snimace, 10) == -1){
            perror("listen() failed.");
            exit(1);
        }
        printf("Cakanie spojenia pre snimace na porte: %d\n", PORT_snimace);
        if ((clientsock_snimace = accept(serversock_snimace, NULL, NULL)) == -1){
            perror("accept() failed");
            exit(1);
        }
        printf("Spojenie na porte %d ok.\n",PORT_snimace);
    }
    if(Wifi_camera == 1){
        struct sockaddr_in server1;
        if ((serversock_camera = socket(AF_INET, SOCK_STREAM, 0)) == -1){
            perror("socket() failed");
            exit(1);
        }
        memset(&server1, 0, sizeof(server1));
        server1.sin_family = AF_INET;
        server1.sin_port = htons(PORT_camera);
        server1.sin_addr.s_addr = INADDR_ANY;
        if (bind(serversock_camera, (struct sockaddr *)&server1, sizeof(server1)) == -1){
            perror("bind() failed");
            exit(1);
        }
        if (listen(serversock_camera, 10) == -1){
            perror("listen() failed.");
            exit(1);
        }
        printf("Cakanie spojenia pre kameru na porte: %d\n", PORT_camera);
        if ((clientsock_camera = accept(serversock_camera, NULL, NULL)) == -1){
            perror("accept() failed");
            exit(1);
        }
        printf("Spojenie na porte %d ok.\n",PORT_camera);
    }
	}
	else{
		exit(0);
	}
}
void closeRobot(){
    SerialClose(portHandle);
    semRem(sem_id);
    gpio_close(17);
    gpio_close(27);
    gpio_close(22);
    gpio_close(26);
    if(Wifi_camera == 1){
    	close(serversock_camera);
    	close(clientsock_camera);
    }
    if(Wifi_snimace == 1){
    	close(serversock_snimace);
    	close(clientsock_snimace);
    }
    struct itimerspec cas;
    cas.it_value.tv_sec=0;
    cas.it_value.tv_nsec=0;
    cas.it_interval.tv_sec=0;
    cas.it_interval.tv_nsec=0;
    timer_settime(casovac,CLOCK_REALTIME,&cas,NULL);
    setMotor(1,0,255,false);
    setMotor(2,0,255,false);
    setMotor(3,0,255,false);
    setMotor(4,0,255,false);
    setMotor(5,0,255,false);
    setMotor(6,0,255,false);

}

void setDevice(unsigned char addr){
    if(addr != lastAddr){
        if (ioctl(file, I2C_SLAVE, addr) < 0) {
            printf("Problem s vytvorenim spojenia so zariadenim s adresou:%d\n",addr);
            exit(1);
        }
        lastAddr = addr;
    }
}
void writeRegister(unsigned char addr,unsigned char reg, unsigned char value){
    unsigned char data[3];
    data[0] = reg;
    data[1] = value;
    setDevice(addr);
    if (write(file, data, 2) != 2) printf("addr:%i, write register %i,val %i",(int)addr,(int)reg,(int)value);
}
unsigned int readRegister16(unsigned char addr,unsigned char reg){
    char data[3];
    data[0] = reg;
    setDevice(addr);
    if (write(file, data, 1) != 1)  printf("addr:%i, write register %i",(int)addr,(int)reg);
    if (read(file, data, 2) != 2)   printf("addr:%i, read register %i",(int)addr,(int)reg);
    return (data[0]<<8)+data[1];
}
signed int readRegister16s(unsigned char addr,unsigned char reg){
    char wdata[2];
    wdata[0] = reg;
    setDevice(addr);
    if (write(file, wdata, 1) != 1)  printf("addr:%i, write register %i",(int)addr,(int)reg);
    signed char data[3];
    if (read(file, data, 2) != 2)   printf("addr:%i, read register %i",(int)addr,(int)reg);
    return (data[0]<<8)+data[1];
}
unsigned char readRegister8(unsigned char addr,unsigned char reg){
    char data[2];
    data[0] = reg;
    setDevice(addr);
    if (write(file, data, 1) != 1)  printf("addr:%i, write register %i",(int)addr,(int)reg);
    if (read(file, data, 1) != 1)   printf("addr:%i, read register %i",(int)addr,(int)reg);
    return data[0];
}

void sendMatImage(Mat img,int quality){
    vector<uchar> buff;
    vector<int> param = vector<int>(2);
    param[0] = CV_IMWRITE_JPEG_QUALITY;
    param[1] = quality;
    imencode(".jpg", img, buff, param);
    char len[10];
    sprintf(len, "%.8d", buff.size());
    send(clientsock_camera, len, strlen(len), 0);
    send(clientsock_camera, &buff[0],buff.size(), 0);
    buff.clear();
}
unsigned char getButton(char pos){
    switch(pos){
        case 1: return !gpio_read(27); break;
        case 2: return !gpio_read(17); break;
        case 3: return !gpio_read(22); break;
	default: return 0;
    }
}
int getSpeedRaw(int pos){
    switch(pos){
        case 1: return readRegister16s(MODRYADDR,1); break;
        case 2: return readRegister16s(ORANZOVYADDR,1); break;
        case 3: return readRegister16s(ORANZOVYADDR,2); break;
        case 4: return readRegister16s(MODRYADDR,2); break;
        case 5: return readRegister16s(ZLTYADDR,2); break;
        case 6: return readRegister16s(ZLTYADDR,1); break;
        default: return 0;
    }
}
float getSpeed(int pos){
	return (float)getSpeedRaw(pos)*((M_PI*OtackomerPriemer)/OtackomerConstant); //??? totoo este nie je dobre ????
}

int getDistanceRaw(int pos){
    switch(pos){
        case 1: return readRegister16s(MODRYADDR,3); break;
        case 2: return readRegister16s(ORANZOVYADDR,3); break;
        case 3: return readRegister16s(ORANZOVYADDR,4); break;
        case 4: return readRegister16s(MODRYADDR,4); break;
        case 5: return readRegister16s(ZLTYADDR,4); break;
        case 6: return readRegister16s(ZLTYADDR,3); break;
        default: return 0;
    }
}
int getDistance(int pos){
	return (int)((float)getDistanceRaw(pos)*((M_PI*OtackomerPriemer)/OtackomerConstant));
}
int getDeltaDistanceRaw(int pos){
    switch(pos){
        case 1: return readRegister16s(MODRYADDR,7); break;
        case 2: return readRegister16s(ORANZOVYADDR,7); break;
        case 3: return readRegister16s(ORANZOVYADDR,8); break;
        case 4: return readRegister16s(MODRYADDR,8); break;
        case 5: return readRegister16s(ZLTYADDR,6); break;
        case 6: return readRegister16s(ZLTYADDR,5); break;
        default: return 0;
    }
}
int getDeltaDistance(int pos){
        return (int)((float)getDeltaDistanceRaw(pos)*((M_PI*OtackomerPriemer)/OtackomerConstant));
}

void resetDistance(int pos){
    switch(pos){
        case 1: writeRegister(MODRYADDR,100,0);  break;
        case 2: writeRegister(ORANZOVYADDR,100,0);  break;
        case 3: writeRegister(ORANZOVYADDR,99,0);   break;
        case 4: writeRegister(MODRYADDR,99,0);   break;
        case 5: writeRegister(ZLTYADDR,99,0);   break;
        case 6: writeRegister(ZLTYADDR,100,0);  break;
    }
}

void setServo(int angle){
    if(angle+91 < 1) writeRegister(ORANZOVYADDR,84,1);
    else if(angle+91 > 181) writeRegister(ORANZOVYADDR,84,181);
    else writeRegister(0x0A,84,angle+91);
}
unsigned int getUltrasonicRaw(){
    return readRegister16(ORANZOVYADDR,6);
}
float getUltrasonic(){
	return (float)readRegister16(ORANZOVYADDR,6)/UltrasonicConstant;
}
int getVoltageRaw(){
    return readRegister16(MODRYADDR,5);
}
float getVoltage(){
    float napHod = (float)getVoltageRaw()*(maxVoltADC/rozlisenieADC)*((R1+R2)/R2);
    return napHod;
}
float getVoltagePercent(){
    float napHod = (float)getVoltage()*(100/(maxNapetie-minNapetie))-(100/(maxNapetie-minNapetie))*minNapetie;
    return napHod;
}
int getAmpRaw(){
    return readRegister16(MODRYADDR,6);
}
float getAmpVolt(){
    return (float)getAmpRaw()*(maxVoltADC/rozlisenieADC);
}
float getAmp(){
    return ((float)getAmpVolt()-maxVoltADC/2)/rozliseniePrud;
}

void setLed(int pos,char color){
    if(pos == 3){
        if(color == 'G'){
            writeRegister(MODRYADDR,96,0);
            writeRegister(MODRYADDR,97,1);
        }
        else if(color == 'R'){
	    writeRegister(MODRYADDR,97,0);
            writeRegister(MODRYADDR,96,1);
        }
        else if(color == 'O'){
            writeRegister(MODRYADDR,97,1);
            writeRegister(MODRYADDR,96,1);
        }
	else{
	    writeRegister(MODRYADDR,97,0);
            writeRegister(MODRYADDR,96,0);
	}
    }
    else if(pos == 1){
        if(color == 'G'){
            writeRegister(ZLTYADDR,96,0);
            writeRegister(ZLTYADDR,97,1);
        }
        else if(color == 'R'){
            writeRegister(ZLTYADDR,97,0);
	    writeRegister(ZLTYADDR,96,1);
        }
        else if(color == 'O'){
            writeRegister(ZLTYADDR,97,1);
            writeRegister(ZLTYADDR,96,1);
        }
	else{
	    writeRegister(ZLTYADDR,97,0);
	    writeRegister(ZLTYADDR,96,0);
	}
    }
    else{
        if(color == 'G'){
            writeRegister(ORANZOVYADDR,96,0);
            writeRegister(ORANZOVYADDR,97,1);
        }
        else if(color == 'R'){
            writeRegister(ORANZOVYADDR,97,0);
            writeRegister(ORANZOVYADDR,96,1);
        }
        else if(color == 'O'){
       	    writeRegister(ORANZOVYADDR,97,1);
            writeRegister(ORANZOVYADDR,96,1);    
        }
	else{
	    writeRegister(ORANZOVYADDR,97,0);
            writeRegister(ORANZOVYADDR,96,0);
	}
    }
}
void setMotorPowerSupply(bool state){
    if(state == false)   gpio_write(26,1);
    else                 gpio_write(26,0);
}
void setMotor(int pos,signed char dir,unsigned char speed,bool onReg){
    if(onReg == true){
        if(dir>=0){
            switch(pos){
                case 1: writeRegister(MODRYADDR,94,speed); break;
                case 2: writeRegister(ORANZOVYADDR,89,speed); break;
                case 3: writeRegister(ORANZOVYADDR,94,speed); break;
                case 4: writeRegister(MODRYADDR,89,speed); break;
                case 5: writeRegister(ZLTYADDR,89,speed); break;
                case 6: writeRegister(ZLTYADDR,94,speed); break;
            }
        }
        else{
            switch(pos){
                case 1: writeRegister(MODRYADDR,93,speed); break;
                case 2: writeRegister(ORANZOVYADDR,88,speed); break;
                case 3: writeRegister(ORANZOVYADDR,93,speed); break;
                case 4: writeRegister(MODRYADDR,88,speed); break;
                case 5: writeRegister(ZLTYADDR,88,speed); break;
                case 6: writeRegister(ZLTYADDR,93,speed); break;
            }
        }
    }
    else{
        if(dir>0){
            switch(pos){
                case 1: writeRegister(MODRYADDR,92,speed); break;
                case 2: writeRegister(ORANZOVYADDR,87,speed); break;
                case 3: writeRegister(ORANZOVYADDR,92,speed); break;
                case 4: writeRegister(MODRYADDR,87,speed); break;
                case 5: writeRegister(ZLTYADDR,87,speed); break;
                case 6: writeRegister(ZLTYADDR,92,speed); break;
            }
        }
        else if(dir==0){
            switch(pos){
                case 1: writeRegister(MODRYADDR,91,speed); break;
                case 2: writeRegister(ORANZOVYADDR,86,speed); break;
                case 3: writeRegister(ORANZOVYADDR,91,speed); break;
                case 4: writeRegister(MODRYADDR,86,speed); break;
                case 5: writeRegister(ZLTYADDR,86,speed); break;
                case 6: writeRegister(ZLTYADDR,91,speed); break;
            }
        }
        else{
            switch(pos){
                case 1: writeRegister(MODRYADDR,90,speed); break;
                case 2: writeRegister(ORANZOVYADDR,85,speed); break;
                case 3: writeRegister(ORANZOVYADDR,90,speed); break;
                case 4: writeRegister(MODRYADDR,85,speed); break;
                case 5: writeRegister(ZLTYADDR,85,speed); break;
                case 6: writeRegister(ZLTYADDR,90,speed); break;
            }
        }
    }
}
int getKbhit(void){
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
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}
GPS_struct getGPS(){
    GPS_struct GPS;
    while(strcmp(SerialRead(portHandle,1),"$") != 0);
	char *dataCMD = SerialRead(portHandle,5);
	int i = 0;
	int x = 0;
	char buffer[20];
	if(!strcmp(dataCMD,"GPVTG")){
	    char data[] = {' '};
        while(data[0] != '\n'){
            char *datas = SerialRead(portHandle,1);
            data[0] = datas[0];
            if(data[0] == ','){
                buffer[x]='\0';
                switch(i++){
                    case 1:  GPS.GPVTG.CourseTrue = atof(buffer);       break;
                    case 3:  GPS.GPVTG.CourseMagnetic = atof(buffer);   break;
                    case 5:  GPS.GPVTG.SpeedKnots = atoi(buffer);       break;
                    case 7:  GPS.GPVTG.SpeedKmh = atof(buffer);          break;
                }
                x = 0;
            }
            else{
                switch(i){
                    case 1:
					case 3:
					case 5:
					case 7:  buffer[x++] = data[0];                 break;
                    case 2:  GPS.GPVTG.ReferenceTrue = data[0];         break;
                    case 4:  GPS.GPVTG.ReferenceMagnetic = data[0];     break;
                    case 6:  GPS.GPVTG.UnitsKnots = data[0];           	break;
                    case 8:  GPS.GPVTG.UnitsKmh = data[0];           	break;
                    case 9:  GPS.GPVTG.Mode = data[0];   		break;
                    case 10: GPS.GPVTG.Checksum[x++] = data[0];         break;
                }
            }
        }
	}
	else if(!strcmp(dataCMD,"GPGGA")){
	    char data[] = {' '};
        while(data[0] != '\n'){
            char *datas = SerialRead(portHandle,1);
            data[0] = datas[0];
			if(data[0] == ','){
			    buffer[x]='\0';
				switch(i++){
                    case 2:  GPS.GPGGA.Latitude = atof(buffer);			break;
                    case 4:  GPS.GPGGA.Longitude = atof(buffer);  		break;
                    case 7:  GPS.GPGGA.SatellitesUsed = atoi(buffer); 	break;
		    case 8:  GPS.GPGGA.HDOP = atof(buffer); 			break;
                    case 9:  GPS.GPGGA.MSLAltitude = atof(buffer); 		break;
                    case 10: GPS.GPGGA.GeoidSeparation = atoi(buffer); 		break;
                    case 12: GPS.GPGGA.AgeofDifferentialCorrections = atoi(buffer);break;
                }
                x = 0;
            }
            else{
                switch(i){
                    case 1:  GPS.GPGGA.UTCTime[x++] = data[0];         break;
                    case 2:
                    case 7:
                    case 8:
                    case 9:
					case 4:  buffer[x++] = data[0];                 break;
					case 10: GPS.GPGGA.Units1 = data[0];		break;
					case 12: GPS.GPGGA.Units2 = data[0];		break;
                    case 3:  GPS.GPGGA.NSIndicator = data[0];          	break;
                    case 5:  GPS.GPGGA.EWindicator = data[0];           break;
                    case 6:  GPS.GPGGA.PositionFixIndictor = data[0];  	break;
					case 13: GPS.GPGGA.Checksum[x++] = data[0];		break;
                }
            }
        }
    }
	else if(!strcmp(dataCMD,"GPGSA")){
	    char data[] = {' '};
        while(data[0] != '\n'){
            char *datas = SerialRead(portHandle,1);
            data[0] = datas[0];
            if(data[0] == ','){
                buffer[x]='\0';
                switch(i++){
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
            else{
                switch(i){
                    case 1:	 GPS.GPGSA.ModeChar = data[0];	break;
                    case 2:  GPS.GPGSA.ModeInt = data[0];      	break;
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
	else if(!strcmp(dataCMD,"GPGLL")){
	    char data[] = {' '};
        while(data[0] != '\n'){
            char *datas = SerialRead(portHandle,1);
            data[0] = datas[0];
            if(data[0] == ','){
                buffer[x]='\0';
                switch(i++){
                    case 1:  GPS.GPGLL.Latitude = atof(buffer);                 break;
                    case 3:  GPS.GPGLL.Longitude = atof(buffer);                break;
                }
                x = 0;
            }
            else{
                switch(i){
                    case 1:
					case 3:	 buffer[x++] = data[0];         break;
                    case 2:  GPS.GPGLL.NSIndicator = data[0];	break;
					case 4:  GPS.GPGLL.EWIndicator = data[0];   break;
					case 5:  GPS.GPGLL.UTCTime[x++] = data[0];  break;
					case 6:  GPS.GPGLL.Status = data[0];  	break;
					case 7:  GPS.GPGLL.ModeIndicator = data[0]; break;
					case 8:  GPS.GPGLL.Checksum[x++] = data[0]; break;
                }
            }
        }
    }
	else if(!strcmp(dataCMD,"GPRMC")){
	    char data[] = {' '};
        while(data[0] != '\n'){
            char *datas = SerialRead(portHandle,1);
            data[0] = datas[0];
            if(data[0] == ','){
                buffer[x]='\0';
                switch(i++){
                    case 3:  GPS.GPRMC.Latitude = atof(buffer); break;
					case 5:  GPS.GPRMC.Longitude = atof(buffer); break;
					case 7:  GPS.GPRMC.SpeedOverGround = atof(buffer); break;
					case 8:  GPS.GPRMC.CourseOverGround = atof(buffer); break;
                }
                x = 0;
            }
            else{
                switch(i){
                    case 1:  GPS.GPRMC.UTCTime[x++] = data[0];  break;
					case 2:  GPS.GPRMC.Status = data[0];        break;
					case 9:  GPS.GPRMC.Date[x++] = data[0];	break;
					case 5:
					case 7:
					case 8:
					case 3:  buffer[x++] = data[0];         break;
					case 10: GPS.GPRMC.Mode = data[0];		break;
					case 4:  GPS.GPRMC.NSIndicator = data[0];   break;
					case 6:  GPS.GPRMC.EWIndicator = data[0];   break;
					case 11: GPS.GPRMC.Checksum[x++] = data[0]; break;
                }
            }
        }
    }
	else if(!strcmp(dataCMD,"GPGSV")){
        char data[] = {' '};
        while(data[0] != '\n'){
            char *datas = SerialRead(portHandle,1);
            data[0] = datas[0];
            if(data[0] == ','){
                buffer[x]='\0';
                switch(i++){
                    case 1:  GPS.GPGSV.NumberOfMessages = atoi(buffer);	break;
                    case 2:  GPS.GPGSV.MessageNumber = atoi(buffer);    break;
					case 3:  GPS.GPGSV.SatellitesInView = atoi(buffer); break;
					case 4:  GPS.GPGSV.SatelliteId1 = atoi(buffer);     break;
					case 5:  GPS.GPGSV.Elevation1 = atoi(buffer);    	break;
					case 6:  GPS.GPGSV.Azimuth1 = atoi(buffer);    	break;
					case 7:  GPS.GPGSV.SNR1 = atoi(buffer);    		break;
					case 8:  GPS.GPGSV.SatelliteId2 = atoi(buffer);    	break;
                    case 9:  GPS.GPGSV.Elevation2 = atoi(buffer);    	break;
                    case 10: GPS.GPGSV.Azimuth2 = atoi(buffer);    	break;
                    case 11: GPS.GPGSV.SNR2 = atoi(buffer);    		break;
					case 12: GPS.GPGSV.SatelliteId3 = atoi(buffer);    	break;
                    case 13: GPS.GPGSV.Elevation3 = atoi(buffer);    	break;
                    case 14: GPS.GPGSV.Azimuth3 = atoi(buffer);    	break;
                    case 15: GPS.GPGSV.SNR3 = atoi(buffer);    		break;
					case 16: GPS.GPGSV.SatelliteId4 = atoi(buffer);    	break;
                    case 17: GPS.GPGSV.Elevation4 = atoi(buffer);    	break;
                    case 18: GPS.GPGSV.Azimuth4 = atoi(buffer);    	break;
                    case 19: GPS.GPGSV.SNR4 = atoi(buffer);    		break;
                }
                x = 0;
            }
            else{
                switch(i){
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
float AccX_offset = 0;
float AccY_offset = 0;
float AccZ_offset = 0;
float GyX_offset = 0;
float GyY_offset = 0;
float GyZ_offset = 0;
float AccScale = 16384;
float GyScale = 131;
float Pitch=0;
float Roll=0;
float Yaw=0;

void MPU6050ResetPRY(){
	Pitch = 0;
	Roll = 0;
	Yaw = 0;
}
void MPU6050ResetOffset(){
	AccX_offset = 0;
	AccY_offset = 0;
	AccZ_offset = 0;
	GyX_offset = 0;
	GyY_offset = 0;
	GyZ_offset = 0;
}
void MPU6050WakeUp(){
	writeRegister(MPU6050ADDR,0x6B,0);
}
MPU6050_struct getMPU6050Raw(){
	MPU6050_struct MPU6050;
	MPU6050.AccX = (float)readRegister16s(MPU6050ADDR,0x3B);
	MPU6050.AccY = (float)readRegister16s(MPU6050ADDR,0x3D);
        MPU6050.AccZ = (float)readRegister16s(MPU6050ADDR,0x3F);
        MPU6050.Temp = (float)readRegister16s(MPU6050ADDR,0x41);
        MPU6050.GyX  = (float)readRegister16s(MPU6050ADDR,0x43);
        MPU6050.GyY  = (float)readRegister16s(MPU6050ADDR,0x45);
        MPU6050.GyZ  = (float)readRegister16s(MPU6050ADDR,0x47);
	return MPU6050;
}
MPU6050_struct getMPU6050(){
	MPU6050_struct MPU6050;
	MPU6050.AccX = (float)readRegister16s(MPU6050ADDR,0x3B)/AccScale-AccX_offset;
        MPU6050.AccY = (float)readRegister16s(MPU6050ADDR,0x3D)/AccScale-AccY_offset;
        MPU6050.AccZ = (float)readRegister16s(MPU6050ADDR,0x3F)/AccScale-AccZ_offset;
        MPU6050.Temp = (float)readRegister16s(MPU6050ADDR,0x41)/340+36.53;
        MPU6050.GyX  = (float)readRegister16s(MPU6050ADDR,0x43)/GyScale-GyX_offset;
        MPU6050.GyY  = (float)readRegister16s(MPU6050ADDR,0x45)/GyScale-GyY_offset;
        MPU6050.GyZ  = (float)readRegister16s(MPU6050ADDR,0x47)/GyScale-GyZ_offset;
/*	if(MPU6050.AccX <0.01 && MPU6050.AccX > -0.01) MPU6050.AccX = 0;
	if(MPU6050.AccY <0.01 && MPU6050.AccY > -0.01) MPU6050.AccY = 0;
	if(MPU6050.AccZ <0.01 && MPU6050.AccZ > -0.01) MPU6050.AccZ = 0;
	if(MPU6050.GyX <0.1 && MPU6050.GyX > -0.1) MPU6050.GyX = 0;
	if(MPU6050.GyY <0.1 && MPU6050.GyY > -0.1) MPU6050.GyY = 0; 
	if(MPU6050.GyZ <0.1 && MPU6050.GyZ > -0.1) MPU6050.GyZ = 0;
*/	return MPU6050;
}

void MPU6050CalibrateOffset(int pocet){
	float acc_x=0,acc_y=0,acc_z=0;
	float gy_x=0,gy_y=0,gy_z=0;
	MPU6050_struct MPU6050;
	for(int i=0;i<pocet;i++){
		MPU6050 = getMPU6050();
		acc_x+=MPU6050.AccX;
		acc_y+=MPU6050.AccY;
		acc_z+=MPU6050.AccZ;
		gy_x+=MPU6050.GyX;
		gy_y+=MPU6050.GyY;
		gy_z+=MPU6050.GyZ;
	}
	AccX_offset = acc_x/pocet;
	AccY_offset = acc_y/pocet;
	AccZ_offset = acc_z/pocet+1;
	
	GyX_offset = gy_x/pocet;
	GyY_offset = gy_y/pocet;
	GyZ_offset = gy_z/pocet;
}
float dist(float a,float b){
	return sqrt(a*a+b*b);
}
//komplementarny filter - http://www.pieter-jan.com/node/11
//https://b94be14129454da9cf7f056f5f8b89a9b17da0be.googledrive.com/host/0B0ZbiLZrqVa6Y2d3UjFVWDhNZms/filter.pdf
//http://husstechlabs.com/projects/atb1/using-the-accelerometer/
MPU6050_struct getMPU6050Full(float dt){
	float pitchAcc;
	float rollAcc;
	float yawAcc;
	MPU6050_struct MPU6050 = getMPU6050();
	MPU6050.Pitch	+=MPU6050.GyX*dt+Pitch;
	MPU6050.Roll	-=MPU6050.GyY*dt+Roll;
	MPU6050.Yaw	+= MPU6050.GyZ*dt+Yaw;
	int forceMagnitudeApprox = abs(MPU6050.AccX)+abs(MPU6050.AccY)+abs(MPU6050.AccZ);
	if(forceMagnitudeApprox >AccScale && forceMagnitudeApprox < 32768){
       	 	//pitchAcc = atan2f(MPU6050.AccY, dist(MPU6050.AccZ,MPU6050.AccX)) * 180 / M_PI;	
        	pitchAcc = atan2f(MPU6050.AccY,MPU6050.AccX*MPU6050.AccX+MPU6050.AccZ*MPU6050.AccZ) * 180 / M_PI;
		MPU6050.Pitch = MPU6050.Pitch * 0.98 + pitchAcc * 0.02;
		Pitch=MPU6050.Pitch;
        	
		//rollAcc = atan2f(MPU6050.AccX, dist(MPU6050.AccZ,MPU6050.AccY)) * 180 / M_PI;
        	rollAcc = atan2f(MPU6050.AccX, MPU6050.AccY*MPU6050.AccY+MPU6050.AccZ*MPU6050.AccZ) * 180 / M_PI;
		MPU6050.Roll = MPU6050.Roll * 0.98 + rollAcc * 0.02;
		Roll=MPU6050.Roll;
		
		yawAcc = atan2f(dist(MPU6050.AccX,MPU6050.AccY),MPU6050.AccZ) * 180 / M_PI;
                MPU6050.Yaw = MPU6050.Yaw * 0.98 + yawAcc * 0.02;
		Yaw=MPU6050.Yaw;
	}
	return MPU6050;
} 

void setMPU6050Sensitivity(unsigned char acc_sens,unsigned char gy_sens){
	switch(acc_sens){
		case 0: AccScale = 16384;	break;		//2g
		case 1:	AccScale = 8192;	break;		//4g
		case 2: AccScale = 4096;	break;		//8g
		case 3:	AccScale = 2048;	break;		//16g
	}
	
	switch(gy_sens){
		case 0: GyScale = 131;      	break;          //250 stup/s
                case 1: GyScale = 65.5;      	break;          //500 stup/s
                case 2: GyScale = 32.75;      break;          //1000 stup/s
                case 3: GyScale = 16.375;     break;          //2000 stup/s
	}
	writeRegister(MPU6050ADDR,0x1B,(gy_sens<<3)|0xE0);
        writeRegister(MPU6050ADDR,0x1C,(acc_sens<<3)|0xE0);
}
void setMPU6050DLPF(unsigned char acc_dlpf,unsigned char gy_dlpf){
	writeRegister(MPU6050ADDR,0x1A,acc_dlpf|(5<<3));
	writeRegister(MPU6050ADDR,0x1A,acc_dlpf|(6<<3));
	writeRegister(MPU6050ADDR,0x1A,acc_dlpf|(7<<3));
	writeRegister(MPU6050ADDR,0x1A,gy_dlpf|(2<<3));
	writeRegister(MPU6050ADDR,0x1A,gy_dlpf|(3<<3));
 	writeRegister(MPU6050ADDR,0x1A,gy_dlpf|(4<<3));
}
int getDistanceL(){
        return ((float)(robotVariables.motors.motor4.distance+robotVariables.motors.motor5.distance+robotVariables.motors.motor6.distance)/3);
}
int getDistanceR(){
        return ((float)(robotVariables.motors.motor1.distance+robotVariables.motors.motor2.distance+robotVariables.motors.motor3.distance)/3);
}

float getSpeedFromDistanceL(float dt){
	return (float)getDistanceL()/dt;
}
float getSpeedFromDistanceR(float dt){
	return (float)getDistanceR()/dt;
}
//http://rossum.sourceforge.net/papers/DiffSteer/DiffSteer.html
void calcRobotPosition(float dt){
	robotVariables.robotPosition.angle += (getDistanceR()-getDistanceL())/vzdialenostKolies;
	robotVariables.robotPosition.x =  ((getSpeedFromDistanceL(dt)+getSpeedFromDistanceR(dt))/2)*cos(robotVariables.robotPosition.angle);
	robotVariables.robotPosition.y = ((getSpeedFromDistanceL(dt)+getSpeedFromDistanceR(dt))/2)*sin(robotVariables.robotPosition.angle);
}
int pocetPosition = 100;
int pocetMotors = 100;
int pocetBattery = 100;
int pocetMPU6050 = 100;
int pocetLeds = 100;
int pocetAmp = 100;
int pocetUltrasonic = 100;

bool compareMotors(Motor_struct motor,Motor_struct lastMotor){
	return motor.direction!=lastMotor.direction || motor.setSpeed!=lastMotor.setSpeed || motor.onRegulator!=lastMotor.onRegulator;
}
void syncModules(int signal , siginfo_t * siginfo, void * ptr){
	switch (signal)
  	{
		case SIGUSR1:
		bool refreshPositionCheck = refreshPosition/refreshModule >= pocetPosition;
		bool refreshMotorsCheck = refreshMotors/refreshModule >= pocetMotors;
		bool refreshBatteryCheck = refreshBattery/refreshModule >= pocetBattery;
		bool refreshMPU6050Check = refreshMPU6050/refreshModule >= pocetMPU6050;
		bool refreshLedsCheck = refreshLeds/refreshModule >= pocetLeds;
		bool refreshAmpCheck = refreshAmp/refreshModule >= pocetAmp;
		bool refreshUltrasonicCheck = refreshUltrasonic/refreshModule >= pocetUltrasonic;
		semWait(sem_id,0);
		if(refreshMPU6050Check){
			robotVariables.MPU6050 = getMPU6050Full((float)refreshMPU6050/1000);
			pocetMPU6050 = 0;
		}
		if(refreshPositionCheck){
			robotVariables.motors.motor1.distance += getDeltaDistance(1);
			robotVariables.motors.motor1.actSpeed = getSpeed(1);
			pocetPosition = 0;
		}
		if(refreshMotorsCheck || compareMotors(robotVariables.motors.motor1,lastRobotVariables.motors.motor1)){
			setMotor(1,robotVariables.motors.motor1.direction,robotVariables.motors.motor1.setSpeed,robotVariables.motors.motor1.onRegulator);
			pocetMotors = 0;
		}
		if(refreshPositionCheck){
			robotVariables.motors.motor4.distance += getDeltaDistance(4);
     	  		robotVariables.motors.motor4.actSpeed = getSpeed(4);
			pocetPosition = 0;
		}
		if(refreshMotorsCheck|| compareMotors(robotVariables.motors.motor4,lastRobotVariables.motors.motor4)){			
			setMotor(4,robotVariables.motors.motor4.direction,robotVariables.motors.motor4.setSpeed,robotVariables.motors.motor4.onRegulator);
			pocetMotors = 0;
		}
		if(refreshPositionCheck){
			robotVariables.motors.motor2.distance += getDeltaDistance(2);
        		robotVariables.motors.motor2.actSpeed = getSpeed(2);
			pocetPosition = 0;
		}
		if(refreshMotorsCheck || compareMotors(robotVariables.motors.motor2,lastRobotVariables.motors.motor2)){
		 	setMotor(2,robotVariables.motors.motor2.direction,robotVariables.motors.motor2.setSpeed,robotVariables.motors.motor2.onRegulator);
			pocetMotors = 0;
		}
		if(refreshPositionCheck){
			robotVariables.motors.motor3.distance += getDeltaDistance(3);
	        	robotVariables.motors.motor3.actSpeed = getSpeed(3);
			pocetPosition = 0;
		}
		if(refreshMotorsCheck || compareMotors(robotVariables.motors.motor3,lastRobotVariables.motors.motor3)){
			setMotor(3,robotVariables.motors.motor3.direction,robotVariables.motors.motor4.setSpeed,robotVariables.motors.motor3.onRegulator);
			pocetMotors = 0;
		}
		if(refreshPositionCheck){
			robotVariables.motors.motor5.distance += getDeltaDistance(5);
        		robotVariables.motors.motor5.actSpeed = getSpeed(5);
			pocetPosition = 0;
		}
		if(refreshMotorsCheck || compareMotors(robotVariables.motors.motor5,lastRobotVariables.motors.motor5)){
			setMotor(5,robotVariables.motors.motor5.direction,robotVariables.motors.motor5.setSpeed,robotVariables.motors.motor5.onRegulator);
			pocetMotors = 0;
		}
		if(refreshPositionCheck){
			robotVariables.motors.motor6.distance += getDeltaDistance(6);
        		robotVariables.motors.motor6.actSpeed = getSpeed(6);
			pocetPosition = 0;
		}		
		if(refreshMotorsCheck || compareMotors(robotVariables.motors.motor6,lastRobotVariables.motors.motor6)){
			setMotor(6,robotVariables.motors.motor6.direction,robotVariables.motors.motor6.setSpeed,robotVariables.motors.motor6.onRegulator);
			pocetMotors = 0;
		}
		if(refreshLedsCheck){
			setLed(1,robotVariables.leds.Led1);
			setLed(2,robotVariables.leds.Led2);
			if(!BatteryLed3Indicate) setLed(3,robotVariables.leds.Led3);
		}
		if(refreshPositionCheck) calcRobotPosition((float)refreshPosition/1000);
		robotVariables.buttons.button1 = getButton(1);
		robotVariables.buttons.button2 = getButton(2);
		robotVariables.buttons.button3 = getButton(3);
		if(refreshBatteryCheck){
			robotVariables.voltage = getVoltage();
			robotVariables.voltagePercent = getVoltagePercent();
			if(BatteryLed3Indicate == 1){
                        	if(robotVariables.voltagePercent > 60)           setLed(3,'G');
                        	else if(robotVariables.voltagePercent > 20)      setLed(3,'O');
                        	else                                             setLed(3,'R');
                	}
		}
		if(refreshAmpCheck)	   robotVariables.amper = getAmp();
		if(refreshUltrasonicCheck) robotVariables.ultrasonic = getUltrasonic();
		memcpy(&robotVariables, &lastRobotVariables, sizeof(robotVariables));
		semPost(sem_id,0);
		break;
	}
}
