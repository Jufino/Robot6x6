#include "robotTerenny.h"

int file;
int lastAddr = 0x00;
char *filename = "/dev/i2c-1";
int serversock_snimace,serversock_camera;
int clientsock_snimace,clientsock_camera;
int portHandle;
GPGGA_struct GPGGA;
GPGLL_struct GPGLL;
GPRMC_struct GPRMC;
GPGSA_struct GPGSA;
GPGSV_struct GPGSV;
GPVTG_struct GPVTG;

void initRobot(){
    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Problem s otvorenim portu.\n");
        exit(1);
    }
    portHandle = SerialOpen("/dev/ttyAMA0",B9600);
    gpio_open(17,0);
    gpio_open(27,0);
    gpio_open(22,0);
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
    if(Wifi_camera == 1 || Wifi_snimace == 1){
        signal(SIGPIPE, sigpipe_fun);
    }
}
void closeRobot(){
    SerialClose(portHandle);
    gpio_close(17);
    gpio_close(27);
    gpio_close(22);
}

void setDevice(int addr){
    if(addr != lastAddr){
        if (ioctl(file, I2C_SLAVE, addr) < 0) {
            printf("Problem s vytvorenim spojenia so zariadenim s adresou:%d\n",addr);
            exit(1);
        }
        lastAddr = addr;
    }
}
void writeRegister(int addr,unsigned char reg, unsigned char value){
    unsigned char data[3];
    data[0] = reg;
    data[1] = value;
    setDevice(addr);
    if (write(file, data, 2) != 2) perror("write register");
}
unsigned int readRegister16(int addr,unsigned char reg){
    char data[3];
    data[0] = reg;
    setDevice(addr);
    if (write(file, data, 1) != 1)  perror("write error");
    if (read(file, data, 2) != 2)   perror("read error");
    return (data[0]<<8)+data[1];
}
signed int readRegisters16(int addr,unsigned char reg){
    char wdata[2];
    wdata[0] = reg;
    setDevice(addr);
    if (write(file, wdata, 1) != 1)  perror("write error");
    signed char data[3];
    if (read(file, data, 2) != 2)   perror("read error");
    return (data[0]<<8)+data[1];
}
unsigned char readRegister8(int addr,unsigned char reg){
    char data[2];
    data[0] = reg;
    setDevice(addr);
    if (write(file, data, 1) != 1)  perror("write register");
    if (read(file, data, 1) != 1)   perror("read error");
    return data[0];
}

void odosliMat(int socket, Mat img,int kvalita){
    vector<uchar> buff;
    vector<int> param = vector<int>(2);
    param[0] = CV_IMWRITE_JPEG_QUALITY;
    param[1] = kvalita;
    imencode(".jpg", img, buff, param);
    char len[10];
    sprintf(len, "%.8d", buff.size());
    send(socket, len, strlen(len), 0);
    send(socket, &buff[0],buff.size(), 0);
    buff.clear();
}
unsigned char tlacitka(char pozicia){
    switch(pozicia){
        case 1: return !gpio_read(27); break;
        case 2: return !gpio_read(17); break;
        case 3: return !gpio_read(22); break;
    }
}
unsigned int rychlost(int poradie){
    switch(poradie){
        case 1: return readRegister16(0x08,1); break;
        case 2: return readRegister16(0x0A,1); break;
        case 3: return readRegister16(0x0A,2); break;
        case 4: return readRegister16(0x08,2); break;
        case 5: return readRegister16(0x09,2); break;
        case 6: return readRegister16(0x09,1); break;
        default: return 0;
    }
}
unsigned int vzdialenost(int poradie){
    switch(poradie){
        case 1: return readRegister16(0x08,3); break;
        case 2: return readRegister16(0x0A,3); break;
        case 3: return readRegister16(0x0A,4); break;
        case 4: return readRegister16(0x08,4); break;
        case 5: return readRegister16(0x09,4); break;
        case 6: return readRegister16(0x09,3); break;
        default: return 0;
    }
}
void resetVzdialenost(int poradie){
    switch(poradie){
        case 1: writeRegister(0x08,100,0);  break;
        case 2: writeRegister(0x0A,100,0);  break;
        case 3: writeRegister(0x0A,99,0);   break;
        case 4: writeRegister(0x08,99,0);   break;
        case 5: writeRegister(0x09,99,0);   break;
        case 6: writeRegister(0x09,100,0);  break;
    }
}
void servo(int pozicia){
    if(pozicia+91 < 1) writeRegister(0x0A,84,1);
    else if(pozicia+91 > 181) writeRegister(0x0A,84,181);
    else writeRegister(0x0A,84,pozicia+91);
}
unsigned int ultrazvuk(){
    return readRegister16(0x0A,6);
}
int napetieRaw(){
    return readRegister16(0x08,5);
}
float napetieVolt(){
    float napHod = (float)napetieRaw()*(maxVolt/rozlisenieADC)*((R1+R2)/R2);
    return napHod;
}
float napetiePercent(){
    float napHod = (float)napetieVolt()*(100/(maxNapetie-minNapetie))-(100/(maxNapetie-minNapetie))*minNapetie;
    return napHod;
}
unsigned int prud(){
    return readRegister16(0x08,6);
}
void led(int poradie,char nazov,bool stav){
    if(poradie == 3){
        if(nazov == 'Z'){
            if(stav == false) writeRegister(0x08,97,0);
            else writeRegister(0x08,97,1);
        }
        else if(nazov == 'C'){
            if(stav == false)       writeRegister(0x08,96,0);
            else    writeRegister(0x08,96,1);
        }
        else{
            if(stav == false){
                led(1,'Z',0);
                led(1,'C',0);
            }
            else{
                led(1,'Z',1);
                led(1,'C',1);
            }
        }
    }
    else if(poradie == 1){
        if(nazov == 'Z'){
            if(stav == false)   writeRegister(0x09,97,0);
            else                writeRegister(0x09,97,1);
        }
        else if(nazov == 'C'){
            if(stav == false)   writeRegister(0x09,96,0);
            else                writeRegister(0x09,96,1);
        }
        else{
            if(stav == false){
                led(2,'Z',0);
                led(2,'C',0);
            }
            else{
                led(2,'Z',1);
                led(2,'C',1);
            }
        }
    }
    else{
        if(nazov == 'Z'){
            if(stav == false)   writeRegister(0x0A,97,0);
            else                writeRegister(0x0A,97,1);
        }
        else if(nazov == 'C'){
            if(stav == false)   writeRegister(0x0A,96,0);
            else                writeRegister(0x0A,96,1);
        }
        else{
            if(stav == false){
                led(3,'Z',0);
                led(3,'C',0);
            }
            else{
                led(3,'Z',1);
                led(3,'C',1);
            }
        }
    }
}
void napajanie(bool stav){
    if(stav == false)   writeRegister(0x08,95,0);
    else                writeRegister(0x08,95,1);
}
void motor(int poradie,signed char smer,unsigned char rychlost,bool reg){
    if(reg == true){
        if(smer>=0){
            switch(poradie){
                case 1: writeRegister(0x08,94,rychlost); break;
                case 2: writeRegister(0x0A,89,rychlost); break;
                case 3: writeRegister(0x0A,94,rychlost); break;
                case 4: writeRegister(0x08,89,rychlost); break;
                case 5: writeRegister(0x09,89,rychlost); break;
                case 6: writeRegister(0x09,94,rychlost); break;
            }
        }
        else{
            switch(poradie){
                case 1: writeRegister(0x08,93,rychlost); break;
                case 2: writeRegister(0x0A,88,rychlost); break;
                case 3: writeRegister(0x0A,93,rychlost); break;
                case 4: writeRegister(0x08,88,rychlost); break;
                case 5: writeRegister(0x09,88,rychlost); break;
                case 6: writeRegister(0x09,93,rychlost); break;
            }
        }
    }
    else{
        if(smer>0){
            switch(poradie){
                case 1: writeRegister(0x08,92,rychlost); break;
                case 2: writeRegister(0x0A,87,rychlost); break;
                case 3: writeRegister(0x0A,92,rychlost); break;
                case 4: writeRegister(0x08,87,rychlost); break;
                case 5: writeRegister(0x09,87,rychlost); break;
                case 6: writeRegister(0x09,92,rychlost); break;
            }
        }
        else if(smer==0){
            switch(poradie){
                case 1: writeRegister(0x08,91,rychlost); break;
                case 2: writeRegister(0x0A,86,rychlost); break;
                case 3: writeRegister(0x0A,91,rychlost); break;
                case 4: writeRegister(0x08,86,rychlost); break;
                case 5: writeRegister(0x09,86,rychlost); break;
                case 6: writeRegister(0x09,91,rychlost); break;
            }
        }
        else{
            switch(poradie){
                case 1: writeRegister(0x08,90,rychlost); break;
                case 2: writeRegister(0x0A,85,rychlost); break;
                case 3: writeRegister(0x0A,90,rychlost); break;
                case 4: writeRegister(0x08,85,rychlost); break;
                case 5: writeRegister(0x09,85,rychlost); break;
                case 6: writeRegister(0x09,90,rychlost); break;
            }
        }
    }
}
int kbhit(void){
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
void parseGPS(){
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
                    case 1:  GPVTG.CourseTrue = atof(buffer);       break;
                    case 3:  GPVTG.CourseMagnetic = atof(buffer);   break;
                    case 5:  GPVTG.SpeedKnots = atoi(buffer);       break;
                    case 7:  GPVTG.SpeedKmh = atof(buffer);          break;
                }
                x = 0;
            }
            else{
                switch(i){
                    case 1:
					case 3:
					case 5:
					case 7:  buffer[x++] = data[0];                 break;
                    case 2:  GPVTG.ReferenceTrue = data[0];         break;
                    case 4:  GPVTG.ReferenceMagnetic = data[0];     break;
                    case 6:  GPVTG.UnitsKnots = data[0];           	break;
                    case 8:  GPVTG.UnitsKmh = data[0];           	break;
                    case 9:  GPVTG.Mode = data[0];   		break;
                    case 10: GPVTG.Checksum[x++] = data[0];         break;
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
                    case 2:  GPGGA.Latitude = atof(buffer);			break;
                    case 4:  GPGGA.Longitude = atof(buffer);  		break;
                    case 7:  GPGGA.SatellitesUsed = atoi(buffer); 	break;
					case 8:  GPGGA.HDOP = atof(buffer); 			break;
                    case 9:  GPGGA.MSLAltitude = atof(buffer); 		break;
                    case 10: GPGGA.GeoidSeparation = atoi(buffer); 		break;
                    case 12: GPGGA.AgeofDifferentialCorrections = atoi(buffer);break;
                }
                x = 0;
            }
            else{
                switch(i){
                    case 1:  GPGGA.UTCTime[x++] = data[0];         break;
                    case 2:
                    case 7:
                    case 8:
                    case 9:
					case 4:  buffer[x++] = data[0];                 break;
					case 10: GPGGA.Units1 = data[0];		break;
					case 12: GPGGA.Units2 = data[0];		break;
                    case 3:  GPGGA.NSIndicator = data[0];          	break;
                    case 5:  GPGGA.EWindicator = data[0];           break;
                    case 6:  GPGGA.PositionFixIndictor = data[0];  	break;
					case 13: GPGGA.Checksum[x++] = data[0];		break;
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
                    case 3:  GPGSA.SatellitesUsedCH1 = atoi(buffer); break;
					case 4:  GPGSA.SatellitesUsedCH2 = atoi(buffer); break;
					case 5:  GPGSA.SatellitesUsedCH3 = atoi(buffer); break;
					case 6:  GPGSA.SatellitesUsedCH4 = atoi(buffer); break;
					case 7:  GPGSA.SatellitesUsedCH5 = atoi(buffer); break;
					case 8:  GPGSA.SatellitesUsedCH6 = atoi(buffer); break;
					case 9:  GPGSA.SatellitesUsedCH7 = atoi(buffer); break;
					case 10:  GPGSA.SatellitesUsedCH8 = atoi(buffer); break;
					case 11:  GPGSA.SatellitesUsedCH9 = atoi(buffer); break;
					case 12:  GPGSA.SatellitesUsedCH10 = atoi(buffer); break;
					case 13:  GPGSA.SatellitesUsedCH11 = atoi(buffer); break;
					case 14:  GPGSA.SatellitesUsedCH12 = atoi(buffer); break;
					case 15:  GPGSA.PDOP = atof(buffer); break;
					case 16:  GPGSA.HDOP = atof(buffer); break;
					case 17:  GPGSA.VDOP = atof(buffer); break;
                }
                x = 0;
            }
            else{
                switch(i){
                    case 1:	 GPGSA.ModeChar = data[0];	break;
                    case 2:  GPGSA.ModeInt = data[0];      	break;
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
                    case 18:  GPGSA.Checksum[x++] = data[0]; break;
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
                    case 1:  GPGLL.Latitude = atof(buffer);                 break;
                    case 3:  GPGLL.Longitude = atof(buffer);                break;
                }
                x = 0;
            }
            else{
                switch(i){
                    case 1:
					case 3:	 buffer[x++] = data[0];         break;
                    case 2:  GPGLL.NSIndicator = data[0];	break;
					case 4:  GPGLL.EWIndicator = data[0];   break;
					case 5:  GPGLL.UTCTime[x++] = data[0];  break;
					case 6:  GPGLL.Status = data[0];  	break;
					case 7:  GPGLL.ModeIndicator = data[0]; break;
					case 8:  GPGLL.Checksum[x++] = data[0]; break;
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
                    case 3:  GPRMC.Latitude = atof(buffer); break;
					case 5:  GPRMC.Longitude = atof(buffer); break;
					case 7:  GPRMC.SpeedOverGround = atof(buffer); break;
					case 8:  GPRMC.CourseOverGround = atof(buffer); break;
                }
                x = 0;
            }
            else{
                switch(i){
                    case 1:  GPRMC.UTCTime[x++] = data[0];  break;
					case 2:  GPRMC.Status = data[0];        break;
					case 9:  GPRMC.Date[x++] = data[0];	break;
					case 5:
					case 7:
					case 8:
					case 3:  buffer[x++] = data[0];         break;
					case 10: GPRMC.Mode = data[0];		break;
					case 4:  GPRMC.NSIndicator = data[0];   break;
					case 6:  GPRMC.EWIndicator = data[0];   break;
					case 11: GPRMC.Checksum[x++] = data[0]; break;
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
                    case 1:  GPGSV.NumberOfMessages = atoi(buffer);	break;
                    case 2:  GPGSV.MessageNumber = atoi(buffer);    break;
					case 3:  GPGSV.SatellitesInView = atoi(buffer); break;
					case 4:  GPGSV.SatelliteId1 = atoi(buffer);     break;
					case 5:  GPGSV.Elevation1 = atoi(buffer);    	break;
					case 6:  GPGSV.Azimuth1 = atoi(buffer);    	break;
					case 7:  GPGSV.SNR1 = atoi(buffer);    		break;
					case 8:  GPGSV.SatelliteId2 = atoi(buffer);    	break;
                    case 9:  GPGSV.Elevation2 = atoi(buffer);    	break;
                    case 10: GPGSV.Azimuth2 = atoi(buffer);    	break;
                    case 11: GPGSV.SNR2 = atoi(buffer);    		break;
					case 12: GPGSV.SatelliteId3 = atoi(buffer);    	break;
                    case 13: GPGSV.Elevation3 = atoi(buffer);    	break;
                    case 14: GPGSV.Azimuth3 = atoi(buffer);    	break;
                    case 15: GPGSV.SNR3 = atoi(buffer);    		break;
					case 16: GPGSV.SatelliteId4 = atoi(buffer);    	break;
                    case 17: GPGSV.Elevation4 = atoi(buffer);    	break;
                    case 18: GPGSV.Azimuth4 = atoi(buffer);    	break;
                    case 19: GPGSV.SNR4 = atoi(buffer);    		break;
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
                    case 20:  GPGSV.Checksum[x++] = data[0]; break;
                }
            }
        }
    }
}

void sigctrl_fun(int param){
  printf("Server sa vypina\n");
  sleep(2);
  //semRem(sem_id);
  if(Wifi_camera == 1){
    close(serversock_camera);
    close(clientsock_camera);
  }
  if(Wifi_snimace == 1){
    close(serversock_snimace);
    close(clientsock_snimace);
  }
  exit(0);
}
void sigpipe_fun(int param){
  printf("Client sa odpojil\n");
  //semRem(sem_id);
  if(Wifi_camera == 1){
    close(serversock_camera);
    close(clientsock_camera);
  }
  if(Wifi_snimace == 1){
    close(serversock_snimace);
    close(clientsock_snimace);
  }
  exit(0);
}
