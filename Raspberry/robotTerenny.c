#include "robotTerenny.h"

int file;
int lastAddr = 0x00;
int serversock_snimace,serversock_camera;
int clientsock_snimace,clientsock_camera;
int portHandle;
int getSocketCamera(){
	return clientsock_camera;
}
int getSocketSnimace(){
	return clientsock_snimace;
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
void closeRobot(){
    SerialClose(portHandle);
    gpio_close(17);
    gpio_close(27);
    gpio_close(22);
    if(Wifi_camera == 1){
    	close(serversock_camera);
    	close(clientsock_camera);
    }
    if(Wifi_snimace == 1){
    	close(serversock_snimace);
    	close(clientsock_snimace);
    }
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
signed int readRegister16s(int addr,unsigned char reg){
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

void odosliMat(Mat img,int kvalita){
    vector<uchar> buff;
    vector<int> param = vector<int>(2);
    param[0] = CV_IMWRITE_JPEG_QUALITY;
    param[1] = kvalita;
    imencode(".jpg", img, buff, param);
    char len[10];
    sprintf(len, "%.8d", buff.size());
    send(clientsock_camera, len, strlen(len), 0);
    send(clientsock_camera, &buff[0],buff.size(), 0);
    buff.clear();
}
unsigned char getTlacitka(char pozicia){
    switch(pozicia){
        case 1: return !gpio_read(27); break;
        case 2: return !gpio_read(17); break;
        case 3: return !gpio_read(22); break;
	default: return 0;
    }
}
unsigned int getRychlost(int poradie){
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
unsigned int getVzdialenost(int poradie){
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
	MPU6050.AccX = (float)readRegister16s(MPU6050ADDR,0x3B)/AccScale-AccX_offset;
        MPU6050.AccY = (float)readRegister16s(MPU6050ADDR,0x3D)/AccScale-AccY_offset;
        MPU6050.AccZ = (float)readRegister16s(MPU6050ADDR,0x3F)/AccScale-AccZ_offset;
        MPU6050.Temp = (float)readRegister16s(MPU6050ADDR,0x41)/340+36.53;
        MPU6050.GyX  = (float)readRegister16s(MPU6050ADDR,0x43)/GyScale-GyX_offset;
        MPU6050.GyY  = (float)readRegister16s(MPU6050ADDR,0x45)/GyScale-GyY_offset;
        MPU6050.GyZ  = (float)readRegister16s(MPU6050ADDR,0x47)/GyScale-GyZ_offset;
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
	AccZ_offset = acc_z/pocet;
	
	GyX_offset = acc_x/pocet;
	GyY_offset = acc_y/pocet;
	GyZ_offset = acc_z/pocet;
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
	if(forceMagnitudeApprox >8192 && forceMagnitudeApprox < 32768){
       	 	pitchAcc = atan2f(MPU6050.AccY, dist(MPU6050.AccZ,MPU6050.AccX)) * 180 / M_PI;	
        	MPU6050.Pitch = MPU6050.Pitch * 0.98 + pitchAcc * 0.02;

        	rollAcc = atan2f(MPU6050.AccX, dist(MPU6050.AccZ,MPU6050.AccY)) * 180 / M_PI;
        	MPU6050.Roll = MPU6050.Roll * 0.98 + rollAcc * 0.02;
		
		yawAcc = atan2f(dist(MPU6050.AccX,MPU6050.AccY),MPU6050.AccZ) * 180 / M_PI;
                MPU6050.Yaw = MPU6050.Yaw * 0.98 + yawAcc * 0.02;
	}
	return MPU6050;
} 

void setMPU6050Sensitivity(int acc_sens,int gy_sens){
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
void setMPU6050DLPF(int acc_dlpf,int gy_dlpf){
	writeRegister(MPU6050ADDR,0x1A,gy_dlpf|(2<<3));
        writeRegister(MPU6050ADDR,0x1A,gy_dlpf|(3<<3));
        writeRegister(MPU6050ADDR,0x1A,gy_dlpf|(4<<3));

	writeRegister(MPU6050ADDR,0x1A,acc_dlpf|(5<<3));
        writeRegister(MPU6050ADDR,0x1A,acc_dlpf|(6<<3));
        writeRegister(MPU6050ADDR,0x1A,acc_dlpf|(7<<3));
}
void setServo(int uhol){
    if(uhol+91 < 1) writeRegister(0x0A,84,1);
    else if(uhol+91 > 181) writeRegister(0x0A,84,181);
    else writeRegister(0x0A,84,uhol+91);
}
unsigned int getUltrazvukRaw(){
    return readRegister16(0x0A,6);
}
float getUltrazvukMeter(){
	return (float)readRegister16(0x0A,6)*(rychlostZvuku/2);
}
int getNapetieRaw(){
    return readRegister16(0x08,5);
}
float getNapetieVolt(){
    float napHod = (float)getNapetieRaw()*(maxVoltADC/rozlisenieADC)*((R1+R2)/R2);
    return napHod;
}
float getNapetiePercent(){
    float napHod = (float)getNapetieVolt()*(100/(maxNapetie-minNapetie))-(100/(maxNapetie-minNapetie))*minNapetie;
    return napHod;
}
int getPrudRaw(){
    return readRegister16(0x08,6);
}
float getPrudVolt(){
    return (float)getPrudRaw()*(maxVoltADC/rozlisenieADC);
}
float getPrudAmp(){
    return ((float)getPrudVolt()-maxVoltADC/2)/rozliseniePrud;
}

void setLed(int poradie,char nazov,bool stav){
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
                setLed(1,'Z',0);
                setLed(1,'C',0);
            }
            else{
                setLed(1,'Z',1);
                setLed(1,'C',1);
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
                setLed(2,'Z',0);
                setLed(2,'C',0);
            }
            else{
                setLed(2,'Z',1);
                setLed(2,'C',1);
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
                setLed(3,'Z',0);
                setLed(3,'C',0);
            }
            else{
                setLed(3,'Z',1);
                setLed(3,'C',1);
            }
        }
    }
}
void setNapajanie(bool stav){
    if(stav == false)   writeRegister(0x08,95,0);
    else                writeRegister(0x08,95,1);
}
void setMotor(int poradie,signed char smer,unsigned char rychlost,bool reg){
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
