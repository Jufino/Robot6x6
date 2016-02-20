	#include <stdio.h>
	#include <stdlib.h>
	#include <string.h>
	#include <errno.h>
	#include <string.h>
	#include <stdio.h>
	#include <stdlib.h>
	#include <unistd.h>
	#include <linux/i2c-dev.h>
	#include <sys/ioctl.h>
	#include <sys/types.h>
	#include <sys/stat.h>
	#include <fcntl.h>
	#include <termios.h>
	#include <inttypes.h>
	#include <unistd.h>	

	#include <opencv2/opencv.hpp>

	#include <netinet/in.h>
	#include <sys/socket.h>
	#include <arpa/inet.h>
	
	extern "C"{
		#include <gpio.h>
		#include "semafor.h"
	}
	int file;
	int lastAddr = 0x00;       
	char *filename = "/dev/i2c-1";
	int serversock_snimace,serversock_camera;
	int clientsock_snimace,clientsock_camera;
	#define Wifi_camera  1
	#define Wifi_snimace 1
	#define PORT_snimace 1213
	#define PORT_camera  1212

	void sigctrl(int param);
	void sigpipe(int param);
	
	using namespace std;
	using namespace cv;

	void initRobot(){
                if ((file = open(filename, O_RDWR)) < 0) {
                        perror("Problem s otvorenim portu.\n");
                        exit(1);
                }
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
			signal(SIGPIPE, sigpipe);
		}
        }
	void closeRobot(){
		gpio_close(17);
                gpio_close(27);
                gpio_close(22);
	}
	void send_img(int socket, Mat img,int kvalita){
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
			case 2:	return !gpio_read(17); break;
			case 3: return !gpio_read(22); break;
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
		if (write(file, data, 1) != 1) 	perror("write error");
		if (read(file, data, 2) != 2)	perror("read error");
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
                if (write(file, data, 1) != 1)	perror("write register");
                if (read(file, data, 1) != 1)	perror("read error");
                return data[0];
        }

	unsigned int rychlost(int poradie){
		switch(poradie){
			case 1:	return readRegister16(0x08,1); break;
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
		if(pozicia+91 < 1) 		writeRegister(0x0A,84,1);
		else if(pozicia+91 > 181) 	writeRegister(0x0A,84,181);
		else				writeRegister(0x0A,84,pozicia+91); 
	}
	unsigned int ultrazvuk(){
                return readRegister16(0x0A,6);
        }
	int napetieRaw(){
		return readRegister16(0x08,5);
	}
	#define R2 5.3L
	#define R1 31.4L
	#define maxVolt 5L
	#define rozlisenieADC 1023L
	float napetieVolt(){
		float napHod = (float)napetieRaw()*(maxVolt/rozlisenieADC)*((R1+R2)/R2);
		return napHod;
	}
	#define maxNapetie 25.2L
	#define minNapetie 22.8L
	float napetiePercent(){
		float napHod = (float)napetieVolt()*(100/(maxNapetie-minNapetie))-(100/(maxNapetie-minNapetie))*minNapetie;
		return napHod;
	}
	unsigned int prud(){
                return readRegister16(0x08,6);
        }
	void Led(int poradie,char nazov,bool stav){
		if(poradie == 3){
			if(nazov == 'Z'){ 
				if(stav == false)	writeRegister(0x08,97,0);
				else			writeRegister(0x08,97,1);
			}
			else if(nazov == 'C'){
				if(stav == false)       writeRegister(0x08,96,0);
                        	else                    writeRegister(0x08,96,1);
			}
			else{
				if(stav == false){ 
					Led(1,'Z',0);
					Led(1,'C',0);
				}
                        	else{ 
                  			Led(1,'Z',1);
                                	Led(1,'C',1);
				}
			}
		}
		else if(poradie == 1){
			if(nazov == 'Z'){
                                if(stav == false)       writeRegister(0x09,97,0);
                                else                    writeRegister(0x09,97,1);
                        }
                        else if(nazov == 'C'){
                                if(stav == false)       writeRegister(0x09,96,0);
                                else                    writeRegister(0x09,96,1);
                        }
                        else{
                                if(stav == false){
                                        Led(2,'Z',0);
                                        Led(2,'C',0);
                                }
                                else{
                                        Led(2,'Z',1);
                                        Led(2,'C',1);
                                }
                        }
		}
		else{
			if(nazov == 'Z'){
                                if(stav == false)       writeRegister(0x0A,97,0);
                                else                    writeRegister(0x0A,97,1);
                        }
                        else if(nazov == 'C'){
                                if(stav == false)       writeRegister(0x0A,96,0);
                                else                    writeRegister(0x0A,96,1);
                        }
                        else{
                                if(stav == false){
                                        Led(3,'Z',0);
                                        Led(3,'C',0);
                                }
                                else{
                                        Led(3,'Z',1);
                                        Led(3,'C',1);
                               } 
                        }
		}
	}
	void napajanie(bool stav){
		if(stav == false)       writeRegister(0x08,95,0);
                else                    writeRegister(0x08,95,1);
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
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}
	int main(void){
		initRobot();
		writeRegister(0x68,0x6B,0);	
	//	napajanie(true);
/*		motor(1,1,255,false);
		motor(2,1,255,false);
		motor(3,1,255,false);
		motor(4,1,255,false);
		motor(5,1,255,false);
		motor(6,1,255,false);
		usleep(2000000);*/
		motor(1,0,255,false);
                motor(2,0,255,false);
                motor(3,0,255,false);
                motor(4,0,255,false);
                motor(5,0,255,false);
                motor(6,0,255,false);
		//motor(3,1,255,false);
		//Led(3,'C',true);
	//	servo(-70);
		char lastznak='0';
		char znak='0';
		int pocet = 0;
		Led(3,'Z',1);	
		//printf("%f,%f\n",napetie(),napetiePercenta());
		while(1){ 
		//	printf("%d,%d,%d\n", tlacitka(1),tlacitka(2),tlacitka(3));
			printf("%d,%f,%f\n",napetieRaw(),napetieVolt,napetiePercent());
			usleep(20000);
	/*		if(kbhit() == 1){
				znak = getchar();
				if(znak != lastznak){
				switch(znak){
					case 119:
						motor(1,1,255,false);
                				motor(2,1,255,false);
                				motor(3,1,255,false);
                				motor(4,1,255,false);
                				motor(5,1,255,false);
                				motor(6,1,255,false);
						usleep(40000);
					break;
					case 115:
                                                motor(1,-1,255,false);
                                                motor(2,-1,255,false);
                                                motor(3,-1,255,false);
                                                motor(4,-1,255,false);
                                                motor(5,-1,255,false);
                                                motor(6,-1,255,false);
						usleep(40000);
                                        break;
					case 97:
                                                motor(1,1,255,false);
                                                motor(2,1,255,false);
                                                motor(3,1,255,false);
                                                motor(4,-1,255,false);
                                                motor(5,-1,255,false);
                                                motor(6,-1,255,false);
						usleep(40000);
                                        break;
					case 100:
                                                motor(1,-1,255,false);
                                                motor(2,-1,255,false);
                                                motor(3,-1,255,false);
                                                motor(4,1,255,false);
                                                motor(5,1,255,false);
                                                motor(6,1,255,false);
						usleep(40000);
                                        break;
				}
					lastznak = znak;
					printf(" - %d \n",znak);
				}
				pocet = 0;	
			}
			else{
				if(pocet == 10){
				lastznak = '0';
				motor(1,0,255,false);
                                motor(2,0,255,false);
                                motor(3,0,255,false);
                                motor(4,0,255,false);
                                motor(5,0,255,false);
                                motor(6,0,255,false);
				usleep(40000);
				pocet = 0;
				}
				else{
					pocet++;
					usleep(5000);
				}
			}*/
}
/*			        signed int accX = readRegister16(0x68,0x3B);
        signed int accY = readRegisters16(0x68,0x3D);
        signed int accZ = readRegisters16(0x68,0x3F);
        signed int Tmp = readRegisters16(0x68,0x41);
        signed int GyX = readRegisters16(0x68,0x43);
        signed int GyY = readRegisters16(0x68,0x45);
        signed int GyZ = readRegisters16(0x68,0x47);

			printf("%d, %d, %d, %f, %d, %d, %d\n",accX,accY,accZ,Tmp/360+36.53,GyX,GyY,GyZ);
			usleep(100000);
		}*/
		gpio_close(22);
		return 0;
}
void sigctrl(int param){
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
void sigpipe(int param){
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

