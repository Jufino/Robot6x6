#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <inttypes.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <robotTerenny.h>
extern "C"{
	#include "semafor.h"
}

#define vyska 240
#define sirka 320
CvCapture* cameraL;
char imageChooseL=0;
IplImage *img1L;
IplImage *img2L;
Mat imgSendL;
void *getImgL(void *arg);

int sem_id=0;
char zap = true;

void sigctrl(int param);
void sigpipe(int param);
void wifiCamera();
int main(void){
	int imageChooseMainL = 0;

    	initRobot();
	napajanie(true);
	motor(1,0,255,false);
        motor(2,0,255,false);
        motor(3,0,255,false);
        motor(4,0,255,false);
        motor(5,0,255,false);
        motor(6,0,255,false);
	
        cameraL = cvCaptureFromCAM(0);

        cvSetCaptureProperty( cameraL, CV_CAP_PROP_FRAME_WIDTH, sirka);
        cvSetCaptureProperty( cameraL, CV_CAP_PROP_FRAME_HEIGHT, vyska);
	sem_id = semCreate(getpid(),3);   //vytvor semafor
        semInit(sem_id,0,1);
        semInit(sem_id,1,1);
	semInit(sem_id,2,1);
        signal(SIGINT, sigctrl);
	signal(SIGPIPE, sigpipe);
	
        pthread_t vlaknoImgL;
        pthread_create(&vlaknoImgL,NULL,&getImgL,NULL);

	writeRegister(0x68,0x6B,0);
	while(1){
		printf("AcX: %d\n",readRegister16s(0x68,0x3B));
		printf("AcY: %d\n",readRegister16s(0x68,0x3D));
		printf("AcZ: %d\n",readRegister16s(0x68,0x3F));
		printf("TMP: %f\n",(float)readRegister16s(0x68,0x41)/360+36.53);
		printf("GyX: %d\n",readRegister16s(0x68,0x43));
		printf("GyY: %d\n",readRegister16s(0x68,0x45));
		printf("GyZ: %d\n\n",readRegister16s(0x68,0x47));
/*
                semWait(sem_id,0);
                imageChooseMainL = imageChooseL;
                semPost(sem_id,0);
		if(imageChooseMainL !=0){
                        if(imageChooseMainL == 1){        
				semWait(sem_id,1);
				cvarrToMat(img1L).copyTo(imgSendL);
				semPost(sem_id,1);
			}
                        else if(imageChooseMainL == 2){   
				semWait(sem_id,2);
				cvarrToMat(img2L).copyTo(imgSendL);
				semPost(sem_id,2);
			}
                        if(Wifi_camera == 1) wifiCamera();
                }

*/
		//printf("napetie:%d,%f,%f\nprud: %d,%f,%f\n",napetieRaw(),napetieVolt,napetiePercent(),prudRaw(),prudVolt(),prudAmp());
		usleep(333000);
	}
        closeRobot();
	return 0;
}
void *getImgL(void *arg){
        while(zap){
		semWait(sem_id,1);
                img1L = cvQueryFrame(cameraL);
		semWait(sem_id,0);
                imageChooseL = 1;
                semPost(sem_id,0);
		semPost(sem_id,1);

		semWait(sem_id,2);	
                img2L = cvQueryFrame(cameraL);
		semWait(sem_id,0);
                imageChooseL = 2;
                semPost(sem_id,0);
                semPost(sem_id,2);
        }
}
void wifiCamera(){
        char recvdata[30];
        int bytes = recv(getSocketCamera(), recvdata, 4, 0);
        if (bytes == 0){
		sigctrl(0);
        }
        if (strcmp(recvdata, "img\n") == 0){ //&& imgSendL.empty() != true){
		odosliMat(imgSendL,80);
        }
}
void sigctrl(int param){
	closeRobot();
  	semRem(sem_id);
  	exit(param);
}
void sigpipe(int param){
  	closeRobot();
  	semRem(sem_id);
  	exit(param);
}

