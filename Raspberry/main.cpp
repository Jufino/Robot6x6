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
	setMotorPowerSupply(true);
	setMotor(1,0,255,false);
        setMotor(2,0,255,false);
        setMotor(3,0,255,false);
        setMotor(4,0,255,false);
        setMotor(5,1,255,true);
        setMotor(6,0,255,false);	
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
	MPU6050WakeUp();
	setMPU6050Sensitivity(1,1);
	setMPU6050DLPF(6,6);
	MPU6050CalibrateOffset(20);
	while(1){
		RobotVariables robot = getRobotVariables();
		/*printf("AcX: %f\n",robot.MPU6050.AccX);
		printf("AcY: %f\n",robot.MPU6050.AccY);
		printf("AcZ: %f\n",robot.MPU6050.AccZ);
		printf("TMP: %f\n",robot.MPU6050.Temp);
		printf("GyX: %f\n",robot.MPU6050.GyX);
		printf("GyY: %f\n",robot.MPU6050.GyY);
		printf("GyZ: %f\n\n",robot.MPU6050.GyZ);
		printf("Roll: %f\n",robot.MPU6050.Roll);
                printf("Pitch: %f\n",robot.MPU6050.Pitch);
                printf("Yaw: %f\n",robot.MPU6050.Yaw);
		printf("Voltage: %f\n",robot.voltage);
		printf("Ultrasonic: %f\n\n",robot.ultrasonic);*/
		printf("x: %f\n",robot.robotPosition.x);
		printf("y: %f\n",robot.robotPosition.y);
		printf("uhol: %f\n\n",robot.robotPosition.angle);
		//printf("dist5: %d\n",robot.motors.motor5.distance);

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
		usleep(10000);
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
		sendMatImage(imgSendL,80);
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

