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

int sem_id1=0;
char zap = true;

void sigctrl(int param);
void sigpipe(int param);
void wifiCamera();
int main(void){
	int imageChooseMainL = 0;

    	initRobot();
//	setMotorPowerSupply(true);
		
        cameraL = cvCaptureFromCAM(0);

        cvSetCaptureProperty( cameraL, CV_CAP_PROP_FRAME_WIDTH, sirka);
        cvSetCaptureProperty( cameraL, CV_CAP_PROP_FRAME_HEIGHT, vyska);
	sem_id1 = semCreate(getpid(),3);   //vytvor semafor
        semInit(sem_id1,0,1);
        semInit(sem_id1,1,1);
	semInit(sem_id1,2,1);
        signal(SIGINT, sigctrl);
	signal(SIGPIPE, sigpipe);
	
        pthread_t vlaknoImgL;
        pthread_create(&vlaknoImgL,NULL,&getImgL,NULL);
/*	MPU6050WakeUp();
	setMPU6050Sensitivity(1,1);
	setMPU6050DLPF(6,6);
	MPU6050CalibrateOffset(20);
*/	while(1){
		//RobotAcculators robot = getRobotAcculators();
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
	//	setMove('F',0,false);

/*	 	RobotSensors robotSensors = getRobotSensors();
		printf("dist1Raw:%d -> dist1: %f,dist2Raw:%d -> dist2: %f,dist3Raw:%d -> dist3 %f => distR:%f\n",
			robotSensors.motors.motor1.distanceRaw,robotSensors.motors.motor1.distance,
			robotSensors.motors.motor2.distanceRaw,robotSensors.motors.motor2.distance,
			robotSensors.motors.motor3.distanceRaw,robotSensors.motors.motor3.distance,robotSensors.robotPosition.distanceR);
		printf("dist4Raw:%d -> dist4: %f,dist5Raw:%d -> dist5: %f,dist6Raw:%d -> dist6 %f => distL:%f\n",
                        robotSensors.motors.motor4.distanceRaw,robotSensors.motors.motor4.distance,
                        robotSensors.motors.motor5.distanceRaw,robotSensors.motors.motor5.distance,
                        robotSensors.motors.motor6.distanceRaw,robotSensors.motors.motor6.distance,robotSensors.robotPosition.distanceL);
		printf("x:%f, y:%f, angle:%f\n",
                        robotSensors.robotPosition.x,
			robotSensors.robotPosition.y,
			robotSensors.robotPosition.angleDeg);
		printf("HMC.X:%f,HMC.Y:%f,HMC.Z:%f,angle rad:%f,angle deg:%f\n",robotSensors.HMC5883L.X,robotSensors.HMC5883L.Y,robotSensors.HMC5883L.Z,robotSensors.HMC5883L.angleRad,robotSensors.HMC5883L.angleDeg);
*/		usleep(100000);

		//printf("Voltage: %f\n",robot.voltagePercent);

		//printf("dist5: %d\n",robot.motors.motor5.distance);
//		RobotAcculators robotAcculators = getRobotAcculators();
//		setRobotAcculators(robotAcculators);
//		setMove('F',255,false);

                semWait(sem_id1,0);
                imageChooseMainL = imageChooseL;
                semPost(sem_id1,0);
		if(imageChooseMainL !=0){
                        if(imageChooseMainL == 1){        
				semWait(sem_id1,1);
				cvarrToMat(img1L).copyTo(imgSendL);
				semPost(sem_id1,1);
			}
                        else if(imageChooseMainL == 2){   
				semWait(sem_id1,2);
				cvarrToMat(img2L).copyTo(imgSendL);
				semPost(sem_id1,2);
			}
                        if(Wifi_camera == 1) wifiCamera();
                }
		//printf("napetie:%d,%f,%f\nprud: %d,%f,%f\n",napetieRaw(),napetieVolt,napetiePercent(),prudRaw(),prudVolt(),prudAmp());
//		usleep(10000);
	}
        closeRobot();
	return 0;
}
void *getImgL(void *arg){
        while(zap){
		semWait(sem_id1,1);
                img1L = cvQueryFrame(cameraL);
		semWait(sem_id1,0);
                imageChooseL = 1;
                semPost(sem_id1,0);
		semPost(sem_id1,1);

		semWait(sem_id1,2);	
                img2L = cvQueryFrame(cameraL);
		semWait(sem_id1,0);
                imageChooseL = 2;
                semPost(sem_id1,0);
                semPost(sem_id1,2);
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
  	semRem(sem_id1);
  	exit(param);
}
void sigpipe(int param){
  	closeRobot();
  	semRem(sem_id1);
  	exit(param);
}

