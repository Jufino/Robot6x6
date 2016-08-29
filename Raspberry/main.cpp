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

extern "C" {
  #include "semafor.h"
}

void sigctrl(int param);
void sigpipe(int param);

int main(void){
  initRobot();
//  signal(SIGINT, sigctrl);
  //signal(SIGPIPE, sigpipe);
 
  while(1){
	RobotSensors robotSensors = getRobotSensors();
    	Callibrate callibrate = getCallibrate();
//	printf("HMC.X:%f,HMC.Y:%f,HMC.Z:%f,angle rad:%f,angle deg:%f",robotSensors.HMC5883L.compassAxis.x,robotSensors.HMC5883L.compassAxis.y,robotSensors.HMC5883L.compassAxis.z,robotSensors.HMC5883L.angle.radian,robotSensors.HMC5883L.angle.degree);
//	printf(" calibX%f,calibY:%f\n",callibrate.HMC5883LOffsetAxis.x,callibrate.HMC5883LOffsetAxis.y); 
	printf("Acc.X:%f,Acc.Y:%f,Acc.Z:%f,Gy.X:%f,Gy.Y:%f,Gy.Z:%f\n",robotSensors.MPU6050.accAxis.x,robotSensors.MPU6050.accAxis.y,robotSensors.MPU6050.accAxis.z,
									robotSensors.MPU6050.gyAxis.x,robotSensors.MPU6050.gyAxis.y,robotSensors.MPU6050.gyAxis.z);

  	usleep(100000);
   }
   closeRobot();
   return 0;
}

void sigctrl(int param){
	closeRobot();
  	exit(param);
}
void sigpipe(int param){
  	closeRobot();
  	exit(param);
}

