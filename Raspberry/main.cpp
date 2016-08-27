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

void sigctrl(int param);
void sigpipe(int param);

int main(void){
	int imageChooseMainL = 0;

  initRobot();
  signal(SIGINT, sigctrl);
	signal(SIGPIPE, sigpipe);
  
	while(1){

/*	 	RobotSensors robotSensors = getRobotSensors();
		printf("x:%f, y:%f, angle:%f\n",
                        robotSensors.robotPosition.x,
			robotSensors.robotPosition.y,
			robotSensors.robotPosition.angleDeg);
		printf("HMC.X:%f,HMC.Y:%f,HMC.Z:%f,angle rad:%f,angle deg:%f\n",robotSensors.HMC5883L.X,robotSensors.HMC5883L.Y,robotSensors.HMC5883L.Z,robotSensors.HMC5883L.angleRad,robotSensors.HMC5883L.angleDeg);
*/		usleep(100000);
/*


                        if(Wifi_camera == 1) wifiCamera();
                }*/
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

