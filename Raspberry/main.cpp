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

int main(void) {
  initRobot();
  while (1) {
    RobotSensors robotSensors = getRobotSensors();
    Callibrate callibrate = getCallibrate();
    printf("acc.pitch:%f,acc.roll:%f,acc.yaw:%f,gy.pitch:%f,gy.roll:%f,gy.yaw:%f,compass.yaw:%f,pitch:%f,roll:%f,yaw:%f\n",
           rad2Deg(robotSensors.MPU6050.accAngle.pitch), rad2Deg(robotSensors.MPU6050.accAngle.roll), rad2Deg(robotSensors.MPU6050.accAngle.yaw),
           rad2Deg(robotSensors.MPU6050.gyAngle.pitch), rad2Deg(robotSensors.MPU6050.gyAngle.roll), rad2Deg(robotSensors.MPU6050.gyAngle.yaw),
           rad2Deg(robotSensors.HMC5883L.yaw),
           rad2Deg(robotSensors.robotPosition.imuAngle.pitch), rad2Deg(robotSensors.robotPosition.imuAngle.roll), rad2Deg(robotSensors.robotPosition.imuAngle.yaw));
    usleep(1000000);
  }
  closeRobot();
  return 0;
}
