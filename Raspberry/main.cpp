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
    /*RobotSensors robotSensors = getRobotSensors();
    Callibrate callibrate = getCallibrate();
    printf("acc.pitch:%f,acc.roll:%f,acc.yaw:%f,gy.pitch:%f,gy.roll:%f,gy.yaw:%f,compass.yaw:%f,pitch:%f,roll:%f,yaw:%f\n",
           rad2Deg(robotSensors.MPU6050.accAngle.pitch), rad2Deg(robotSensors.MPU6050.accAngle.roll), rad2Deg(robotSensors.MPU6050.accAngle.yaw),
           rad2Deg(robotSensors.MPU6050.gyAngle.pitch), rad2Deg(robotSensors.MPU6050.gyAngle.roll), rad2Deg(robotSensors.MPU6050.gyAngle.yaw),
           rad2Deg(robotSensors.HMC5883L.yaw),
           rad2Deg(robotSensors.robotPosition.imuAngle.pitch), rad2Deg(robotSensors.robotPosition.imuAngle.roll), rad2Deg(robotSensors.robotPosition.imuAngle.yaw));
    */
    //RobotAcculators robotAcculators = getRobotAcculators();
 /*   robotAcculators.kinect.roll = (rand() % 30) - 15;

    robotAcculators.leds.LedKinect = LED_OFF;
    RobotSensors robotSensors = getRobotSensors();
    printf("%f %f %f %f\n",robotSensors.kinect.accAxis.x,robotSensors.kinect.accAxis.y,robotSensors.kinect.accAxis.z,robotSensors.kinect.accAngle.roll);
    setRobotAcculators(robotAcculators);
    sleep(1);
    robotAcculators.leds.LedKinect = LED_RED;*/
    
    /*robotAcculators.leds.LedUp = COLOR_GREEN;
    robotAcculators.leds.LedMiddle = COLOR_ORANGE;
    robotAcculators.leds.LedDown = COLOR_RED;
    setRobotAcculators(robotAcculators);*/

        RobotSensors robotSensors = getRobotSensors();
    printf("bUp:%d;bMiddle:%d;bDown:%d\n",robotSensors.buttons.buttonUp,robotSensors.buttons.buttonMiddle,robotSensors.buttons.buttonDown);

    sleep(1);
  }
  closeRobot();
  return 0;
}
