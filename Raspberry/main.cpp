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
    /*RobotAcculators robotAcculators = getRobotAcculators();
    RobotSensors robotSensors = getRobotSensors();*/
    /*   robotAcculators.kinect.roll = (rand() % 30) - 15;

       robotAcculators.leds.LedKinect = LED_OFF;
       RobotSensors robotSensors = getRobotSensors();
       printf("%f %f %f %f\n",robotSensors.kinect.accAxis.x,robotSensors.kinect.accAxis.y,robotSensors.kinect.accAxis.z,robotSensors.kinect.accAngle.roll);
       setRobotAcculators(robotAcculators);
       sleep(1);
       robotAcculators.leds.LedKinect = LED_RED;*/
 /*   robotAcculators.robotDirection = FORWARD;
    if (robotSensors.buttons.buttonUp)
      robotAcculators.robotSpeed = 300;
    else if (robotSensors.buttons.buttonMiddle)
      robotAcculators.robotSpeed = 500;
    else if (robotSensors.buttons.buttonDown)
      robotAcculators.robotSpeed = 1000;
    else
      robotAcculators.robotSpeed = 0;
    robotAcculators.leds.LedUp = COLOR_GREEN;
    robotAcculators.leds.LedMiddle = COLOR_ORANGE;
    robotAcculators.leds.LedDown = COLOR_GREEN;
    robotAcculators.kinect.roll=10;
    robotAcculators.ledKinect = LEDKINECT_BLINK_RED_ORANGE;
    setRobotAcculators(robotAcculators);

    printf("x: %f, y: %f, z: %f,roll: %f, pitch: %f, yaw: %f\n",robotSensors.robotPosition.axisPossition.x,robotSensors.robotPosition.axisPossition.y,robotSensors.robotPosition.axisPossition.z,robotSensors.robotPosition.anglePossition.roll*(180/3.14),robotSensors.robotPosition.anglePossition.pitch*(180/3.14),robotSensors.robotPosition.anglePossition.yaw*(180/3.14));
*/
    //printf("bUp:%d;bMiddle:%d;bDown:%d\n", robotSensors.buttons.buttonUp, robotSensors.buttons.buttonMiddle, robotSensors.buttons.buttonDown);

//    printf("acc angle:%f,motorStatus:%d\n",robotSensors.kinect.accAngle.roll,robotSensors.kinect.motorStatus);

    sleep(1);
  }
  closeRobot();
  return 0;
}
