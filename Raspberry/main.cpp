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
      RobotAcculators robotAcculators = getRobotAcculators();
          robotAcculators.kinect.roll = 10*(M_PI/180);
          setRobotAcculators(robotAcculators);
  while (1) {


    RobotSensors robotSensors = getRobotSensors();

    printf("%f\n",robotSensors.robotPosition.anglePossition.yaw*(180/M_PI));
    //robotAcculators.leds.LedMiddle = COLOR_ORANGE;
    //robotAcculators.ledKinect = LEDKINECT_RED;
//robotAcculators.kinect.yaw = 90*(M_PI/180);
  //     setRobotAcculators(robotAcculators);
    //printf("%f;%f\n",robotSensors.voltage.volts ,robotSensors.voltage.capacityPercent);*/
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

    sleep(0.2);
  }
  closeRobot();
  return 0;
}
