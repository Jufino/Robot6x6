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
#include <math.h>


#define casJednehoCyklu 5000 //[us]
#define casZmenyUhla 150 //[ms]
#define casKolkoMaCakatPredPrvymOtocenimPoStrateOperatora 100 //[ms]

int main(void) {
  //---------------------------------------------
  initRobot(); //inilizacia robota
  //---------------------------------------------
  RobotAcculators robotAcculators = getRobotAcculators();
  robotAcculators.kinect.roll = 20 * (M_PI / 180); //nastavi naklonenie kinectu vertikalne na nejaky uhol

  setRobotAcculators(robotAcculators);
  sleep(10);
  robotAcculators = getRobotAcculators();
  robotAcculators.direction = STOP;
  robotAcculators.speed = 0;
  robotAcculators.isManual = true;
  setRobotAcculators(robotAcculators);
  //---------------------------------------------
  //pomocne premenne
  int lastOperatorDirectionLocal = 0;
  long numberCycleLostOperator = 0;
  long numberCycleChangeAngle = 0;
  bool changeRotateKinect = false;
  //---------------------------------------------
  while (1) {
    //---------------------------------------------
    RobotAcculators robotAcculators = getRobotAcculators(); //nacita posledne vlozene akcne zasahy
    RobotSensors robotSensors = getRobotSensors(); //nacita posledne hodnoty senzorov
    //---------------------------------------------
    //otacanie sa kinectu za operatorom
    if ( robotSensors.operatorPossition.x == -1 &&
         robotSensors.operatorPossition.y == -1 &&
         robotSensors.operatorPossition.z == -1) {

      if (numberCycleChangeAngle > casZmenyUhla / (casJednehoCyklu / 1000)) {
        if (numberCycleLostOperator > casKolkoMaCakatPredPrvymOtocenimPoStrateOperatora / (casJednehoCyklu / 1000)) { //caka nejaky cas pred prvym otocenim
          //---------------------------------------------
          if (lastOperatorDirectionLocal < 0)
            robotAcculators.kinect.yaw -= 5 * (M_PI / 180);
          else
            robotAcculators.kinect.yaw += 5 * (M_PI / 180);
          //---------------------------------------------
          if (robotAcculators.kinect.yaw > M_PI / 2) {
            changeRotateKinect = true;
            lastOperatorDirectionLocal = -1;
          }
          else if (robotAcculators.kinect.yaw < -M_PI / 2) {
            changeRotateKinect = true;
            lastOperatorDirectionLocal = 1;
          }
          //---------------------------------------------
          numberCycleChangeAngle = 0;
        }
        else {
          numberCycleLostOperator++;
        }
      }
      else {
        numberCycleChangeAngle++;
      }
    }
    else {
      lastOperatorDirectionLocal = robotSensors.lastOperatorDirection;
      numberCycleLostOperator = 0;
    }
    //---------------------------------------------
    //chod na poziciu
    //sem pride kod, ktory posle robota na poziciu
    //---------------------------------------------
    vector<Point> bodyPohybu = getJorneyPoints();

    Point bodNaKtorySaChcemeDostatNaMape(-1, -1);

    if (bodyPohybu.size() > 1)
      bodNaKtorySaChcemeDostatNaMape = bodyPohybu.at(1);

    if (bodNaKtorySaChcemeDostatNaMape.x != -1 && bodNaKtorySaChcemeDostatNaMape.y != -1) {
      robotAcculators.distance = sqrt(pow(bodNaKtorySaChcemeDostatNaMape.x - MAP_WIDTH / 2, 2) + pow(bodNaKtorySaChcemeDostatNaMape.y - MAP_HEIGHT / 2, 2));
      robotAcculators.angle = acos(((double)(bodNaKtorySaChcemeDostatNaMape.x - MAP_WIDTH / 2)) / robotAcculators.distance);
      robotAcculators.isManual = false;
      printf("%f\n",(robotSensors.robotPosition.anglePossition.yaw-robotAcculators.angle)*(180/M_PI));
    }
    else {
      //nemame cestu - pri zmene otacania kinect otoc robota o 90 stupnov v poslednom znamom smere
      if (changeRotateKinect == true) {
        if (robotSensors.lastOperatorDirection < 0)
          robotAcculators.angle = robotSensors.robotPosition.anglePossition.yaw - 90 * (M_PI / 180);
        else
          robotAcculators.angle = robotSensors.robotPosition.anglePossition.yaw + 90 * (M_PI / 180);
        robotAcculators.distance = 0;
        robotAcculators.isManual = false;
        changeRotateKinect = false;
      }
    }

    setRobotAcculators(robotAcculators);
    usleep(casJednehoCyklu); //cas jedneho cyklu
  }
  closeRobot();
  return 0;
}
