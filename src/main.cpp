#include <iostream>
#include <math.h>
#include <cmath>

#include <robot.h>
#include <envir.h>
#include <sensor.h>
#include <sensor_range.h>
#include <sensor_bearing.h>

using namespace std;
using namespace arpro;

int main(int argc, char **argv)
{

  // default environment with moving target
  Environment envir;
  // sensors gets measurements from this environment
  Sensor::setEnvironment(envir);
  
  double r = 0.07;
  double r2 = 0.05;
  double e = 0.3;
  double maxWheelSpeed = 10;
  double minRange = 0.1;

  // init robot at (0,0,0)
  Robot robot("R2D2", 0,0,0);
  Robot robot2("R2D2 - Small", 0,0,0);
  robot.initWheel(r, e, maxWheelSpeed);
  robot2.initWheel(r2, e, maxWheelSpeed);

  envir.addRobot(robot);
  envir.addRobot(robot2);

  RangeSensor rSensor(robot,0.1,0,0,minRange);
  // RangeSensor rSensor2(robot2,0.1,0,0,minRange);
  BearingSensor bSensor(robot2,0.1,0,0);

  // simulate 100 sec
  while(envir.time() < 100)
  {
    cout << "---------------------" << endl;

    // update target position
    envir.updateTarget();

    // try to follow target
    robot.goTo(envir.target());
    robot2.moveWithSensor(Twist(0.4,0,0));

  }

  // plot trajectory
  envir.plot();

}
