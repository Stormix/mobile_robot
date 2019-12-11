
#ifndef SENSOR_BEARING_H
#define SENSOR_BEARING_H

#include <sensor.h>
#include <limits>
#include <algorithm>
#include <cmath>

using std::cout;
using std::endl;
using namespace arpro;

class BearingSensor : public Sensor
{
public:
  BearingSensor(Robot &_robot, double _x, double _y, double _theta) : Sensor(_robot, _x, _y, _theta)
  { }

  // update from current sensor pose
  void update(const Pose &_p) override
  {
    double angle;
    auto robots = envir_->getRobots();
    for (auto other : robots){
      if (other != robot_)
      {
        Pose p = other->pose();
        cout << "Robot at ("<< p.x << ", "<< p.y <<") and sensor at ("<< _p.x << ", "<< _p.y <<")" << endl;
        angle = atan2(p.y - _p.y, p.x - _p.x) - _p.theta;
        break;
      }
    }
    // set angle back to [ - pi , pi ]
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;
    s_ = angle;
  }

  // correct twist in sensor frame
  void correctTwist(Twist &_v)
  {
    double g = 0.2;
    _v.w = _v.w - g * s_;
  }
};

#endif