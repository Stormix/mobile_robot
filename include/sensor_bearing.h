
#ifndef SENSOR_BEARING_H
#define SENSOR_BEARING_H

#include <sensor.h>
#include <limits>
#include <algorithm>

using std::endl;
using std::cout;
using namespace arpro;

class BearingSensor : public Sensor
{
  public :
    BearingSensor ( Robot & _robot , double _x , double _y , double _theta) : 
      Sensor ( _robot , _x , _y , _theta ),
    {

    } // the BearingSensor constructor does nothing more

    // update from current sensor pose
    void update(const Pose &_p){
      double angle;
      for(auto other: envir_->getRobots())
        if(other != robot_)
        {
          angle = atan2(other->pose_.y - _p.y,other->pose_.x - _p.x) - _p.theta;
          break;
        }
      s_ = angle % (2*M_PI) - M_PI;
    }

    // correct twist in sensor frame
    void correctTwist(Twist &_v) {
      return;
    }
};

#endif