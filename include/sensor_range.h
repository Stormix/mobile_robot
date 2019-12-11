
#ifndef SENSOR_RANGE_H
#define SENSOR_RANGE_H

#include <sensor.h>
#include <limits>
#include <algorithm>

using std::endl;
using std::cout;
using namespace arpro;

class RangeSensor : public Sensor
{
  public :
    RangeSensor ( Robot & _robot , double _x , double _y , double _theta, double minRange) : 
      Sensor ( _robot , _x , _y , _theta ),
      minRange(minRange) // call the Sensor constructor
    {

    } // the RangeSensor constructor does nothing more

    // update from current sensor pose
    void update(const Pose &_p){
      Pose p1, p2;
      std::vector<double> distances;
      // TODO: use auto for getWAlls()
      for (int i = 0; i < envir_->getWalls().size(); i++)
      {
        p1 = envir_->getWalls()[i];
        p2 = envir_->getWalls()[(i+1)%envir_->getWalls().size()];

        //TODO move to a separate function
        double nom = (p1.x * p2.y - p1.x*_p.y - p2.x*p1.y + p2.x*_p.y + _p.x*p1.y-_p.x*p2.y);
        double denom = (p1.x*sin(_p.theta) - p2.x*sin(_p.theta) - p1.y*cos(_p.theta) + p2.y*cos(_p.theta));
        double d = denom != 0 ? nom/denom :-1;
        if(d > 0){
          distances.push_back(d); 
        }
      }
      cout << endl;
      s_ = *std::min_element(distances.begin(), distances.end());
      cout << "Nearest wall is " << s_ << "m away" << endl;
    }

    // correct twist in sensor frame
    void correctTwist(Twist &_v) {
      double g = .3;
      if ( _v.vx > g * (s_ - minRange) ){
        _v.vx = g * (s_ - minRange);
      }
    }
  
  private:
    double minRange;
};

#endif