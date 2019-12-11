#include <iostream>
#include <math.h>
#include <robot.h>
#include <sensor.h>
#include <algorithm>

using namespace arpro;
using namespace std;

Environment* Sensor::envir_ = nullptr;

Robot::Robot(string _name, double _x, double _y, double _theta)
{    
    pose_.x = _x;
    pose_.y = _y;
    pose_.theta = _theta;

    name_ = _name;

    // init position history
    x_history_.push_back(_x);
    y_history_.push_back(_y);
}


void Robot::moveXYT(double _vx, double _vy, double _omega)
{
    // update position
    pose_.x += _vx*dt_;
    pose_.y += _vy*dt_;
    pose_.theta += _omega*dt_;

    // store position history
    x_history_.push_back(pose_.x);
    y_history_.push_back(pose_.y);
}

void Robot::rotateWheels(double _left, double _right)
{
    if(!wheels_init_)
        return;

    double a = max(abs(_left)/maxWheelSpeed,abs(_right)/maxWheelSpeed);
    a = a < 1 ? 1 : a;
    if(a == 1){
      cout << "Velocity setpoint is too high" << endl;
    }
    _left = _left/a;
    _right = _right/a;

    double v = wheelRadius * (_left + _right) / 2;
    double w = wheelRadius * (_left - _right) / (2*baseDistance);
    double _vx = v * cos(pose_.theta);
    double _vy= v * sin(pose_.theta);
    moveXYT(_vx, _vy, w);
}

// move robot with linear and angular velocities
void Robot::moveVW(double _v, double _omega)
{
    double _left = (_v + baseDistance * _omega)/wheelRadius;
    double _right = (_v - baseDistance * _omega)/wheelRadius;
    rotateWheels(_left, _right);
}


// try to go to a given x-y position
void Robot::goTo(const Pose &_p)
{
    // error in robot frame
    Pose error = _p.transformInverse(pose_);

    // try to do a straight line with sensor constraints
    moveWithSensor(Twist(error.x, error.y, 0));
}


void Robot::moveWithSensor(Twist _twist)
{
    
    for(auto& sensor : sensors_)
    {
        sensor->updateFromRobotPose(pose_);        
        sensor->correctRobotTwist(_twist);
    }
    
    double v = _twist.vx;
    double w = 20 *_twist.vy + _twist.w;
    
    moveVW(v,w);
}


void Robot::printPosition()
{
    cout << "Current position: " << pose_.x << ", " << pose_.y << endl;
}

