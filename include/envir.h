#ifndef ENVIR_H
#define ENVIR_H

#include <vector>
#include <geom.h>

namespace arpro
{


class Robot;

class Environment
{

    Pose target_;
    std::vector<double> x_hist, y_hist;
    std::vector<Robot*> robots_;

    double dt = 0.1;
    double t = 0;
    std::vector<Pose> walls;

public:

    inline const std::vector<Pose>& getWalls() const {return walls;}
    inline const std::vector<Robot*>& getRobots() const {return robots_;}
    Environment();

    double time() const
    {
      return t;
    }

    // the target draws a cardoid curve
    void updateTarget();

    Pose target() const
    {
      return target_;
    }

    void addRobot(Robot &_robot);

    // plots the trajectory in the given environment
    void plot();
};

}

#endif // ENVIR_H
