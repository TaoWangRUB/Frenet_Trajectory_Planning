#ifndef FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H
#define FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H

#include "helper/CubicSpline2D.h"
//#include "Obstacle.h"
//#include "Car.h"

#include "Eigen/Dense"
#include <vector>
#include <tuple>

using namespace std;
using namespace Eigen;

class GlobalPath {
public:

    // Euclidean attributes
    vector<double> x;          // x position
    vector<double> y;          // y position
    vector<double> yaw;        // yaw in rad
    vector<double> ds;         // speed
    vector<double> k;          // curvature
    vector<double> s;          // s position along spline

    GlobalPath(shared_ptr<CubicSpline2D> csp);
    bool set_global_path();

private:
    // spline is stored
    shared_ptr<CubicSpline2D> _csp;
};

#endif //FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H
