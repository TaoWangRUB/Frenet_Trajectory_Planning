#ifndef FRENET_H
#define FRENET_H
#include "helper/CubicSpline2D.h"
#include <vector>
using namespace std;
struct TRAJECTORY{
    double dt{0.1};
    // course time
    vector<double> t;
    // course position
    vector<double> s;
    // course velocity
    vector<double> s_d;
    // course acc
    vector<double> s_dd;
    // course jerk
    vector<double> s_ddd;
    // lateral position
    vector<double> d;
    // lateral velocity
    vector<double> d_d;
    // lateral acc
    vector<double> d_dd;
    // lateral jerk
    vector<double> d_ddd;

    // cartisian
    vector<double> x;          // x position
    vector<double> y;          // y position
    vector<double> yaw;        // yaw in rad
    vector<double> ds;         // speed
    vector<double> c;          // curvature

    // cost
    double cd{0.}; // lateral cost
    double cv{0.}; // velocity cost
    double cf{0.}; // total cost
};

class FRENET
{
public:
    FRENET(shared_ptr<CubicSpline2D>);
    void get_frenet_traj(double s0, double s_d, double d, double d_d, double d_dd);
    bool get_global_path(TRAJECTORY& traj);
    void get_cartisian();
    TRAJECTORY get_optimal_path();
private:
    vector<TRAJECTORY> trajs;
    // central path
    shared_ptr<CubicSpline2D> _csp;
};

#endif // FRENET_H
