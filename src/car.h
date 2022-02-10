#ifndef CAR_H
#define CAR_H

#include "utils.h"

#include <vector>
#include <tuple>

using namespace std;

// Lincoln MKZ configuration
const double VEHICLE_LENGTH = 4.93;
const double VEHICLE_WIDTH = 1.86;

class Car {
public:
    Car() {
        length = VEHICLE_LENGTH;
        width = VEHICLE_WIDTH;
    };
    Car(Pose pose_): pose(pose_) {
        length = VEHICLE_LENGTH;
        width = VEHICLE_WIDTH;
    };
    void set_pos(Pose p);
    vector<Point> get_bounding_box();
private:
    double length;
    double width;
    Pose pose; // x, y, yaw
};

#endif // CAR_H
