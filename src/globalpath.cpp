#include "globalpath.h"
#include "utils.h"

#include <algorithm>

const float COLLISION_CHECK_THRESHOLD = 6; // don't check unless within 6m

GlobalPath::GlobalPath(shared_ptr<CubicSpline2D> csp) : _csp(csp) {
    int cnts = (int) ((_csp->get_s().back() - _csp->get_s().front()) / 0.1);
    for(int i = 0; i < cnts; ++i) s.push_back(_csp->get_s().front() + 0.1*i);
    set_global_path();
}

// Convert the frenet path to global path in terms of x, y, yaw, velocity
bool GlobalPath::set_global_path() {

    // calc global positions
    for (size_t i = 0; i < s.size(); i++) {
        auto ix = _csp->calc_x(s[i]);
        auto iy = _csp->calc_y(s[i]);
        if (isnan(ix) || isnan(iy)) break;
        auto iyaw = _csp->calc_yaw(s[i]);
        auto kappa = _csp->calc_curvature(s[i]);
        x.push_back(ix);
        y.push_back(iy);
        yaw.push_back(iyaw);
        k.push_back(kappa);
    }

    // not enough points to construct a valid path
    if (x.size() <= 1) {
        return false;
    }

    // calc curvature

    return true;
}

