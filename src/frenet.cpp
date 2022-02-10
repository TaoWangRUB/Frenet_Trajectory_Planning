#include "frenet.h"
#include "quintic_poly.h"
#include "quartic_poly.h"

FRENET::FRENET(shared_ptr<CubicSpline2D> csp) : _csp(csp)
{

}

// compute all the trajectories
void FRENET::get_frenet_traj(double s0, double s_d, double d, double d_d, double d_dd)
{
    //vector<TRAJECTORY> trajs;

    int num_paths = 0;
    int num_viable_paths = 0;

    vector<double> dn, tn, vn;
    auto DMIN = -5., DMAX = 5., DELTA_D = 0.5;
    int n = (DMAX-DMIN) / DELTA_D;
    for(int i = 0; i < n; i++) dn.push_back(DMIN + i*DELTA_D);
    auto TMIN = 3., TMAX = 5., DELTA_T = 0.5;
    int m = (TMAX-TMIN) / DELTA_T;
    for(int i = 0; i < m; i++) tn.push_back(TMIN + i*DELTA_T);
    auto TARGET_V = 30. / 3.6, DELTA_V = 5. / 3.6;
    int N_V = 1;
    for(int i = 1; i <= N_V; i++) vn.push_back(TARGET_V - N_V*DELTA_V + i*DELTA_V);

    // cost weight
    double KJ{0.1}, KT{0.1}, KD{1.0}, KLAT{1.0}, KLONG{1.0};
    // compute the trajectories
    for(int i = 0; i < dn.size(); ++i){

        // lateral motion planning
        for(int j = 0; j < tn.size(); ++j){
            //
            double lat_d{0}, lat_v{0}, lat_acc{0}, lat_jerk{0};
            double long_d{0}, long_v{0}, long_acc{0}, long_jerk{0};
            // get trajectory points
            auto traj_lat = TRAJECTORY();
            auto lat = quintic_poly(d, d_d, d_dd, dn[i], 0., 0., tn[j]);
            double t = 0.;
            // construct frenet trajectory
            while (t <= tn[j]) {
                traj_lat.t.push_back(t);
                traj_lat.d.push_back(lat.calc_point(t));
                traj_lat.d_d.push_back(lat.calc_first_derivative(t));
                traj_lat.d_dd.push_back(lat.calc_second_derivative(t));
                traj_lat.d_ddd.push_back(lat.calc_third_derivative(t));
                lat_d += abs(lat.calc_point(t));
                lat_v += abs(lat.calc_first_derivative(t));
                lat_acc += abs(lat.calc_second_derivative(t));
                lat_jerk += pow(abs(lat.calc_third_derivative(t)), 2.);
                t += traj_lat.dt;
            }
            // longitudinal motion planning
            for(auto& tv : vn){
                auto traj_long = TRAJECTORY();
                // copy frenet path
                traj_long.t.assign(traj_lat.t.begin(), traj_lat.t.end());
                traj_long.d.assign(traj_lat.d.begin(), traj_lat.d.end());
                traj_long.d_d.assign(traj_lat.d_d.begin(), traj_lat.d_d.end());
                traj_long.d_dd.assign(traj_lat.d_dd.begin(), traj_lat.d_dd.end());
                traj_long.d_ddd.assign(traj_lat.d_ddd.begin(), traj_lat.d_ddd.end());
                auto lon_qp = quartic_poly(s0, s_d, 0.0, tv, 0.0, tn[j]);
                for(auto& ti : traj_long.t){
                    traj_long.s.push_back(lon_qp.calc_point(ti));
                    traj_long.s_d.push_back(lon_qp.calc_first_derivative(ti));
                    traj_long.s_dd.push_back(lon_qp.calc_second_derivative(ti));
                    traj_long.s_ddd.push_back(lon_qp.calc_third_derivative(ti));
                    long_acc += abs(lon_qp.calc_second_derivative(ti));
                    long_jerk += pow(abs(lon_qp.calc_third_derivative(ti)), 2.);
                }
                auto ds = pow(TARGET_V - traj_long.s_d.back(), 2.);
                traj_long.cd = KJ * lat_jerk + KT * tn[j] + KD * pow(traj_long.d.back(), 2.);
                traj_long.cv = KJ * long_jerk + KT * tn[j] + KD * ds;
                traj_long.cf = KLAT * traj_long.cd + KLONG * traj_long.cv;
                trajs.push_back(traj_long);
                num_paths++;
            }
        }
    }
    return;
}

// transform Frenet to Cartisian
bool FRENET::get_global_path(TRAJECTORY& traj){
    // calc global positions
    for (size_t i = 0; i < traj.s.size(); i++) {
        auto ix = _csp->calc_x(traj.s[i]);
        auto iy = _csp->calc_y(traj.s[i]);
        if (isnan(ix) || isnan(iy)) break;
        auto di = traj.d[i];
        auto iyaw = _csp->calc_yaw(traj.s[i]);
        //auto kappa = _csp->calc_curvature(traj.s[i]);
        traj.x.emplace_back(ix + di * cos(iyaw + M_PI_2));
        traj.y.emplace_back(iy + di * sin(iyaw + M_PI_2));
    }
    // not enough points to construct a valid path
    if (traj.x.size() <= 1) {
        return false;
    }
    // calc yaw and ds
    vector<double> ds;
    for (size_t i = 0; i < traj.x.size() - 1; i++) {
        auto dx = traj.x[i+1] - traj.x[i];
        auto dy = traj.y[i+1] - traj.y[i];
        traj.yaw.push_back(atan2(dy, dx));
        ds.push_back(hypot(dx, dy));
    }
    traj.yaw.push_back(traj.yaw.back());
    ds.push_back(ds.back());

    // calc curvature
    for (size_t i = 0; i < traj.yaw.size() - 1; i++) {
        auto dyaw = traj.yaw[i+1] - traj.yaw[i];
        if (dyaw > M_PI_2) {
            dyaw -= M_PI;
        } else if (dyaw < -M_PI_2) {
            dyaw += M_PI;
        }
        traj.c.push_back(dyaw / ds[i]);
    }

    return true;
}
// transfor Frenet to Cartisian
void FRENET::get_cartisian(){
    for(auto& traj : trajs)
        get_global_path(traj);
}
// get optimal trajectory
TRAJECTORY FRENET::get_optimal_path(){
    auto minCost = numeric_limits<double>::max();
    auto ans = TRAJECTORY();
    for(auto& traj : trajs){
        if(minCost >= traj.cf){
            minCost = traj.cf;
            ans = traj;
        }
    }
    return ans;
}
