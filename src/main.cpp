#include <vector>
#include <iomanip>

#include "common.h"
#include "mapper.h"
#include "measurement_package.h"
#include "frenet.h"
#include "draw.h"

#include "helper/CubicSpline2D.h"
#include "globalpath.h"
#include "frenet.h"

using namespace std;

void check_arguments(int argc, char* argv[]) {
    string usage_instructions = "Usage instruction: ";
    usage_instructions += argv[0];
    usage_instructions += " path/to/world.txt path/to/sensor.dat";
    bool has_valid_args = false;

    // make sure the user has provided input and output files
    if (argc == 1) {
      cerr << usage_instructions << endl;
    } else if (argc == 2) {
      cerr << "Please include an output file.\n" << usage_instructions << endl;
    } else if (argc == 3) {
      has_valid_args = true;
    } else if (argc > 3) {
      cerr << "Too many arguments.\n" << usage_instructions << endl;
    }

    if (!has_valid_args) {
      exit(EXIT_FAILURE);
    }
}

int main(int arc, char* argv[])
{
    check_arguments(arc, argv);
    string in_map_name = argv[1];
    string in_sensor_name = argv[2];
  
    //read the map data for all landmarks
    Mapper mapper;
    mapper.initialize(in_map_name);
    // manual input wayPoint
    vector<vector<double>> wayPoints{{0., 0.},
                                     {10., -6.},
                                     {20.5, 5.},
                                     {35., 6.5},
                                     {70.5, 0.}};
    //read the measurements with odometry and radar data
    MeasurementPackage measurements;
    measurements.initialize(in_sensor_name);
    cout << measurements.data.size() << endl;
    // manual input obstacle lists
    vector<vector<double>> obstacles{{20., 10.},
                                     {30., 6.},
                                     {30., 8.},
                                     {35., 8.},
                                     {50., 3.}};
    // construct guide spline path
    shared_ptr<CubicSpline2D> csp(new CubicSpline2D(wayPoints));
    GlobalPath path(csp);
    map<string, string> pathStyle{{"color", "black"},
                                  {"linestyle", "-"}};
    map<string, string> trajStyle{{"color", "red"},
                                  {"linestyle", " "},
                                  {"marker", "o"},
                                  {"fillstyle", "none"}};
    Draw draw;

    // initial state
    auto speed = 10. / 3.6; // longitudinal speed m/s
    auto d = 2.0; // lateral position to path m
    auto d_d = 0.; // lateral speed m/s
    auto d_dd = 0.; // lateral acceleration m/s^2
    auto s0 = 0.; // course position

    auto area = 20.; // animation length [m]

    // save figure
    string imagesFolder = "./images";
    if (mkdir(imagesFolder.c_str(), 0777) == -1)
        cerr << "Error :  " << strerror(errno) << endl;

    //cout<<"shared times "<<csp.use_count()<<endl;
    for (int i = 0; i < 200; i++) {
        draw.Clear(i);
        // initialize global path
        FRENET traj(csp);
        // get optimized trajectory
        traj.get_frenet_traj(s0, speed, d, d_d, d_dd);
        traj.get_cartisian();
        auto best = traj.get_optimal_path();
        // update initial condition
        s0 = best.s[1];
        speed = best.s_d[1];
        d = best.d[1];
        d_d = best.d_d[1];
        d_dd = best.d_dd[1];

        draw.plot_xy(path.x, path.y, pathStyle);
        draw.plot_xy(best.x, best.y, trajStyle);
        draw.Pause();
        stringstream ss;
        ss << setfill('0') << setw(3) << i;
        draw.Save(imagesFolder+"/"+ss.str());
    }
    // convert png files to gif and if successed then delete pngs
    // convert command
    string cmd1 = "convert -delay 1 -loop 0 "+ imagesFolder +"/*.png ./frenet.gif";
    string cmd2 = "rm -r " + imagesFolder;
    if(system(cmd1.c_str()))
        system(cmd2.c_str());
    draw.Show();

    return 1;
}
