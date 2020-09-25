#include <math.h>
#include <cmath>
#include <string>
#include <algorithm>  
#include <vector>
<<<<<<< HEAD
#include "spline.h"
#include "car.h"
#include "helpers.h"
#include "HighwayMap.h"
#include <iostream>

using namespace std;
=======
#include "Global.h"
#include "spline.h"
#include "car.h"
#include "helpers.h"
#include <iostream>

using namespace std;
using namespace highway;
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285

class TrajectoryPlanner
{
private:
<<<<<<< HEAD
    // trajectory planner
    double _newSize = 50;
    double _refVel = 0.0;
    bool _laneChange = false;

    double getAcc(double speed, double targetSpeed) const;
    const HighwayMap* _h;

public:
    TrajectoryPlanner(const HighwayMap* h);

    vector<vector<double>> generateTrajectory(
        const vector<double>& previous_path_x, const vector<double>& previous_path_y,
        const car& carCurr,
        const double ref_vel, const int finalLane);
=======
    double getAcc(double speed, double targetSpeed) const;
public:
    vector<vector<double>> generateTrajectory(
        vector<double> previous_path_x, vector<double> previous_path_y,
        car& carCurr,
        vector<double> map_waypoints_x, vector<double> map_waypoints_y,
        vector<double> map_waypoints_s, double ref_vel, int finalLane) const;
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
};




