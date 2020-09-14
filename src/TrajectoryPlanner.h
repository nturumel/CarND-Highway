#include <math.h>
#include <cmath>
#include <string>
#include <algorithm>  
#include <vector>
#include "Global.h"
#include "spline.h"
#include "car.h"
#include "helpers.h"
#include <iostream>

using namespace std;
using namespace highway;

class TrajectoryPlanner
{
private:
    double getAcc(double speed, double targetSpeed) const;
public:
    vector<vector<double>> generateTrajectory(
        vector<double> previous_path_x, vector<double> previous_path_y,
        car& carCurr,
        vector<double> map_waypoints_x, vector<double> map_waypoints_y,
        vector<double> map_waypoints_s, double ref_vel, int finalLane) const;
};




