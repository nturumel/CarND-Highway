#include <math.h>
#include <cmath>
#include <string>
#include <algorithm>  
#include <vector>
#include "Global.h"
#include "spline.h"
#include "car.h"
#include "helpers.h"
#include "HighwayMap.h"
#include <iostream>

using namespace std;
using namespace highway;

class TrajectoryPlanner
{
private:
    double getAcc(double speed, double targetSpeed) const;
    vector<vector<double>> _oldTrajectory; // previous generated trajectory

public:
    TrajectoryPlanner();

    vector<vector<double>> generateTrajectory(
        vector<double> previous_path_x, vector<double> previous_path_y,
        car& carCurr,
        HighwayMap* h,
        double ref_vel, int finalLane);
};




