#include <math.h>
#include <cmath>
#include <string>
#include <algorithm>  
#include <vector>
#include "spline.h"
#include "car.h"
#include "helpers.h"
#include "HighwayMap.h"
#include <iostream>

using namespace std;

class TrajectoryPlanner
{
private:
    double getAcc(double speed, double targetSpeed) const;
    vector<vector<double>> _oldTrajectory; // previous generated trajectory
    HighwayMap* _h;

public:
    TrajectoryPlanner(HighwayMap* h);

    vector<vector<double>> generateTrajectory(
        vector<double> previous_path_x, vector<double> previous_path_y,
        car& carCurr,
        double ref_vel, int finalLane);
};




