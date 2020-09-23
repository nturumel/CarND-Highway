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
    // trajectory planner
    int _maxUsePrev = 50;
    double _newSize = 50;
    double _refVel = 0.0;

    double getAcc(double speed, double targetSpeed) const;
    const HighwayMap* _h;

public:
    TrajectoryPlanner(const HighwayMap* h);

    vector<vector<double>> generateTrajectory(
        const vector<double>& previous_path_x, const vector<double>& previous_path_y,
        const car& carCurr,
        const double ref_vel, const int finalLane);
};




