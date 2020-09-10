#include "TrajectoryPlanner.h"

double TrajectoryPlanner::getAcc(double speed, double targetSpeed) const
{
    if (fabs(speed) < 1)
    {
        return maxAcc;
    }

    return (maxAcc - maxAcc * log(speed) / log(targetSpeed));    
}

vector<vector<double>> TrajectoryPlanner::generateTrajectory(
    vector<double> previous_path_x, vector<double> previous_path_y,
    car& carCurr,
    vector<double> map_waypoints_x, vector<double> map_waypoints_y,
    vector<double> map_waypoints_s, double ref_vel, int finalLane) const
{

    double car_vel = carCurr._speed;
    double ref_x = carCurr._x;
    double ref_y = carCurr._y;
    double ref_yaw = deg2rad(carCurr._yaw);
    int& lane = finalLane;

    int previous_path_size = previous_path_x.size();

    // std::cout << " Car values: " << car_vel << "," << lane << "," << vel_add << std::endl;

    vector<double> ptsx, ptsy;

    if (previous_path_size < 2)
    {
        double prev_car_x = carCurr._x - cos(carCurr._yaw);
        double prev_car_y = carCurr._y - sin(carCurr._yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(ref_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(ref_y);
    }
    else
    {
        ref_x = previous_path_x[previous_path_size - 1];
        ref_y = previous_path_y[previous_path_size - 1];


        double prev_ref_x = previous_path_x[previous_path_size - 2];
        double prev_ref_y = previous_path_y[previous_path_size - 2];
        ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

        //Use two points to tangent
        ptsx.push_back(prev_ref_x);
        ptsx.push_back(ref_x);

        ptsy.push_back(prev_ref_y);
        ptsy.push_back(ref_y);
    }


    //In Frenet
    vector<double> next_wp0 = getXY((carCurr._s + 40), (laneWidth * (lane)+laneWidth / 2)
        , map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY((carCurr._s + 80), (laneWidth * (lane)+laneWidth / 2),
        map_waypoints_s, map_waypoints_x, map_waypoints_y);



    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);


    //transform to car coordinates
    global2Car(ptsx, ref_yaw, ref_x, ref_y);
    global2Car(ptsy, ref_yaw, ref_x, ref_y);

    
    //we are pushing back all the pts left from our previous cycle
    int maxPrev = std::min(maxUsePrev, previous_path_size);

    vector<double> next_x_vals, next_y_vals;


    for (int i = 0; i < maxPrev; ++i)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    } 
    
    //create a spline
    tk::spline sp;
    sp.set_points(ptsx, ptsy);

    // spline spacing
    double target_x = 40.0;
    double target_y = sp(target_x);
    
    // note: straight line distance
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    // add new points to next path
    double prev_x = 0.0;
    double prev_y = 0.0;
    double prev_speed = (previous_path_size < 2) ? carCurr._speed :
        sqrt(ptsx[0] * ptsx[0] + ptsy[0] * ptsy[0]) / 0.02;

    double acc = 0.0;

    for (int i = 0; i < 50 - maxPrev; ++i)
    {
        acc = getAcc(prev_speed, ref_vel);
        if (prev_speed > ref_vel && acc > 0.0) {
            acc = 0.0;
            prev_speed = ref_vel;
        }
        double x = prev_x + prev_speed * .02 + .5 * .004 * acc;
        double y = sp(x);
        prev_speed = sqrt((x - prev_x) * (x - prev_x) + (y - prev_y) * (y - prev_y)) / .02;

        prev_x = x;
        prev_y = y;

        car2Global(x, y, ref_yaw, ref_x, ref_y);

        next_x_vals.push_back(x);
        next_y_vals.push_back(y);

    }

    return {next_x_vals ,next_y_vals};

}

