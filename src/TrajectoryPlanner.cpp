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
    std::cout << "generating trajectory" << std::endl;
    vector<vector<double>> nextVals(2, vector<double>(newSize, 0));

    double carVel = carCurr._speed;
    double refX = carCurr._x;
    double refY = carCurr._y;
    double refYaw = deg2rad(carCurr._yaw);
    int& lane = finalLane;

    int previous_path_size = previous_path_x.size();

    
    vector<double> ptsx, ptsy;

    if (previous_path_size < 2)
    {
        double prev_car_x = carCurr._x - cos(carCurr._yaw);
        double prev_car_y = carCurr._y - sin(carCurr._yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(refX);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(refY);
    }
    else
    {
        refX = previous_path_x[previous_path_size - 1];
        refY = previous_path_y[previous_path_size - 1];


        double prev_ref_x = previous_path_x[previous_path_size - 2];
        double prev_ref_y = previous_path_y[previous_path_size - 2];
        refYaw = atan2(refY - prev_ref_y, refX - prev_ref_x);

        //Use two points to tangent
        ptsx.push_back(prev_ref_x);
        ptsx.push_back(refX);

        ptsy.push_back(prev_ref_y);
        ptsy.push_back(refY);
    }


    //In Frenet
    vector<double> next_wp0 = getXY((carCurr._s + 40), (laneWidth * (lane)+laneWidth / 2)
        , map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY((carCurr._s + 80), (laneWidth * (lane)+laneWidth / 2),
        map_waypoints_s, map_waypoints_x, map_waypoints_y);

    std::cout << "Spline points created" << std::endl;


    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);

   
    //transform to car coordinates
    global2Car(ptsx, ptsy, refYaw, refX, refY);
   
   
    std::cout << "Have the spline points" << std::endl;
    for (int i = 0; i < ptsx.size(); ++i)
    {
        std::cout << ptsx[i] << ",";
    }

    std::cout << std::endl;
    for (int i = 0; i < ptsy.size(); ++i)
    {
        std::cout << ptsy[i] << ",";
    }

    std::cout << "\n Spline points transformed" << std::endl;

    //we are pushing back all the pts left from our previous cycle
    int maxPrev = std::min(maxUsePrev, previous_path_size);

    std::cout << "Prev size is: " << maxPrev << std::endl;


    
    
    // create a spline
    tk::spline sp;
    sp.set_points(ptsx, ptsy);

    std::cout << "Spline Created" << std::endl;

    // pushing previous points
    for (int i = 0; i < maxPrev; ++i)
    {
        nextVals[0][i] = (previous_path_x[i]);
        nextVals[1][i] = (previous_path_y[i]);
    }


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

    std::cout << "Speed is: " << prev_speed << std::endl;

    for (int i = maxPrev; i < newSize; ++i)
    {
        acc = getAcc(prev_speed, ref_vel);
        if (prev_speed > ref_vel && acc > 0.0) {
            acc = 0.0;
            prev_speed = ref_vel;
        }

        std::cout << "acceleration: " << acc << std::endl;

        double x = prev_x + prev_speed * .02 + .5 * .004 * acc;
        double y = sp(x);
        prev_speed = sqrt((x - prev_x) * (x - prev_x) + (y - prev_y) * (y - prev_y)) / .02;

        std::cout << "Speed updated: " << prev_speed << std::endl;


        prev_x = x;
        prev_y = y;

        car2Global(x, y, refYaw, refX, refY);

        std::cout << "x and y: " << x << "," << y << std::endl;

        nextVals[0][i] = x;
        nextVals[1][i] = y;

    }

    std::cout << "the end: " << nextVals[0].back() << "," << nextVals[1].back() << std::endl;

    std::cout << "got the trajectory" << std::endl;
    return nextVals;

}

