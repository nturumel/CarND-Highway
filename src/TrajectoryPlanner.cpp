#include "TrajectoryPlanner.h"

double TrajectoryPlanner::getAcc(double speed, double targetSpeed) const
{
<<<<<<< HEAD
    
    if (speed < targetSpeed)
        return _h->_maxAcc;
    else if (speed > targetSpeed)
        return -(_h->_maxAcc);
    return 0;
}

vector<vector<double>> TrajectoryPlanner::generateTrajectory(
    const vector<double>& previous_path_x, const vector<double>& previous_path_y,
    const car& carCurr,
    const double desiredVel, const int desiredLane)
{
    // prev vel
    static double prevRefVel = 0.0;


    // other variables
    double refX = carCurr._x;
    double refY = carCurr._y;
    double refYaw = carCurr._yaw;
    int previousPathSize = static_cast<int> (previous_path_x.size());
    _laneChange = (carCurr._lane != desiredLane);

    // spline    
    vector<double> ptsX, ptsY;

    if (previousPathSize < 2)
    {
        double prevCarX = refX - cos(carCurr._yaw);
        double prevCarY = refY - sin(carCurr._yaw);

        ptsX.push_back(prevCarX);
        ptsX.push_back(refX);

        ptsY.push_back(prevCarY);
        ptsY.push_back(refY);

        _refVel = carCurr._speed;
    }
    else
    {
        // redefine reference state as previous path end point
        refX = previous_path_x[previousPathSize - 1];
        refY = previous_path_y[previousPathSize - 1];


        double prevRefX = previous_path_x[previousPathSize - 2];
        double prevRefY = previous_path_y[previousPathSize - 2];

        refYaw = atan2(refY - prevRefY, refX - prevRefX);

        //Use two points to tangent
        ptsX.push_back(prevRefX);
        ptsX.push_back(refX);

        ptsY.push_back(prevRefY);
        ptsY.push_back(refY);

        _refVel = prevRefVel;
=======
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
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
    }


    //In Frenet
<<<<<<< HEAD
    double endPointS;
    if (previousPathSize > 0)
    {
        // convert x,y of the last point in prev
        // ref x and ref y 
        // debug
        endPointS = carCurr._endS;
    }
    else
    {
        endPointS = carCurr._s;
    }
    double laneCenter = _h->getLaneCenter(desiredLane);
    if (!_laneChange)
    {
        vector<double> next_wp0 = _h->frenet2cartesian({ endPointS + 15, laneCenter });
        vector<double> next_wp1 = _h->frenet2cartesian({ endPointS + 30, laneCenter });
        vector<double> next_wp2 = _h->frenet2cartesian({ endPointS + 45, laneCenter });
        vector<double> next_wp3 = _h->frenet2cartesian({ endPointS + 60, laneCenter });
        vector<double> next_wp4 = _h->frenet2cartesian({ endPointS + 75, laneCenter });
        vector<double> next_wp5 = _h->frenet2cartesian({ endPointS + 90, laneCenter });
        vector<double> next_wp6 = _h->frenet2cartesian({ endPointS + 105, laneCenter });
        vector<double> next_wp7 = _h->frenet2cartesian({ endPointS + 120, laneCenter });
        vector<double> next_wp8 = _h->frenet2cartesian({ endPointS + 135, laneCenter });


      
        ptsX.push_back(next_wp0[0]);
        ptsX.push_back(next_wp1[0]);
        ptsX.push_back(next_wp2[0]);
        ptsX.push_back(next_wp3[0]);
        ptsX.push_back(next_wp4[0]);
        ptsX.push_back(next_wp5[0]);
        ptsX.push_back(next_wp6[0]);
        ptsX.push_back(next_wp7[0]);
        ptsX.push_back(next_wp8[0]);

        ptsY.push_back(next_wp0[1]);
        ptsY.push_back(next_wp1[1]);
        ptsY.push_back(next_wp2[1]);
        ptsY.push_back(next_wp3[1]);
        ptsY.push_back(next_wp4[1]);
        ptsY.push_back(next_wp5[1]);
        ptsY.push_back(next_wp6[1]);
        ptsY.push_back(next_wp7[1]);
        ptsY.push_back(next_wp8[1]);

        //transform to car coordinates
        global2Car(ptsX, ptsY, refYaw, refX, refY);

    }
    else
    {
        vector<double> next_wp0 = _h->frenet2cartesian({ endPointS + 30, laneCenter });
        vector<double> next_wp1 = _h->frenet2cartesian({ endPointS + 60, laneCenter });
        vector<double> next_wp2 = _h->frenet2cartesian({ endPointS + 90, laneCenter });
        


        ptsX.push_back(next_wp0[0]);
        ptsX.push_back(next_wp1[0]);
        ptsX.push_back(next_wp2[0]);
        

        ptsY.push_back(next_wp0[1]);
        ptsY.push_back(next_wp1[1]);
        ptsY.push_back(next_wp2[1]);
        
        //transform to car coordinates
        global2Car(ptsX, ptsY, refYaw, refX, refY);
    }

         
    // create a spline
    tk::spline sp;
    sp.set_points(ptsX, ptsY);

   
    //we are pushing back all the pts left from our previous cycle
    // result
    vector<vector<double>> nextVals(2);
    
    
    // pushing previous points
    for (int i = 0; i < previousPathSize; ++i)
    {
        nextVals[0].push_back(previous_path_x[i]);
        nextVals[1].push_back(previous_path_y[i]);
       
    }   

    // spline spacing
    // note: straight line distance
    double targetX = 15.0 + static_cast<int>(_laneChange) * 15;
    double targetY = sp(targetX);
    double targetDist = sqrt(pow(targetX, 2) + pow(targetY, 2));
    double xAddOn = 0.0;
    double acc = 0;

    
    while(nextVals[0].size() < _newSize)
    {
        acc = getAcc(_refVel, desiredVel);
        
        // not really required except for the curves... it seems, just make max 0.7 ...
        if(_laneChange)
            acc -= (0.02);

        // update the speed
        _refVel = std::max(0.5, _refVel + acc);
        _refVel = std::min(_h->_maxVel, _refVel);

       
        double N = (targetDist / (0.02 * _refVel));
        double x = xAddOn + targetX / N;
        double y = sp(x);
        
        xAddOn = x;
        car2Global(x, y, refYaw, refX, refY);


        nextVals[0].push_back(x);
        nextVals[1].push_back(y);

     }

    
    prevRefVel = _refVel;

=======
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
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
    return nextVals;

}

<<<<<<< HEAD
TrajectoryPlanner::TrajectoryPlanner(const HighwayMap* h) : _h(h) {};
=======
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
