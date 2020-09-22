#include "TrajectoryPlanner.h"

double TrajectoryPlanner::getAcc(double speed, double targetSpeed) const
{
    if (speed < targetSpeed)
        return _h->_maxAcc;
    else if (speed > targetSpeed)
        return -(_h->_maxAcc);
    return 0;
}

vector<vector<double>> TrajectoryPlanner::generateTrajectory(
    const vector<double>& previous_path_x, const vector<double>& previous_path_y,
    const car& carCurr,
    const double desiredVel, const int finalLane, const double endS)
{
    // prev vel
    static double prevRefVel = 0.0;
    
    
    // other variables
    double refX = carCurr._x;
    double refY = carCurr._y;
    double refYaw = carCurr._yaw;
    int previousPathSize = static_cast<int> (previous_path_x.size());

    
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
    }

    
    //In Frenet
    double endPointS;
    if (previousPathSize > 0)
    {
        // convert x,y of the last point in prev
        // ref x and ref y 
        // debug
        endPointS = endS;
    }
    else
    {
        endPointS = carCurr._s;
    }
    double laneCenter = _h->getLaneCenter(finalLane);

    vector<double> next_wp0 = _h->frenet2cartesian({ endPointS + 15, laneCenter });
    vector<double> next_wp1 = _h->frenet2cartesian({ endPointS + 30, laneCenter });
    vector<double> next_wp2 = _h->frenet2cartesian({ endPointS + 45, laneCenter });
    vector<double> next_wp3 = _h->frenet2cartesian({ endPointS + 60, laneCenter });
    vector<double> next_wp4 = _h->frenet2cartesian({ endPointS + 75, laneCenter });
    vector<double> next_wp5 = _h->frenet2cartesian({ endPointS + 90, laneCenter });
    vector<double> next_wp6 = _h->frenet2cartesian({ endPointS + 105, laneCenter });
    vector<double> next_wp7 = _h->frenet2cartesian({ endPointS + 120, laneCenter });
    vector<double> next_wp8 = _h->frenet2cartesian({ endPointS + 135, laneCenter });
    

    // std::cout << "Spline points created" << std::endl;


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
    double targetX = 15.0;
    double targetY = sp(targetX);
    double targetDist = sqrt(pow(targetX, 2) + pow(targetY, 2));
    double xAddOn = 0.0;
    double acc = 0;

    // // std::cout << "Speed is: " << prev_speed << std::endl;

    while(nextVals[0].size() < _newSize)
    {
        acc = getAcc(_refVel, desiredVel);

        // update the speed
        _refVel = std::max(0.5, _refVel + acc);
        _refVel = std::min(_h->_maxVel, _refVel);

        
        // std::cout << " Assumed speed  and Acceleration are: " << _refVel << "," << acc << std::endl;

        double N = (targetDist / (0.02 * _refVel));
        double x = xAddOn + targetX / N;
        double y = sp(x);
        
        xAddOn = x;
        
        car2Global(x, y, refYaw, refX, refY);

       
       // // std::cout << "x and y: " << x << "," << y << std::endl;

        nextVals[0].push_back(x);
        nextVals[1].push_back(y);

     }

    
    // std::cout << "got the trajectory" << std::endl;
    prevRefVel = _refVel;

    return nextVals;

}

TrajectoryPlanner::TrajectoryPlanner(const HighwayMap* h) : _h(h) {};