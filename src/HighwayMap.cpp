#include "HighwayMap.h"

const bool smooth = true;
HighwayMap* HighwayMap::_instance = 0;

HighwayMap::HighwayMap() : _splineMap(4)
{
    std::ifstream in_map_(_filename.c_str(), std::ifstream::in);

    // getting the highway map Values

    string line;
    while (getline(in_map_, line))
    {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        _wayPointsX.push_back(x);
        _wayPointsY.push_back(y);
        _wayPointsS.push_back(s);
        _wayPointsDx.push_back(d_x);
        _wayPointsDy.push_back(d_y);
    }

    // creating spline of the road in x, y and d
    _splineMap[0].set_points(_wayPointsS, _wayPointsX);
    _splineMap[1].set_points(_wayPointsS, _wayPointsY);
    _splineMap[2].set_points(_wayPointsS, _wayPointsDx);
    _splineMap[3].set_points(_wayPointsS, _wayPointsDy);

    // initialising the constants
    streamIn();
}

HighwayMap* HighwayMap::getInstance()
{
    if (_instance)
    {
        return _instance;
    }
    _instance = new HighwayMap();
    return _instance;
}

double HighwayMap::getLaneCenter(int lane)
{
    return laneWidth * (lane)+(laneWidth / 2);
}

vector<double> HighwayMap::frenet2cartesian(const vector<double> frenetPosition) const
{
    const double& s = frenetPosition[0];
    const double& d = frenetPosition[1];

    if (smooth)
    {
        double x = _splineMap[0](s) + _splineMap[2](s) * d;
        double y = _splineMap[1](s) + _splineMap[3](s) * d;
        return { x, y };
    }
    else
        return getXY(s, d, _wayPointsS, _wayPointsX, _wayPointsY);
}

void HighwayMap::streamIn()
{
    ifstream in;
    in.open("/home/workspace/CarND-Path-Planning-Project/src/values.txt");
    in >> laneChangeFactor;
    in >> speedChangeFactor;
    in >> speedFactor;
    in >> bufferFactor;
    in >> safetyFactor;
    cout << "The order is: " << "laneChangeFactor , speedChangeFactor , speedFactor , bufferFactor , safetyFactor" << endl;
    cout << "The values are: " << laneChangeFactor << "," << speedChangeFactor << "," << speedFactor << "," << bufferFactor << "," << safetyFactor << endl;
    in.close();
}

HighwayMap::~HighwayMap()
{
    delete _instance;
}
