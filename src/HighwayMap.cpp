#include "HighwayMap.h"

HighwayMap* HighwayMap::_instance = 0;

HighwayMap::HighwayMap()
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

double HighwayMap::getLaneCenter(const int lane) const 
{
    return _laneWidth * (lane) + (_laneWidth / 2);
}

vector<double> HighwayMap::frenet2cartesian(const vector<double>& frenetPosition) const
{
    double s = fmod(frenetPosition[0], _maxS);
    const double& d = frenetPosition[1];

    return getXY(s, d, _wayPointsS, _wayPointsX, _wayPointsY);
}

vector<double> HighwayMap::cartesian2frenet(const vector<double>& cartPosition) const
{
    const double& x = cartPosition[0];
    const double& y = cartPosition[1];
    const double& theta = cartPosition[3];

    return getFrenet(x, y, theta, _wayPointsX, _wayPointsY);

}


HighwayMap::~HighwayMap()
{
    delete _instance;
}
