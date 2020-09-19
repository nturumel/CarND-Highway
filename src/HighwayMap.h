#pragma once
#include <iostream>
#include "spline.h"
#include "Global.h"
#include <fstream>
#include "helpers.h"
#include <sstream>

using namespace std;
using namespace highway;


// we are making this a singleton class
class HighwayMap
{
private:
	// constructor
	HighwayMap(string filename);

	// destructor
	~HighwayMap();
	
	// instance
	static HighwayMap* _instance;

	// Loaded waypoints defining the road curve
	// Waypoints represent the center of the road. Drivable lanes are to the right
	vector<double> _wayPointsX;
	vector<double> _wayPointsY;
	vector<double> _wayPointsS;
	vector<double> _wayPointsDx;
	vector<double> _wayPointsDy;


	// road spline
	vector<tk::spline> _splineMap;
public:
	// singleton get instance
	static HighwayMap* getInstance(string filename);

	// helper functions
	double getLaneCenter(int lane);

	// spline functions map
	vector<double> frenet2cartesian(const vector<double> frenetPosition) const;

	// disabling others
	HighwayMap(HighwayMap& other) = delete;
	HighwayMap& operator=(const HighwayMap& other) = delete;



};

