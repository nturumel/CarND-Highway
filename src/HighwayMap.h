#pragma once
#include <iostream>
#include "spline.h"
#include <fstream>
#include "helpers.h"
#include <sstream>
#include <math.h>

using namespace std;


// we are making this a singleton class
class HighwayMap
{
private:
	// constructor
	HighwayMap();

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


	
public:
	// singleton get instance
	static HighwayMap* getInstance();

	// helper functions
	double getLaneCenter(const int lane) const;

	// spline functions map
	vector<double> frenet2cartesian(const vector<double>& frenetPosition) const;
	vector<double> cartesian2frenet(const vector<double>& cartPosition) const;

	// disabling others
	HighwayMap(HighwayMap& other) = delete;
	HighwayMap& operator=(const HighwayMap& other) = delete;

	//highway constants
	 int _laneWidth = 4;
	 int _nlane = 3;
	 double _maxAcc = 0.09;
	 double _maxVel = 47.5 / 2.24;
	 double _maxS = 6945.554;

	 string _filename = "../data/highway_map.csv";
	 
};

