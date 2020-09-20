#pragma once
#include <iostream>
#include "spline.h"
#include <fstream>
#include "helpers.h"
#include <sstream>

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


	// road spline
	vector<tk::spline> _splineMap;
public:
	// singleton get instance
	static HighwayMap* getInstance();

	// helper functions
	double getLaneCenter(int lane);

	// spline functions map
	vector<double> frenet2cartesian(const vector<double> frenetPosition) const;

	// disabling others
	HighwayMap(HighwayMap& other) = delete;
	HighwayMap& operator=(const HighwayMap& other) = delete;

	//highway constants
	 int laneWidth = 4;
	 int nlane = 3;
	 double maxAcc = 5;
	 double maxVel = 60;
	 double maxReturn = 1e6;
	 int maxUsePrev = 50;
	 double newSize = 50;
	 double redZone = 30.0;
	 double laneChangeFactor = 0;
	 double  speedChangeFactor = 0;
	 double speedFactor = 0;
	 double bufferFactor = 0;
	 double safetyFactor = 0;
	 string _filename = "../data/highway_map.csv";

	 // this is temporary the weights will be fixed
	 void streamIn();
};

