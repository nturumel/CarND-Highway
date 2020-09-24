#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include "helpers.h"
#include "json.hpp"
#include "HighwayMap.h"
#include <math.h>
#include <climits>
#include <fstream>
#include "car.h"
#include <chrono>
#include <ratio>
#include <chrono>

using namespace std::chrono;
using namespace std;


class BehaviourPlanner
{
private:
	// behaviour planner
	double _maxReturn = 1e6;
	double _redZone = 30.0;
	int _suggestedLane = 1;
	bool _laneChanged = true;
	steady_clock::time_point _lastLaneChange = steady_clock::now();
	seconds _minTimeRequired{10};

	double _laneChangeFactor = 1.0;
	double  _speedChangeFactor = 1.0;
	double _speedFactor = 0.01;
	double _bufferFactor = 0.01;
	double _safetyFactor = 1;
	
	// highway
	const HighwayMap* _h;

	unordered_map <int, vector<car>> _hashCar;
	const car* _carCurr;
	vector <double> _maxLaneSpeeds;
	vector<vector<car*>> _relCars;


	// keep track of size of previous point vector
	int _prevSize;

	// see if collision imminent
	bool collision();

	// collide
	bool _collide;

	// populate nearest cars
	void nearestCar();

	
	
	// cost functions
	double laneChangeCost(int lane);
	double speedChangeCost(int lane);
	double speedCost(int lane);
	double safetyCost(int lane);
	double bufferCost(int lane);

	// next action
	pair<double, int> _next;
	pair<double, int> choseAction();


public:
	BehaviourPlanner(const HighwayMap* h);
	void setEnvironment(const car& carCurr, int prevSize, const vector<vector<double>>& sensor_fusion);
	pair<double, int> returnNextAction();
};
