#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include "helpers.h"
#include "json.hpp"
#include <math.h>
#include <climits>
#include <fstream>
#include "Global.h"
#include "car.h"

using namespace std;
using namespace highway;


class BehaviourPlanner
{
private:
	unordered_map <int, vector<car>> _hashCar;
	car& _carCurr;
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
	BehaviourPlanner(car& carCurr, int prevSize, vector<vector<double>>& sensor_fusion);
	pair<double, int> returnNextAction();

	
};
