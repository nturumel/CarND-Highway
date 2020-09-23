

#include "BehaviourPlanner.h"



void BehaviourPlanner::nearestCar()
{

	// initialise maxSpeed vector and rel car vector
	_maxLaneSpeeds = vector<double>(_h->_nlane, _h->_maxVel);
	_relCars = vector<vector<car*>>(_h->_nlane);

	
	// initialise inevstiagation constant
	// the faster the car moves, the further out it needs to look
	double investigationConstant = ((_carCurr->_speed - 0) * (0.7 / 21.2) + 0.8);


	//// debug 
	//cout << "Investigation Zone: " << investigationConstant * _redZone << endl;

	// fill up rel vector
	for (int i = 0; i < _h->_nlane; ++i)
	{
		car* closestBack = nullptr;
		car* closestFront = nullptr;

		double backDistance = _maxReturn;
		double frontDistance = _maxReturn;

		for (car& othercar : _hashCar[i])
		{
			if (othercar._distance >= 0) // other car in front
			{
				if (othercar._distance < frontDistance)
				{
					frontDistance = othercar._distance;
					closestFront = &othercar;
				}
			}
			else
			{
				if (-othercar._distance < backDistance)
				{
					backDistance = -othercar._distance;
					closestBack = &othercar;
				}

			}

		}

		_relCars[i] = vector<car*>{ closestBack ,closestFront };

		

		if (closestFront && (closestFront->_distance < investigationConstant * _redZone))
		{
			_maxLaneSpeeds[i] = closestFront->_speed * 0.9;
		}
		else
		{
			_maxLaneSpeeds[i] = _h->_maxVel;
		}

	}

	
	// debug info

	/*
	cout << "hashcars" << endl;
	for (int i = 0; i < _h->_laneWidth; ++i)
	{
		cout << "i: " << i << endl;
		for (int j = 0; j < _hashCar[i].size(); ++j)
		{
			cout << (_hashCar[i][j])._speed << ":" << (_hashCar[i][j])._distance << ":" << (_hashCar[i][j])._s << ",";
		}
		cout << endl;
	}
	
	cout << "relcars" << endl;
	for (int i = 0; i < _h->_nlane; ++i)
	{
		cout << "i: " << i << endl;
		for (int j = 0; j < _relCars[i].size(); ++j)
		{
			if (_relCars[i][j])
				cout << _relCars[i][j]->_distance << "," << _relCars[i][j]->_speed << ",";
		}
		cout << endl;
	}
	
	
	cout << "speeds" << endl;	
	for (int i = 0; i < _maxLaneSpeeds.size(); ++i)
	{
		cout << "i: " << i << ":" << _maxLaneSpeeds[i] << endl;
	}
	cout << "Sensed data end" << endl;
	*/

}

// cost functions
double BehaviourPlanner::laneChangeCost(int lane)
{
	// if last lane change occured within 10 seconds return max
	seconds delta = duration_cast<seconds> (steady_clock::now() - _lastLaneChange);
	if (delta < _minTimeRequired)
	{
		return _maxReturn;
	}

	return (fabs(_carCurr->_lane - lane));
}
double BehaviourPlanner::speedChangeCost(int lane)
{
	car* front = _relCars[lane][1];
	if (!front)
	{
		return 0;
	}

	double deltaVel = fabs(_carCurr->_speed - _maxLaneSpeeds[lane]);

	return deltaVel;
}
double BehaviourPlanner::speedCost(int lane)
{
	return fabs(_maxLaneSpeeds[lane] - _h->_maxVel);
}
double BehaviourPlanner::safetyCost(int lane)
{
	// if same lane the return 0
	if (_carCurr->_lane == lane)
	{
		return 0;
	}

	// we only consider the cars in front

	// cost of current lane, if there is a car within red zone return max
	car* front = _relCars[_carCurr->_lane][1];
	if (front && front->_distance < _redZone / 2)
	{
		return _maxReturn;
	}

	front = _relCars[lane][1];
	if (front)
	{
		if (fabs(front->_s) < _redZone)
		{
			return _maxReturn;
		}
		else
		{
			return (1 / front->_distance);
		}

	}

	return 0;

}
double BehaviourPlanner::bufferCost(int lane)
{

	// Initial lane, see if any in 30
	car* front = (_relCars[_carCurr->_lane][1]);
	car* back = (_relCars[_carCurr->_lane][0]);

	double cost = 0;

	if (front || back)
	{
		double initialCost = 1;
		if (front)
		{
			initialCost *= (1 / fabs(front->_s));
		}
		if (back)
		{
			initialCost *= (1 / fabs(back->_s));
		}
		if (initialCost > 1)
		{
			cost = initialCost;
		}
	}

	front = (_relCars[lane][1]);
	back = (_relCars[lane][0]);
	if (front || back)
	{
		double finalCost = 1;
		if (front)
		{
			finalCost *= (1 / fabs(front->_s));
		}
		if (back)
		{
			finalCost *= (1 / fabs(back->_s));
		}
		if (finalCost > 1)
		{
			if (cost)
			{
				cost *= finalCost;
			}
			else
			{
				cost = finalCost;
			}

		}
	}

	return cost;
}

bool BehaviourPlanner::collision()
{
	car* front = _relCars[_carCurr->_lane][1];
	if (front)
	{ 
		return ((_relCars[_carCurr->_lane][1])->_distance < _redZone * 1.5);

	}
	return false;
}

void BehaviourPlanner::setEnvironment(const car& carCurr, int prevSize, const vector<vector<double>>& sensor_fusion)
{
	//initialise
	_carCurr = &carCurr;


	// cout << "\t" << "In Constuctor" << endl;
	
	// initialise hashCar
	for (auto sensed : sensor_fusion)
	{
		double x = sensed[1];
		double y = sensed[2];
		double vx = sensed[3];
		double vy = sensed[4];
		double s = sensed[5];
		double d = sensed[6];

		double speed = sqrt(vx * vx + vy * vy);
		double yaw = atan2(vy, vx);
		int lane = (d / _h->_laneWidth);

		if (0 <= lane && lane < _h->_nlane)
		{
			car temp;
			temp.setValues(x, y, s, d, yaw, lane, speed);
			if (lane == _carCurr->_lane && _carCurr->_s > s)
			{
				// cout << "Right Behind" << endl;
				temp._distance = (s)-_carCurr->_s;

			}
			else
			{
				temp._distance = (s + prevSize * 0.02 * speed) - _carCurr->_endS;
			}
			_hashCar[lane].emplace_back(temp);
		}
	}

	// populate the lane vector
	nearestCar();

	// cout << "\t" << "Nearest Car" << endl;

	// collise or not
	_collide = collision();

	
	// set next action
	if (_collide && _suggestedLane == _carCurr->_lane)
	{
		cout << "Collision imminent" << endl;
		_next = choseAction();
	}
	else
	{
		_next = make_pair(_maxLaneSpeeds[_suggestedLane], _suggestedLane);
		
	}

	// cout << "\t" << "Next action and end: " << _next.first << "," << _next.second << endl;

	// clear _hashCars and reset
	_hashCar.clear();
	_maxLaneSpeeds.clear();
	_relCars.clear();
}

pair<double, int> BehaviourPlanner::choseAction()
{

	double max = _maxReturn;
	pair<double, int> result = make_pair(_maxLaneSpeeds[_suggestedLane], _carCurr->_lane);

	if (_carCurr->_speed < 10)
	{
		return result;
	}
	
	for (int i = 0; i < _h->_nlane; ++i)
	{
		if (i > _carCurr->_lane + 1 || i < _carCurr->_lane - 1)
		{
			continue;
		}
		else
		{
			double cost = 0;

			double costAdd;
			costAdd = laneChangeCost(i);
			// cout << "Lane change cost: " << costAdd << endl;
			cost += _laneChangeFactor * costAdd;

			costAdd = speedChangeCost(i);
			// cout << "Speed change cost: " << costAdd << endl;
			cost += _speedChangeFactor * costAdd;

			costAdd = speedCost(i);
			// cout << "Max speed change cost: " << costAdd << endl;
			cost += _speedFactor * costAdd;

			costAdd = bufferCost(i);
			// cout << "Buffer cost: " << costAdd << endl;
			cost += _bufferFactor * costAdd;


			costAdd = safetyCost(i);
			// cout << "Safety cost: " << costAdd << endl;
			cost += _safetyFactor * costAdd;

			if (max > cost)
			{
				max = cost;
				result = make_pair(_maxLaneSpeeds[i], i);
			}

		}

	}
	
	// update _lastLane Change if lane change recommended 
	if (result.second != _carCurr->_lane)
	{
		cout << "Lane Changed" << endl;
		_suggestedLane = result.second;
		_lastLaneChange = steady_clock::now();
	}

	return result;
}

pair<double, int> BehaviourPlanner::returnNextAction()
{
	return _next;
}

void BehaviourPlanner::streamIn()
{
	ifstream in;
	in.open("/home/workspace/CarND-Path-Planning-Project/src/values.txt");
	in >> _laneChangeFactor;
	in >> _speedChangeFactor;
	in >> _speedFactor;
	in >> _bufferFactor;
	in >> _safetyFactor;
	in.close();
}

BehaviourPlanner::BehaviourPlanner(const HighwayMap* h): _h(h) 
{
	// initialise
	streamIn();
}