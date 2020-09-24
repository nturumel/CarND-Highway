

#include "BehaviourPlanner.h"



void BehaviourPlanner::nearestCar()
{

	// initialise maxSpeed vector and rel car vector
	_maxLaneSpeeds = vector<double>(_h->_nlane, _h->_maxVel);
	_relCars = vector<vector<car*>>(_h->_nlane);

	
	// initialise inevstiagation constant
	// the faster the car moves, the further out it needs to look
	double investigationConstant = ((_carCurr->_speed - 0) * (0.7 / 21.2) + 0.8);


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

	// we only consider the cars in front for the current lane if there is a car within red zone return max
	car* interestedVehicle = _relCars[_carCurr->_lane][1];
	if (interestedVehicle && interestedVehicle->_distance < (_redZone / 2))
	{
		return _maxReturn;
	}

	interestedVehicle = _relCars[lane][1];
	if (interestedVehicle && interestedVehicle->_distance < (2 / 3) * _redZone)
	{
		return _maxReturn;
	}

	interestedVehicle = _relCars[lane][0];
	if (interestedVehicle && fabs(interestedVehicle->_distance) < (2 / 3) * _redZone)
	{
		return _maxReturn;
		
	}

	return 0;

}
double BehaviourPlanner::bufferCost(int lane)
{

	car* front = (_relCars[lane][1]);
	car* back = (_relCars[lane][0]);
	double cost = 1;

	if (front || back)
	{		
		if (front)
		{
			cost *= (1 / fabs(front->_distance));
		}
		if (back)
		{
			cost *= (1 / fabs(back->_distance));
		}
		
	}

	if (cost != 1)
	{
		return cost;
	}

	return 0; // no vehicle in that lane
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

	
	// collise or not
	_collide = collision();

	
	// set next action
	if (_collide && _suggestedLane == _carCurr->_lane)
	{
		_next = choseAction();
	}
	else
	{
		_next = make_pair(_maxLaneSpeeds[_suggestedLane], _suggestedLane);
		
	}

	
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
			cost += _laneChangeFactor * costAdd;

			costAdd = speedChangeCost(i);
			cost += _speedChangeFactor * costAdd;

			costAdd = speedCost(i);
			cost += _speedFactor * costAdd;

			costAdd = bufferCost(i);
			cost += _bufferFactor * costAdd;


			costAdd = safetyCost(i);
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
		_suggestedLane = result.second;
		_lastLaneChange = steady_clock::now();
	}

	return result;
}

pair<double, int> BehaviourPlanner::returnNextAction()
{
	return _next;
}

BehaviourPlanner::BehaviourPlanner(const HighwayMap* h) : _h(h) {}