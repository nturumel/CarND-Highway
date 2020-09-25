<<<<<<< HEAD


#include "BehaviourPlanner.h"


void BehaviourPlanner::nearestCar()
{

	// initialise maxSpeed vector and rel car vector
	_maxLaneSpeeds = vector<double>(_h->_nlanes, _h->_maxVel);
	_relCars = vector<vector<car*>>(_h->_nlanes);

	
	// initialise inevstiagation constant
	// the faster the car moves, the further out it needs to look
	double investigationConstant = ((_carCurr->_speed - 0) * (0.7 / 21.2) + 0.8);


	// fill up rel vector
	for (int i = 0; i < _h->_nlanes; ++i)
=======
#include "BehaviourPlanner.h"



void BehaviourPlanner::nearestCar()
{
	
	// initialise maxSpeed vector
	_maxLaneSpeeds = vector<double>(nlane, maxVel);
	for (int i = 0; i < nlane; ++i)
	{
		_relCars.push_back({ nullptr,nullptr });
	}

	// fill up rel vector
	for (int i = 0; i < nlane; ++i)
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
	{
		car* closestBack = nullptr;
		car* closestFront = nullptr;

<<<<<<< HEAD
		double backDistance = _maxReturn;
		double frontDistance = _maxReturn;
=======
		double backDistance = maxReturn;
		double frontDistance = maxReturn;
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285

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

<<<<<<< HEAD
		

		if (closestFront && (closestFront->_distance < investigationConstant * _redZone))
		{
			_maxLaneSpeeds[i] = closestFront->_speed * 0.9;
		}
		else
		{
			_maxLaneSpeeds[i] = _h->_maxVel;
=======
		if (closestBack && closestFront && ((closestBack->_distance < 1.5 * redZone)))
		{
			_maxLaneSpeeds[i] = (closestBack->_speed + closestFront->_speed);
			_maxLaneSpeeds[i] /= 2;
		}
		else if (closestFront && (closestFront->_distance < 1.5 * redZone))
		{
			_maxLaneSpeeds[i] = closestFront->_speed;
		}
		else
		{
			_maxLaneSpeeds[i] = maxVel;
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
		}

	}

<<<<<<< HEAD
	
}

// cost functions
double BehaviourPlanner::laneChangeCost(int lane)
{
	// if last lane change occured within 10 seconds return max
	if (lane != _carCurr->_lane)
	{
		seconds delta = duration_cast<seconds> (steady_clock::now() - _lastLaneChange);
		if (delta < _minTimeRequired)
		{
			return _maxReturn;
		}
	}
	
	return (fabs(_carCurr->_lane - lane));
=======
	/*

	// debug info
	cout << "hashcars" << endl;
	for (int i = 0; i < nlane; ++i)
	{
		cout << "i: " << i << endl;
		for (int j = 0; j < _hashCar[i].size(); ++j)
		{
			cout << (_hashCar[i][j])._speed << ":" << (_hashCar[i][j])._distance << ":" << (_hashCar[i][j])._s << ",";
		}
		cout << endl;
	}
	cout << "relcars" << endl;
	for (int i = 0; i < nlane; ++i)
	{
		cout << "i: " << i << endl;
		for (int j = 0; j < _relCars[i].size(); ++j)
		{
			if (_relCars[i][j])
				cout << _relCars[i][j]->_speed << ",";
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
	return (fabs(_carCurr._lane - lane));
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
}
double BehaviourPlanner::speedChangeCost(int lane)
{
	car* front = _relCars[lane][1];
	if (!front)
	{
		return 0;
	}

<<<<<<< HEAD
	double deltaVel = fabs(_carCurr->_speed - _maxLaneSpeeds[lane]);
=======
	double deltaVel = fabs(_carCurr._speed - _maxLaneSpeeds[lane]);
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285

	return deltaVel;
}
double BehaviourPlanner::speedCost(int lane)
{
<<<<<<< HEAD
	return fabs(_maxLaneSpeeds[lane] - _h->_maxVel);
=======
	return fabs(_maxLaneSpeeds[lane] - maxVel);
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
}
double BehaviourPlanner::safetyCost(int lane)
{
	// if same lane the return 0
<<<<<<< HEAD
	if (_carCurr->_lane == lane)
=======
	if (_carCurr._lane == lane)
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
	{
		return 0;
	}

<<<<<<< HEAD
	// we only consider the cars in front for the current lane if there is a car within red zone return max
	car* interestedVehicle = _relCars[_carCurr->_lane][1];
	if (interestedVehicle && fabs(interestedVehicle->_distance) < (_redZone / 2))
	{
		return _maxReturn;
	}

	interestedVehicle = _relCars[lane][1];
	if (interestedVehicle && fabs(interestedVehicle->_distance) < (_redZone / 2))
	{
		return _maxReturn;
	}

	interestedVehicle = _relCars[lane][0];
	if (interestedVehicle && fabs(interestedVehicle->_distance) < (_redZone / 2))
	{
		return _maxReturn;
=======
	// we only consider the cars in front

	// cost of current lane, if there is a car within red zone return max
	car* front = _relCars[_carCurr._lane][1];
	if (front && front->_distance < redZone / 2)
	{
		return maxReturn;
	}

	front = _relCars[lane][1];
	if (front)
	{
		if (fabs(front->_s) < redZone)
		{
			return maxReturn;
		}
		else
		{
			return (1 / front->_distance);
		}

>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
	}

	return 0;

}
double BehaviourPlanner::bufferCost(int lane)
{

<<<<<<< HEAD
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
		return cost;
		
	}
	
	return 0; // no vehicle in that lane
=======
	// Initial lane, see if any in 30
	car* front = (_relCars[_carCurr._lane][1]);
	car* back = (_relCars[_carCurr._lane][0]);

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
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
}

bool BehaviourPlanner::collision()
{
<<<<<<< HEAD
	car* front = _relCars[_carCurr->_lane][1];
	if (front)
	{ 
		return ((_relCars[_carCurr->_lane][1])->_distance < _redZone * 1.5);
=======
	car* front = _relCars[_carCurr._lane][1];
	if (front)
	{
		return ((_relCars[_carCurr._lane][1])->_distance < redZone);
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285

	}
	return false;
}

<<<<<<< HEAD
void BehaviourPlanner::setEnvironment(const car& carCurr, int prevSize, const vector<vector<double>>& sensor_fusion)
{
	//initialise
	_carCurr = &carCurr;

=======
BehaviourPlanner::BehaviourPlanner(car& carCurr, int prevSize, vector<vector<double>>& sensor_fusion) :_carCurr(carCurr), _prevSize(prevSize)
{
	// load the weights 
	streamIn();


	cout << "\t" << "In Constuctor" << endl;
	//set prevSize
	_prevSize = std::min(maxUsePrev, prevSize);

	cout << "\t" << "Previous size: " << _prevSize << endl;
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285

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
<<<<<<< HEAD
		int lane = (d / _h->_laneWidth);

		if (0 <= lane && lane < _h->_nlanes)
		{
			car temp;
			temp.setValues(x, y, s, d, yaw, lane, speed);
			if (lane == _carCurr->_lane && _carCurr->_s > s)
			{
				temp._distance = (s)-_carCurr->_s;
=======
		int lane = (d / laneWidth);

		if (0 <= lane && lane < nlane)
		{
			car temp(x, y, s, d, yaw, lane, speed);
			if (lane == _carCurr._lane && _carCurr._s > s)
			{
				cout << "Right Behind" << endl;
				temp._distance = (s)-_carCurr._s;
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285

			}
			else
			{
<<<<<<< HEAD
				temp._distance = (s + prevSize * 0.02 * speed) - _carCurr->_endS;
=======
				temp._distance = (s + _prevSize * 0.02 * speed) - _carCurr._s;
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
			}
			_hashCar[lane].emplace_back(temp);
		}
	}

	// populate the lane vector
	nearestCar();

<<<<<<< HEAD
	
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
=======
	cout << "\t" << "Nearest Car" << endl;

	// collise or not
	_collide = collision();

	cout << "\t" << "Collision: " << _collide << endl;

	// set next action
	if (!_collide)
	{
		_next = make_pair(_maxLaneSpeeds[_carCurr._lane], _carCurr._lane);
	}
	else
	{
		_next = choseAction();
	}

	cout << "\t" << "Next action and end: " << _next.first << "," << _next.second << endl;

>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
}

pair<double, int> BehaviourPlanner::choseAction()
{

<<<<<<< HEAD
	double max = _maxReturn;
	pair<double, int> result = make_pair(_maxLaneSpeeds[_suggestedLane], _carCurr->_lane);

	if (_carCurr->_speed < 10)
	{
		return result;
	}
	
	for (int i = 0; i < _h->_nlanes; ++i)
	{
		if (i > _carCurr->_lane + 1 || i < _carCurr->_lane - 1)
=======
	double max = maxReturn;
	pair<double, int> result = make_pair(_maxLaneSpeeds[_carCurr._lane], _carCurr._lane);

	if (_carCurr._speed < 10)
	{
		return result;
	}

	for (int i = 0; i < nlane; ++i)
	{
		if (i > _carCurr._lane + 1 || i < _carCurr._lane - 1)
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
		{
			continue;
		}
		else
		{
<<<<<<< HEAD
			// if safety is max, then we cannot consider it
			if (safetyCost(i) == _maxReturn)
			{
				continue;
			}
			double cost = 
				_laneChangeFactor * laneChangeCost(i) +
				_speedChangeFactor * speedChangeCost(i) + 
				_speedFactor * speedCost(i) + 
				_bufferFactor * bufferCost(i);
=======
			double cost = 0;

			double costAdd;
			costAdd = laneChangeCost(i);
			cout << "Lane change cost: " << costAdd << endl;
			cost += laneChangeFactor * costAdd;

			costAdd = speedChangeCost(i);
			cout << "Speed change cost: " << costAdd << endl;
			cost += speedChangeFactor * costAdd;

			costAdd = speedCost(i);
			cout << "Max speed change cost: " << costAdd << endl;
			cost += speedFactor * costAdd;

			costAdd = bufferCost(i);
			cout << "Buffer cost: " << costAdd << endl;
			cost += bufferFactor * costAdd;


			costAdd = safetyCost(i);
			cout << "Safety cost: " << costAdd << endl;
			cost += safetyFactor * costAdd;
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285

			if (max > cost)
			{
				max = cost;
				result = make_pair(_maxLaneSpeeds[i], i);
			}

		}

	}
<<<<<<< HEAD
	
	// update _lastLane Change if lane change recommended 
	if (result.second != _carCurr->_lane)
	{
		_suggestedLane = result.second;
		_lastLaneChange = steady_clock::now();
	}

=======
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
	return result;
}

pair<double, int> BehaviourPlanner::returnNextAction()
{
	return _next;
}
<<<<<<< HEAD

BehaviourPlanner::BehaviourPlanner(const HighwayMap* h) : _h(h) {}
=======
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
