#include "Global.h"
#include <iostream>
#include <fstream>

using namespace std;
using namespace highway;

namespace highway
{
	extern int laneWidth(4);
	extern int nlane(3);
	extern double maxAcc(5);
	extern double maxVel(60);
	extern double maxReturn(1e6);
	extern int maxUsePrev(50);
	extern double newSize(50);
	extern double redZone(30.0);
	extern double laneChangeFactor(0);
	extern double  speedChangeFactor(0);
	extern double speedFactor(0);
	extern double bufferFactor(0);
	extern double safetyFactor(0);
	
	void streamIn()
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
}