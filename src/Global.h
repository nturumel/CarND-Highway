#pragma once
namespace highway
{
	extern int laneWidth;
	extern int nlane;
	extern double maxAcc;
	extern double maxVel;
	extern double maxReturn;
	extern int maxUsePrev;
	extern double newSize;
	extern double redZone;
	extern double laneChangeFactor; 
	extern double  speedChangeFactor;
	extern double speedFactor;
	extern double bufferFactor;
	extern double safetyFactor;
	void streamIn();
}