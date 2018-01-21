#pragma once

#include <vector>
#include "Point.h"


extern "C" {
#include "extApi.h"
}

class DroneNavController
{

#define PI 3.14159265359

private:
	

	enum State
	{
		Landed,
		TookOff,
		ReadyToLand,
		Navigating,
		BackToBase
	};


	
	std::vector<MyPoint> activeNavPoints;
	std::vector<int> activeWaitTimes;
	std::vector<float> activeRotAtPoint;

	std::vector<MyPoint> baseNavPointsStart;
	std::vector<int> waitTimesStart;
	std::vector<float> rotAtPointStart;


	std::vector<MyPoint> baseNavPointsLand;
	std::vector<int> waitTimesLand;
	std::vector<float> rotAtPointLand;


	std::vector<MyPoint> navPoints;
	std::vector<int> waitTimes;
	std::vector<float> rotAtPoint;

	float desiredAltitude = 8;
	float currentTargetRotation;


	State currentState;

	int clientID;
	
	int targetHandle;
	int gpsHandle;
	int proxhandle;

	int currentDestIndex;

	int ticksWaited;

	float targetDestErrorMargin = 1.5;

	float maxEnergy = 8;
	float currentEnergy = 8;
	float energyThreshold = 1;
	bool initOnce = false;

	float energyConsumptionPerTick = 0.005;

	MyPoint base = MyPoint(0, -10, 1);

	bool obsticalDetected = false;

	MyPoint getDronePos();

	void createNavPointsOnCircle(const MyPoint& center, float radius, int numberOfPoints);

	void generateNavGridNavPoints();

	double degToRad(double deg);

	void calcRotations(std::vector<MyPoint> points,std::vector<int>& waitTimes, std::vector<float>& rotAtPoint);
	
	double calcAngle(MyPoint vct1, MyPoint vct2);

	double angleAtCorner(MyPoint p1, MyPoint p2, MyPoint p3);



public:
	//dbug stuff

	int droneHandle;

	bool first = true;



public:
	DroneNavController();
	~DroneNavController();

	void genNavPoints();

	void setComVars(int cID,int th,int ph);

	void update();
	
	void newUpdate();

	void startNavigation();

	void setupStartAndLand();
};

