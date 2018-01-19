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


	
	std::vector<Point> activeNavPoints;
	std::vector<int> activeWaitTimes;
	std::vector<float> activeRotAtPoint;

	std::vector<Point> baseNavPointsStart;
	std::vector<int> waitTimesStart;
	std::vector<float> rotAtPointStart;


	std::vector<Point> baseNavPointsLand;
	std::vector<int> waitTimesLand;
	std::vector<float> rotAtPointLand;


	std::vector<Point> navPoints;
	std::vector<int> waitTimes;
	std::vector<float> rotAtPoint;

	float desiredAltitude = 6;
	float currentTargetRotation;


	State currentState;

	int clientID;
	
	int targetHandle;
	int gpsHandle;
	int proxhandle;

	int currentDestIndex;

	int ticksWaited;

	float targetDestErrorMargin = 1.5;

	float maxEnergy = 5;
	float currentEnergy = 5;
	float energyThreshold = 1;
	bool initOnce = false;

	float energyConsumptionPerTick = 0.005;

	Point base = Point(0, -10, 1);

	bool obsticalDetected = false;

	Point getDronePos();

	void createNavPointsOnCircle(const Point& center, float radius, int numberOfPoints);

	void generateNavGridNavPoints();

	double degToRad(double deg);

	void calcRotations(std::vector<Point> points,std::vector<int>& waitTimes, std::vector<float>& rotAtPoint);
	
	double calcAngle(Point vct1, Point vct2);

	double angleAtCorner(Point p1, Point p2, Point p3);



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

