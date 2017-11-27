#pragma once

#include <vector>
#include "Point.h"


extern "C" {
#include "extApi.h"
}

class DroneNavController
{

private:
	
	std::vector<Point> navPoints;
	std::vector<int> waitTimes;
	std::vector<float> rotAtPoint;

	float desiredAltitude = 6;
	float currentTargetRotation;


	int clientID;
	
	int targetHandle;
	int gpsHandle;

	int currentDestIndex;

	int ticksWaited;

	float targetDestErrorMargin = 1.5;

	Point getDronePos();

	void createNavPointsOnCircle(const Point& center, float radius, int numberOfPoints);

	void generateNavGridNavPoints();

	
public:
	//dbug stuff

	int droneHandle;

	bool first = true;



public:
	DroneNavController();
	~DroneNavController();

	void genNavPoints();

	void setComVars(int cID,int th);

	void update();

	void startNavigation();
};

