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
	std::vector<float> rotateAtPoint;

	float desiredAltitude = 1;

	int clientID;
	
	int targetHandle;
	int gpsHandle;

	int currentDestIndex;

	float targetDestErrorMargin = 2;

	Point getDronePos();

	void createNavPointsOnCircle(const Point& center, float radius, int numberOfPoints);

	void generateNavGridNavPoints();

	

public:
	DroneNavController();
	~DroneNavController();

	void genNavPoints();

	void setComVars(int cID,int th);

	void update();

	void startNavigation();
};

