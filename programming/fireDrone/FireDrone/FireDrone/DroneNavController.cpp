#include "DroneNavController.h"
#include <iostream>


DroneNavController::DroneNavController()
{
	genNavPoints();
}


DroneNavController::~DroneNavController()
{
}


void DroneNavController::createNavPointsOnCircle(const Point& center, float radius, int numberOfPoints)
{
	float pi = 3.14;

	float stepSize = (pi*2)  / numberOfPoints;
	float rad = 0;

	

	for (int i = 0; i < numberOfPoints; i++)
	{
		Point p;

		p.x = center.x + radius * cos(rad);
		p.y = center.y + radius * sin(rad);
		p.z = desiredAltitude;
		navPoints.push_back(p);
		rotateAtPoint.push_back(0);


		rad = rad + stepSize;
	}
}

void DroneNavController::setComVars(int cID, int th)
{
	clientID = cID;
	targetHandle = th;

}

void DroneNavController::startNavigation()
{
	Point newDestPoint = navPoints.at(currentDestIndex);

	//set a new dest
	float newDest[3] = { newDestPoint.x, newDestPoint.y, newDestPoint.z };
	simxSetObjectPosition(clientID, targetHandle, -1, newDest, simx_opmode_oneshot);
}

Point DroneNavController::getDronePos()
{

	float xOffset = -1;
	float yOffset = +1;
	float zOffset = +0.5;

	float gpsX = 1;
	float gpsY = 1;
	float gpsZ = 1;

	//update gps
	simxGetFloatSignal(clientID, "gpsX", &gpsX, simx_opmode_buffer);
	simxGetFloatSignal(clientID, "gpsY", &gpsY, simx_opmode_buffer);
	simxGetFloatSignal(clientID, "gpsZ", &gpsZ, simx_opmode_buffer);

	Point dronePos;
	dronePos.x = gpsX + xOffset;
	dronePos.y = gpsY + yOffset;
	dronePos.z = gpsZ + zOffset;

	return dronePos;
}

void  DroneNavController::update()
{
	//check the distance between the target and the drone	

	Point dronePos = getDronePos();

	Point currentDest = navPoints.at(currentDestIndex);

	if (dronePos.distance(currentDest) <= targetDestErrorMargin)
	{
		//move to the next pos and wrap around if needed
		currentDestIndex++;
		currentDestIndex = currentDestIndex%navPoints.size();

		Point newDestPoint =  navPoints.at(currentDestIndex);

		//set a new dest
		float newDest[3] = { newDestPoint.x, newDestPoint.y, newDestPoint.z };
		simxSetObjectPosition(clientID, targetHandle, -1, newDest, simx_opmode_oneshot);

	}
	else{
		//the drone is not therer yet	
	}

}

void DroneNavController::generateNavGridNavPoints()
{
	int size = 5;

	Point p1;
	p1.x = 0;
	p1.y = 0;
	p1.z = desiredAltitude;
	navPoints.push_back(p1);
	rotateAtPoint.push_back(90);

	Point p2;
	p2.x = size;
	p2.y = 0;
	p2.z = desiredAltitude;
	navPoints.push_back(p2);
	rotateAtPoint.push_back(90);

	Point p3;
	p3.x = size;
	p3.y = size;
	p3.z = desiredAltitude;
	navPoints.push_back(p3);
	rotateAtPoint.push_back(90);

	Point p4;
	p4.x = 0;
	p4.y = size;
	p4.z = desiredAltitude;
	navPoints.push_back(p4);
	rotateAtPoint.push_back(90);


}

void DroneNavController::genNavPoints() 
{
	bool createCircle = true;

	if (createCircle)
	{
		Point center;
		center.x = 0;
		center.y = 0;
		center.z = 0;

		createNavPointsOnCircle(center, 10, 36);
	}
	else{
		generateNavGridNavPoints();
	}
	
	currentDestIndex = 0;

}