#include "DroneNavController.h"
#include <iostream>


DroneNavController::DroneNavController()
{
	ticksWaited = 0;
	currentTargetRotation = -1.6;
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
		waitTimes.push_back(0);
		rotAtPoint.push_back(0);


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

	float xOffset = 0;
	float yOffset = 0;
	float zOffset = 0;

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
	
	float dronePos2[3] = { 0,0,0 };

	if (first)
	{
		simxGetObjectPosition(clientID, droneHandle, -1, dronePos2, simx_opmode_streaming);
		first = false;
	}
	else {
		simxGetObjectPosition(clientID, droneHandle, -1, dronePos2, simx_opmode_buffer);
	}
	
	//overwrite the drone pos we get from the gps because the gps coordinates change when the drone is rotated, which makes no sense

	//dronePos.x = dronePos2[0];
	//dronePos.y = dronePos2[1];
	//dronePos.z = dronePos2[2];

	if (dronePos.distance(currentDest) <= targetDestErrorMargin)
	{
	
		int timeToWait = waitTimes.at(currentDestIndex);
		
		if (timeToWait == ticksWaited)
		{

			//move to the next pos and wrap around if needed
			currentDestIndex++;
			currentDestIndex = currentDestIndex%navPoints.size();

			Point newDestPoint = navPoints.at(currentDestIndex);

			//set a new dest
			float newDest[3] = { newDestPoint.x, newDestPoint.y, newDestPoint.z };
			simxSetObjectPosition(clientID, targetHandle, -1, newDest, simx_opmode_oneshot);
			ticksWaited = 0;
		}
		else {
			
			//during half time we rotate the drone
			
			

			if (ticksWaited == timeToWait / 2) {
				currentTargetRotation = currentTargetRotation + rotAtPoint.at(currentDestIndex);

				float eulerAngle[3] = { 0,0,currentTargetRotation };

				simxSetObjectOrientation(clientID, targetHandle, -1, eulerAngle, simx_opmode_oneshot);
			}

			


			ticksWaited++;
		}

		

	}
	else{
		//the drone is not therer yet	
	}

}

void DroneNavController::generateNavGridNavPoints()
{
	int size = 10;
	int waitTime = 150;


	Point p1;
	p1.x = 0;
	p1.y = 0;
	p1.z = desiredAltitude;
	navPoints.push_back(p1);
	waitTimes.push_back(waitTime);
	rotAtPoint.push_back(1.6);

	Point p2;
	p2.x = size;
	p2.y = 0;
	p2.z = desiredAltitude;
	navPoints.push_back(p2);
	waitTimes.push_back(waitTime);
	rotAtPoint.push_back(1.6);

	Point p3;
	p3.x = size;
	p3.y = size;
	p3.z = desiredAltitude;
	navPoints.push_back(p3);
	waitTimes.push_back(waitTime);
	rotAtPoint.push_back(1.6);

	Point p4;
	p4.x = 0;
	p4.y = size;
	p4.z = desiredAltitude;
	navPoints.push_back(p4);
	waitTimes.push_back(waitTime);
	rotAtPoint.push_back(1.6);


}

void DroneNavController::genNavPoints() 
{
	bool createCircle = false;

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