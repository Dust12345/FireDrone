#include "DroneNavController.h"
#include <iostream>

DroneNavController::DroneNavController()
{
	ticksWaited = 0;
	currentTargetRotation = 1.57;
	genNavPoints();
	currentState = Landed;
	activeNavPoints = baseNavPointsStart;
	activeWaitTimes = waitTimesStart;
	activeRotAtPoint = rotAtPointStart;
}


DroneNavController::~DroneNavController()
{
}


void DroneNavController::createNavPointsOnCircle(const MyPoint& center, float radius, int numberOfPoints)
{
	

	float stepSize = (PI*2)  / numberOfPoints;
	float rad = 0;

	for (int i = 0; i < numberOfPoints; i++)
	{
		MyPoint p;

		p.x = center.x + radius * cos(rad);
		p.y = center.y + radius * sin(rad);
		p.z = desiredAltitude;
		navPoints.push_back(p);
		waitTimes.push_back(0);
		rotAtPoint.push_back(0);


		rad = rad + stepSize;
	}
}

void DroneNavController::setComVars(int cID, int th,int ph)
{
	clientID = cID;
	targetHandle = th;
	proxhandle = ph;

}

void DroneNavController::startNavigation()
{
	MyPoint newDestPoint = activeNavPoints.at(currentDestIndex);

	//set a new dest
	float newDest[3] = { newDestPoint.x, newDestPoint.y, newDestPoint.z };
	simxSetObjectPosition(clientID, targetHandle, -1, newDest, simx_opmode_oneshot);
}

MyPoint DroneNavController::getDronePos()
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

	MyPoint dronePos;
	dronePos.x = gpsX + xOffset;
	dronePos.y = gpsY + yOffset;
	dronePos.z = gpsZ + zOffset;

	return dronePos;
}

double DroneNavController::degToRad(double deg)
{
	return deg*(PI / 180);
}

void  DroneNavController::update()
{
	//check the distance between the target and the drone	

	MyPoint dronePos = getDronePos();

	MyPoint currentDest = navPoints.at(currentDestIndex);

	if (dronePos.distance(currentDest) <= targetDestErrorMargin)
	{
	
		int timeToWait = waitTimes.at(currentDestIndex);
		
		if (timeToWait == ticksWaited)
		{

			//move to the next pos and wrap around if needed
			currentDestIndex++;
			currentDestIndex = currentDestIndex%navPoints.size();

			MyPoint newDestPoint = navPoints.at(currentDestIndex);

			//set a new dest
			float newDest[3] = { newDestPoint.x, newDestPoint.y, newDestPoint.z };
			simxSetObjectPosition(clientID, targetHandle, -1, newDest, simx_opmode_oneshot);
			ticksWaited = 0;
		}
		else {
			
			//during half time we rotate the drone
			if (ticksWaited == timeToWait / 2)
			{
				//double rot = degToRad(rotAtPoint.at(currentDestIndex));

				currentTargetRotation = currentTargetRotation + rotAtPoint.at(currentDestIndex);
				float eulerAngle[3] = { 0,0,currentTargetRotation };

				simxSetObjectOrientation(clientID, targetHandle, -1, eulerAngle, simx_opmode_oneshot);
			}

			ticksWaited++;
		}

		

	}
	else{
		simxUChar* detectState = new simxUChar();
		//check for collision
		simxReadProximitySensor(clientID, proxhandle, detectState, NULL, NULL, NULL, simx_opmode_buffer);

		if ((*detectState) == 0) {
			//no detection
			if (obsticalDetected) {
				obsticalDetected = false;
				//detection
				MyPoint newDestPoint = navPoints.at(currentDestIndex);

			
				float newDest[3] = { newDestPoint.x, newDestPoint.y, newDestPoint.z };
				simxSetObjectPosition(clientID, targetHandle, -1, newDest, simx_opmode_oneshot);
			}
		}
		else {		
			
			if (!obsticalDetected)
			{
				
				//detection
				MyPoint newDestPoint = navPoints.at(currentDestIndex);

				float newDest[3] = { newDestPoint.x, newDestPoint.y, newDestPoint.z+3 };
				simxSetObjectPosition(clientID, targetHandle, -1, newDest, simx_opmode_oneshot);
				obsticalDetected = true;
			}

		
		}
	}

}

void DroneNavController::generateNavGridNavPoints()
{
	int size = 30;
	int waitTime =200;


	MyPoint p1;
	p1.x = 0;
	p1.y = 0;
	p1.z = desiredAltitude;
	navPoints.push_back(p1);

	MyPoint p2;
	p2.x = size;
	p2.y = 0;
	p2.z = desiredAltitude;
	navPoints.push_back(p2);

	MyPoint p3;
	p3.x = size;
	p3.y = size;
	p3.z = desiredAltitude;
	navPoints.push_back(p3);

	MyPoint p4;
	p4.x = 0;
	p4.y = size;
	p4.z = desiredAltitude;
	navPoints.push_back(p4);

	calcRotations(navPoints, waitTimes, rotAtPoint);
}


double DroneNavController::calcAngle(MyPoint vct1, MyPoint vct2)
{
	double dot = vct1.x*vct2.x + vct1.y*vct2.y;     // dot product between[x1, y1] and [x2, y2]
	double det = vct1.x*vct2.y - vct1.y*vct2.x;     // determinant
	double angle = atan2(det, dot);					//atan2(y, x) or atan2(sin, cos)

	return angle;
}

double DroneNavController::angleAtCorner(MyPoint p1, MyPoint p2, MyPoint p3)
{
	MyPoint vct1;
	vct1.x = p2.x - p1.x;
	vct1.y = p2.y - p1.y;

	MyPoint vct2;
	vct2.x = p3.x - p2.x;
	vct2.y = p3.y - p2.y;

	double angle = calcAngle(vct1, vct2);

	return angle;
}

void DroneNavController::calcRotations(std::vector<MyPoint> points,std::vector<int>& wt, std::vector<float>& rt)
{
	double waitPerAngle = 127;

	//special case for the first point
	MyPoint p1 = points.at(points.size() - 1);
	MyPoint p2 = points.at(0);
	MyPoint p3 = points.at(1);
	
	double angle = angleAtCorner(p1, p2, p3);

	wt.push_back(angle*waitPerAngle);
	rt.push_back(angle);

	std::cout << angle << std::endl;

	for (int i = 1; i < points.size(); i++)
	{		
		p1 = points.at(points.size() - 1);
		p2 = points.at(0);
		p3 = points.at(1);

		angle = angleAtCorner(p1, p2, p3);

		wt.push_back(angle*waitPerAngle);
		rt.push_back(angle);
	}


	//special case at last corner
	p1 = points.at(points.size() - 2);
	p2 = points.at(points.size() - 1);
	p3 = points.at(0);

	angle = angleAtCorner(p1, p2, p3);

	wt.push_back(angle*waitPerAngle);
	rt.push_back(angle);

	

}



void DroneNavController::newUpdate()
{

	currentEnergy = currentEnergy - energyConsumptionPerTick;

	if (!initOnce)
	{
		startNavigation();
		initOnce = true;
		return;
	}

	//check the distance between the target and the drone	

	MyPoint dronePos = getDronePos();

	MyPoint currentDest = activeNavPoints.at(currentDestIndex);

	if (dronePos.distance(currentDest) <= targetDestErrorMargin)
	{

		int timeToWait = activeWaitTimes.at(currentDestIndex);

		if (timeToWait == ticksWaited)
		{

			//move to the next pos and wrap around if needed
			currentDestIndex++;


			if(currentDestIndex >= activeNavPoints.size())
			{
				if (currentState == BackToBase)
				{
					currentState = Landed;
					activeNavPoints = baseNavPointsStart;
					activeRotAtPoint = rotAtPointStart;
					activeWaitTimes = waitTimesStart;
					currentDestIndex = 0;
					currentEnergy = maxEnergy;
				}else if (currentState == Landed)
				{
					currentState = Navigating;
					activeNavPoints = navPoints;
					activeRotAtPoint = rotAtPoint;
					activeWaitTimes = waitTimes;
					currentDestIndex = 0;
					initOnce = false;
				}
			}

			currentDestIndex = currentDestIndex%activeNavPoints.size();
			if (currentDestIndex == 0)
			{
				if (currentState == Navigating) {
					if (currentEnergy <= energyThreshold) {
						currentState = BackToBase;
						activeNavPoints = baseNavPointsLand;
						activeRotAtPoint = rotAtPointLand;
						activeWaitTimes = waitTimesLand;
						currentDestIndex = 0;
					}
				}
			}


			currentDestIndex = currentDestIndex%activeNavPoints.size();

			MyPoint newDestPoint = activeNavPoints.at(currentDestIndex);

			//set a new dest
			float newDest[3] = { newDestPoint.x, newDestPoint.y, newDestPoint.z };
			simxSetObjectPosition(clientID, targetHandle, -1, newDest, simx_opmode_oneshot);
			ticksWaited = 0;
		}
		else {

			//during half time we rotate the drone
			if (ticksWaited == timeToWait / 2)
			{
				//double rot = degToRad(rotAtPoint.at(currentDestIndex));

				currentTargetRotation = currentTargetRotation + activeRotAtPoint.at(currentDestIndex);
				float eulerAngle[3] = { 0,0,currentTargetRotation };

				simxSetObjectOrientation(clientID, targetHandle, -1, eulerAngle, simx_opmode_oneshot);
			}
			ticksWaited++;
		}
	}
	else {

		if (currentState == Navigating) {
			simxUChar* detectState = new simxUChar();
			//check for collision
			simxReadProximitySensor(clientID, proxhandle, detectState, NULL, NULL, NULL, simx_opmode_buffer);

			if ((*detectState) == 0) {
				//no detection
				if (obsticalDetected) {
					obsticalDetected = false;
					//detection
					MyPoint newDestPoint = activeNavPoints.at(currentDestIndex);

					float newDest[3] = { newDestPoint.x, newDestPoint.y, newDestPoint.z };
					simxSetObjectPosition(clientID, targetHandle, -1, newDest, simx_opmode_oneshot);
				}
			}
			else {

				if (!obsticalDetected)
				{
					//detection
					MyPoint newDestPoint = activeNavPoints.at(currentDestIndex);

					float newDest[3] = { newDestPoint.x, newDestPoint.y, newDestPoint.z + 4 };
					simxSetObjectPosition(clientID, targetHandle, -1, newDest, simx_opmode_oneshot);
					obsticalDetected = true;
				}


			}
		}


	}
}


void DroneNavController::setupStartAndLand()
{

	MyPoint readyPoint = base;

	readyPoint.z = desiredAltitude;

	baseNavPointsStart.push_back(readyPoint);
	waitTimesStart.push_back(130);
	rotAtPointStart.push_back(0);
	baseNavPointsStart.push_back(navPoints.at(0));
	waitTimesStart.push_back(400);
	rotAtPointStart.push_back(-3.14159);

	baseNavPointsLand.push_back(readyPoint);
	waitTimesLand.push_back(130);
	rotAtPointLand.push_back(0);
	baseNavPointsLand.push_back(base);
	waitTimesLand.push_back(400);
	rotAtPointLand.push_back(-3.14159);
}

void DroneNavController::genNavPoints() 
{


	

	bool createCircle = false;

	if (createCircle)
	{
		MyPoint center;
		center.x = 0;
		center.y = 0;
		center.z = 0;

		createNavPointsOnCircle(center, 10, 36);
	}
	else{
		generateNavGridNavPoints();
	}
	
	currentDestIndex = 0;

	setupStartAndLand();

}