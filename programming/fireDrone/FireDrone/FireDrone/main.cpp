
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include "DroneNavController.h"


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "FireDetection.h"


extern "C" {
	#include "extApi.h"
}

using namespace cv;




void placeTrees(int clientID, int density,int size,float xStart, float yStart,int variance)
{
	cv::RNG rng = RNG(12345);

	for (int i = 0; i < size; i++)
	{
		for (int j = 0; j < size; j++)
		{
			
			int han = 0;
			std::string treePath = "C:/Program Files/V-REP3/V-REP_PRO_EDU/models/nature/Tree.ttm";
			simxLoadModel(clientID, treePath.c_str(), 0, &han, simx_opmode_blocking);

			int xRandom = rng.uniform(variance*-1, variance);
			int yRandom = rng.uniform(variance*-1, variance);


			float xPos = xStart + (i*density) + xRandom;
			float yPos = yStart + (j*density) + yRandom;

			float newDest[3] = { xPos, yPos, 2 };

			simxSetObjectPosition(clientID, han, -1, newDest, simx_opmode_oneshot);

		}
	}

}


void placeFires(int clientID, int numberOfFires, int bounds)
{

	cv::RNG rng = RNG(589);

	for (int j = 0; j < numberOfFires; j++)
	{

		int han = 0;
		std::string firePath = "C:/Program Files/V-REP3/V-REP_PRO_EDU/programming/fireDrone/FireDrone/Scene/InfraRedFire.ttm";
		simxLoadModel(clientID, firePath.c_str(), 0, &han, simx_opmode_blocking);

		int xRandom = rng.uniform(bounds*-1, bounds);
		int yRandom = rng.uniform(bounds*-1, bounds);


		float xPos = xRandom;
		float yPos = yRandom;

		//float xPos = 0;
		//float yPos = 0;



		float newDest[3] = { xPos, yPos, 0 };

		simxSetObjectPosition(clientID, han, -1, newDest, simx_opmode_oneshot);

	}
}



int main(int argc, char* argv[])
{
	int portNb =20001;
	int clientID = simxStart((simxChar*)"127.0.0.1", portNb, true, true, 2000, 5);
	std::string droneName = "Quadricopter";
	std::string frontCamName = "Quadricopter_frontCamera";
	std::string gyroscope = "GyroSensor";


	DroneNavController nc;
	FireDetection fd;

	//placeTrees(clientID,2,10,0,0,5);
	//placeFires(clientID,8,10);

	if (clientID != -1){

		//camera stuff
		


		namedWindow("Display window", WINDOW_AUTOSIZE);// Create a window for display.

		int resX = 256;
		int resY = 256;
		int res[2] = { 256, 256 };

		int gyrohandle = 0;
		int camHandle = 0;
		unsigned char* img;



		simxGetObjectHandle(clientID, "Infrared_Sensor", &camHandle, simx_opmode_blocking);
		simxGetObjectHandle(clientID, "GyroSensor", &gyrohandle, simx_opmode_blocking);
		
		char options = 1;
		int resolutionConnection2[2] = { 0,0 };

		int proxhandle;
		simxGetObjectHandle(clientID, "Proximity_sensor0", &proxhandle, simx_opmode_blocking);


		if (simxReadVisionSensor(clientID, camHandle, NULL, NULL, NULL, simx_opmode_oneshot_wait) == simx_error_noerror)
		{
			printf("Test ok.......simxReadVisionSensor is OK.....\n");
		}if (simxGetVisionSensorImage(clientID, camHandle, &resolutionConnection2[0], &img, 1, simx_opmode_oneshot_wait) == simx_error_noerror)
		{
			printf("Test ok......simxGetVisionSensorImage() is OK.....\n");
			printf("after Get Image:\n");
			printf("resolutionConnection2[0]=%d\n", resolutionConnection2[0]);
			printf("resolutionConnection2[1]=%d\n", resolutionConnection2[1]);
		}

		simxGetVisionSensorImage(clientID, camHandle, &resolutionConnection2[0], &img, 1, simx_opmode_streaming);

		


		int droneHandle = 0;
		simxGetObjectHandle(clientID, "Quadricopter", &droneHandle, simx_opmode_blocking);

		int thandle = 0;
		simxGetObjectHandle(clientID, "Quadricopter_target", &thandle, simx_opmode_blocking);
	
	

		float gpsX = 1;
		float gpsY = 1;
		float gpsZ = 1;

		simxGetFloatSignal(clientID, "gpsX", &gpsX, simx_opmode_streaming);
		simxGetFloatSignal(clientID, "gpsY", &gpsY, simx_opmode_streaming);
		simxGetFloatSignal(clientID, "gpsZ", &gpsZ, simx_opmode_streaming);

		float newPos[3] = { 0, 0, 0 };

		simxGetObjectPosition(clientID, droneHandle, -1, newPos, simx_opmode_streaming);
		
		bool initOnce = false;

	
		fd.setComVars(clientID, camHandle, gyrohandle);
		nc.setComVars(clientID, thandle, proxhandle);
		fd.init();


		simxGetObjectPosition(clientID, thandle, -1, newPos, simx_opmode_buffer);


		float eulerAngle[3] = { 0,0,-1.6 };

		simxSetObjectOrientation(clientID, thandle, -1, eulerAngle, simx_opmode_oneshot);


		bool first = true;

		nc.droneHandle = droneHandle;


		simxUChar* detectState = new simxUChar();
		float detectPos[3] = { 0, 0, 0 };
		simxReadProximitySensor(clientID, proxhandle, detectState, detectPos, NULL, NULL, simx_opmode_streaming);

		while (simxGetConnectionId(clientID) != -1)
		{
			
			nc.newUpdate();
			fd.update();
			extApi_sleepMs(50);
		}
		simxFinish(clientID);
	}
	else
	{
		//connection failed
		return 0;
	}

}