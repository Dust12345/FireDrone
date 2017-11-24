
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include "DroneNavController.h"


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>


extern "C" {
	#include "extApi.h"
}

using namespace cv;


void getmemory(unsigned char **p, int rX, int rY) {
	*p = (unsigned char *)malloc(rX*rY * 3 * sizeof(unsigned char));
}

Mat createCvImageFromBuffer(unsigned char *bufferImage, int resolutionX, int resolutionY)
{
	Mat cvImage(resolutionX, resolutionY,CV_8UC1, double(0));
	int print;
	int count;
	count = resolutionX*resolutionY;
	for (print = 0; print != count; print++)
	{
		cvImage.at<uchar>(cv::Point(print%resolutionX, print / resolutionX)) = bufferImage[print];
	}
	//Mat dst;
	//normalize(cvImage, dst, 0, 1, cv::NORM_MINMAX);
	return cvImage;
}



int main(int argc, char* argv[])
{
	int portNb =20001;
	int clientID = simxStart((simxChar*)"127.0.0.1", portNb, true, true, 2000, 5);
	std::string droneName = "Quadricopter";
	std::string frontCamName = "Quadricopter_frontCamera";


	DroneNavController nc;

	if (clientID != -1){

		//camera stuff


		namedWindow("Display window", WINDOW_AUTOSIZE);// Create a window for display.

		int resX = 256;
		int resY = 256;
		int res[2] = { 256, 256 };

		int camHandle = 0;
		unsigned char* img;
		simxGetObjectHandle(clientID, "Infrared_Sensor", &camHandle, simx_opmode_blocking);
		//simxGetVisionSensorImage(clientID, camHandle, res, &img, 0, simx_opmode_streaming);
		getmemory(&img, resX, resY);
		char options = 1;
		int resolutionConnection2[2] = { 0,0 };


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

		//----


		int droneHandle = 0;
		simxGetObjectHandle(clientID, "Quadricopter", &droneHandle, simx_opmode_blocking);

		//int res[2] = { 400, 400};

		//int camHandle = 0;
		//unsigned char* img[1];

		int thandle = 0;
		simxGetObjectHandle(clientID, "Quadricopter_target", &thandle, simx_opmode_blocking);

		//simxGetObjectHandle(clientID, "Quadricopter_frontCamera", &camHandle, simx_opmode_blocking);
		//simxGetVisionSensorImage(clientID, camHandle, res, img, 1, simx_opmode_streaming);

		//unsigned char* gpsSignal[1];
		//int gpsDataLength = 0;

		float gpsX = 1;
		float gpsY = 1;
		float gpsZ = 1;

		simxGetFloatSignal(clientID, "gpsX", &gpsX, simx_opmode_streaming);
		simxGetFloatSignal(clientID, "gpsY", &gpsY, simx_opmode_streaming);
		simxGetFloatSignal(clientID, "gpsZ", &gpsZ, simx_opmode_streaming);

		float newPos[3] = { 0, 0, 0 };

		simxGetObjectPosition(clientID, droneHandle, -1, newPos, simx_opmode_streaming);
		
		bool initOnce = false;
		
		nc.setComVars(clientID, thandle);
		
		simxGetObjectPosition(clientID, thandle, -1, newPos, simx_opmode_buffer);


		int prophandle;
		//test stuff below
		
		float v1 = 0;
		float v2 = 0;
		float v3 = 0;
		float v4 = 0;

		bool first = true;

		while (simxGetConnectionId(clientID) != -1)
		{
		
			simxGetVisionSensorImage(clientID, camHandle, &resolutionConnection2[0], &img, 1, simx_opmode_buffer);

			Mat cvImage = createCvImageFromBuffer(img, resX, resY);
			imshow("Display window", cvImage);
			waitKey(1);

			if (!initOnce){
				nc.startNavigation();
				initOnce = true;
			}
			else{
				nc.update();
			}		
		

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