#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include "DroneNavController.h"
#include "FireDetection.h"
#include "math.h"


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>



extern "C" {
#include "extApi.h"
}

using namespace cv;

class FireDetection
{
public:
	FireDetection();
	~FireDetection();
	
	void init();
	void update();
	void setComVars(int clientId, int camHandle, int gyrohandle);
	void getmemory(unsigned char **p, int rX, int rY);
	Mat eulerAnglesToRotationMatrix(Vec3f &theta);
	Mat createCvImageFromBuffer(unsigned char *bufferImage, int resolutionX, int resolutionY, Mat &mapImage, float currentX, float currentY, float currentZ, Mat cameraOrientation);
	Vec3f getWorldCoordinatesOfPixel(int x, int y, Mat cameraOrientation, int resolutionX, int resolutionY, float currentX, float currentY, float currentZ);
};

