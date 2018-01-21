#include "FireDetection.h"



int clientID;
int camHandle;
int gyrohandle;
char options = 1;
int resolutionConnection2[2] = { 0,0 };
int resX = 256;
int resY = 256;
float gpsX = 1;
float gpsY = 1;
float gpsZ = 1;
float gyroX = 5;
float gyroY = 5;
float gyroZ = 5;
std::vector<cv::Point> fireList = vector<cv::Point>();

string mapwindow = "Map Display";
string sensorwindow = "Sensor Display";

static Mat mapImage;
unsigned char* img;

using namespace cv;
FireDetection::FireDetection()
{
}


FireDetection::~FireDetection()
{
}

void FireDetection::setComVars(int clientId, int camhandle, int gyroHandle)
{
	clientID = clientId;
	camHandle = camhandle;
	gyrohandle = gyroHandle;
}

void FireDetection::init() {
	simxGetFloatSignal(clientID, "gpsX", &gpsX, simx_opmode_streaming);
	simxGetFloatSignal(clientID, "gpsY", &gpsY, simx_opmode_streaming);
	simxGetFloatSignal(clientID, "gpsZ", &gpsZ, simx_opmode_streaming);
	simxGetFloatSignal(clientID, "gyroX", &gyroX, simx_opmode_streaming);
	simxGetFloatSignal(clientID, "gyroY", &gyroY, simx_opmode_streaming);
	simxGetFloatSignal(clientID, "gyroZ", &gyroZ, simx_opmode_streaming);

	mapImage = Mat(256, 256, CV_8UC1, double(0));

	namedWindow(sensorwindow, WINDOW_AUTOSIZE);// Create a window for Sensor display.
	namedWindow(mapwindow, WINDOW_AUTOSIZE);// Create a window for Map display.

	getmemory(&img, resX, resY);

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
}

void FireDetection::update() {
	simxGetVisionSensorImage(clientID, camHandle, &resolutionConnection2[0], &img, 1, simx_opmode_buffer);

	simxGetFloatSignal(clientID, "gpsX", &gpsX, simx_opmode_buffer);
	simxGetFloatSignal(clientID, "gpsY", &gpsY, simx_opmode_buffer);
	simxGetFloatSignal(clientID, "gpsZ", &gpsZ, simx_opmode_buffer);
	simxGetFloatSignal(clientID, "gyroX", &gyroX, simx_opmode_buffer);
	simxGetFloatSignal(clientID, "gyroY", &gyroY, simx_opmode_buffer);
	simxGetFloatSignal(clientID, "gyroZ", &gyroZ, simx_opmode_buffer);
	Vec3f eulerAngles = Vec3f(gyroX, gyroY, gyroZ*2);
	//std::cout << "gyrox: " << gyroX << std::endl;
	//std::cout << "gyroy: " << gyroY << std::endl;
	//std::cout << "gyroz: " << gyroZ << std::endl;

	Mat rotationMatrix = eulerAnglesToRotationMatrix(eulerAngles);
	Mat cvImage = createCvImageFromBuffer(img, resX, resY, mapImage, gpsY, gpsX, gpsZ, rotationMatrix);
	imshow(sensorwindow, cvImage);
	imshow(mapwindow, mapImage);
	waitKey(1);
}

// Calculates rotation matrix given euler angles.
Mat FireDetection::eulerAnglesToRotationMatrix(Vec3f &theta)
{
	// Calculate rotation about x axis
	Mat R_x = (Mat_<float>(3, 3) <<
		1, 0, 0,
		0, cos(theta[0]), -sin(theta[0]),
		0, sin(theta[0]), cos(theta[0])
		);

	// Calculate rotation about y axis
	Mat R_y = (Mat_<float>(3, 3) <<
		cos(theta[1]), 0, sin(theta[1]),
		0, 1, 0,
		-sin(theta[1]), 0, cos(theta[1])
		);

	// Calculate rotation about z axis
	Mat R_z = (Mat_<float>(3, 3) <<
		cos(theta[2]), -sin(theta[2]), 0,
		sin(theta[2]), cos(theta[2]), 0,
		0, 0, 1);


	// Combined rotation matrix
	Mat R = R_x * R_y * R_z;

	return R;

}

void FireDetection::getmemory(unsigned char **p, int rX, int rY) {
	*p = (unsigned char *)malloc(rX*rY * 3 * sizeof(unsigned char));
}

Vec3f FireDetection::getWorldCoordinatesOfPixel(int x, int y, Mat cameraOrientation, int resolutionX, int resolutionY, float currentX, float currentY, float currentZ) {
	//90deg angle
	float xdir = (x - (resolutionX / 2.0)) / (resolutionX / 2.0);
	float ydir = (y - (resolutionY / 2.0)) / (resolutionY / 2.0);
	float zdir = -1;
	Vec3f dir = Vec3f(xdir, ydir, zdir);
	Mat tmp = cameraOrientation * Mat(dir, false);
	tmp.copyTo(Mat(dir, false));
	float length = (currentZ / dir[2]);
	return Vec3f(currentX + length*dir[0], currentY + length*dir[1], 0);
}

Mat FireDetection::createCvImageFromBuffer(unsigned char *bufferImage, int resolutionX, int resolutionY, Mat &mapomage, float currentX, float currentY, float currentZ, Mat cameraOrientation)
{
	Mat cvImage(resolutionX, resolutionY, CV_8UC1, double(0));
	//Mat a = cv::perspectiveTransform();
	//calculate viewed area size
	float height = currentZ;
	//field of view, maparea
	//TODO: get from Sensor, groundsegment
	float fow = 90;
	float sizeOfMapArea = 50;
	float sizeTerrainInView = 2 * height*tan((fow / 2)*3.14159 / 180);
	float percentageOfMapInView = sizeTerrainInView / sizeOfMapArea;
	//get from scene eventually
	float leftXborder = -10;
	float bottomYBorder = -10;
	//in pixels
	int pixelRangeOfView = 256 * percentageOfMapInView;
	int xPixelOfCurrentPos = (currentX - leftXborder) / sizeOfMapArea * 256;
	int yPixelOfCurrentPos = (currentY - leftXborder) / sizeOfMapArea * 256;
	int ViewLeftXBorderPixel = xPixelOfCurrentPos - (pixelRangeOfView / 2);
	int ViewBottomYBorderPixel = yPixelOfCurrentPos - (pixelRangeOfView / 2);

	//image creation
	int print;
	int count;
	count = resolutionX*resolutionY;
	for (print = 0; print != count; print++)
	{
		cv::Point currentPixel = cv::Point(print%resolutionX, print / resolutionX);
		if (bufferImage[print] != 0)
		{
			//color mappixel
			//int mapPixelX = ViewLeftXBorderPixel + (print%resolutionX)*percentageOfMapInView;
			//int mapPixelY = ViewBottomYBorderPixel + (print / resolutionX) * percentageOfMapInView;
			Vec3f mapCoordinates = getWorldCoordinatesOfPixel(print%resolutionX, print / resolutionX, cameraOrientation, resolutionX, resolutionY, currentX, currentX, currentZ);
			int mapPixelX = ((mapCoordinates[0] + sizeOfMapArea / 2) / sizeOfMapArea) * 256;
			int mapPixelY = ((mapCoordinates[1] + sizeOfMapArea / 2) / sizeOfMapArea) * 256;
			if (mapPixelX >= 0 && mapPixelY >= 0 && mapPixelX < 256 && mapPixelY < 256) {
				cv::Point currentMapPixel = cv::Point(mapPixelX, mapPixelY);
				mapImage.at<uchar>(currentMapPixel) = bufferImage[print];
				float mindistance = 10000;
				for each (cv::Point fire in fireList)
				{
					float distance = cv::norm(fire - currentMapPixel);
					if (distance < mindistance) {
						mindistance = distance;
					}
				}
				if (mindistance > 10) {
					std::cout << "fire detected at:" << mapCoordinates[0] << "/" << mapCoordinates[1] << std::endl;
					fireList.push_back(currentMapPixel);
				}
			}

		}
		cvImage.at<uchar>(currentPixel) = bufferImage[print];
	}
	//Mat dst;
	//normalize(cvImage, dst, 0, 1, cv::NORM_MINMAX);
	return cvImage;
}