#include "Point.h"
#include <math.h>

MyPoint::~MyPoint()
{
}

float MyPoint::distance(const MyPoint& p)
{
	return sqrt((x - p.x)*(x - p.x) + (y - p.y)*(y - p.y) + (z - p.z)*(z - p.z));
}