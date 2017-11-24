#include "Point.h"
#include <math.h>

Point::~Point()
{
}

float Point::distance(const Point& p)
{
	return sqrt((x - p.x)*(x - p.x) + (y - p.y)*(y - p.y) + (z - p.z)*(z - p.z));
}