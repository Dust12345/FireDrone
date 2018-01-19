#pragma once
class Point
{
public:

	float x;
	float y;
	float z;

	Point() :x(0),y(0),z(0) {};
	Point(float xp,float yp,float zp) :x(xp), y(yp), z(zp) {};
	~Point();

	float distance(const Point& p);
};

