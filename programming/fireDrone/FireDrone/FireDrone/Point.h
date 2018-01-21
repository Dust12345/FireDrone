#pragma once


class MyPoint
{
public:

	float x;
	float y;
	float z;

	MyPoint() :x(0),y(0),z(0) {};
	MyPoint(float xp,float yp,float zp) :x(xp), y(yp), z(zp) {};
	~MyPoint();

	float distance(const MyPoint& p);
};

