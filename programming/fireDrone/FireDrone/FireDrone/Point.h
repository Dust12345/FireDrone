#pragma once
class Point
{
public:

	float x;
	float y;
	float z;

	Point() :x(0),y(0),z(0) {};
	~Point();

	float distance(const Point& p);
};

