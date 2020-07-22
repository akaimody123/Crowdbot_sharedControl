#ifndef POINT_H
#define POINT_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <unistd.h>
#include <pthread.h>
#include <sstream>
#include <stdlib.h>
#include <cstdint>
#include <memory>
#include <vector>
#define equals(x, y) (fabs(x-y) < 0.00001)


using namespace std;

template<class T>
class Point {
public:
	Point() {
		x = y = 0;
	}
	virtual ~Point() {
	}
	Point(T x_t, T y_t) :
			x(x_t), y(y_t) {
	}

	virtual Point operator=(const Point& b) {
		this->x = b.x;
		this->y = b.y;
		return *this;
	}
	virtual Point operator+=(const Point& b) {
		this->x += b.x;
		this->y += b.y;
		return *this;
	}
	virtual Point operator-(const Point& b) {
		T x = this->x - b.x;
		T y = this->y - b.y;
		return Point(x,y);
	}
	virtual bool operator==(const Point& b) {
		if (equals(this->x, b.x) && equals(this->y, b.y))
			return true;
		else
			return false;
	}
	T x;
	T y;
};

class IntPoint: public Point<int> {
public:
	IntPoint() :
			Point() {
	}
	IntPoint(int xt, int yt) :
			Point(xt, yt) {
	}

	bool operator==(const IntPoint& b) {
		if (this->x == b.x && this->y == b.y)
			return true;
		else
			return false;
	}
};

class RealPoint: public Point<float> {
public:
	RealPoint() :
			Point() {
	}
	RealPoint(float xt, float yt) :
			Point(xt, yt) {
	}
};

#endif
