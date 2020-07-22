#ifndef POSE_H
#define POSE_H
#include "point.h"
#include "helper.h"

using namespace std;
class Pose: public Point<float> {
public:
	float th;
	Pose(float xt, float yt, float tht = 0) :
			Point(xt, yt), th(tht) {
	}
	Pose() :
			Point() {
		th = 0;
	}
	bool operator==(Pose pose) // copy/move constructor is called to construct arg
			{
		if (equals(this->x, pose.x) && equals(this->y, pose.y)
		&& equals(this->th, pose.th)) {
			return true;
		} else {
			return false;
		}
	}
	Pose& operator=(Pose pose) // copy/move constructor is called to construct arg
			{
		this->x = pose.x;
		this->y = pose.y;
		this->th = pose.th;
		return *this;
	}
	Pose operator+(Pose pose) // copy/move constructor is called to construct arg
			{
		pose.x += this->x;
		pose.y += this->y;
		pose.th = angAdd(pose.th, this->th);
		return pose;
	}
	float bearingToPose(Pose pose) {
		float y = pose.y - this->y;
		float x = pose.x - this->x;
		float bearing = atan2(y, x);
		return bearing;
	}

};
#endif
