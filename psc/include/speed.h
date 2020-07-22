#ifndef SPEED_H
#define SPEED_H
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
#define MAX_LIN_VEL  1.3  
#define MIN_LIN_VEL 0.88 

#define MAX_ANG_VEL 0.785
#define MIN_ANG_VEL 0.785

using namespace std;
struct Speed {
	float v;   
	float w;

	Speed(const Speed& speed) :
			v(speed.v), w(speed.w) {
	}
	Speed(float vt, float wt) :
			v(vt), w(wt) {
	}
	Speed() {
		w = v = 0;
	}
	Speed operator*(float s) // copy/move constructor is called to construct arg
				{
			Speed speed = Speed();
			speed.v = this->v * s;
			speed.w = this->w * s;
			return speed;
		}
	Speed operator+(Speed rhs) // copy/move constructor is called to construct arg
			{
		Speed speed = Speed();
		speed.v = this->v + rhs.v;
		speed.w = this->w + rhs.w;
		return speed;
	}

	Speed operator-(Speed rhs) // copy/move constructor is called to construct arg
				{
			Speed speed = Speed();
			speed.v = this->v - rhs.v;
			speed.w = this->w - rhs.w;
			return speed;
		}
	bool operator==(Speed s) // copy/move constructor is called to construct arg
			{
		if (equals(this->v,s.v) && equals(this->w, s.w)) {
			return true;
		} else {
			return false;
		}
	}
};

#endif
