#ifndef HELPER_H
#define HELPER_H

#include "speed.h"
#include <vector>
#include <chrono>
#include <math.h>
#include <numeric>
#include <unistd.h>
#include <stdlib.h>
#include "ros/ros.h"

#include "tf/transform_datatypes.h"
#include "geometry_msgs/Quaternion.h"

using namespace std;
using namespace std::chrono;

template<typename T>
float vectorNorm(T p) {
	std::vector<float> v { p.x, p.y };
	float res = inner_product(v.begin(), v.end(), v.begin(), 0.0f);
	return sqrt(res);
}

float vectorNorm(Speed p);

Speed normaliseSpeed(Speed speed);
float getYaw(geometry_msgs::Quaternion q);
float angDiff(float a, float b);
float wraparound(float th);
float angAdd(float a, float b);
float magSquared(Speed p);

#endif
