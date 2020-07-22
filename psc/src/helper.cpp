#include "helper.h"
#include "DeOscillator.h"

using namespace std;

float DeOscillator::lin_dist_thres = .1;
float DeOscillator::ang_dist_thres_lower = M_PI * 10 / 180;
float DeOscillator::ang_dist_thres_upper = M_PI * 110 / 180;

float vectorNorm(Speed p) {
	return sqrt(magSquared(p));
}
float magSquared(Speed p) {
	std::vector<float> v { p.v, p.w };
	float res = inner_product(v.begin(), v.end(), v.begin(), 0.0f);
	return res;
}
Speed normaliseSpeed(Speed speed_old) {
	Speed speed = Speed(speed_old);
	if (speed.v >= 0) {
		speed.v /= (MAX_LIN_VEL);
	} else {
		speed.v /= (MIN_LIN_VEL);
	}
	if (speed.w >= 0) {
		speed.w /= (MAX_ANG_VEL);
	} else {
		speed.w /= (MIN_ANG_VEL);
	}
	return speed;
}

float getYaw(geometry_msgs::Quaternion q) {
	tf::Quaternion qt;
	tf::quaternionMsgToTF(q, qt);
	tf::Matrix3x3 m(qt);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	return yaw;
	}
float angDiff(float a1, float a2) {
	float a = wraparound(a1) - wraparound(a2);
	a = fmod((a + M_PI), (2 * M_PI)) - M_PI;
	return wraparound(a);
}

float wraparound(float ang) { // [-pi, pi]
	if (equals(ang, 0) || equals(fabs(ang), M_PI))
		return ang;
	if ((ang <= M_PI) && (ang >= -M_PI))
		return ang;
	if (ang > M_PI) {
		ang -= 2 * M_PI;
	}
	if (ang < -M_PI) {
		ang += 2 * M_PI;
	}
	return wraparound(ang); //again wraparound here?
}

float angAdd(float a, float b) {
	float c = wraparound(a + b);
	return c;
}
