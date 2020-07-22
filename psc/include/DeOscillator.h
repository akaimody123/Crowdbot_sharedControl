#ifndef DEOSCILLATOR_H
#define DEOSCILLATOR_H

#include <unistd.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <numeric>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "pose.h"
#include "tf/transform_datatypes.h"
#include "helper.h"

#define INVALID_DIR -12


class DeOscillator {
private:
	float upperbound, lowerbound;
	bool front; 
	Pose start_pose; // The pose at the start of deoscillation.
	float velDir; // instantanouse direction of motion in the body frame.
	static float lin_dist_thres, ang_dist_thres_lower, ang_dist_thres_upper;
	bool first;
public:
	DeOscillator() {
		upperbound = M_PI + .01;
		lowerbound = -M_PI - .01;
		start_pose = Pose();
		velDir = 0;
		front = true;
		first = true;
	}

	

	// This function examines if we have travelled far enough to ensure deoscillation.
	void updateOdom(const nav_msgs::Odometry& cmd) {
		float xt = cmd.pose.pose.position.x;
		float yt = cmd.pose.pose.position.y;
		float tht = getYaw(cmd.pose.pose.orientation);

		if (first) {
			start_pose = Pose(xt, yt, tht);
			first = false;
			return;
		}
		float lin_dist = sqrt(
				pow(start_pose.x - xt, 2) + pow(start_pose.y - yt, 2));
		float ang_dist = abs(angDiff(start_pose.th, tht));
		if ((lin_dist > lin_dist_thres) || ((ang_dist > ang_dist_thres_lower)
//					&& (ang_dist < ang_dist_thres_upper)
				)) {
			start_pose = Pose(xt, yt, tht);
			velDir = atan2(cmd.twist.twist.angular.z, cmd.twist.twist.linear.x);

		}

	}

	void getAdmissibleDirection(float& upperbound, float& lowerbound) {
		upperbound = velDir + M_PI * 90 / 180;
		upperbound = wraparound(upperbound);
		lowerbound = velDir - M_PI * 90 / 180;
		lowerbound = wraparound(lowerbound);
	}

	// This is called only once anytime the goal pose changes.
	void changeDir(Pose currPose, Pose goalPose, float dir = INVALID_DIR) {
		float bearing = currPose.bearingToPose(goalPose);
		float trueBearing = angDiff( bearing,currPose.th);
		bool front_t = true;
		int sign = -1;
		if (front) sign = 1;
		if (fabs(trueBearing) > (M_PI / 2 +sign*.1) && bearing!=0) {
			
			front_t = false;
		}

		if (dir != INVALID_DIR) {
			velDir = dir;
		}
		if (front ^ front_t) {
			// Useful when DWA is used and complex update of velDir is not required.
			// This is because DWA only takes one goal and does not change its goal.
			if (dir == INVALID_DIR) {
				if (front_t) {
					velDir = 0;
				} else {
					velDir = M_PI;
				}
			}
			front = front_t;
		//	cout << "CHANGING DIRECTION, isFront= " << front << ". Veldir: "
		//			<< velDir << " !!!" << endl;
		} else {
		//	cout << "New goal but direction Remains the same. isFront= "
		//			<< front << ". Veldir: " << velDir << endl;
		}

	}

	bool isFront() const {
		return front;
	}
}
;
#endif
