
//Script to test PSC node
#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

#include "../include/psc.h"


using namespace std;
using namespace std::chrono;

Speed test_PSC(PSC psc)
{
  //simulate user input. In reality should be subscribed from ROS

  geometry_msgs::TwistStamped cmd;
  cmd.header.frame_id = 'DIRECT';
  cmd.twist.linear.x = 1;
  cmd.twist.linear.y=0;
  cmd.twist.angular.z = 0; 

  //simulate current robot pose
  Pose currentPose;
  currentPose.x = 0;
  currentPose.y = 0;
  currentPose.th = 0;

  
  psc.usercommandCallback(cmd,currentPose);

  //simulate candidate velocity pairs from reactive navigation
  concurrent_vector<Speed> reactive_result;
  reactive_result.clear();
  reactive_result.emplace_back(Speed(0,0));
  reactive_result.emplace_back(Speed(1,0));
  reactive_result.emplace_back(Speed(1,0.2));
  reactive_result.emplace_back(Speed(1,0.6));
  reactive_result.emplace_back(Speed(1,1));
    
  Speed chosenSpeed=psc.computePSCVelocity(reactive_result);
  return chosenSpeed;

}

int main(int argc, char** argv) {

	const char* ns = "PSC";
	ros::init(argc, argv, ns);
	ros::NodeHandle n;

	PSC psc = PSC(ns,n);

	Speed psc_result;
  while (ros::ok()) 
  {
	 psc_result = test_PSC(psc);
	 cout<<"psc_result [v="<<psc_result.v<<",w="<<psc_result.w<<"]"<<endl;
  }
}

