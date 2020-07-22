#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include <unistd.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <numeric>
#include "concurrent_vector.h"
#include "speed.h"
#include "DeOscillator.h"

using namespace std;

typedef struct {Speed speed;float pval;} Distribution;

class PSC{
public:
	PSC(const char * topic, ros::NodeHandle &n_t);

	void getInputCandidates(Speed input, vector<Distribution> &candidates);
	Speed computePSCVelocity(concurrent_vector<Speed> reactive_candidate);
	void usercommandCallback( geometry_msgs::TwistStamped cmd,Pose currentPose);
	geometry_msgs::TwistStamped usercmd;
	
protected:
	ros::NodeHandle n;	
	string topic; // Namespace for yaml variables. 
private:

	Pose currPose;
	Pose goalPose;
	Speed chosenSpeed;
	float dt;
	DeOscillator deOscillator;
	float coupling;
	
	




}
;


