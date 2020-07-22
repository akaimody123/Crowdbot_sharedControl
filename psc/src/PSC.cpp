
//PSC node
#include "ros/ros.h"
#include "psc.h"
#include "helper.h"


using namespace std;
using namespace std::chrono;


PSC::PSC(const char * topic, ros::NodeHandle &n_t):topic(topic), n(n_t)
{

  this->chosenSpeed = Speed(0,0);
  this->dt = 0.1; //delta time
  
}

 
 void PSC::usercommandCallback(geometry_msgs::TwistStamped cmd,Pose currentPose) 
 {
  //user command callback -- In reality should be used with subscriber
  this->usercmd = geometry_msgs::TwistStamped(cmd);
  Speed humanInput = Speed(this->usercmd.twist.linear.x,
      this->usercmd.twist.angular.z);
  this->currPose = currentPose;
 
  // Here, we estimate the goal pose from the user command.
  float dx, dy, th;
  goalPose = currPose;

  if (humanInput.v != 0)
  {
    if (humanInput.v<0)
    {
      humanInput.w = -humanInput.w;
    }

    th = atan2(humanInput.w, humanInput.v); // estimate user desired direction
    float length = 2; // Assume the goal is 2 meters ahead in user's desired direction
  	dx = length * cos(th);
  	dy = length * sin(th);
  
  	// Now dx, dy are in the body frame and will need to be rotated to the global frame.
  	float xt = cos(currPose.th) * dx - sin(currPose.th) * dy;
  	float yt = sin(currPose.th) * dx + cos(currPose.th) * dy;

  	//xt, yt is the delta distance user want to go in global frame
    goalPose = goalPose + Pose(xt, yt, th);  //set new goal pose  
  }
  else 
  {
    //pure roation, assume target direction is turn left or turn right 
    if (humanInput.w >0){th = M_PI/2;}
    else if (humanInput.w<0){th=-M_PI/2;}
    else{th=0;}
    float dth = humanInput.w * this->dt;
    goalPose.th = angAdd(goalPose.th, dth);     
  }
  this->deOscillator.changeDir(currPose, goalPose);
}



void PSC::getInputCandidates(Speed input, vector<Distribution> &candidates) 
{
  /*estimate user input 
  At this stage, we only used a simple assumption --  a normal distribution over candidate velocity pairs centred at the direction user indicated
  Interface difference is taken into account by setting different std value */

  Distribution p;
  float std = 0;
  if (this->usercmd.header.frame_id == "DIRECT") 
  {  //assume user's input as user's intention, no distribution
    p.speed = input;
    p.pval = 1;
    candidates.emplace_back(p);
    return;
  } 
  else 
  {
    //assume normal distribution
    std = M_PI / 5;  //user defined value. 
    float var = std * std; 
    p.speed = input;
    p.pval = 1/pow(2 * var * M_PI, 0.5);

    candidates.emplace_back(p);

    //if not going straight forward, add all the input candidate from the normal distribution 
    if (!(equals(input.w, 0) && input.v > 0)) 
    {
      // find magnitude and angle.
      float th_0 = atan2(input.w, input.v);
      float mag = vectorNorm(input);
      int step = 1;
      float d = std / step;
      for (float dth = d; dth <= std; dth += d) 
      {
        if (equals(dth, 0))
          continue;

        //left theta
        float th = th_0 - dth;
        float v = cos(th) * mag;
        float w = sin(th) * mag;
        float pval = exp(-dth * dth / (2 * var)) / pow(2 * var * M_PI, 0.5); //normal distribution
        Distribution p;
        p.speed.v = v;
        p.speed.w = w; 
        p.pval = pval;
        candidates.emplace_back(p);

        //right theta
        th = th_0 + dth;
        v = cos(th) * mag;
        w = sin(th) * mag;
        p.speed = Speed(v, w);
        candidates.emplace_back(p);

      } 
    }
  }  
}

  
Speed PSC::computePSCVelocity(concurrent_vector<Speed> reactive_candidate) {
  //Use PSC to find next desired linear velocity & angular velocity pair
  Speed chosenSpeed; //desired velocity pair
  Speed humanInput = Speed(this->usercmd.twist.linear.x,this->usercmd.twist.angular.z); //user input (v,w pair)
  //If user input is (0,0), stop the wheelchair
  if (equals(humanInput.v, 0) && equals(humanInput.w, 0)) {
    return Speed(0,0);
  }

  //calculate cost function based on heading, velocity and clearance (or other user defined cost function)
  //coupling parameter
  float a = 100; // user defined value. agreement factor between user command and resultant velocity. 
  //For pure rotation, set a =0.1
  if (equals(humanInput.v, 0) && !equals(humanInput.w, 0)) {
    a = .1; //user defined value
  }

  float inva = 1 / a;
  float alpha = 0.03;   // user defined value. weight for heading.  
  float sigma = 0.1; // user defined value. weight for clearance  
  float gamma = 0.75;   // user defined value. weight for velocity.
  float final_clearance = 0;

  float max_clearance = 0;
  float max_cost = 0;

  //get user input distribution 
  vector<Distribution> inputDistribution;
  inputDistribution.clear();

  //pure rotation input distribution 
  if (equals(humanInput.v, 0) && !equals(humanInput.w, 0)) {
    Distribution d;
    d.speed = humanInput;
    d.pval =0.01;
    inputDistribution.emplace_back(d);
  } else {
    getInputCandidates(humanInput, inputDistribution);
  }
    
  std::mutex mylock;

  if (reactive_candidate.size()==0)
  {  
    ROS_INFO("No candidate error");
    return Speed(0,0);
  }
  else
  {

    #pragma omp parallel for collapse(2)
    //loop through each input distribution & candidate velocity pairs
    for (int j = 0; j < inputDistribution.size(); j++) {
        for (int i = 0; i < reactive_candidate.size(); i++) {

          Distribution d = inputDistribution[j];
          Speed input = d.speed;
          Speed realspeed = reactive_candidate[i];
        
          float ang = atan2(realspeed.w,realspeed.v);
          float global_ang = angAdd(ang,currPose.th);
          if (realspeed.v == 0)
            {
              //pure rotation or stop
              global_ang = currPose.th+realspeed.w*dt;
            }
          
          float clearance = 1; //computeClearance(realspeed);  //define a function that calculate clearance for each candidate vel pair. 
          //The function we used is related with DWA. In general, this is the distance to the nearest obstacle when the robot moves one step using this candidate velocity.
          //TODO: you can use your own function here.

          float heading = ((M_PI-fabs(angDiff(atan2(goalPose.y-currPose.y,goalPose.x-currPose.x),global_ang)))/M_PI);
          
          Speed norm_speed = normaliseSpeed(realspeed);  
          float velocity = abs(norm_speed.v); 

          float G = alpha * heading +  gamma * velocity + sigma*clearance;  
          //G: probability distribution of candidate safe velocity pairs -- reflected in the cost value

          Speed diff = norm_speed - input;
          float x = magSquared(diff);
          x *= -0.5 * inva; 
          float coupling = expf(x);
          //speed difference between user input and candidate vel
        
          float cost = G * coupling * d.pval *1; 
          //couping: probability distribution of user input
          //can further add an interaction probability with moving obstacles(agents) 


          mylock.lock();

          if (cost>max_cost) {  
            max_cost = cost;
            chosenSpeed = realspeed;
            final_clearance = clearance;
          }
          mylock.unlock();
      
      
      } 
    }
  
    
    geometry_msgs::Vector3Stamped c;
    c.header.stamp = ros::Time::now();
    c.vector.z = final_clearance;
    c.vector.x = chosenSpeed.v;
    c.vector.y = chosenSpeed.w;

    return chosenSpeed; 
  
  }
}





 