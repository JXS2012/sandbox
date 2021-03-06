#include <ros/ros.h>
#include "HighBird.h"

HighBird *bird;

void timerCallback(const ros::TimerEvent&)
{
  bird->drive();  
}

int main(int argc, char** argv)
{
  double freq;

  ros::init(argc,argv,"quadrotor");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  if (!nh_private.getParam("freq",freq))
    freq = 30.0;
  ROS_INFO("call back freq %.3f",freq);

  bird = new HighBird(nh,nh_private);
  ROS_INFO("create bird");

  ros::Timer timer = nh.createTimer(ros::Duration(1.0/freq),timerCallback);
  ROS_INFO("create timer");
  
  while (nh.ok() && !bird->finish())
    ros::spinOnce();

  bird->writeLog();

  delete bird;
  return 0;
}
