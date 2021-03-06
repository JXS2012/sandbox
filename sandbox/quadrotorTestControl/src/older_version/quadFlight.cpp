#include <ros/ros.h>
#include "quadrotor.h"
#include <boost/ref.hpp>
quadrotor *hummingbird;

void timerCallback(const ros::TimerEvent&,quadrotor *hummingbird)
{
  hummingbird->drive();  
}

int main(int argc, char** argv)
{
  float freq = 30.0;

  ros::init(argc,argv,"quadrotor");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  //  quadrotor *hummingbird = new quadrotor(nh,nh_private);
  hummingbird = new quadrotor(nh,nh_private);
  printf("create hummingbird\n");

  ros::Timer timer = nh.createTimer(ros::Duration(1.0/freq),boost::bind(timerCallback,_1,boost::ref(hummingbird)));
  printf("create timer\n");
  
  while (nh.ok() && !hummingbird->finish())
    ros::spinOnce();

  hummingbird->write_log();

  return 0;
}
