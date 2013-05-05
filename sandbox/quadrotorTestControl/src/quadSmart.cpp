#include <ros/ros.h>
//#include "smartbird.h"
#include "highbird.h"

//#include <boost/ref.hpp>
//smartbird *hummingbird;
highbird *hummingbird;

void timerCallback(const ros::TimerEvent&)
{
  hummingbird->drive();  
}

int main(int argc, char** argv)
{
  float freq = 30.0;

  ros::init(argc,argv,"quadrotor");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  //hummingbird = new smartbird(nh,nh_private);
  hummingbird = new highbird(nh,nh_private);
  printf("create hummingbird\n");

  ros::Timer timer = nh.createTimer(ros::Duration(1.0/freq),timerCallback);
  printf("create timer\n");
  
  while (nh.ok() && !hummingbird->finish())
    ros::spinOnce();

  hummingbird->write_log();

  return 0;
}
