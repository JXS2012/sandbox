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
  double freq;

  ros::init(argc,argv,"quadrotor");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  if (!nh_private.getParam("freq",freq))
    freq = 30.0;
  printf("call back freq %.3f\n",freq);

  hummingbird = new highbird(nh,nh_private);
  printf("create hummingbird\n");

  ros::Timer timer = nh.createTimer(ros::Duration(1.0/freq),timerCallback);
  printf("create timer\n");
  
  while (nh.ok() && !hummingbird->finish())
    ros::spinOnce();

  hummingbird->write_log();

  delete hummingbird;
  return 0;
}
