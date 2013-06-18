#include <ros/ros.h>
#include "birdeye.h"

birdeye *hummingbird;
int counter = 0;

void timerCallback(const ros::TimerEvent&)
{
  counter++;
  hummingbird->updateFlightStatus();  
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

  hummingbird = new birdeye(nh,nh_private);
  ROS_INFO("create hummingbird eye");

  ros::Timer timer = nh.createTimer(ros::Duration(1.0/freq),timerCallback);
  ROS_INFO("create timer");
  
  while (nh.ok() && counter <=20*freq)
    {
      ros::spinOnce();
    }

  std::FILE * pFile;
  
  pFile = fopen("/home/jianxin/log.txt","w");
  if (pFile!=NULL)
    {
      for (int i = 0; i<hummingbird->getLogSize();i++)
	fprintf(pFile, "x %.3f y %.3f z %.3f velX %.6f velY %.6f velZ %.6f accX %.6f accY %.6f accZ %.6f\n", hummingbird->getLog_x(i), hummingbird->getLog_y(i), hummingbird->getLog_z(i), hummingbird->getLog_velX(i),hummingbird->getLog_velY(i),hummingbird->getLog_velZ(i), hummingbird->getLog_accX(i),hummingbird->getLog_accY(i),hummingbird->getLog_accZ(i));
    }

  delete hummingbird;
  return 0;
}