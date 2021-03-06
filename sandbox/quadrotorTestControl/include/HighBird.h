#include <ros/ros.h>
#include "BirdEye.h"
#include "DummyBird.h"

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "VectorComputation.h"

#include "math.h"
#include <vector>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>

class HighBird
{
 private:
  DummyBird *dummy;
  BirdEye *eye;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  //constants
  double flight_height; //unit mm
  double max_outter_radius;
  double total_time;
  double freq;

  bool hover; //flag for hover controller
  bool fly; //turn motor on or not. If false, quadrotor won't fly. For testing purpose

  //variables
  pcl::PointXYZ targetPoint;
  bool landing; //flag showing whether quadrotor is landing
  bool outRange;
  bool reachStartPoint;
  bool highbird_finish;
  int counter; //used in main loop for tracking time, current time = counter/freq
  std::string strTargets;
  std::vector<pcl::PointXYZ> targets;
  int targetIndex;

  //functions
  //initiation
  void initiation();
  void initParameters();
  void readTargets();

  //high level controllers
  void invokeController(); //deciding which controller to use
  void checkStartPoint();
  void safeOutRange(); //check shifted away from center of room
  void land();   //land quadrotor once conditions are met(here is counter>1000 i.e. time constraint)
  void switchHover();
  void hoverFlight();
  void switchNextTarget();

 public:
  HighBird(ros::NodeHandle nh, ros::NodeHandle nh_private);
  virtual ~HighBird();

  //drive quadrotor to achieve goals
  void drive();
  //store flight data(x,y,z,roll,pitch,yaw,control inputs)
  void writeLog();
  //return true when finished flying
  bool finish();
};
