#include <ros/ros.h>
#include "birdeye.h"
#include "dummybird.h"

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "vector_computation.h"

#include "math.h"
#include <sstream>
#include <vector>
#include <stdio.h>

class highbird
{
 private:
  dummybird *dummy;
  birdeye *eye;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  //constants
  double flight_height; //unit mm
  double max_outter_radius;
  float shift_no; //number of shifts performed during flight if in hovering mode
  float shift_time; //total time for each shift = shift_time/freq
  float shift_distance;
  bool hover; //flag for hover controller
  bool fly; //turn motor on or not. If false, quadrotor won't fly. For testing purpose

  //variables
  pcl::PointXYZ targetPoint;
  bool landing; //flag showing whether quadrotor is landing
  bool outRange;
  bool reachStartPoint;
  bool highbird_finish;
  float current_shift; //counting current shift steps,
  int counter; //used in main loop for tracking time, current time = counter/freq

  //functions
  //initiation
  void initiation();
  void init_parameters();

  //high level controllers
  void invokeController(); //deciding which controller to use
  void checkStartPoint();
  void shiftTargetPoint(); //change target point in flight for hover controller
  void safeOutRange(); //check shifted away from center of room
  void land();   //land quadrotor once conditions are met(here is counter>1000 i.e. time constraint)
  void switchHover();
  
 public:
  highbird(ros::NodeHandle nh, ros::NodeHandle nh_private);
  virtual ~highbird();

  //drive quadrotor to achieve goals
  void drive();
  //store flight data(x,y,z,roll,pitch,yaw,control inputs)
  void write_log();
  //return true when finished flying
  bool finish();
};
