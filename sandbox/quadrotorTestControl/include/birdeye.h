#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <LinearMath/btMatrix3x3.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "vector_computation.h"

#include "math.h"
#include <sstream>
#include <vector>
#include <stdio.h>

class birdeye
{
 private:
  //ros related
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  tf::TransformListener *listener;
  tf::StampedTransform *transform;

  //constants;
  double flight_radius,freq;
  std::string vicon;
  pcl::PointXYZ shiftedOrigin,zeroVector;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,cloud_vel,cloud_acc;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  int averageStep;

  //variables;
  pcl::PointXYZ currentPoint,prePoint,originPoint;
  pcl::PointXYZ e_p,e_v,acc_t,currentVel,currentAcc,intError;
  double phi,psi,theta;
  //std::vector<float> x_log, y_log, z_log, phi_log, theta_log, psi_log;
  std::vector<pcl::PointXYZ> pointLog,angleLog,velLog,accLog;

  //handle vicon lost
  int freezeCounter,currentIndex;
  bool lostVicon;

  //functions
  //initiation
  void initiation();
  void init_position(); //init x,y,z,x_pre,.. 
  void init_parameters();
  void flightOrigin();
  void init_pointcloud(); //initiate path

  //in flight data updating
  void transformZYXtoZXY(double,double,double);

  //flight log record
  void flightLog();

  //vicon lost handle
  void viconLostHandle();
  pcl::PointXYZ differentiate(const std::vector<pcl::PointXYZ>&);
  bool checkZeroVector(pcl::PointXYZ);

 public:
  birdeye(ros::NodeHandle nh, ros::NodeHandle nh_private);
  virtual ~birdeye();

  //in flight data updating
  void updateFlightStatus(); //update current position,velocity,etc
  void updateIntError(pcl::PointXYZ); //update integrated error for hover
  void resetIntError(); //reset integrated error to zero
  void calculate_position_velocity_error(); //used for path control

  //getters
  pcl::PointXYZ getShiftedOrigin();
  pcl::PointXYZ getStartPoint();
  pcl::PointXYZ getCurrentPoint();
  pcl::PointXYZ getOriginPoint();
  pcl::PointXYZ getE_p();
  pcl::PointXYZ getE_v();
  pcl::PointXYZ getAcc_t();
  pcl::PointXYZ getCurrentVel();
  pcl::PointXYZ getIntError();

  double getPhi();
  double getPsi();
  double getTheta();

  float getLog_x(int);
  float getLog_y(int);
  float getLog_z(int);
  float getLog_phi(int);
  float getLog_theta(int);
  float getLog_psi(int);
  float getLog_velX(int);
  float getLog_velY(int);
  float getLog_velZ(int);
  float getLog_accX(int);
  float getLog_accY(int);
  float getLog_accZ(int);

  int getLogSize();
};
