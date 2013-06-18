#include "birdeye.h"

birdeye::birdeye(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{

  ros::NodeHandle node  (nh_,"birdeye");
  listener = new(tf::TransformListener);
  transform = new(tf::StampedTransform);
  
  //initiate variables
  initiation();

}

birdeye::~birdeye()
{
  delete listener;
  delete transform;
  ROS_INFO("Destroying birdeye "); 
}

void birdeye::initiation()
{
  init_position();
  init_parameters();
  flightOrigin();
  init_pointcloud();

  //defining pre points for later velocity calculation
  prePoint = originPoint;
  ROS_INFO("origin at %.3f %.3f %.3f",originPoint.x,originPoint.y,originPoint.z);

}

void birdeye::init_parameters()
{
  if (!nh_private_.getParam("flight_radius", flight_radius))
    flight_radius = 300;
  if (!nh_private_.getParam("freq",freq))
    freq = 30;
  if (!nh_private_.getParam("vicon",vicon))
    vicon = "/vicon/bird/bird";
  if (!nh_private_.getParam("averageStep",averageStep))
    averageStep = 10;

  ROS_INFO("Radius initialized %.3f",flight_radius);
  ROS_INFO("Frequency initialized %.3f",freq);
  ROS_INFO("vicon %s",vicon.c_str());
  ROS_INFO("moving average %d",averageStep);

  lostVicon = false;
  freezeCounter = 0;
  currentIndex = 0;

  zeroVector.x = 0;
  zeroVector.y = 0;
  zeroVector.z = 0;
}

void birdeye::init_position()
{
  currentPoint = zeroVector;
  prePoint = zeroVector;
  originPoint = zeroVector;
  currentVel = zeroVector;
  intError = zeroVector;
  shiftedOrigin = zeroVector;
  //shiftedOrigin.y = 800;

  phi = 0;
  psi = 0;
  theta = 0;
}

void birdeye::flightOrigin()
{
  //finding start point of flight
  pcl::PointXYZ originAngle;
  while (true)
    {
      try{
	listener->lookupTransform("/world", vicon,  
				 ros::Time(0), *transform);
      }
      catch (tf::TransformException ex){
	ROS_DEBUG("%s",ex.what());
	usleep(100);
	continue;
      }
      originPoint.x = transform->getOrigin().x()*1000;
      originPoint.y = transform->getOrigin().y()*1000;
      originPoint.z = transform->getOrigin().z()*1000;

      double yaw,pitch,roll;
      btMatrix3x3(transform->getRotation()).getEulerYPR(yaw,pitch,roll);
      transformZYXtoZXY(yaw,pitch,roll);
      
      originAngle.x = phi;
      originAngle.y = theta;
      originAngle.z = psi;
      break;
    }
  pointLog.push_back(originPoint);
  angleLog.push_back(originAngle);
  velLog.push_back(zeroVector);
  accLog.push_back(zeroVector);
}

void birdeye::init_pointcloud()
{
  cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_vel.reset(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_acc.reset(new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width = 1000;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  cloud_vel->width = 1000;
  cloud_vel->height = 1;
  cloud_vel->points.resize (cloud->width * cloud->height);

  cloud_acc->width = 1000;
  cloud_acc->height = 1;
  cloud_acc->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = flight_radius*cos(i*3.14/180.0)+shiftedOrigin.x;
    cloud->points[i].y = flight_radius*sin(i*3.14/180.0)+shiftedOrigin.y;
    cloud->points[i].z = 300;
    cloud_vel->points[i].x = -flight_radius*sin(i*3.14/180.0);
    cloud_vel->points[i].y = flight_radius*cos(i*3.14/180.0);
    cloud_vel->points[i].z = 0;
    cloud_acc->points[i].x = -flight_radius*cos(i*3.14/180.0);
    cloud_acc->points[i].y = -flight_radius*sin(i*3.14/180.0);
    cloud_acc->points[i].z = 0;
  }

  kdtree.setInputCloud(cloud);
}

void birdeye::transformZYXtoZXY(double yaw, double pitch, double roll)
{
    phi = asin(sin(roll)*cos(pitch));
    double psi_1 = cos(yaw)*sin(roll)*sin(pitch)-cos(roll)*sin(yaw);
    double psi_2 = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);
    psi = atan2(-psi_1,psi_2);
    theta = atan2(tan(pitch),cos(roll));  
}

void birdeye::updateFlightStatus()
{
  double roll,pitch,yaw;

  try{
    listener->lookupTransform("/world", vicon,  
			     ros::Time(0), *transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  
  //get angles in ZYX
  btMatrix3x3(transform->getRotation()).getEulerYPR(yaw,pitch,roll);
  
  //transform from euler zyx to euler zxy
  transformZYXtoZXY(yaw,pitch,roll);
  
  //getting positions x,y,z
  currentPoint.x = transform->getOrigin().x()*1000;
  currentPoint.y = transform->getOrigin().y()*1000;
  currentPoint.z = transform->getOrigin().z()*1000;

  flightLog();
  
  //calculate velocity using one step differentiate, can be improved by moving average method.
  /*
  if (currentIndex >= averageStep)
      prePoint = pointLog[currentIndex-averageStep];
  else
    prePoint = pointLog[0];

  currentVel = scalarProduct(freq/averageStep, vectorMinus(currentPoint,prePoint));
  */

  viconLostHandle();
  currentVel = differentiate(pointLog);
  velLog.push_back(currentVel);
  currentAcc = differentiate(velLog);
  accLog.push_back(currentAcc);
}

pcl::PointXYZ birdeye::differentiate(const std::vector<pcl::PointXYZ>& x)
{
  pcl::PointXYZ dx;
  if (lostVicon)
    {
      dx = zeroVector;
    }
  else
    {
      if (freezeCounter!=0)
	{
	  if (freezeCounter>averageStep) 
	    dx = scalarProduct(freq/(freezeCounter+1), vectorMinus(x.back(),x[x.size()-1-freezeCounter-1]));
	  else
	    dx = scalarProduct(freq/averageStep, vectorMinus(x.back(),x[x.size()-1-averageStep]));
	  freezeCounter = 0;
	}
      else
	{
	  if (x.size()>(unsigned)(1+averageStep))
	    {
	      if (!checkZeroVector(x[x.size()-1-averageStep]))
		dx = scalarProduct(freq/averageStep, vectorMinus(x.back(),x[x.size()-1-averageStep]));
	      else
		{
		  int tempCounter = 1;
		  while (checkZeroVector(x[x.size()-1-averageStep-tempCounter]))
		    tempCounter++;
		  dx = scalarProduct(freq/(averageStep+tempCounter), vectorMinus(x.back(),x[x.size()-1-averageStep-tempCounter]));
		}
	    }
	  else
	    {
	      dx = scalarProduct(freq/x.size(),vectorMinus(x.back(),x.front()));
	    }
	}
    }
  return dx;
}

void birdeye::viconLostHandle()
{
  if (pointLog.size()<(unsigned)averageStep)
    {
      lostVicon = false;
      return;
    }
  if (checkZeroVector(vectorMinus(pointLog.back(),pointLog[pointLog.size()-2-freezeCounter])))
    {
      lostVicon = true;
      freezeCounter++;
      pointLog.pop_back();
      pointLog.push_back(zeroVector);
    }
  else
    {
      lostVicon = false;
      freezeCounter = 0;
    }
}

bool birdeye::checkZeroVector(pcl::PointXYZ x)
{
  if (x.x == 0 && x.y == 0 && x.z==0)
    return true;
  else
    return false;
}

/*
void birdeye::viconLostHandle()
{
  //when vicon recovered, recalculate currentVel using current point and last valid point, divided by duration of vicon lost
  if (lostVicon && (currentVel.x != 0 || currentVel.y != 0 || currentVel.z != 0))
    {
      currentVel = scalarProduct(freq/freezeCounter, vectorMinus(currentPoint,prePoint));
      freezeCounter = 0;
    }

  //detect vicon lost and measure the duration of vicon lost
  if (currentVel.x == 0 && currentVel.y == 0 && currentVel.z ==0)
    {
      lostVicon = true;
      freezeCounter ++;
    }
  else
    lostVicon = false;  

}
*/
void birdeye::updateIntError(pcl::PointXYZ targetPoint)
{
  intError = vectorPlus(intError, vectorMinus(currentPoint,targetPoint) );
}

void birdeye::resetIntError()
{
  intError = zeroVector;
}

void birdeye::flightLog()
{
  currentIndex++;
  pointLog.push_back(currentPoint);

  pcl::PointXYZ tempAngle;
  tempAngle.x = phi;
  tempAngle.y = theta;
  tempAngle.z = psi;
  angleLog.push_back(tempAngle);
}

void birdeye::calculate_position_velocity_error()
{
  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);

  if (kdtree.nearestKSearch(currentPoint,1,pointIdxNKNSearch,pointNKNSquaredDistance)>0)
    {
      for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
	{
	  pcl::PointXYZ pathPoint,pathPoint_vel,pathPoint_acc,pathTangent,pathNormal,pathBinormal;
	  pathPoint = cloud->points[ pointIdxNKNSearch[i] ];
	  pathPoint_vel = cloud_vel->points[ pointIdxNKNSearch[i] ];
	  pathPoint_acc = cloud_acc->points[ pointIdxNKNSearch[i] ];

	  pathTangent = normalize(pathPoint_vel);
	  //pathNormal = normalize(pathPoint_acc);
	  //pathBinormal = crossProduct(pathTangent,pathNormal);

	  e_p = vectorMinus(currentPoint,pathPoint);
	  e_p = vectorMinus(e_p,scalarProduct( dotProduct(e_p,pathTangent),pathTangent) );
	  ROS_DEBUG("current point: %.3f,%.3f,%.3f",currentPoint.x,currentPoint.y,currentPoint.z);
	  ROS_DEBUG("path point: %.3f,%.3f,%.3f",pathPoint.x,pathPoint.y,pathPoint.z);

	  e_v = vectorMinus(currentVel,pathPoint_vel);

	  acc_t = pathPoint_acc;
	  ROS_DEBUG("error in position: %.3f,%.3f,%.3f",e_p.x,e_p.y,e_p.z);
	}
      
    }
}

pcl::PointXYZ birdeye::getShiftedOrigin()
{
  return shiftedOrigin;
}

pcl::PointXYZ birdeye::getStartPoint()
{
  return cloud->points[0];
}

pcl::PointXYZ birdeye::getCurrentPoint()
{
  return currentPoint;
}
 
pcl::PointXYZ birdeye::getOriginPoint()
{
  return originPoint;
}

pcl::PointXYZ birdeye::getE_p()
{
  return e_p;
}
 
pcl::PointXYZ birdeye::getE_v()
{
  return e_v;
}
 
pcl::PointXYZ birdeye::getAcc_t()
{
  return acc_t;
}
 
pcl::PointXYZ birdeye::getCurrentVel()
{
  return currentVel;
}

pcl::PointXYZ birdeye::getIntError()
{
  return intError;
}

double birdeye::getPhi()
{
  return phi;
}
 
double birdeye::getPsi()
{
  return psi;
}
 
double birdeye::getTheta()
{
  return theta;
}

float birdeye::getLog_x(int i)
{
  return pointLog[i].x;
}

float birdeye::getLog_y(int i)
{
  return pointLog[i].y;
}

float birdeye::getLog_z(int i)
{
  return pointLog[i].z;
}

float birdeye::getLog_phi(int i)
{
  return angleLog[i].x;
}

float birdeye::getLog_theta(int i)
{
  return angleLog[i].y;
}

float birdeye::getLog_psi(int i)
{
  return angleLog[i].z;
}

float birdeye::getLog_velX(int i)
{
  return velLog[i].x;
}

float birdeye::getLog_velY(int i)
{
  return velLog[i].y;
}

float birdeye::getLog_velZ(int i)
{
  return velLog[i].z;
}

float birdeye::getLog_accX(int i)
{
  return accLog[i].x;
}

float birdeye::getLog_accY(int i)
{
  return accLog[i].y;
}

float birdeye::getLog_accZ(int i)
{
  return accLog[i].z;
}


int birdeye::getLogSize()
{
  return pointLog.size();
}
