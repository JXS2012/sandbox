#include "highbird.h"

highbird::highbird(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{

  ros::NodeHandle node  (nh_,"highbird");

  dummy = new dummybird(nh_,nh_private_);
  eye = new birdeye(nh_,nh_private_);

  initiation();
  //initiate variables

}

highbird::~highbird()
{

  delete dummy;
  delete eye;
  ROS_INFO("Destroying highbird "); 

}

void highbird::initiation()
{
  initParameters();

  //defining first flight target point
  targetPoint = eye->getStartPoint();

  //readTargets();
  
  //targetPoint = eye->getOriginPoint();
  //targetPoint.z += flight_height;
  //targetPoint.x += flight_height;
  //targetPoint.y += flight_height;

  ROS_INFO("hover to %.3f %.3f %.3f",targetPoint.x,targetPoint.y,targetPoint.z);
  sleep(5);
  if (fly)
    dummy->on();
}

void highbird::readTargets()
{
  pcl::PointXYZ temp;
  std::istringstream iss(strTargets);

  while (iss)
    {
      std::string sub1,sub2,sub3;
      iss >>sub1 >>sub2 >>sub3;
      temp.x = atof(sub1.c_str());
      temp.y = atof(sub2.c_str());
      temp.z = atof(sub3.c_str());
      targets.push_back(temp);
    }

  targets.pop_back();
  for (int i = 0; (unsigned)i < targets.size(); i++)
    ROS_INFO("Target point %d is (%.3f,%.3f,%.3f)",i+1,targets[i].x,targets[i].y,targets[i].z);

  targetPoint = targets[0];
}

void highbird::initParameters()
{
  if (!nh_private_.getParam("flight_height", flight_height))
    flight_height = 300; //unit mm

  if (!nh_private_.getParam("max_outter_radius", max_outter_radius))
    max_outter_radius = 1200; //unit mm

  if (!nh_private_.getParam("fly", fly))
    fly = false; //turn motor on or not. 

  if (!nh_private_.getParam("targets",strTargets) )
    strTargets = "0,600,300";

  if (!nh_private_.getParam("freq",freq))
    freq = 30;

  if (!nh_private_.getParam("total_time",total_time))
    total_time = 30;

  ROS_INFO("Flight height initialized %.3f",flight_height);
  ROS_INFO("Boundary radius initialized %.3f",max_outter_radius);
  ROS_INFO("Fly mode initialized %d",fly);
  ROS_INFO("Fly duration %.3f",total_time);
  ROS_DEBUG("%s",strTargets.c_str());

  targetIndex = 0;
  counter = 0; //used in main loop for tracking time
  
  landing = false; //flag showing whether quadrotor is landing
  outRange = false;
  reachStartPoint = false;
  highbird_finish = false;
  hover = true;
}

void highbird::invokeController()
{
  //in case vicon lost track of model
  if (eye->getCurrentVel().x == 0 && eye->getCurrentVel().y == 0 && eye->getCurrentVel().z == 0)
    {
      dummy->direct_drive(0,0,0,eye->getPsi());
    }
  else
    {
      if (hover)
	{
	  eye->updateIntError(targetPoint);
	  dummy->pid_hover_controller(vectorMinus(eye->getCurrentPoint(),targetPoint),eye->getCurrentVel(),eye->getIntError(),eye->getPsi());
	}
      else
	{
	  eye->calculate_position_velocity_error();
	  dummy->pid_path_controller(eye->getE_p(),eye->getE_v(),eye->getAcc_t(),eye->getPsi());
	}
      /*
      dummy->pid_hover_controller(vectorMinus(eye->getCurrentPoint(),targetPoint),eye->getCurrentVel(),eye->getIntError(),eye->getPsi());
      */

    }
}

void highbird::checkStartPoint()
{
  if (norm(vectorMinus(eye->getCurrentPoint(),eye->getStartPoint()))<100 && !reachStartPoint)
    {
      reachStartPoint = true;
      hover = false;
      ROS_INFO("Reached start point!");
    }
}

void highbird::safeOutRange()
{
  pcl::PointXYZ temp = eye->getShiftedOrigin();
  temp.z = eye->getCurrentPoint().z;
  if (norm(vectorMinus(eye->getCurrentPoint(),temp))>max_outter_radius && !outRange)
    {
      ROS_INFO("origin %.3f %.3f %.3f",temp.x,temp.y,temp.z);
      ROS_INFO("out of range at point %.3f %.3f %.3f!",eye->getCurrentPoint().x,eye->getCurrentPoint().y,eye->getCurrentPoint().z);
      outRange = true;
      
      switchHover();
      targetPoint = eye->getOriginPoint();
      targetPoint.z += flight_height;
      ROS_INFO("reset target point %.3f %.3f %.3f!",targetPoint.x,targetPoint.y,targetPoint.z);
      if (counter<700)
	counter = 700;
      /*
      if (fly)
	dummy->off();
      highbird_finish = true;
      */
    }
}

void highbird::land()
{
  //if counter over 800, start landing
  if (counter > total_time*freq && !landing)
    {
      switchHover();
      targetPoint = eye->getCurrentPoint();
      targetPoint.z = eye->getOriginPoint().z;
      landing = true;
      ROS_INFO("start landing off");
    }
  //if in landing process and height less than 10cm, turn off motor and land
  if (eye->getCurrentPoint().z-eye->getOriginPoint().z < 100 && landing)
    {
      if (fly)
	dummy->off();
      highbird_finish = true;
    }    
}

void highbird::switchHover()
{
  hover = true;
  eye->resetIntError();
}

void highbird::hoverFlight()
{
  if ( norm(vectorMinus(eye->getCurrentPoint(),targetPoint))<50 
       && (unsigned)targetIndex < targets.size()-1 )
    switchNextTarget();
}

void highbird::switchNextTarget()
{
  targetPoint = targets[++targetIndex];
  switchHover();
}

void highbird::drive()
{
  //update flight status
  eye->updateFlightStatus();
  
  //check whether arrived at start point
  checkStartPoint();

  //hover to points
  //hoverFlight();

  //check landing condition
  land();

  //check out range condition
  safeOutRange();

  //call hover controller or path controller according to (bool hover)
  invokeController();
    
  counter ++;
}

bool highbird::finish()
{
  return highbird_finish;
}

void highbird::write_log()
{
  std::FILE * pFile;

  pFile = fopen("/home/jianxin/log.txt","w");
  if (pFile!=NULL)
    {
      for (int i = 0; i<eye->getLogSize();i++)
	fprintf(pFile, "x %.3f y %.3f z %.3f phi %.3f theta %.3f psi %.3f roll %.6f pitch %.6f thrust %.6f\n", eye->getLog_x(i), eye->getLog_y(i), eye->getLog_z(i), eye->getLog_phi(i),eye->getLog_theta(i),eye->getLog_psi(i), dummy->getLog_roll(i), dummy->getLog_pitch(i), dummy->getLog_thrust(i));
    }
  
}
