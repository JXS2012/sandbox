#include "DummyBird.h"

DummyBird::DummyBird(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{
  ros::NodeHandle node  (nh_);

  //control cmds publisher
  pub_thrust = node.advertise<std_msgs::Float64>("mav/cmd_thrust", 1000);
  pub_pitch = node.advertise<std_msgs::Float64>("mav/cmd_pitch", 1000);
  pub_roll = node.advertise<std_msgs::Float64>("mav/cmd_roll", 1000);
  pub_yaw = node.advertise<std_msgs::Float64>("mav/cmd_yaw", 1000);
  //motor OnOff services
  client_SetMotorsOnOff = node.serviceClient<mav_srvs::SetMotorsOnOff>("mav/setMotorsOnOff");
  client_GetMotorsOnOff = node.serviceClient<mav_srvs::GetMotorsOnOff>("mav/getMotorsOnOff");

  initParameters();
  initPid();

}

DummyBird::~DummyBird()
{
  ROS_INFO("Destroying DummyBird ");
}

void DummyBird::initPid()
{  
  //pid constants found through trial and error
  if (quadrotorType.compare("Hummingbird") == 0)
    {
      float root_x = 3;
      float root_y = 3;

      kdHover.x = 1.4*root_x;
      kdHover.y = 1.4*root_y;
      kdHover.z = 4;

      kpHover.x = root_x;
      kpHover.y = root_y;
      kpHover.z = 2;

      kiHover.x = 0.05;
      kiHover.y = 0.05;
      kiHover.z = 0.05;

      kdHoverVel.x = 4.2;
      kdHoverVel.y = 4.2;
      kdHoverVel.z = 4;

      kpHoverVel.x = 3;
      kpHoverVel.y = 3;
      kpHoverVel.z = 2;

      kiHoverVel.x = 0.05;
      kiHoverVel.y = 0.05;
      kiHoverVel.z = 0.05;


      kdPath.x = 4.2;
      kdPath.y = 4.2;
      kdPath.z = 4;

      kpPath.x = 3;
      kpPath.y = 3;
      kpPath.z = 2;
    }

  if (quadrotorType.compare("Pelican")==0)
    {
      float root_x = 3.5;
      float root_y = 3.5;

      kdHover.x = 1.6*root_x;
      kdHover.y = 1.6*root_y;
      kdHover.z = 4;

      kpHover.x = root_x;
      kpHover.y = root_y;
      kpHover.z = 2;

      kiHover.x = 0.05;
      kiHover.y = 0.05;
      kiHover.z = 0.05;

      kdHoverVel.x = 0.5;
      kdHoverVel.y = 0.5;
      kdHoverVel.z = 4;

      kpHoverVel.x = 5.6;
      kpHoverVel.y = 5.6;
      kpHoverVel.z = 2;

      kiHoverVel.x = 3.5;
      kiHoverVel.y = 3.5;
      kiHoverVel.z = 0.05;

      kdPath.x = 5.6;
      kdPath.y = 5.6;
      kdPath.z = 4;

      kpPath.x = 3.5;
      kpPath.y = 3.5;
      kpPath.z = 2;
    }    

}

void DummyBird::initParameters()
{ 
  if (!nh_private_.getParam("gravity_constant", g))
    g = 9.81;
  if (!nh_private_.getParam("heli_mass",heli_mass))
    heli_mass = 615;
  if (!nh_private_.getParam("ROS_thrust_cmd_table",strThrustCmd))
    strThrustCmd = "0 0.05 0.1 0.15 0.2 0.25 0.3 0.35 0.4 0.45 0.5";
  if (!nh_private_.getParam("actual_thrust_table",strActThrust))
    strActThrust = "57.8 95.3 138.3 192.3 265.3 350.3 430.3 540.3 665.3 790.3 932.3";
  if (!nh_private_.getParam("quadrotor_type",quadrotorType))
    quadrotorType = "Hummingbird";

  thrustCmd = readTable(strThrustCmd);
  actualThrust = readTable(strActThrust);
  ROS_INFO("Gravity constant initialized %.3f",g);
  ROS_INFO("Helicopter mass initialized %.3f",heli_mass);
  ROS_INFO("Quadrotor Type %s",quadrotorType.c_str());
  ROS_INFO("Thrust table: ");
  for (int i = 0; (unsigned)i<thrustCmd.size(); i++)
    ROS_INFO("%.3f : %.3f",thrustCmd[i],actualThrust[i]);
}

std::vector<double> DummyBird::readTable(std::string strTable)
{
  std::vector<double> result;
  std::istringstream iss(strTable);

  while (iss)
    {
      std::string sub;
      iss >>sub;
      result.push_back(atof(sub.c_str()));
    }

  result.pop_back();

  return result;
}

double DummyBird::interpolateThrust(double input)
{
  int i = 4;
  while (input>actualThrust[i]-heli_mass)
    i++;

  double output; 
  
  //lower bound on output is 0.2 and higher bound on output is 0.4
  if (i >= 9)
    output = 0.4;

  if (i <= 4)
    output = 0.2;

  if ((i<9)&&(i>4))
    output  = (input + heli_mass - actualThrust[i-1]) / (actualThrust[i] - actualThrust[i-1]) * 0.05 + thrustCmd[i-1];
  
  return output;
}

void DummyBird::pidHoverController(pcl::PointXYZ e_p,pcl::PointXYZ e_v,pcl::PointXYZ e_int, double psi)
{
  double acc_x_des, acc_y_des, acc_z_des;
  acc_x_des = -kpHover.x*e_p.x - kdHover.x*e_v.x - kiHover.x*e_int.x;
  acc_y_des = -kpHover.y*e_p.y - kdHover.y*e_v.y - kiHover.y*e_int.y;
  acc_z_des = -kpHover.z*e_p.z - kdHover.z*e_v.z - kiHover.z*e_int.z;

  acc_des2control(acc_x_des,acc_y_des,acc_z_des,psi);

}

void DummyBird::pidHoverVelController(pcl::PointXYZ e_p,pcl::PointXYZ e_v,pcl::PointXYZ e_acc, pcl::PointXYZ e_int, double psi)
{
  double acc_x_des, acc_y_des, acc_z_des;

  acc_x_des = -kpHoverVel.x*e_v.x - kdHoverVel.x*e_acc.x - kiHoverVel.x*e_p.x;
  acc_y_des = -kpHoverVel.y*e_v.y - kdHoverVel.y*e_acc.y - kiHoverVel.y*e_p.y;
  acc_z_des = -kpHover.z*e_p.z - kdHover.z*e_v.z - kiHover.z*e_int.z;

  acc_des2control(acc_x_des,acc_y_des,acc_z_des,psi);
}

void DummyBird::pidPathController(pcl::PointXYZ e_p,pcl::PointXYZ e_v,pcl::PointXYZ acc_t, double psi)
{
  double acc_x_des, acc_y_des, acc_z_des;
  acc_x_des = -kpPath.x*(e_p.x) - kdPath.x*e_v.x + acc_t.x;
  acc_y_des = -kpPath.y*(e_p.y) - kdPath.y*e_v.y + acc_t.y;
  acc_z_des = -kpPath.z*(e_p.z) - kdPath.z*e_v.z + acc_t.z;

  acc_des2control(acc_x_des,acc_y_des,acc_z_des,psi);
}

void DummyBird::acc_des2control(double acc_x_des, double acc_y_des, double acc_z_des, double psi)
{
  float thrust,pitch,roll,yaw;
  
  //using linearized invert dynamics to calculate desired roll,pitch,thrust cmd
  if (quadrotorType.compare("Hummingbird") == 0)
    {
      roll =  -(acc_x_des/1000.0*sin(psi) - acc_y_des/1000.0*cos(psi))/g;
      pitch = (acc_x_des/1000.0*cos(psi) + acc_y_des/1000.0*sin(psi))/g;
      thrust = interpolateThrust(acc_z_des/1000.0*heli_mass/g);
    }

  if (quadrotorType.compare("Pelican") == 0)
    {
      roll =  (acc_x_des/1000.0*sin(psi) - acc_y_des/1000.0*cos(psi))/g;
      pitch = -(acc_x_des/1000.0*cos(psi) + acc_y_des/1000.0*sin(psi))/g;
      thrust = interpolateThrust(acc_z_des/1000.0*heli_mass/g);
    }
 
  roll_log.push_back(roll);
  pitch_log.push_back(pitch);
  thrust_log.push_back(thrust);

  msg_thrust.data = thrust;
  msg_pitch.data = pitch;
  msg_roll.data = roll;
    
  pub_pitch.publish(msg_pitch);
  pub_roll.publish(msg_roll);
  pub_thrust.publish(msg_thrust);
}

void DummyBird::directDrive(double acc_x,double acc_y, double acc_z,double psi)
{
  acc_des2control(acc_x,acc_y,acc_z,psi);
}

void DummyBird::on()
{
  mav_srvs::SetMotorsOnOff srv_SetMotorsOnOff;
  mav_srvs::GetMotorsOnOff srv_GetMotorsOnOff;
  srv_SetMotorsOnOff.request.on = true;
  client_SetMotorsOnOff.call(srv_SetMotorsOnOff);
  client_GetMotorsOnOff.call(srv_GetMotorsOnOff);
  
  while (!srv_GetMotorsOnOff.response.on)
    {
      //ROS_ERROR("Failed to turn motor on\n");
      client_SetMotorsOnOff.call(srv_SetMotorsOnOff);
      client_GetMotorsOnOff.call(srv_GetMotorsOnOff);
      usleep(100);
    } 

  ROS_INFO("Motor on\n");
}

void DummyBird::off()
{
  mav_srvs::SetMotorsOnOff srv_SetMotorsOnOff;
  mav_srvs::GetMotorsOnOff srv_GetMotorsOnOff;
  srv_SetMotorsOnOff.request.on = false;
  client_SetMotorsOnOff.call(srv_SetMotorsOnOff);
  client_GetMotorsOnOff.call(srv_GetMotorsOnOff);
  
  while (srv_GetMotorsOnOff.response.on)
    {
      //ROS_ERROR("Failed to turn motor off\n");
      client_SetMotorsOnOff.call(srv_SetMotorsOnOff);
      client_GetMotorsOnOff.call(srv_GetMotorsOnOff);
      usleep(100);
    } 

  ROS_INFO("Motor off\n");
}
