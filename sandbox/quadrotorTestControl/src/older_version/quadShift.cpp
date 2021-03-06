#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <LinearMath/btMatrix3x3.h>
#include "std_msgs/Float64.h"
#include "math.h"
#include <sstream>
#include <vector>
#include <stdio.h>
#include <mav_srvs/SetMotorsOnOff.h>
#include <mav_srvs/GetMotorsOnOff.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

std_msgs::Float64 msg_thrust;
std_msgs::Float64 msg_pitch;
std_msgs::Float64 msg_roll;
std_msgs::Float64 msg_yaw;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vel (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_acc (new pcl::PointCloud<pcl::PointXYZ>);
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
pcl::PointXYZ currentPoint,prePoint,originPoint,targetPoint;
pcl::PointXYZ e_p,e_v,acc_t,currentVel,intError;
pcl::PointXYZ kd_hover, kp_hover, ki_hover, kd_path, kp_path;

float e_multiplier; //used for calculating tangent vector

double phi,psi,theta;

float thrust_in,pitch_in,roll_in,yaw_in;
std::vector<float> x_log, y_log, z_log, phi_log, theta_log, psi_log, pitch_log, roll_log, thrust_log;

//constants
float g = 9.81;
float freq = 30.0;
//float pitch_roll_range = 0.14;
//float heli_mass = 615;
float heli_mass = 615;
float root_x = 0.003;
float root_y = 0.003;
float flight_height = 300; //unit mm
float shift_no = 0; //shift in certain direction for three times
float shift_time = 300; //hovering time at each point * freq = 10*30 = 300
float shift_distance = 300;
int control_mode = 0; //0 for hover, 1 for path following
bool fly = true;

void  init_value(); //init x,y,z,x_pre,.. 
void  init_pid(); //defining pid constants
void  init_pointcloud(); //initiate path

void  calculate_position_velocity_error();
double  interpolate_thrust(double); //use pre found table for thrust cmd interpolating
void  pid_hover_control();  //compute pitch,roll,thrust inputs
void  pid_path_control();

void  write_log(); //store flight data(x,y,z,roll,pitch,yaw,control inputs)

void transformZYXtoZXY(double,double,double);

//set of functions for vector calculations
pcl::PointXYZ  normalize(pcl::PointXYZ);
double norm(pcl::PointXYZ);
pcl::PointXYZ crossProduct(pcl::PointXYZ , pcl::PointXYZ );
double dotProduct(pcl::PointXYZ, pcl::PointXYZ);
pcl::PointXYZ scalarProduct(double , pcl::PointXYZ );
pcl::PointXYZ vectorMinus(pcl::PointXYZ , pcl::PointXYZ );
pcl::PointXYZ vectorPlus(pcl::PointXYZ , pcl::PointXYZ );

void turnMotorOn(ros::ServiceClient, ros::ServiceClient);
void turnMotorOff(ros::ServiceClient, ros::ServiceClient);

void flightOrigin(tf::TransformListener *, tf::StampedTransform *);

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  //declaring ros publisher and service client

  ros::NodeHandle node;
  ros::NodeHandle n_thrust;
  ros::NodeHandle n_pitch;
  ros::NodeHandle n_roll;
  ros::NodeHandle n_yaw;
  //control cmds publisher
  ros::Publisher pub_thrust = n_thrust.advertise<std_msgs::Float64>("mav/cmd_thrust", 1000);
  ros::Publisher pub_pitch = n_pitch.advertise<std_msgs::Float64>("mav/cmd_pitch", 1000);
  ros::Publisher pub_roll = n_roll.advertise<std_msgs::Float64>("mav/cmd_roll", 1000);
  ros::Publisher pub_yaw = n_yaw.advertise<std_msgs::Float64>("mav/cmd_yaw", 1000);
  //motor onoff services
  ros::ServiceClient client_SetMotorsOnOff = node.serviceClient<mav_srvs::SetMotorsOnOff>("mav/setMotorsOnOff");
  ros::ServiceClient client_GetMotorsOnOff = node.serviceClient<mav_srvs::GetMotorsOnOff>("mav/getMotorsOnOff");

  mav_srvs::SetMotorsOnOff srv_SetMotorsOnOff;
  mav_srvs::GetMotorsOnOff srv_GetMotorsOnOff;

  //set motor on
  if (fly)
    turnMotorOn(client_SetMotorsOnOff,client_GetMotorsOnOff);
  
  //initiate variables
  init_value();
  init_pid();

  tf::TransformListener *listener = new(tf::TransformListener);
  tf::StampedTransform *transform = new(tf::StampedTransform);
  ros::Rate rate(freq);

  flightOrigin(listener,transform);
  while (true)
    {
      try{
	listener->lookupTransform("/world", "/vicon/bird/bird",  
				 ros::Time(0), *transform);
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	rate.sleep();
	continue;
      }
      originPoint.x = transform->getOrigin().x()*1000;
      originPoint.y = transform->getOrigin().y()*1000;
      originPoint.z = transform->getOrigin().z()*1000;
      break;
    }

  init_pointcloud();

  //defining first flight target point
  targetPoint.x = originPoint.x;
  targetPoint.y = originPoint.y;
  targetPoint.z = originPoint.z+flight_height;

  //defining pre points for later velocity calculation
  prePoint = originPoint;

  printf("origin at %.3f %.3f %.3f\n",originPoint.x,originPoint.y,originPoint.z);

  int counter = 0; //used in main loop for tracking time,not accurate
  bool landing = false; //flag showing whether quadrotor is landing
  float current_shift = 0; //counting current shift steps,

  //main loop
  /*
  while (node.ok()){
    double roll,pitch,yaw;
    
    try{
      listener.lookupTransform("/world", "/vicon/bird/bird",  
			       ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    //get angles in ZYX
    btMatrix3x3(transform.getRotation()).getEulerZYX(yaw,pitch,roll);

    //transform from euler zyx to euler zxy
    transformZYXtoZXY(roll,pitch,yaw);

    //getting positions x,y,z
    currentPoint.x = transform.getOrigin().x()*1000;
    currentPoint.y = transform.getOrigin().y()*1000;
    currentPoint.z = transform.getOrigin().z()*1000;

    //calculate cummulated error
    intError = vectorPlus(intError, vectorMinus(currentPoint,targetPoint) );

    //calculate velocity using one step differentiate, can be improved by moving average method.
    currentVel = scalarProduct(freq, vectorMinus(currentPoint,prePoint));

    prePoint = currentPoint;
    calculate_position_velocity_error();

    //keep flight log
    x_log.push_back(currentPoint.x);
    y_log.push_back(currentPoint.y);
    z_log.push_back(currentPoint.z);
    phi_log.push_back(phi);
    theta_log.push_back(theta);
    psi_log.push_back(psi);

    //printf("x %.3f y %.3f z %.3f\n",x,y,z);

    //if vicon lost track of quadrotor, switch to zero pitch roll input and floating thrust force
    if (currentVel.x == 0 && currentVel.y == 0 && currentVel.z == 0)
      {
	pitch_in = 0;
	roll_in = 0;
	thrust_in = interpolate_thrust(0);
      }
    else
      {
	if (control_mode == 0)
	  pid_hover_control();
	else
	  if (control_mode == 1)
	    pid_path_control();
      }

    counter ++;
    //set new target point to achieve shifting, only available at hovering
    if (counter > shift_time*current_shift && current_shift<shift_no && control_mode == 0)
      {
	printf ("%.3f shift\n",current_shift);
	current_shift++;
	targetPoint.x += shift_distance;
	targetPoint.y += shift_distance;
      }

    //if counter over 1000, start landing
    if (counter > 1000 && !landing)
      {
	targetPoint.z -= flight_height;
	landing = true;
	printf("start landing off\n");
      }

    //if in landing process and height less than 10cm, turn off motor and lan
    if (currentPoint.z-originPoint.z < 100 && landing && fly)
      {
	turnMotorOff(client_SetMotorsOnOff,client_GetMotorsOnOff);
	break;
      }    
    
    //the first 0.5s, only thrust cmd is passed to quadrotor, to make sure that quadrotor gets to a safety height before moving around
    if (counter > freq*0.5)
      {
	msg_thrust.data = thrust_in;
	msg_pitch.data = pitch_in;
	msg_roll.data = roll_in;
	msg_yaw.data = yaw_in; 
	
	pub_pitch.publish(msg_pitch);
	pub_roll.publish(msg_roll);
	pub_yaw.publish(msg_yaw);
	pub_thrust.publish(msg_thrust);
      }
    else{
      msg_thrust.data = thrust_in;
      pub_thrust.publish(msg_thrust);
    }
    
    rate.sleep();

  }

  write_log();
  return 0;
  */
};

void init_value()
{
  currentPoint.x = 0;
  currentPoint.y = 0;
  currentPoint.z = 0;
  prePoint = currentPoint;
  originPoint = currentPoint;
  targetPoint = currentPoint;
  currentVel = currentPoint;
  intError = currentPoint;

  phi = 0;
  psi = 0;
  theta = 0;
}

void init_pid()
{  
  //pid constants found through trial and error
  kd_hover.x = 1.4*root_x;
  kd_hover.y = 1.4*root_y;
  kd_hover.z = 4;

  kp_hover.x = root_x;
  kp_hover.y = root_y;
  kp_hover.z = 2;

  ki_hover.x = 0.00005;
  ki_hover.y = 0.00005;
  ki_hover.z = 0.05;

  kd_path.x = 0.0;
  kd_path.y = 0.0;
  kd_path.z = 4;

  kp_path.x = 0.3;
  kp_path.y = 0.3;
  kp_path.z = 2;

}

void flightOrigin(tf::TransformListener *listener,tf::StampedTransform *transform)
{
  //finding start point of flight
  while (true)
    {
      try{
	listener->lookupTransform("/world", "/vicon/bird/bird",  
				 ros::Time(0), *transform);
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	usleep(100);
	continue;
      }
      originPoint.x = transform->getOrigin().x()*1000;
      originPoint.y = transform->getOrigin().y()*1000;
      originPoint.z = transform->getOrigin().z()*1000;
      break;
    }

}

void transformZYXtoZXY(double roll, double pitch, double yaw)
{
    phi = asin(sin(roll)*cos(pitch));
    double psi_1 = cos(yaw)*sin(roll)*sin(pitch)-cos(roll)*sin(yaw);
    double psi_2 = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);
    psi = atan2(-psi_1,psi_2);
    theta = atan2(tan(pitch),cos(roll));  
}

double interpolate_thrust(double input)
{
  double ros_output_table[11] = {0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5};
  double input_mass[11] = {57.8, 95.3, 138.3, 192.3, 265.3, 350.3, 430.3, 540.3, 665.3, 790.3, 932.3};
  int i = 4;
  while (input>input_mass[i]-heli_mass)
    i++;

  double output; 
  
  //lower bound on output is 0.2 and higher bound on output is 0.4
  if (i >= 9)
    output = 0.4;

  if (i <= 4)
    output = 0.2;

  if ((i<9)&&(i>4))
    output  = (input + heli_mass - input_mass[i-1]) / (input_mass[i] - input_mass[i-1]) * 0.05 + ros_output_table[i-1];
  
  return output;
}

void pid_hover_control()
{
  //using pid to find desired acc in x,y,z direction
  float acc_x, acc_y, acc_z;

  acc_x = -kp_hover.x*(currentPoint.x-targetPoint.x) - kd_hover.x*currentVel.x - ki_hover.x*intError.x;
  acc_y = -kp_hover.y*(currentPoint.y-targetPoint.y) - kd_hover.y*currentVel.y - ki_hover.y*intError.y;
  acc_z = -kp_hover.z*(currentPoint.z-targetPoint.z) - kd_hover.z*currentVel.z - ki_hover.z*intError.z;
  //printf("acc x %.3f, y %.3f z %.3f\n",acc_x,acc_y,acc_z);

  //using linearized invert dynamics to calculate desired roll,pitch,thrust cmd
  roll_in =  -(acc_x*sin(psi) - acc_y*cos(psi))/g;
  pitch_in = (acc_x*cos(psi) + acc_y*sin(psi))/g;
  thrust_in = interpolate_thrust(acc_z/1000.0*heli_mass/g);
  //printf("roll %.3f pitch %.3f thrust %.3f\n",roll_in,pitch_in,thrust_in);

  //no control in yaw angle
  yaw_in = 0;

  roll_log.push_back(roll_in);
  pitch_log.push_back(pitch_in);
  thrust_log.push_back(thrust_in);
  //printf("Input roll %.3f pitch %.3f\n",roll_in,pitch_in);
}

void pid_path_control()
{
  //using pid to find desired acc in x,y,z direction
  float acc_x, acc_y, acc_z;
  acc_x = -kp_path.x*(e_p.x) - kd_path.x*e_v.x + acc_t.x;
  acc_y = -kp_path.y*(e_p.y) - kd_path.y*e_v.y + acc_t.y;
  acc_z = -kp_path.z*(e_p.z) - kd_path.z*e_v.z + acc_t.z;
  //printf("x %.3f y %.3f, z %.3f error x%.3f ,y%.3f ,z%.3f acc_t in x %.3f, y%.3f,z%.3f\n",currentPoint.x,currentPoint.y,currentPoint.z,e_p.x,e_p.y,e_p.z,acc_t.x,acc_t.y,acc_t.z);
  printf("component kp_x %.3f kd_x %.3f acc %.3f\n", kp_path.x*(e_p.x),kd_path.x*e_v.x,acc_t.x);

  //using linearized invert dynamics to calculate desired roll,pitch,thrust cmd
  roll_in =  -(acc_x/1000.0*sin(psi) - acc_y/1000.0*cos(psi))/g;
  pitch_in = (acc_x/1000.0*cos(psi) + acc_y/1000.0*sin(psi))/g;
  thrust_in = interpolate_thrust(acc_z/1000.0*heli_mass/g);
  printf("input cmd roll %.3f pitch %.3f thrust %.3f\n",roll_in,pitch_in,thrust_in);
  //printf("thrust %.3f\n",thrust_in);

  //no control in yaw angle
  yaw_in = 0;

  roll_log.push_back(roll_in);
  pitch_log.push_back(pitch_in);
  thrust_log.push_back(thrust_in);
  //printf("Input roll %.3f pitch %.3f\n",roll_in,pitch_in);

}

void write_log()
{
  std::FILE * pFile;

  pFile = fopen("log.txt","w");
  if (pFile!=NULL)
    {
      for (int i = 0; i<x_log.size();i++)
	fprintf(pFile, "x %.3f y %.3f z %.3f vel_x %.3f vel_y %.3f vel_z %.3f roll %.6f pitch %.6f thrust %.6f\n", x_log[i], y_log[i], z_log[i], phi_log[i], theta_log[i], psi_log[i], roll_log[i], pitch_log[i], thrust_log[i]);
    }
  
}

void land_off()
{
  roll_in = 0;
  pitch_in = 0;
  thrust_in = interpolate_thrust(heli_mass-100.0);
}

void init_pointcloud()
{
  cloud->width = 1000;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 500*cos(i*3.14/180.0);
    cloud->points[i].y = 500*sin(i*3.14/180.0)+500.0;
    cloud->points[i].z = 300;
    cloud_vel->points[i].x = -500*sin(i*3.14/180.0);
    cloud_vel->points[i].y = 500*cos(i*3.14/180.0);
    cloud_vel->points[i].z = 0;
    cloud_acc->points[i].x = -500*cos(i*3.14/180.0);
    cloud_acc->points[i].y = -500*sin(i*3.14/180.0);
    cloud_acc->points[i].z = 0;
  }

  kdtree.setInputCloud(cloud);
}

double norm(pcl::PointXYZ input)
{
  return sqrt(pow(input.x,2)+pow(input.y,2)+pow(input.z,2));
}

pcl::PointXYZ normalize(pcl::PointXYZ input)
{
  pcl::PointXYZ output;
  output.x = input.x/norm(input);
  output.y = input.y/norm(input);
  output.z = input.z/norm(input);
  return output;
}

pcl::PointXYZ crossProduct(pcl::PointXYZ v1, pcl::PointXYZ v2)
{
  pcl::PointXYZ output;
  output.x = v1.y*v2.z-v1.z*v2.y;
  output.y = v1.z*v2.x -v1.x*v2.z;
  output.z = v1.x*v2.y-v1.y*v2.x;
  return output;
}

double dotProduct(pcl::PointXYZ v1, pcl::PointXYZ v2)
{
  return v1.x*v2.x+v1.y*v2.y+v1.z*v2.z;
}

pcl::PointXYZ scalarProduct(double k, pcl::PointXYZ v)
{
  pcl::PointXYZ output;
  output.x = k*v.x;
  output.y = k*v.y;
  output.z = k*v.z;
  return output;
}

pcl::PointXYZ vectorMinus(pcl::PointXYZ v1, pcl::PointXYZ v2)
{
  pcl::PointXYZ output;
  output.x = v1.x - v2.x;
  output.y = v1.y - v2.y;
  output.z = v1.z - v2.z;
  return output;
}

pcl::PointXYZ vectorPlus(pcl::PointXYZ v1, pcl::PointXYZ v2)
{
  pcl::PointXYZ output;
  output.x = v1.x + v2.x;
  output.y = v1.y + v2.y;
  output.z = v1.z + v2.z;
  return output;
}

void calculate_position_velocity_error()
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
	  //std::cout <<currentPoint.x<<"  "<<currentPoint.y<<"    "<<currentPoint.z<<"path"<<pathPoint.x <<"   "<<pathPoint.y<<"   "<<pathPoint.z<<std::endl;

	  e_v = vectorMinus(currentVel,pathPoint_vel);

	  acc_t = pathPoint_acc;
	  //std::cout <<"   "<<e_p.x<<"    "<<e_p.y<<"    "<<e_p.z<<std::endl;
	}
      
    }
}

void turnMotorOn(ros::ServiceClient client_SetMotorsOnOff, ros::ServiceClient client_GetMotorsOnOff)
{
  mav_srvs::SetMotorsOnOff srv_SetMotorsOnOff;
  mav_srvs::GetMotorsOnOff srv_GetMotorsOnOff;
  srv_SetMotorsOnOff.request.on = true;
  client_SetMotorsOnOff.call(srv_SetMotorsOnOff);
  client_GetMotorsOnOff.call(srv_GetMotorsOnOff);
  
  while (!srv_GetMotorsOnOff.response.on)
    {
      ROS_ERROR("Failed to turn motor on\n");
      client_SetMotorsOnOff.call(srv_SetMotorsOnOff);
      client_GetMotorsOnOff.call(srv_GetMotorsOnOff);
      usleep(100);
    } 

  ROS_INFO("Motor on\n");
}

void turnMotorOff(ros::ServiceClient client_SetMotorsOnOff, ros::ServiceClient client_GetMotorsOnOff)
{
  mav_srvs::SetMotorsOnOff srv_SetMotorsOnOff;
  mav_srvs::GetMotorsOnOff srv_GetMotorsOnOff;
  srv_SetMotorsOnOff.request.on = false;
  client_SetMotorsOnOff.call(srv_SetMotorsOnOff);
  client_GetMotorsOnOff.call(srv_GetMotorsOnOff);
  
  while (srv_GetMotorsOnOff.response.on)
    {
      ROS_ERROR("Failed to turn motor off\n");
      client_SetMotorsOnOff.call(srv_SetMotorsOnOff);
      client_GetMotorsOnOff.call(srv_GetMotorsOnOff);
      usleep(100);
    } 
  ROS_INFO("Motor off\n");
}

/*
void init_origin()
{
  tf::TransformListener listener;
  tf::StampedTransform transform;

  while (true)
    {
      try{
	listener.lookupTransform("/world", "/vicon/hummingbird/hummingbird",  
				 ros::Time(0), transform);
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	rate.sleep();
	continue;
      }
      x_origin = transform.getOrigin().x()*1000;
      y_origin = transform.getOrigin().y()*1000;
      break;
    }
  printf("origin at %.3f %.3f\n",x_origin,y_origin);
}

void get_ZXY()
{
  double roll,pitch,yaw;
  tf::TransformListener listener;
  tf::StampedTransform transform;

  try{
    listener.lookupTransform("/world", "/vicon/hummingbird/hummingbird",  
			     ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  
  btMatrix3x3(transform.getRotation()).getEulerZYX(yaw,pitch,roll);
  //printf("ZYX roll %.3f pitch %.3f yaw %.3f\n",roll,pitch,yaw);
  
  phi = asin(sin(roll)*cos(pitch));
  double psi_1 = cos(yaw)*sin(roll)*sin(pitch)-cos(roll)*sin(pitch);
  double psi_2 = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);
  psi = atan(-psi_1/psi_2);
  theta = atan(tan(pitch)/cos(roll));  
  //printf("ZXY roll %.3f pitch %.3f yaw %.3f\n",phi,theta,psi);
}

void get_loc_vel()
{
  tf::StampedTransform transform;

    x = transform.getOrigin().x()*1000-x_origin;
    y = transform.getOrigin().y()*1000-y_origin;
    vel_x = (x-x_pre)*freq;
    vel_y = (y-y_pre)*freq;
    x_pre = x;
    y_pre = y;
    x_log.push_back(x);
    y_log.push_back(y);
    //printf("x %.3f y %.3f vel_x %.3f vel_y %.3f\n",x,y,vel_x,vel_y);
}
*/
