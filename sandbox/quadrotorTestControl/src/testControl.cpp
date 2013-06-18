#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "math.h"

#include <sstream>

using namespace std;

int main(int argc, char **argv)
{


  ros::init(argc, argv, "talker");

  ros::NodeHandle thrust;
  ros::NodeHandle pitch;
  ros::NodeHandle roll;
  ros::NodeHandle yaw;


  ros::Publisher pub_thrust = thrust.advertise<std_msgs::Float64>("mav/cmd_thrust", 1000);
  ros::Publisher pub_pitch = pitch.advertise<std_msgs::Float64>("mav/cmd_pitch", 1000);
  ros::Publisher pub_roll = roll.advertise<std_msgs::Float64>("mav/cmd_roll", 1000);
  ros::Publisher pub_yaw = yaw.advertise<std_msgs::Float64>("mav/cmd_yaw", 1000);

  ros::Rate loop_rate(20);
  
  while (ros::ok())
  {

    std_msgs::Float64 msg_thrust;
    std_msgs::Float64 msg_pitch;
    std_msgs::Float64 msg_roll;
    std_msgs::Float64 msg_yaw;

    
    float thrust_in,pitch_in,roll_in,yaw_in;
    /*
    cout << "please input thrust(0,0.4),pitch(-1,1),roll(-1,1),yaw(-4.43rad/s,4.43rad/s):\n";
    cin >> thrust_in >> pitch_in >> roll_in >> yaw_in;

    if (pitch_in < -1)
      pitch_in = -1;
    if (pitch_in > 1)
      pitch_in = 1;

    if (roll_in < -1)
      roll_in = -1;
    if (roll_in > 1)
      roll_in = 1;

    if (yaw_in < -4.43)
      yaw_in = -4.43;
    if (yaw_in > 4.43)
      yaw_in = 4.43;
    */
    /*    
    cout << "please input thrust(0,0.6): ";
    while (!(cin >> thrust_in))
      {
	cout << "Wrong input, please input thrust(0,0.6) again: ";
	cin.clear();
	cin.ignore();
      }

    if (thrust_in < 0)
      {
	thrust_in = 0;
	cout <<"capped to 0\n";
      }
    if (thrust_in > 0.6)
      {
	thrust_in = 0.6;
	cout <<"capped to 0.4\n";
      }

    roll_in = 0;
    pitch_in = 0;
    yaw_in = 0;
    */
    thrust_in = 0.1;
    pitch_in = 0;
    roll_in = 0.2;
    yaw_in = 0;
    msg_thrust.data = thrust_in;
    msg_pitch.data = pitch_in;
    msg_roll.data = roll_in;
    msg_yaw.data = yaw_in; 

    int counter = 0;
    while(counter < 120)
      {
	pub_pitch.publish(msg_pitch);
	pub_roll.publish(msg_roll);
	pub_yaw.publish(msg_yaw);
	pub_thrust.publish(msg_thrust);
	//cout << "publish!\n";
	ros::spinOnce();
	
	loop_rate.sleep();
	counter ++;
      }

    msg_thrust.data = 0;
    msg_pitch.data = 0;
    msg_roll.data = 0;
    msg_yaw.data = 0; 
    pub_pitch.publish(msg_pitch);
    pub_roll.publish(msg_roll);
    pub_yaw.publish(msg_yaw);
    pub_thrust.publish(msg_thrust);

    char flag;
    cout << "continue?(y or n): ";
    cin >> flag;
    if (flag == 'n')
	break;
    else
      continue;
  }
  
  
  return 0;
}
