#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <geometry_msgs/Twist.h>

#define BUTTON1 0
#define BUTTON2 1
#define BUTTON3 2
#define BUTTON4 3
#define BUTTON5 4
#define BUTTON6 5
#define BUTTON7 6
#define BUTTON8 7
#define BUTTON9 8
#define BUTTON10 9
#define BUTTON11 10
#define BUTTON12 11

const double lower_treshold = -0.15;
const double upper_treshold = 0.15;

class Teleop_joy_wrapper
{
	public:
		Teleop_joy_wrapper();
	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
		ros::NodeHandle nh;
		ros::Publisher axes_vel_pub;
		ros::Subscriber joy_sub;
};
