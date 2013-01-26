#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <geometry_msgs/Twist.h>

#include <brics_actuator/JointPositions.h>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

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

const double lower_treshold = 0.0;//-0.15;
const double upper_treshold = 0.0; //0.15;

class Teleop_joy_wrapper
{
	public:
		Teleop_joy_wrapper();
		
	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
		ros::NodeHandle nh;
		ros::Publisher axes_vel_pub;
		ros::Subscriber joy_sub;
		ros::Publisher gripperPositionPublisher;
		double gripperValue;
		brics_actuator::JointPositions command;
};
