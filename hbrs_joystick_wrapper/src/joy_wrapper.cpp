#include "joy_wrapper.h"

Teleop_joy_wrapper::Teleop_joy_wrapper()//: linear_vel(1), angular_vel(2)
{
  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop_joy_wrapper::joyCallback, this);
  axes_vel_pub = nh.advertise<geometry_msgs::Twist>("cc_node/CartesianParameters", 1000);
}

void Teleop_joy_wrapper::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 	
  geometry_msgs::Twist twist;	
  
  twist.linear.x = joy->buttons[BUTTON7]* joy->axes[0];
  twist.linear.y = joy->buttons[BUTTON7]* joy->axes[1];
  twist.linear.z = joy->buttons[BUTTON7]* joy->axes[3];
   
  twist.angular.x = joy->buttons[BUTTON7]* joy->axes[5];
  twist.angular.y = joy->buttons[BUTTON7]* joy->axes[4];
  twist.angular.z = joy->buttons[BUTTON7]* joy->axes[2];
    //std::vector<double> angular = {x_ang_vel, y_ang_vel, z_ang_vel};
  
  
  axes_vel_pub.publish(twist);
  
}

//*
int main(int argc, char** argv)
{
	ros::init(argc, argv, "joy_wrapper");
	Teleop_joy_wrapper joy_wrapper;
	while(ros::ok()) {	
		ros::spin();
	}
}
//*/
