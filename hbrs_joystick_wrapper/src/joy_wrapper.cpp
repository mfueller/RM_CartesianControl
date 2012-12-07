#include "joy_wrapper.h"

Teleop_joy_wrapper::Teleop_joy_wrapper()//: linear_vel(1), angular_vel(2)
{
  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop_joy_wrapper::joyCallback, this);
  axes_vel_pub = nh.advertise<geometry_msgs::Twist>("cc_node/CartesianParameters", 1000);
}

void Teleop_joy_wrapper::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 	
  geometry_msgs::Twist twist;	
  
  if( joy->axes[0] > lower_treshold && joy->axes[0] < upper_treshold) { 
	twist.linear.x = 0.0; 
  } 
  else { 
	twist.linear.x = joy->buttons[BUTTON7] * velocity * joy->axes[0];
  }
  
  if( joy->axes[1] > lower_treshold && joy->axes[1] < upper_treshold ) { 
	twist.linear.y = 0.0; 
  } 
  else {
	twist.linear.y = joy->buttons[BUTTON7] * velocity * joy->axes[1];
  }
  
  if( joy->axes[3] > lower_treshold && joy->axes[3] < upper_treshold ) { 
	twist.linear.z = 0.0; 
  } 
  else {
	twist.linear.z = joy->buttons[BUTTON7] * velocity * joy->axes[3];
  }
  
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;
    //std::vector<double> angular = {x_ang_vel, y_ang_vel, z_ang_vel};
  
  
  axes_vel_pub.publish(twist);
  
}

//*
int main(int argc, char** argv)
{
	ros::init(argc, argv, "joy_wrapper");
	Teleop_joy_wrapper test;
	while(ros::ok()) {	
		ros::spin();
	}
}
//*/
