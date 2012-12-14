/*
 *  cc_node.cpp
 *
 *  Created on: 30.11.2012
 * Version: v0.1
 *      Author: Jeyaprakash, Marc, and Yashar
 */
 
 //joint_states 9-13

#include <ros/ros.h>
#include <std_srvs/Empty.h> // ?
#include <geometry_msgs/Twist.h>
#include <brics_actuator/JointVelocities.h>
#include "sensor_msgs/Joy.h"
#include "my_functional_class.h"
#include <sensor_msgs/JointState.h>

sensor_msgs::JointState joint_state;
geometry_msgs::Twist cartesian_pose;

bool serviceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    std::cout << "service call triggered" << std::endl;    

	return true;
}

void joystickCallback(const geometry_msgs::Twist::ConstPtr& cartesianPose) {
	
	cartesian_pose = *cartesianPose;
}

// List of joint angles 
//*
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& jointState) {
	joint_state = *jointState;
}
//*/



int main(int argc, char **argv)
{
	std::cout << "Run cc_node" << std::endl;
    MyFunctionalClass my_func_class;

    /* init ROS node with a name and a node handle*/	
    ros::init(argc, argv, "hbrs_cc_node");
    ros::NodeHandle nh("~");
	
    /* create a loop rate to let your node run only with maximum frequency, here 2Hz */
	ros::Rate loop_rate(2);
	
	// subscribe to topic published by joypad
	
	
	// publish joint velocities to the arm (/arm_1/arm_controller/velocity_command)
	ros::Publisher youbot_arm_joint_vel_cmd_pub = nh.advertise<brics_actuator::JointVelocities>("/arm_1/arm_controller/velocity_command", 1000);
	
	ros::Subscriber joy_sub, node_input, jointState;
	node_input =nh.subscribe<geometry_msgs::Twist>("/cc_node/CartesianParameters", 1000, joystickCallback);
	jointState =nh.subscribe<sensor_msgs::JointState>("/joint_states", 1000, jointStateCallback);
	// get values for joint_value and store it into joint_velocities[i]
	
	std::vector<double> joint_velocities(5);
	brics_actuator::JointVelocities joint_velocities_msg;
	//put in function name 
	

	while (ros::ok())
	{
		//joint_velocities = my_func_class.getJointVelocities();
		//joint_velocities_msg = my_func_class.generateArmMsg(joint_velocities);
		
		//youbot_arm_joint_vel_cmd_pub.publish(joint_velocities_msg);
		
		
        /* process topic callbacks and service request */
		ros::spinOnce();

        /* wait to ensure that is not running than the predefined loop rate */
		//loop_rate.sleep();
	}

	return 0;
}
