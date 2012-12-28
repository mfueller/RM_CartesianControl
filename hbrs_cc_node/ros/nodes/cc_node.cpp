/*
 *  cc_node.cpp
 *
 *  Created on: 30.11.2012
 * Version: v0.1
 *      Author: Jeyaprakash, Marc, and Yashar
 */
 

#include <ros/ros.h>
#include <std_srvs/Empty.h> // ?
#include <geometry_msgs/Twist.h>
#include <brics_actuator/JointVelocities.h>
#include "sensor_msgs/Joy.h"
#include "my_functional_class.h"
#include <sensor_msgs/JointState.h>
#include "hbrs_cc_library.h"

#include <boost/units/systems/si/angular_velocity.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/io.hpp>

sensor_msgs::JointState joint_state;
geometry_msgs::Twist cartesian_pose;
bool jointCallBack = false, joyCallBack = false;

bool serviceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    std::cout << "service call triggered" << std::endl;    

	return true;
}

//TODO change naming
void joystickCallback(const geometry_msgs::Twist::ConstPtr& cartesianPose) {
	
	cartesian_pose = *cartesianPose;
	joyCallBack = true;
	
}

// List of joint angles 
//*
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& jointState) {
	joint_state = *jointState;
	jointCallBack = true;
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
	
	//std::vector<double> joint_velocities(5);
	//brics_actuator::JointVelocities joint_velocities_msg;
	//put in function name 
	
    Hbrs_cc_Library hbrs_cc_lib("");
    
	while (ros::ok()) {
		if(jointCallBack && joyCallBack)
		{	
		
			std::vector<double> jointPosition, transVel, rotVel, jointVelocity;
			for (unsigned short i = 8; i < 13; i++) {
				jointPosition.push_back(joint_state.position[i]);
			}
	
			transVel.push_back(cartesian_pose.linear.x);
			transVel.push_back(cartesian_pose.linear.y);
			transVel.push_back(cartesian_pose.linear.z);
			rotVel.push_back(cartesian_pose.angular.x);
			rotVel.push_back(cartesian_pose.angular.y);
			rotVel.push_back(cartesian_pose.angular.z);
		
			hbrs_cc_lib.getJointVelocity(jointPosition, transVel, rotVel, jointVelocity);
			
			brics_actuator::JointVelocities command;
			std::vector <brics_actuator::JointValue> setPoints;
			setPoints.resize(5);
			std::stringstream jointName;
			
			for (int i = 0; i < 5; ++i) {

				jointName.str("");
				jointName << "arm_joint_" << (i + 1);

				setPoints[i].joint_uri = jointName.str();
				setPoints[i].value = jointVelocity.at(i);

				setPoints[i].unit = boost::units::to_string(boost::units::si::radian_per_second);
				std::cout << "Joint " << setPoints[i].joint_uri << " = " << setPoints[i].value << " " << setPoints[i].unit << std::endl;
			}
			
			command.velocities = setPoints;
			youbot_arm_joint_vel_cmd_pub.publish(command);

			jointCallBack = false;
			joyCallBack = false;	
		}
		ros::spinOnce();

        /* wait to ensure that is not running than the predefined loop rate */
		loop_rate.sleep();
	}

	return 0;
}
