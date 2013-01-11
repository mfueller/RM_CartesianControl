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
#include <brics_actuator/JointPositions.h>
#include "sensor_msgs/Joy.h"
#include "my_functional_class.h"
#include <sensor_msgs/JointState.h>
#include "hbrs_cc_library.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>

#include <boost/units/systems/si/angular_velocity.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

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
	ros::Rate rate(20); //Hz
	
	// subscribe to topic published by joypad
	
	
	// publish joint velocities to the arm (/arm_1/arm_controller/velocity_command)
	ros::Publisher youbot_arm_joint_vel_cmd_pub = nh.advertise<brics_actuator::JointVelocities>("/arm_1/arm_controller/velocity_command", 1000);
	
	
	ros::Publisher armPositionsPublisher = nh.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command", 1000);
	brics_actuator::JointPositions positionCommand;
	std::vector <brics_actuator::JointValue> armJointPositions;
	armJointPositions.resize(5);
	
	ros::Subscriber joy_sub, node_input, jointState;
	node_input =nh.subscribe<geometry_msgs::Twist>("/cc_node/CartesianParameters", 1000, joystickCallback);
	jointState =nh.subscribe<sensor_msgs::JointState>("/joint_states", 1000, jointStateCallback);
	// get values for joint_value and store it into joint_velocities[i]

	//put in function name 
	
    Hbrs_cc_Library hbrs_cc_lib("");
    
    int firstLoop = 0;
    bool keepArm = false;
    
	while (ros::ok()) {
		if(firstLoop++ < 10)
		{
			for(unsigned short i = 0 ; i < 5 ; i++){
				std::stringstream jointName;
				jointName << "arm_joint_" << (i+1);
				armJointPositions.at(i).joint_uri =  jointName.str();
				armJointPositions.at(i).value = 55.0 * (M_PI / 180.0);
				armJointPositions.at(i).unit = boost::units::to_string(boost::units::si::radians);
			}
			
			positionCommand.positions = armJointPositions;
			armPositionsPublisher.publish(positionCommand);
			rate.sleep();
		}	
		else if(jointCallBack && joyCallBack)
		{	
			std::vector<double> jointPosition, transVel, rotVel, jointVelocity;
			for (unsigned short i = 8; i < 13; i++) {
				jointPosition.push_back(joint_state.position[i]);
			}
		
	
			transVel.push_back(cartesian_pose.linear.x );
			transVel.push_back(cartesian_pose.linear.y );
			transVel.push_back(cartesian_pose.linear.z );
			//transVel.push_back(0.0 );
			//transVel.push_back(1.0 );
			//transVel.push_back(0.0 );
			rotVel.push_back(cartesian_pose.angular.x);
			rotVel.push_back(cartesian_pose.angular.y);
			rotVel.push_back(cartesian_pose.angular.z);
		
			hbrs_cc_lib.getJointVelocity(jointPosition, transVel, rotVel, jointVelocity);
			
			brics_actuator::JointVelocities command;
			std::vector <brics_actuator::JointValue> setPoints;
			setPoints.resize(5);
			std::stringstream jointName;
			
			for (unsigned short i = 0; i < 5; ++i) {

				jointName.str("");
				jointName << "arm_joint_" << (i + 1);

				setPoints[i].joint_uri = jointName.str();
				setPoints[i].value = jointVelocity.at(i);

				setPoints[i].unit = boost::units::to_string(boost::units::si::radian_per_second);
				//std::cout << "Joint " << setPoints[i].joint_uri << " = " << setPoints[i].value << " " << setPoints[i].unit << " , ";
			}
			std::cout << std::endl;
			std::cout << std::endl;
			
			command.velocities = setPoints;
			youbot_arm_joint_vel_cmd_pub.publish(command);

			jointCallBack = false;
			joyCallBack = false;
			keepArm = true;	
		} /*
		else if(keepArm){
			for(unsigned short i = 0 ; i < 5 ; i++){
				std::stringstream jointName;
				jointName << "arm_joint_" << (i+1);
				armJointPositions.at(i).joint_uri =  jointName.str();
				armJointPositions.at(i).value = joint_state.position[i+8];
				armJointPositions.at(i).unit = boost::units::to_string(boost::units::si::radians);
				
				std::cout << joint_state.position[i+8] <<" , ";
			}
			keepArm = false;
			std::cout << std::endl;
			positionCommand.positions = armJointPositions;
			armPositionsPublisher.publish(positionCommand);
		} */
		ros::spinOnce();

        /* wait to ensure that is not running than the predefined loop rate */
		loop_rate.sleep();
	}

	return 0;
}
