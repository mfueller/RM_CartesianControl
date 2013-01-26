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

//#include <tf/transform_listener.h>
#include <kdl/chain.hpp>
#include "ros_urdf_loader.h" 

sensor_msgs::JointState joint_state;
geometry_msgs::Twist cartesian_pose;
ros::Time processTime;
bool joyCallBack = false;
bool jointStateChecket = false;

bool serviceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    std::cout << "service call triggered" << std::endl;    

	return true;
}

//TODO change naming
void joystickCallback(const geometry_msgs::Twist::ConstPtr& cartesianPose) {
	
	cartesian_pose = *cartesianPose;
	processTime = ros::Time::now();	
	joyCallBack = true;
}

// List of joint angles 
//*
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& jointState) {
	joint_state = *jointState;
	jointStateChecket = true;
}
//*/



int main(int argc, char **argv)
{
	int indx = 0;
	std::cout << "Run cc_node " <<std::endl;
    MyFunctionalClass my_func_class;
	
    /* init ROS node with a name and a node handle*/	
    ros::init(argc, argv, "hbrs_cc_node");
    ros::NodeHandle nh("~");
	
    /* create a loop rate to let your node run only with maximum frequency, here 2Hz */
	ros::Rate loop_rate(20);
	ros::Rate rate(2); //Hz
	// subscribe to topic published by joypad
	
	
	// publish joint velocities to the arm (/arm_1/arm_controller/velocity_command)
	ros::Publisher youbot_arm_joint_vel_cmd_pub = nh.advertise<brics_actuator::JointVelocities>("/arm_1/arm_controller/velocity_command", 1);
	ros::Publisher armPositionsPublisher = nh.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command", 1);	
	
	brics_actuator::JointPositions positionCommand;
	std::vector <brics_actuator::JointValue> armJointPositions;
	armJointPositions.resize(5);
	
	ros::Subscriber joy_sub, node_input, jointState;
	node_input =nh.subscribe<geometry_msgs::Twist>("/cc_node/CartesianParameters", 1, joystickCallback);
	jointState =nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, jointStateCallback);
	 
    arm_cc::ROS_URDF_Loader robot_description;
    std::vector<boost::shared_ptr<urdf::JointLimits> > jointlimits;
    KDL::Chain chain;
    robot_description.loadModel(nh, "arm_link_0", "arm_link_5", chain, jointlimits);
	
	//Watchdog ccWatchdog("hbrs_cc_watchdog",ros::Duration(10));
	
	//tf::TransformListener listener;
	
    Hbrs_cc_Library hbrs_cc_lib(chain);
    int firstLoop = 0;
    double jointAngle[] = {170.0, 50.0, -50.0, 100.0, 167.5};
    
    std::vector<double> jointPosition;
    std::vector<double> transVel, rotVel, jointVelocity;
    ros::Duration watchdogTime(1.0);
    std::stringstream armName;
    
    
	while (ros::ok()) {
		jointPosition.clear();
		ros::spinOnce();
		/*
		if(ros::Time::now() - processTime >  watchdogTime)
		{
			for (unsigned short i = 0; i < joint_state.name.size(); i++) {
				for(unsigned short j = 0; j < 5 ; j++){
					armName.str("");
					armName << "arm_joint_" << j+1;
					if(joint_state.name[i].compare(armName.str()) == 0 ){
						
						jointPosition.push_back(joint_state.position[i]);
						break;
					}
				}		
			}
			for(unsigned short i = 0 ; i < 5 ; i++){
				std::stringstream jointName;
				jointName << "arm_joint_" << (i+1);
				armJointPositions.at(i).joint_uri =  jointName.str();
				armJointPositions.at(i).value = jointPosition[i];
				armJointPositions.at(i).unit = boost::units::to_string(boost::units::si::radians);
			}
			
			positionCommand.positions = armJointPositions;
			armPositionsPublisher.publish(positionCommand);
			rate.sleep();
			std::cout << " Watchdog: No Joystick callback "<< std::endl;
			exit(-1);
		}
		std::cout << "Run cc_node " << indx++ <<std::endl;
		*/
		if(firstLoop++ < 10)
		{
			for(unsigned short i = 0 ; i < 5 ; i++){
				std::stringstream jointName;
				jointName << "arm_joint_" << (i+1);
				armJointPositions.at(i).joint_uri =  jointName.str();
				armJointPositions.at(i).value = jointAngle[i] * (M_PI / 180.0);
				armJointPositions.at(i).unit = boost::units::to_string(boost::units::si::radians);
			}
			
			positionCommand.positions = armJointPositions;
			armPositionsPublisher.publish(positionCommand);
			rate.sleep();
		}	
		else
		{
			while(!jointStateChecket || !joyCallBack)
			{	}	
			transVel.clear();
			rotVel.clear(); 
			jointVelocity.clear();
			
			for (unsigned short i = 0; i < joint_state.name.size(); i++) {
				for(unsigned short j = 0; j < 5 ; j++){
					armName.str("");
					armName << "arm_joint_" << j+1;
					if(joint_state.name[i].compare(armName.str()) == 0 ){
						
						jointPosition.push_back(joint_state.position[i]);
						break;
					}
				}		
			}
			
			transVel.push_back(cartesian_pose.linear.y );
			transVel.push_back(cartesian_pose.linear.x );
			transVel.push_back(cartesian_pose.linear.z );
			//transVel.push_back(0.0 );
			//transVel.push_back(1.0 );
			//transVel.push_back(0.0 );
			rotVel.push_back(cartesian_pose.angular.x );
			rotVel.push_back(cartesian_pose.angular.y );
			rotVel.push_back(cartesian_pose.angular.z );
		
			std::cout << "start joint velcity "<< std::endl;
			hbrs_cc_lib.getJointVelocity(jointPosition, transVel, rotVel, jointVelocity);
			std::cout << "finish joint velcity "<< std::endl;
			
			brics_actuator::JointVelocities command;
			std::vector <brics_actuator::JointValue> setPoints;
			setPoints.resize(5);
			std::stringstream jointName;
			
			for (unsigned short i = 0; i < jointVelocity.size(); ++i) {

				jointName.str("");
				jointName << "arm_joint_" << (i + 1);

				setPoints[i].joint_uri = jointName.str();
				setPoints[i].value = jointVelocity.at(i);

				setPoints[i].unit = boost::units::to_string(boost::units::si::radian_per_second);
				//std::cout << "Joint " << setPoints[i].joint_uri << " = " << setPoints[i].value << std::endl;
			}
			
			command.velocities = setPoints;
			youbot_arm_joint_vel_cmd_pub.publish(command);
		}
		loop_rate.sleep();
	}
	for(unsigned short i = 0 ; i < 5 ; i++){
		std::stringstream jointName;
		jointName << "arm_joint_" << (i+1);
		armJointPositions.at(i).joint_uri =  jointName.str();
		armJointPositions.at(i).value = jointPosition[i];
		armJointPositions.at(i).unit = boost::units::to_string(boost::units::si::radians);
	}
		positionCommand.positions = armJointPositions;
		armPositionsPublisher.publish(positionCommand);
	return 0;
}
