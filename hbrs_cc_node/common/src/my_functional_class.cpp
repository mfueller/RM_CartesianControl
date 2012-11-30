/*
 * my_functional_class.h
 *
 *  Created on: Aug 14, 2011
 *      Author: Frederik Hegger
 *  Modified by Jeyaprakash, Marc, Yashar
 */

#include "my_functional_class.h"


MyFunctionalClass::MyFunctionalClass()
{
    /* init and configure here */
}

MyFunctionalClass::~MyFunctionalClass()
{
    /* clean up here */
}

brics_actuator::JointVelocities MyFunctionalClass::generateArmMsg(std::vector<double> joint_velocity_value)
{
	// Msg type brics_actuator::JointVelocities
	brics_actuator::JointVelocities joint_velocities;
	std::vector<brics_actuator::JointValue> brics_joint_value(5); 
	ros::Time now = ros::Time::now();
	
	std::string joint_names[5];
	joint_names[0] = "arm_joint_1";
	joint_names[1] = "arm_joint_2";
	joint_names[2] = "arm_joint_3";
	joint_names[3] = "arm_joint_4";
	joint_names[4] = "arm_joint_5";
	 
	for (int i=0;i<5;i++) { 
		brics_joint_value[i].joint_uri = joint_names[i];
		brics_joint_value[i].value = joint_velocity_value[i];
		brics_joint_value[i].unit = "rad"; // TODO set the right unit  
		brics_joint_value[i].timeStamp = now;
    }
    
    joint_velocities.velocities = brics_joint_value; 
    
    return joint_velocities;
}

std::vector<double> MyFunctionalClass::getJointVelocities()
{
	std::vector<double> my_data(5);
    return my_data;
}


