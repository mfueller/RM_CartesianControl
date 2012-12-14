/*
 * my_functional_class.cpp
 *
 *  Created on: Aug 14, 2011
 *      Author: Frederik Hegger
 */

#ifndef MY_FUNCTIONAL_CLASS_H_
#define MY_FUNCTIONAL_CLASS_H_

#include <brics_actuator/JointVelocities.h>
#include <sensor_msgs/JointState.h>

class MyFunctionalClass
{
public:
	MyFunctionalClass();
	~MyFunctionalClass();
    /* example function which does your main computation */
    brics_actuator::JointVelocities generateArmMsg(std::vector<double>);
    std::vector<double> getJointVelocities();	
};


#endif /* MY_FUNCTIONAL_CLASS_H_ */
