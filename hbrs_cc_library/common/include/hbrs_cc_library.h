/*
 * cc_library.cpp
 *
 *  Created on: Nov 30, 2012
 * 
 */
 

#ifndef HBRS_C_LIBRARY_H_
#define HBRS_CC_LIBRARY_H_

#include <iostream>
#include <stdio.h>
#include <vector>



class Hbrs_cc_Library
{
	
public:
	struct Hbrs_twist
	{	
		std::vector<double> linear;
		std::vector<double> angular;
	};
	Hbrs_cc_Library(std::string);
	~Hbrs_cc_Library();
	std::vector<double> ikSolver(Hbrs_twist , std::vector<double>);
	std::vector<double> getJointVelocity(Hbrs_twist);
	//internal function
	// 1. calculate the new joint position from old joint position, joint velocity and time step
	// 2. singularity and limit checking for joint
	// 3. reading arm description from file and set it in a structure
	// 4. 
	
	
};


#endif /* HBRS_CC_LIBRARY_H_ */
