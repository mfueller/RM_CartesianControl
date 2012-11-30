/*
 * cc_library.cpp
 *
 *  Created on: Nov 30, 2012
 * 
 */

#ifndef CC_LIBRARY_H_
#define CC_LIBRARY_H_


class Cc_Library
{
public:
	Cc_Library();
	~Cc_Library();
	struct twist;
	std::vector<double> ikSolver(struct twist , std::vector<double>);
	std::vector<double> getJointVelocity(struct twist);
	//internal function
	// 1. calculate the new joint position from old joint position, joint velocity and time step
	// 2. singularity and limit checking for joint
	// 3. reading arm description from file and set it in a structure
	// 4. 
	
	
};


#endif /* CC_LIBRARY_H_ */
