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
#include "hbrs_ik_solver.h"
#include <kdl/chain.hpp>



class Hbrs_cc_Library
{
	
public:
	struct Hbrs_twist
	{	
		std::vector<double> linear;
		std::vector<double> angular;
	};
	Hbrs_cc_Library(KDL::Chain&);
	~Hbrs_cc_Library();
	KDL::Chain ccChain;
	std::vector<double> ikSolver(Hbrs_twist , std::vector<double>);
	void getJointVelocity(std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double> &);
	void jointVelocityNormalizer(std::vector<double> &);
	void jointLimitAdaptor(std::vector<double>, std::vector<double> &);
private:
	// 0.01 is the youbot max velocity ,       remember to make it general for other robots too
	double maxJointVelocity;
	//double jointMinLimit[];
	//double jointMaxLimit[];
	double softJointLimit;
	double hardJointLimit;
	double softLimitFactor;
	Hbrs_ik_solver* ik_solver;
};


#endif /* HBRS_CC_LIBRARY_H_ */
