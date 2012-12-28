/*
 * hbrs_ik_solver.h
 *
 *  Created on: Dec 14, 2012
 * 
 * Jeyaprakash, Marc, Yashar
 * 
 */

#ifndef HBRS_IK_SOLVER_H_
#define HBRS_IK_SOLVER_H_

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>

#define DEG_2_RAD (M_PI / 180.0)


class Hbrs_ik_solver
{
public:
	KDL::Chain chain;
	unsigned int numberOfJoints;
	KDL::JntArray jointpositions;
	std::vector<double> jointAngle;
	
	Hbrs_ik_solver();
	~Hbrs_ik_solver();
	void solver(std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double> &);
};
#endif /* HBRS_IK_SOLVER_H_ */
