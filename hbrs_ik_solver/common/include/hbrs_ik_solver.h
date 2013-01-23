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
//#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <kdl/kdl.hpp>
#include <Eigen/Dense>
#define DEG_2_RAD (M_PI / 180.0)


class Hbrs_ik_solver
{
public:
	KDL::Chain ikChain;
	unsigned int numberOfJoints;
	KDL::JntArray jointpositions;
	std::vector<double> jointAngle;
	KDL::ChainIkSolverVel_wdls* iksolver;
	Eigen::MatrixXd Mq;
	
	Hbrs_ik_solver(KDL::Chain);
	~Hbrs_ik_solver();
	void solver(std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double> &);
};
#endif /* HBRS_IK_SOLVER_H_ */
