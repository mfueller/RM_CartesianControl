#include "hbrs_ik_solver.h" 

using namespace KDL;

Hbrs_ik_solver::Hbrs_ik_solver()
{   
    
    // youbot chain with our DH parameters from the RM lecture compared with matlab toolbox
    chain.addSegment(Segment(Joint(Joint::None), Frame::DH(0.167, M_PI , 0.161, 0.0)));
    chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(0.033, M_PI / 2.0 , 0.0, 0.0)));
    chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(0.155, 0.0, 0.0, 0.0)));
    chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(0.135, 0.0, 0.0, 0.0)));
    chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(0.0, M_PI / 2.0, 0.0, 0.0)));
    chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(0.0, M_PI, -0.218, 0.0)));    
 
    // Create joint array
    numberOfJoints = chain.getNrOfJoints();
    jointpositions = JntArray(numberOfJoints);
    jointAngle.assign(numberOfJoints,0.0);
    
    jointpositions(0) =  jointAngle.at(0) + (-169.0 * DEG_2_RAD);
    jointpositions(1) =  jointAngle.at(1) + (-65.0 - 90.0) * DEG_2_RAD;
    jointpositions(2) =  jointAngle.at(2) + 146.0 * DEG_2_RAD;
    jointpositions(3) =  jointAngle.at(3) + (-102.5 - 90.0) * DEG_2_RAD;
    jointpositions(4) =  jointAngle.at(4) + 167.0 * DEG_2_RAD;
}

Hbrs_ik_solver::~Hbrs_ik_solver()
{
    /* clean up here */
}
 
using namespace KDL;
 
void Hbrs_ik_solver::solver(std::vector<double> jointPosition, std::vector<double> transVel, std::vector<double> rotVel, std::vector<double> &jointVelocity)
{
	KDL::Vector vel(transVel.at(0), transVel.at(1), transVel.at(2));
	KDL::Vector rot(rotVel.at(0), rotVel.at(1), rotVel.at(2));
	for (unsigned short i = 0; i < numberOfJoints; i++) {
		jointpositions(i) = jointPosition.at(i);
	}
	
	KDL::Twist twist(vel,rot);
	
	
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
    ChainIkSolverVel_pinv_givens iksolver = ChainIkSolverVel_pinv_givens(chain);
    
    KDL::JntArray jntArray(chain.getNrOfJoints());
    iksolver.CartToJnt(jointpositions , twist, jntArray);
    for (unsigned short i = 0; i < numberOfJoints; i++) {
		jointVelocity.push_back(jntArray(i));
	}
}
