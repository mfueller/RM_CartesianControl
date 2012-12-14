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
}

Hbrs_ik_solver::~Hbrs_ik_solver()
{
    /* clean up here */
}
 
using namespace KDL;
 
int Hbrs_ik_solver::fkSolver()
{
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
    ChainIkSolverVel_pinv iksolver = ChainIkSolverVel_pinv(chain);
    //*
    std::cout << numberOfJoints << std::endl;
    jointpositions(0) =  jointAngle.at(0) + (-169.0 * DEG_2_RAD);
    jointpositions(1) =  jointAngle.at(1) + (-65.0 - 90.0) * DEG_2_RAD;
    jointpositions(2) =  jointAngle.at(2) + 146.0 * DEG_2_RAD;
    jointpositions(3) =  jointAngle.at(3) + (-102.5 - 90.0) * DEG_2_RAD;
    jointpositions(4) =  jointAngle.at(4) + 167.0 * DEG_2_RAD;
    //*/
    
    // Create the frame that will contain the results
    KDL::Frame cartpos;    
 
    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0) {
        std::cout << cartpos <<std::endl;
        std::cout << "Succes, thanks KDL!"<< std::endl;
    }
    else {
        std::cout << "Error: could not calculate forward kinematics :(" << std::endl;
    }
    
    //iksolver.CartToJnt(jointpositions, , output);
}
