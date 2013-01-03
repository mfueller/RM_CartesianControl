#include "hbrs_cc_library.h"


Hbrs_cc_Library::Hbrs_cc_Library(std::string armDescriptionFile)
{
	
    /* init and configure here */
}

Hbrs_cc_Library::~Hbrs_cc_Library()
{
    /* clean up here */
}


void Hbrs_cc_Library::getJointVelocity(std::vector<double> jointPosition, std::vector<double> transVel, std::vector<double> rotVel, std::vector<double> &jointVelocity) {
	Hbrs_ik_solver ik_solver;
	if (transVel.at(0) == 0.0 && transVel.at(1) == 0.0 && transVel.at(2) == 0.0) {
		jointVelocity.clear();
		jointVelocity.assign(5, 0.0);
	}
	else {
		ik_solver.solver(jointPosition, transVel, rotVel, jointVelocity);
	}
}



