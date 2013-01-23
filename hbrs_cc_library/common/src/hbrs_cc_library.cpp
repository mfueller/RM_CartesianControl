#include "hbrs_cc_library.h"


Hbrs_cc_Library::Hbrs_cc_Library(KDL::Chain& chain)
{
    maxJointVelocity = 0.1;
    softJointLimit = 10.0 * (M_PI / 180.0);
	hardJointLimit = 2.0 * (M_PI / 180.0);
	softLimitFactor = 10.0;
	ccChain = chain;
	ik_solver = new Hbrs_ik_solver(ccChain);
}

Hbrs_cc_Library::~Hbrs_cc_Library()
{
    /* clean up here */
}


void Hbrs_cc_Library::getJointVelocity(std::vector<double> jointPosition, std::vector<double> transVel, std::vector<double> rotVel, std::vector<double> &jointVelocity) {
	ik_solver->solver(jointPosition, transVel, rotVel, jointVelocity);
	jointVelocityNormalizer(jointVelocity);
	jointLimitAdaptor(jointPosition, jointVelocity);
}

void Hbrs_cc_Library::jointVelocityNormalizer(std::vector<double> &jointVelocity){
	double currentMaxVelocity = 0.0;
	for(unsigned short i = 0 ; i < jointVelocity.size() ; i++){
		if(std::fabs(jointVelocity[i]) > currentMaxVelocity)
		{
			currentMaxVelocity = std::fabs(jointVelocity[i]);
		}
		std::cout << jointVelocity[i] << " ";
	}
	std::cout << std::endl;
	std::cout << "currentMaxVelocity " << currentMaxVelocity << std::endl;
	std::cout << "\njointVelocityNormalizer: ";
	if(currentMaxVelocity > maxJointVelocity){
		for(unsigned short i = 0 ; i < jointVelocity.size() ; i++){
			jointVelocity[i] = jointVelocity[i] * (maxJointVelocity / currentMaxVelocity);
			std::cout << jointVelocity[i] << " ";
		}
	}
	std::cout << std::endl;
}

void Hbrs_cc_Library::jointLimitAdaptor(std::vector<double> jointPosition, std::vector<double> &jointVelocity) { 
	double jointMinLimit[] = {0.000891336 , 0.0000727539 , -5.18122 , 0.00166331 , 0.000929437};
    double jointMaxLimit[] = {5.89556 , 2.70532 , -0.00116239 , 3.56719 , 5.8388};  
	for (unsigned short i=0; i < jointPosition.size() ; i++) {
		if(jointPosition[i] < (jointMinLimit[i] + softJointLimit) && jointVelocity[i] < 0){
			if(jointPosition[i] < jointMinLimit[i] + hardJointLimit){
				std::cout << "jointLimitAdaptor hard min limit " << i << " , "<< jointPosition[i] << " < "<< (jointMinLimit[i] + hardJointLimit) <<std::endl;
				jointVelocity.assign(jointVelocity.size(),0);
				break;
			} 
			else {
				for (unsigned short j=0; j < jointVelocity.size() ; j++)
					jointVelocity[j] /= softLimitFactor;
				break;
			} 
		}
		else if(jointPosition[i] > (jointMaxLimit[i] - softJointLimit) && jointVelocity[i] > 0){
			if(jointPosition[i] > jointMaxLimit[i] - hardJointLimit){
				jointVelocity.assign(jointVelocity.size(),0);	
				std::cout << "jointLimitAdaptor hard max limit " << i <<" , " << jointPosition[i] << " > " << jointMaxLimit[i] << " - " << hardJointLimit <<std::endl;
				break;
			}
			else {
				for (unsigned short j=0; j < jointVelocity.size() ; j++)
					jointVelocity[j] /= softLimitFactor;
				break;
			}
		}
	} 
}


