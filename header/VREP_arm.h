/*
 * VREP_arm.h
 *
 *  Created on: 2017年12月7日
 *      Author: xielong
 */

#ifndef HEADER_VREP_ARM_H_
#define HEADER_VREP_ARM_H_

#include "../header/VREP.h"

class VREP_arm: public VREP{
public:
	int link_num;
	int link_handle[7];
public:
	VREP_arm(const char* name, int clientID, int mode);
	Eigen::MatrixXd getPosition();
	void setJointPos(float *joint);
};




#endif /* HEADER_VREP_ARM_H_ */
