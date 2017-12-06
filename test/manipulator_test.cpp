/*
 * manipulator_test.cpp
 *
 *  Created on: Dec 6, 2017
 *      Author: xielong
 */
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Geometry>

#include "../header/manipulator.h"


int main(){
	Eigen::MatrixXd manipulator_dh(7,4);
	manipulator_dh <<  0, 0.20386, 0, M_PI/2,
				0, 0, 0, -M_PI/2,
				0, 0.29126, 0, M_PI/2,
				0, 0, 0, -M_PI/2,
				0, 0.32363,  0, M_PI/2,
				0, 0, 0, -M_PI/2,
				0, 0.15512, 0, 0;
	manipulator ma(manipulator_dh);

	Eigen::MatrixXd joint_angle(7,1);
	joint_angle << 90, 0, 0, 0, 0, 0, 0;
	ma.fkine(joint_angle);
}
