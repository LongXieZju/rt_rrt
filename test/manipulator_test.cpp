/*
 * manipulator_test.cpp
 *
 *  Created on: 2017年12月6日
 *      Author: xielong
 */

#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Geometry>

#include "../src/manipulator.h"

//int main(){
//	Eigen::MatrixXd manipulator_dh(7,4);
//	manipulator_dh <<  0, 0.20386, 0, M_PI/2,
//				0, 0, 0, -M_PI/2,
//				0, 0.29126, 0, M_PI/2,
//				0, 0, 0, -M_PI/2,
//				0, 0.32363,  0, M_PI/2,
//				0, 0, 0, -M_PI/2,
//				0, 0.15512, 0, 0;
//	manipulator ma(manipulator_dh);
//
//	Eigen::MatrixXd joint_angle(7,1);
//	joint_angle << 90, 0, 45, 0, 0, 0, 0;
////	std::cout << ma.jacob(joint_angle) <<std::endl;
//	Eigen::MatrixXd goal_position(1, 3);
//	goal_position << -0.1498, -0.4, -0.2697;
//	std::cout << ma.ikine(goal_position) <<std::endl;
//}


