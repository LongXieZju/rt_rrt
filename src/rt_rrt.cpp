//============================================================================
// Name        : rt_rrt.cpp
// Author      : Xielong
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Geometry>
#include "../header/manipulator.h"


//int main(){
//	Eigen::MatrixXd manipulator_dh(7,4);
//	manipulator_dh <<  0, 0.20386, 0, M_PI/2,
//			0, 0, 0, -M_PI/2,
//			0, 0.29126, 0, M_PI/2,
//			0, 0, 0, -M_PI/2,
//			0, 0.32363,  0, M_PI/2,
//			0, 0, 0, -M_PI/2,
//			0, 0.15512, 0, 0;
//	manipulator ma(manipulator_dh);
//
//	Eigen::MatrixXd m = Eigen::MatrixXd::Random(1,7);
//	std::cout << manipulator_dh << std::endl;
//	return 0;
//
//}
