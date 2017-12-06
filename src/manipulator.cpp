/*
 * manipulator.cpp
 *
 *  Created on: 2017年12月6日
 *      Author: xielong
 */

#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Geometry>

#include "manipulator.h"

manipulator::manipulator(Eigen::MatrixXd dh_param){
	Eigen::MatrixXd q_init = Eigen::MatrixXd::Zero(7,1);
	setJointAngle(q_init);
	setDhParam(dh_param);
}

void manipulator::setJointAngle(Eigen::MatrixXd joint_angle){
	manipulator::joint_angle = joint_angle;
}

void manipulator::setDhParam(Eigen::MatrixXd dh_param){
	manipulator::dh_param = dh_param;
}

Eigen::MatrixXd manipulator::getDhParam(){
	return manipulator::dh_param;
}

Eigen::MatrixXd manipulator::jacob(Eigen::MatrixXd joint_angle){
	Eigen::MatrixXd Jn = Eigen::MatrixXd::Zero(6, 7);
	Eigen::MatrixXd J0 = Eigen::MatrixXd::Zero(6, 7);
	Eigen::MatrixXd J_temp = Eigen::MatrixXd::Zero(6, 6);
	Eigen::MatrixXd UT = Eigen::MatrixXd::Identity(4, 4);
	Eigen::MatrixXd dh = manipulator::getDhParam();
	Eigen::Matrix4Xd T_temp(4,4);
	Eigen::MatrixXd delta(3, 1);
	Eigen::MatrixXd D(3, 1);
	for(int i = 6; i >= 0; i--){
		float q = joint_angle(i);
		float d = dh(i,1);
		float a = dh(i,2);
		float alpha = dh(i,3);
		T_temp << cos(q), -sin(q)*cos(alpha), sin(q)*sin(alpha), a*cos(q),
			      sin(q),  cos(q)*cos(alpha),-cos(q)*sin(alpha), a*sin(q),
					0,         sin(alpha),         cos(alpha),       d,
					0,            0,                 0,              1;
		UT = T_temp*UT;
		D << -UT(0,0)*UT(1,3)+UT(1,0)*UT(0,3),
		     -UT(0,1)*UT(1,3)+UT(1,1)*UT(0,3),
		     -UT(0,2)*UT(1,3)+UT(1,2)*UT(0,3);
		delta = UT.block(2,0,1,3).transpose();
		Jn.block(0,i,3,1) = D;
		Jn.block(3,i,3,1) = delta;
	}
	J_temp.block(0,0,3,3) = UT.block(0,0,3,3);
	J_temp.block(3,3,3,3) = UT.block(0,0,3,3);
	J0 = J_temp * Jn;
	return J0;
}

Eigen::MatrixXd manipulator::fkine(Eigen::MatrixXd joint_angle){
	Eigen::MatrixXd joint_position(4,3);
	joint_position.row(0) << 0, 0, 0;
	joint_position.row(1) << 0, 0, 0.20386;
	Eigen::MatrixXd dh = manipulator::getDhParam();
	Eigen::Matrix4Xd T = Eigen::Matrix4Xd::Identity(4,4);
	for(int i = 0; i < joint_angle.rows(); i++){
		float q = joint_angle(i);
		float d = dh(i,1);
		float a = dh(i,2);
		float alpha = dh(i,3);
		Eigen::Matrix4Xd T_temp(4,4);
		T_temp << cos(q), -sin(q)*cos(alpha), sin(q)*sin(alpha), a*cos(q),
			      sin(q),  cos(q)*cos(alpha),-cos(q)*sin(alpha), a*sin(q),
			        0,         sin(alpha),         cos(alpha),       d,
			        0,            0,                 0,              1;
		T = T*T_temp;
//		std::cout << T <<std::endl;
//		std::cout << T.block(0,3,3,1) <<std::endl;
		if(i == 3){
			joint_position.row(2) = T.block(0,3,3,1).transpose();
		}
	}
	joint_position.row(3) = T.block(0,3,3,1).transpose();
//	std::cout << T <<std::endl;
//	std::cout << "Joint position: " <<std::endl;
//	std::cout << joint_position <<std::endl;
	return joint_position;
}

Eigen::MatrixXd manipulator::ikine(Eigen::MatrixXd goal_position){
	float lamda = 0.2;           // damping constant
	float stol = 1e-3;           // tolerance
	int nm_error = 100;        // initial error
	int count = 0;             // iteration count
	int ilimit = 1000;         // maximum iteration
	Eigen::MatrixXd joint_angle(7,1);
	joint_angle << 0, 0, 0, M_PI/2, 0, M_PI/2, 0;
	while(nm_error > stol){
		Eigen::MatrixXd end_position = manipulator::fkine(joint_angle).row(3);
		Eigen::MatrixXd error = goal_position - end_position;

	}
//	while nm_error > stol
//	                end_position = this.model.fkine(joint_angle);
//	                end_position = end_position.t;
//	                error = goal_position - end_position;
//	                jacob = this.model.jacob0(joint_angle);
//	                f = linsolve(jacob*jacob' + lamda^2 * eye(size(this.model.links, 2) - 1), [error; 0; 0; 0]);
//	                d_joint_angle = jacob' * f;
//	                joint_angle = joint_angle + d_joint_angle;
//	                nm_error = norm(error);
//	                count = count + 1;
//	                if count > ilimit
//	                    disp('Solution wouldn''t converge');
//	                    break;
//	                end
//	            end
}













Eigen::Matrix4Xd manipulator::transMatrix (Eigen::MatrixXd link, float q){
	float d = link(1);
	float a = link(2);
	float alpha = link(3);
	Eigen::Matrix4Xd result(4,4);
	result << cos(q), -sin(q)*cos(alpha), sin(q)*sin(alpha), a*cos(q),
		 sin(q),  cos(q)*cos(alpha),-cos(q)*sin(alpha), a*sin(q),
		   0,         sin(alpha),         cos(alpha),       d,
		   0,            0,                 0,              1;
	return result;
}


