/*
 * manipulator.h
 *
 *  Created on: 2017年12月6日
 *      Author: xielong
 */

#ifndef MANIPULATOR_H_
#define MANIPULATOR_H_

class manipulator{
private:
	Eigen::MatrixXd joint_angle;
	Eigen::MatrixXd dh_param;
	Eigen::MatrixXd T[4];
public:
	manipulator(Eigen::MatrixXd dh_param);
	void setJointAngle(Eigen::MatrixXd q);
	void setDhParam(Eigen::MatrixXd dh_param);
	Eigen::MatrixXd getDhParam();

	Eigen::MatrixXd jacob(Eigen::MatrixXd joint_angle);
	Eigen::MatrixXd fkine(Eigen::MatrixXd joint_angle);
	Eigen::MatrixXd ikine(Eigen::MatrixXd end_effector);

	Eigen::Matrix4Xd transMatrix (Eigen::MatrixXd manipulator_dh, float q);
};

#endif /* MANIPULATOR_H_ */
