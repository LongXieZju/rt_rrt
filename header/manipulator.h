/*
 * manipulator.h
 *
 *  Created on: Dec 6, 2017
 *      Author: xielong
 */

#ifndef HEADER_MANIPULATOR_H_
#define HEADER_MANIPULATOR_H_

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

	Eigen::MatrixXd fkine(Eigen::MatrixXd joint_angle);
	Eigen::MatrixXd ikine(Eigen::MatrixXd end_effector);

	Eigen::Matrix4Xd transMatrix (Eigen::MatrixXd manipulator_dh, float q);
};

#endif /* HEADER_MANIPULATOR_H_ */
