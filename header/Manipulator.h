/*
 * manipulator.h
 *
 *  Created on: 2017年12月6日
 *      Author: xielong
 */

#ifndef MANIPULATOR_H_
#define MANIPULATOR_H_

#include "../header/Struct.h"

class Manipulator{
private:
	Eigen::MatrixXd joint_angle;
	Eigen::MatrixXd dh_param;
	Eigen::MatrixXd T[4];

public:
	int link_num;
	int max_iter;
	int node_added;
	int step_div;
	int obstacle_num;
	float arm_radius;
	float goal_bais;
	float node_max_step;
	float obs_radius[3] = {0.11, 0.1, 0.07};

	Eigen::MatrixXd goal_position;
	Eigen::MatrixXd goal_angle;
	Eigen::MatrixXd start_angle;
	Eigen::MatrixXd max_ang;
	Eigen::MatrixXd min_ang;

	Eigen::MatrixXd tree;
//	Eigen::MatrixXd;
//	Eigen::MatrixXd;
public:
	Manipulator(Eigen::MatrixXd dh_param);
	void setJointAngle(Eigen::MatrixXd q);
	void setDhParam(Eigen::MatrixXd dh_param);
	Eigen::MatrixXd getDhParam();
	void setGoalPosition(Eigen::MatrixXd goal_position);
	void setStartState(Eigen::MatrixXd joint_angle);

	Eigen::MatrixXd jacob(Eigen::MatrixXd joint_angle);
	Eigen::MatrixXd fkine(Eigen::MatrixXd joint_angle);
	Eigen::MatrixXd ikine(Eigen::MatrixXd end_effector);

	Eigen::Matrix4Xd transMatrix(Eigen::MatrixXd manipulator_dh, float q);

	Eigen::MatrixXd sampleNode();
	Eigen::MatrixXd steer(Eigen::MatrixXd new_node, int nearest_node_ind);
	NearestNode getNearestNode(Eigen::MatrixXd node);
	Eigen::MatrixXd getNeighbors(Eigen::MatrixXd new_node, int nearest_node_ind);
	int obstacle_collision(Eigen::MatrixXd new_node, int nearest_node_ind, Eigen::MatrixXd obs_position);
};

#endif /* MANIPULATOR_H_ */
