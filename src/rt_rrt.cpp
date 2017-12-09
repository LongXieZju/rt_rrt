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
#include "../header/Manipulator.h"
#include "../header/VREP.h"
#include "../header/VREP_arm.h"

Eigen::MatrixXd getObstaclesPos(int* obstacles, VREP v){
	int num = sizeof(obstacles)/sizeof(*obstacles);
	Eigen::MatrixXd pos(3, num);
	for(int i = 0; i < num; i++){
		pos.col(i) = v.getPosition(obstacles[i]);
	}
	return pos;
}

int main(){
	srand((unsigned)time(0));  //random sample differently every time

	// Manipulator model, contains forward and backward kinematics models
	Eigen::MatrixXd manipulator_dh(7,4);
	manipulator_dh <<  0, 0.20386, 0, M_PI/2,
			0, 0, 0, -M_PI/2,
			0, 0.29126, 0, M_PI/2,
			0, 0, 0, -M_PI/2,
			0, 0.32363,  0, M_PI/2,
			0, 0, 0, -M_PI/2,
			0, 0.15512, 0, 0;
	Eigen::MatrixXd joint_start(7,1);
	joint_start << 0, 0, 0, M_PI/2, 0, M_PI/2, 0;
	Manipulator seven_arm(manipulator_dh);

	// VREP, to gain environment info
	VREP v;
	v.simStop();
	v.connect();
	Eigen::MatrixXd goal_position = v.getPosition(v.getHandle("goal"));
	std::cout << "****goal_position****" << std::endl;
	std::cout << goal_position.transpose() << std::endl;
	int obstacles[3] = {v.getHandle("obstacle_1"), v.getHandle("obstacle_2"), v.getHandle("obstacle_3")};
	Eigen::MatrixXd obs_position = getObstaclesPos(obstacles, v);

	// Setting up
	seven_arm.setGoalPosition(goal_position);
	seven_arm.setStartState(joint_start);

	//RRT
	Eigen::MatrixXd new_node(seven_arm.link_num,1);
	NearestNode nearest_node;
	clock_t start_jacob = clock();
	///test
//	Eigen::MatrixXd A(3,2);
//	A << 1,2,3,4,5,6;
//	Eigen::MatrixXd B(3,2);
//	B << 2,3,4,5,6,7;
//	std::cout << A << std::endl;
	///
	for(int i = 0; i < seven_arm.max_iter; i++){
		new_node = seven_arm.sampleNode();
		nearest_node = seven_arm.getNearestNode(new_node);
		std::cout << "****nearest_dist/index****" << std::endl;
		std::cout << nearest_node.nearest_dist << "/"<< nearest_node.ind << std::endl;
		if(nearest_node.nearest_dist > seven_arm.node_max_step){
			new_node = seven_arm.steer(new_node, nearest_node.ind);
		}
//		if(!seven_arm.obstacle_collision(new_node, nearest_node.ind)){
//
//		}
		seven_arm.getNeighbors(new_node, nearest_node.ind);
//		std::cout << "********" << std::endl;
//		std::cout << nearest_node.ind << std::endl;
	}
	clock_t ends_jacob = clock();
	std::cout <<"RRT Running Time : "<<(double)(ends_jacob - start_jacob)/ CLOCKS_PER_SEC << std::endl;

	Eigen::MatrixXd m = Eigen::MatrixXd::Random(1,7);
	std::cout << manipulator_dh << std::endl;
	return 0;

}
