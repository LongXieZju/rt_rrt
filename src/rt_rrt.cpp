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
	int num = sizeof(obstacles)/sizeof(*obstacles) + 1;
	Eigen::MatrixXd pos(3, num);
	for(int i = 0; i < num; i++){
		pos.col(i) = v.getPosition(obstacles[i]);
	}
	return pos;
}

int main(){
//	srand((unsigned)time(0));  //random sample differently every time

	///test
	Eigen::MatrixXd A(3,2);
	A << -2,2,3,4,5,6;
	Eigen::MatrixXd B(3,2);
	B << 0,0,0,0,0,0;
	Eigen::MatrixXd C;
	C = (A.array()>0).select(A,0);
//	C.select(A, 0);
	Eigen::MatrixXd D;
	std::cout <<  C << std::endl;
	std::cout <<  ((A.array() == 3).select(A, B)) << std::endl;
	std::cout << "********" << std::endl;
	std::cout <<  (A.array() > 0).matrix()(1,1) << std::endl;
	///

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
	std::cout << "****obs_handle****" << std::endl;
	std::cout << obstacles[0] << "," << obstacles[1] << "," << obstacles[2] << std::endl;
	Eigen::MatrixXd obs_position = getObstaclesPos(obstacles, v);
	// VREP manipulator model
	VREP_arm v_arm("redundantRob", v.clientID, v.mode);


	// Setting up
	seven_arm.setGoalPosition(goal_position);
	seven_arm.setStartState(joint_start);
	std::cout << "****goal_angle****" << std::endl;
	std::cout << seven_arm.goal_angle.transpose() << std::endl;
	std::cout << "****start_angle****" << std::endl;
	std::cout << seven_arm.start_angle.transpose() << std::endl;
	std::cout << "****obs_position****" << std::endl;
	std::cout << obs_position.transpose() << std::endl;

	//RRT
	Eigen::MatrixXd new_node(seven_arm.link_num,1);
	NearestNode nearest_node;
	int new_node_ind;


	clock_t start_jacob = clock();
	int count = 0;
	for(int i = 0; i < seven_arm.max_iter; i++){
		count++;
		new_node = seven_arm.sampleNode();
		nearest_node = seven_arm.getNearestNode(new_node);
		if(nearest_node.nearest_dist > seven_arm.node_max_step){
			new_node = seven_arm.steer(new_node, nearest_node.ind);
		}
		if(seven_arm.obstacleCollision(new_node, nearest_node.ind, obs_position)){
			new_node_ind = seven_arm.insertNode(new_node, nearest_node.ind);
			if((new_node - seven_arm.goal_angle).norm() < seven_arm.node_max_step){
				if(seven_arm.obstacleCollision(new_node, seven_arm.goal_angle, obs_position)){
					std::cout << "Find path" << std::endl;
					seven_arm.findPath(new_node_ind);
					break;
				}
			}
		}
	}
	clock_t ends_jacob = clock();
	std::cout << "****iterations****" << std::endl;
		std::cout << count << " " << "Nodes" << std::endl;
	std::cout << "****node_added****" << std::endl;
	std::cout << seven_arm.node_added << " " << "Nodes" << std::endl;
	std::cout <<"RRT Running Time : "<<(double)(ends_jacob - start_jacob)/ CLOCKS_PER_SEC << std::endl;
	std::cout << "****path****" << std::endl;
	int path_ind;
	Eigen::MatrixXd joint_angle(7, 1);
	v.simStart();
	while(!seven_arm.back_trace.empty()){
		path_ind = seven_arm.back_trace.top();
		joint_angle = seven_arm.tree.col(path_ind);
		v_arm.setJointPos(joint_angle - seven_arm.start_angle);
//		std::cout << seven_arm.back_trace.top() << " " << std::endl;
		seven_arm.back_trace.pop();
		v.simSleep(1);
	}
	v.simStop();
//	std::cout << "****tree****" << std::endl;
//	std::cout << seven_arm.tree.leftCols(10) << std::endl;
	return 0;

}
