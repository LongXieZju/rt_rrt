/*
 * vrep_connection.h
 *
 *  Created on: 2017年12月7日
 *      Author: xielong
 */

#ifndef HEADER_VREP_H_
#define HEADER_VREP_H_

class VREP{
public:
	int clientID;
	int mode;

public:
	void connect();
	void simPause();
	void simStart();
	void simStop();
	void simSleep(int time);
	int getHandle(const char* name);
	Eigen::MatrixXd getPosition(int handle);
	void setJointPos(int handle, float joint);
};



#endif /* HEADER_VREP_H_ */
