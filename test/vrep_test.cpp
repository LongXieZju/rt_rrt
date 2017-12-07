/*
 * vrep_test.cpp
 *
 *  Created on: 2017年12月7日
 *      Author: xielong
 */

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <eigen3/Eigen/Geometry>

#include "../header/VREP.h"
#include "../header/VREP_arm.h"

//extern "C" {
//    #include "extApi.h"
//	#include "extApi.c"
//	#include "extApiPlatform.c"
//}

int main()
{
    VREP v;
    v.connect();
    if(v.clientID == -1)
    		return 0;
    int h = v.getHandle("goal");

    VREP_arm arm("goal", v.clientID, v.mode);

    	clock_t start = clock();
    	arm.getPosition();
    	clock_t ends = clock();
    	std::cout <<"Running Time : "<<(double)(ends - start)/ CLOCKS_PER_SEC << std::endl;

    	std::cout <<"**********************************************************"<< std::endl;

    	float joint[] = {M_PI/2, 0, 0,  0,  0,  0,  0};
    	clock_t startgetPosition = clock();
    	arm.setJointPos(joint);
    	clock_t endsgetPosition = clock();
    	std::cout <<"Running Time : "<<(double)(endsgetPosition - startgetPosition)/ CLOCKS_PER_SEC << std::endl;

    	std::cout <<"**********************************************************"<< std::endl;

    return 0;
}











