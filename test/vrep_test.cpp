/*
 * vrep_test.cpp
 *
 *  Created on: 2017年12月7日
 *      Author: xielong
 */
// Copyright 2006-2017 Coppelia Robotics GmbH. All rights reserved.
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
//
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
//
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.4.0 rev. 1 on April 5th 2017

// Make sure to have the server side running in V-REP!
// Start the server from a child script with following command:
// simExtRemoteApiStart(portNumber) -- starts a remote API server service on the specified port

// Copyright 2006-2017 Coppelia Robotics GmbH. All rights reserved.
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
//
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
//
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.4.0 rev. 1 on April 5th 2017

// Make sure to have the server side running in V-REP!
// Start the server from a child script with following command:
// simExtRemoteApiStart(portNumber) -- starts a remote API server service on the specified port

#include <stdio.h>
#include <stdlib.h>

extern "C" {
    #include "extApi.h"
	#include "extApi.c"
	#include "extApiPlatform.c"
}

int main()
{
    int Port = 19997;
    int PositionControlHandle;
    simxChar* Adresse = "127.0.0.1";
    float position[3];


    int clientID = simxStart(Adresse, Port, true,true,2000,5);

    if (clientID != -1)
    {
        printf("V-rep connected.");
        extApi_sleepMs(300);
        while (simxGetConnectionId(clientID) != -1)
        {
            simxGetObjectHandle(clientID,"IRB140_manipulationSphere", &PositionControlHandle, simx_opmode_oneshot);
            simxGetObjectPosition(clientID,PositionControlHandle,-1,position, simx_opmode_oneshot);
            printf("(%f,%f,%f)\r\n",position[0],position[1],position[2]);
        }

        simxFinish(clientID);
    }
    else {
        printf("V-rep can't be connected.");
        extApi_sleepMs(300);
    }

    return 0;
}











