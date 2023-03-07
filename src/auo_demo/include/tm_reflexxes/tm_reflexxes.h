/*********************************************************************
 * tm_reflexxes.h
 *
 * Copyright (c) 2017, ISCI / National Chiao Tung University (NCTU)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************
 * 
 * Author: Howard Chen
 */

#ifndef TM_REFLEXXES_H
#define TM_REFLEXXES_H

#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <sys/time.h>
#include <iostream>
#include <termios.h>
#include <math.h>
#include <iostream>

#include "tm_driver/tm_driver.h"
#include "tm_reflexxes/tm_reflexxes.h"
#include "tm_kinematics/tm_kin.h"

#include "tm_msgs/SendScript.h"
#include "tm_msgs/SetVelocity.h"

#include "ReflexxesAPI.h"
#include "RMLPositionFlags.h"
#include "RMLPositionInputParameters.h"
#include "RMLPositionOutputParameters.h"

#include "RMLVelocityFlags.h"
#include "RMLVelocityInputParameters.h"
#include "RMLVelocityOutputParameters.h"


#define CYCLE_TIME_IN_SECONDS 0.025
#define NUMBER_OF_DOFS 		  6
#define DEG2RAD 			  0.01745329252
#define RAD2DEG 			  57.29577951

#define MAX_VELOCITY 		  1.5
#define MAX_ACC 			  0.0375*40 // 0.0375 : acc in 25ms

static struct termios oldt, newt;

namespace tm_reflexxes {

	void initTermios(int echo);

    void resetTermios();

    int kbhit();

	void ReflexxesSmoothStop(       ros::ServiceClient& vel_client,
	                                RMLVelocityInputParameters& InputState,
	                                double SynTime);

	void ReflexxesSmoothStop(       ros::ServiceClient& vel_client,
	                                RMLPositionInputParameters *InputState,
	                                double SynTime);

	bool ReflexxesVelocityRun(      ros::ServiceClient& vel_client,
	                                RMLPositionInputParameters& InputState, 
	                                std::vector<double> TargetVelocity, 
	                                double SynTime);


	bool ReflexxesPositionRun(      ros::ServiceClient& vel_client,
	                                RMLPositionInputParameters &InputState,
	                                std::vector<double> TargetPosition, 
	                                std::vector<double> TargetVelocity,
	                                double SynTime);


	//**************************************************************************
	//Simulation Function : Only generate trajectory
	
	void ReflexxesSmoothStop_sim(   RMLVelocityInputParameters *InputState, 
                                    double SynTime);

	void ReflexxesSmoothStop_sim(   RMLPositionInputParameters *InputState, 
                                    double SynTime);

	bool ReflexxesVelocityRun_sim(  RMLVelocityInputParameters &InputState, 
                                    std::vector<double> TargetVelocity, 
                                    double SynTime);

	bool ReflexxesVelocityRun_sim(  RMLVelocityInputParameters &InputState, 
                                    std::vector<double> TargetVelocity);

	bool ReflexxesPositionRun_sim(  RMLPositionInputParameters &InputState, 
                                    std::vector<double> TargetPosition,
                                    std::vector<double> TargetVelocity, 
                                    double SynTime);
	
    bool ReflexxesPositionSafetyRun_sim(  RMLPositionInputParameters &InputState, 
                                      std::vector<double> TargetPosition,
                                      std::vector<double> TargetVelocity, 
                                      double SynTime);
	
	//Simulation Function : ONly generate trajectory 
	//**************************************************************************
}
#endif //TM_REFLEXXES_H
