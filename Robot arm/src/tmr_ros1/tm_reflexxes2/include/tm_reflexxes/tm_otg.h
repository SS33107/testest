
/*********************************************************************
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

#ifndef TM_OTG
#define TM_OTG

#include <eigen3/Eigen/Dense>

#include <stdio.h>
#include <string>
#include <vector>
#include <stdlib.h>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <sys/time.h>
#include <iostream>
#include <termios.h>
#include <math.h>

#include "tm_driver/tm_driver.h"
#include "tm_reflexxes/tm_reflexxes.h"
#include "tm_kinematics/tm_kin.h"

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>


#define MAX_VELOCITY          1.5
#define MAX_ACC               0.0375*40 // 0.0375 : acc in 25ms
#define DEG2RAD 0.01745329252
#define RAD2DEG 57.29577951
#define REPEATABILITY 0.01//0.00005

#define CYCLE_TIME_IN_SECONDS                   0.025
#define NUMBER_OF_DOFS                          6

#define MAX_ACC 0.0375*40 // 0.0375 : acc in 25ms
#define DANGEROUS_ZONE 0.3
#define JOINTLIMIT_SPD_123 150
#define JOINTLIMIT_SPD_456 200

#define STOP 0
#define PASS 1
#define REACH 2

typedef float Scalar;
const float EPS = 1e-6;
const float LAMBDA_MAX = 0.3;
const float EPSQ = 1e-15;

namespace tm_otg{

	bool CheckJointLimit(double *q);

    bool CheckVelocityLimit(std::vector<double> qd, double &MaxScalingFactor);

    bool pinv_QR(const Eigen::MatrixXd &A, Eigen::MatrixXd &invA, Scalar eps);

    bool pinv(const Eigen::MatrixXd &A, Eigen::MatrixXd &invA, Scalar eps);
    
    bool pinv_SVD(const Eigen::MatrixXd &J, Eigen::MatrixXd &invJ);

    bool GetQfromInverseKinematics( std::vector<double> CartesianPosition, double *q_inv);

    bool GetQdfromInverseJacobian(std::vector<double> CurrentPosition,std::vector<double> EFF_Velocity, std::vector<double>& qd);

    bool GetQdfromLinearJacobian(std::vector<double> CurrentPosition,std::vector<double> EFF_Velocity, std::vector<double>& qd);

    bool GetQdfromCartesianConstrain(std::vector<double> CurrentPosition,
                                     std::vector<double> EFF_Velocity, 
                                     std::vector<double>& qd);

    bool OnlineCartesianConstrainGeneration(Eigen::Vector3d &TaskVelocity,
                                        std::vector<double> ConstrainPoint, 
                                        double *q);

    bool OnlineCartesianConstrainGeneration_wall(Eigen::Vector3d &TaskVelocity,
                                                 std::vector<double> ConstrainPoint, 
                                                 double *q);

    bool OnlineRepulsiveForceGeneration(std::vector<double>& PotentialField_qd,
                                        double *ObstaclePoint,  // obstacle position        : for min_distance
                                        double *TCP_position,   // robot TCP position       : for min_distance
                                        double *q,              // robot joint position     : for jacobian
                                        double dis_repuslive);   // eff to obstacle distance : for repulsive

    bool OnlineAttractiveForceGeneration(   std::vector<double>& action_qd,
                                        	double *TCP_position,             // robot TCP position       : for min_distance
                                        	double *q,                        // robot joint position     : for jacobian
                                        	std::vector<double> GoalPoint,    // eff to obstacle goal     : for att
                                        	std::vector<double> &EFF_Velocity); // eff velocity             : for att
}
#endif //TM_REFLEXXES_H
