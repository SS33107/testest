/*********************************************************************
 * tm_bringup_reflexxes.cpp
 *
 * Copyright (c) 2017, ISCI / National Chiao Tung University (NCTU)
 *
 * Author: Howard Chen (s880367@gmail.com)
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

/* Based on original source from Yun-Hsuan Tsai */

/*********************************************************************
 * tm_ros_wrapper.cpp
 *
 * Copyright 2016 Copyright 2016 Techman Robot Inc.
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
 *********************************************************************/

#include "tm_driver/tm_print.h"
#include "tm_driver/tm_driver.h"
#include "tm_driver/tm_communication.h"
#include "tm_driver/tm_robot_state_rt.h"
#include "tm_driver/tm_hardware_interface.h"

#include "tm_reflexxes/tm_reflexxes.h"
#include "tm_kinematics/tm_kin.h"

#include "tm_msgs/RobotStateMsgRT.h"
#include "tm_msgs/SetIO.h"

#include <eigen3/Eigen/Dense>

#include <stdio.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>

#ifdef USE_BOOST
  #include <boost/lexical_cast.hpp>
  #include <boost/thread/thread.hpp>
  #include <boost/thread/mutex.hpp>
  #include <boost/thread/condition_variable.hpp>
  #include <boost/chrono/chrono.hpp>
#else
  #include <thread>
  #include <mutex>
  #include <condition_variable>
  #include <chrono>
#endif

#define DEG2RAD 0.01745329252
#define RAD2DEG 57.29577951

#define DANGEROUS_ZONE 0.3
#define JOINTLIMIT_SPD_123 90
#define JOINTLIMIT_SPD_456 150
using namespace std;

double g_distance;
std::vector<double> g_obstacle_position(3);
std::vector<double> g_obstacle_velocity(3);

bool CheckJointLimit(double *q)
{
    bool valid = true;

    if(abs(q[0]) > 270*DEG2RAD)
    {
        print_warning("[WARN] the 1th joint : %lf\n",q[0] );
        valid = false;
    }
    else if(abs(q[1]) > 1.57)
    {
        print_warning("[WARN] the 2th joint : %lf\n",q[1] );
        valid = false;
    }
    else if(abs(q[2]) > 155*DEG2RAD)
    {
        print_warning("[WARN] the 3th joint : %lf\n",q[2] );
        valid = false;
    }
    else if(abs(q[3]) > 180*DEG2RAD)
    {
        print_warning("[WARN] the 4th joint : %lf\n",q[3] );
        valid = false;
    }
    else if(abs(q[4]) > 180*DEG2RAD)
    {
        print_warning("[WARN] the 5th joint : %lf\n",q[4] );
        valid = false;
    }
    else if(abs(q[5]) > 270*DEG2RAD)
    {
        print_warning("[WARN] the 6th joint : %lf\n",q[5] );
        valid = false;
    }
    else
        valid = true;

    return valid;
}

bool CheckVelocityLimit(std::vector<double> qd)
{
    bool valid = true;

    if(abs(qd[0]) > 90*DEG2RAD || abs(qd[1]) > 90*DEG2RAD || abs(qd[2]) > 90*DEG2RAD)
    {
        print_warning("[WARN] the 1th~3th joint : %10.4lf %10.4lf %10.4lf\n",qd[0],qd[1],qd[2] );
        valid = false;
    }
    else if(abs(qd[3]) > 150*DEG2RAD || abs(qd[4]) > 150*DEG2RAD || abs(qd[5]) > 150*DEG2RAD)
    {
        print_warning("[WARN] the 4th~6th joint : %10.4lf %10.4lf %10.4lf\n",qd[3],qd[4],qd[5] );
        valid = false;
    }
    else
        valid = true;

    return valid;
}

bool GetQfromInverseKinematics( std::vector<double> CartesianPosition, double *q_inv)
{
    Eigen::Matrix<float,4,4> T_;
    Eigen::AngleAxisf rollAngle (CartesianPosition[3], Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf yawAngle  (CartesianPosition[4], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf pitchAngle(CartesianPosition[5], Eigen::Vector3f::UnitX());
    Eigen::Quaternion<float> q = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix<float,3,3> RotationMatrix = q.matrix();
    double *T = new double[16];

    
    T_ <<   0., 0., 0., CartesianPosition[0],
            0., 0., 0., CartesianPosition[1],
            0., 0., 0., CartesianPosition[2],
            0., 0., 0., 1.;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            T_(i,j) = RotationMatrix(i,j);
        }
    }

    tm_jacobian::Matrix2DoubleArray(T_,T);
    cout << ">>>> T " << endl;
    tm_jacobian::printMatrix(T,4,16);

    int num_sol =  tm_kinematics::inverse(T, q_inv);

    delete [] T;
    return CheckJointLimit(q_inv);
}

bool GetQdfromInverseJacobian(std::vector<double> CurrentPosition,std::vector<double> EFF_Velocity, std::vector<double>& qd)
{

    Eigen::Matrix<float, 6, 1> home,q;
    home << 0, -PI*0.5, 0, PI*0.5, 0, 0;
    Eigen::Matrix<float,6,1> effspd,jointspd;

    home   << 0, -PI*0.5, 0, PI*0.5, 0, 0;
    effspd << EFF_Velocity[0], EFF_Velocity[1], EFF_Velocity[2], EFF_Velocity[3], EFF_Velocity[4], EFF_Velocity[5];
    q      << CurrentPosition[0], CurrentPosition[1], CurrentPosition[2], CurrentPosition[3], CurrentPosition[4], CurrentPosition[5];
    q += home;

    Eigen::Matrix<float, 6, 6> Inverse_Jacobian = tm_jacobian::Inverse_Jacobian(q);
    jointspd = Inverse_Jacobian*effspd;
    //cout << ">>>> Inverse jacobian" << endl;
    //tm_jacobian::printMatrix(Inverse_Jacobian);

    tm_jacobian::Matrix2DoubleVector(jointspd,qd);

    return CheckVelocityLimit(qd);
}

// x:TCP[3] 
// y:TCP[7] 
// z:TCP[11] 
//TODO: need obstacle velocity as input for dynamic obstacle
bool OnlinePotentialForceGeneration(vector<double>& PotentialField_qd,
                                    double *GoalPoint,      // goal position            : for attractive
                                    double *ObstaclePoint,  // obstacle position        : for min_distance
                                    double *TCP,            // robot TCP position       : for min_distance
                                    double *q,              // robot joint position     : for jacobian
                                    double dis_repuslive)   // eff to obstacle distance : for repulsive
{
    bool succeed = false;
    double Vmax_rep = 0.5;
    double Vmax_att = 0.025;
    double Survelliance_rep = 2/DANGEROUS_ZONE;//0.2;
    double Survelliance_att = 0.4;
    double ShapingFactor_rep = 8;
    double ShapingFactor_att = 5;

    Eigen::Vector3d Obstacle2Eff,Eff2Goal;
    Eff2Goal     <<     0,     GoalPoint[1]-TCP[7],     0;              // x = 0
    Obstacle2Eff << 0, TCP[7]-ObstaclePoint[1], TCP[11]-ObstaclePoint[2]; // x = 0
    //Eff2Goal     <<     GoalPoint[0]-TCP[3],     GoalPoint[1]-TCP[7],     GoalPoint[2]-TCP[11];
    //Obstacle2Eff << TCP[3]-ObstaclePoint[0], TCP[7]-ObstaclePoint[1], TCP[11]-ObstaclePoint[2];

    double dis_goal      = sqrt( pow(Eff2Goal(0),2)     + pow(Eff2Goal(1),2)     + pow(Eff2Goal(2),2)     );
    //double dis_repuslive = sqrt( pow(Obstacle2Eff(0),2) + pow(Obstacle2Eff(1),2) + pow(Obstacle2Eff(2),2) );

    Eigen::Vector3d AttractiveVecotor = Eff2Goal/dis_goal;
    Eigen::Vector3d RepulsiveVector   = Obstacle2Eff/dis_repuslive;

    double AttractiveForce = Vmax_att - Vmax_att*exp(-(dis_goal*ShapingFactor_att)/Survelliance_att); 
    double RepulsiveForce  = Vmax_rep / (1 + exp((dis_repuslive*Survelliance_rep-1)*ShapingFactor_rep));

    Eigen::Vector3d AttractiveVelocity = AttractiveForce*AttractiveVecotor; 
    Eigen::Vector3d RepulsiveVelocity  = RepulsiveForce *RepulsiveVector;
    Eigen::Vector3d PotentialVelocity  = AttractiveVelocity + RepulsiveVelocity;

    std::vector<double> CurrentPosition = {q[0], q[1], q[2], q[3], q[4], q[5]}; 
    std::vector<double> repulsive_qd(6);
    std::vector<double> EFF_Velocity = {PotentialVelocity(0), PotentialVelocity(1), PotentialVelocity(2), 0,0,0}; 
    succeed = tm_jacobian::GetQdfromInverseJacobian(CurrentPosition,EFF_Velocity,repulsive_qd);   
    PotentialField_qd = repulsive_qd;


    cout << "================================================"<<endl;
    cout << "[ INFO ] Eff2Goal     : " ;
    tm_jacobian::printVector(Eff2Goal) ;
    cout << "[ INFO ] Obstacle2Eff : " ;
    tm_jacobian::printVector(Obstacle2Eff) ;
    printf("[ INFO ] dis_goal      = [ %lf ] \n",dis_goal );
    printf("[ INFO ] dis_repuslive = [ %lf ] \n",dis_repuslive );
    cout << "[ INFO ] AttractiveVecotor : ";
    tm_jacobian::printVector(AttractiveVecotor);
    cout << "[ INFO ] RepulsiveVector   : ";
    tm_jacobian::printVector(RepulsiveVector);
    printf("[ INFO ] AttractiveForce = [ %lf]  \n",AttractiveForce );
    printf("[ INFO ] RepulsiveForce  = [ %lf]  \n",RepulsiveForce );
    cout << "[ INFO ] AttractiveVelocity :";
    tm_jacobian::printVector(AttractiveVelocity);
    cout << "[ INFO ] RepulsiveVelocity  :";
    tm_jacobian::printVector(RepulsiveVelocity);
    cout << "[ INFO ] PotentialVelocity  :";
    tm_jacobian::printVector(PotentialVelocity);
    cout << "[ INFO ] PotentialField_qd  :";
    tm_jacobian::printVector(repulsive_qd);
    cout << "================================================"<<endl;

    return succeed;

}

// x:TCP_position[3] 
// y:TCP_position[7] 
// z:TCP_position[11] 
// relative speed is obstacle-TCP
bool OnlineRepulsiveForceGeneration(vector<double>& PotentialField_qd,
                                    double *ObstaclePoint,  // obstacle position        : for min_distance
                                    double *TCP_position,   // robot TCP position       : for min_distance
                                    /*double *TCP_velocity,*/   // robot TCP velocity       : for repulsive vector
                                    double *q,              // robot joint position     : for jacobian
                                    double dis_repuslive)   // eff to obstacle distance : for repulsive
{
    bool succeed = false;
    double Vmax_rep = 1.0;
    double Survelliance_rep = 2/DANGEROUS_ZONE;//0.2;
    double ShapingFactor_rep = 8;

    Eigen::Vector3d Obstacle2Eff;
    Obstacle2Eff << TCP_position[3]-ObstaclePoint[0], TCP_position[7]-ObstaclePoint[1], TCP_position[11]-ObstaclePoint[2];
    Eigen::Vector3d RepulsiveVector   = Obstacle2Eff/dis_repuslive;


    //Eigen::Vector3d relative_speed, relative_speed_unitvector;
    //relative_speed << g_obstacle_velocity[0]-TCP_velocity[0], g_obstacle_velocity[1]-TCP_velocity[1], g_obstacle_velocity[2]-TCP_velocity[2];
    //double obstacle_speed_factor = sqrt( pow(relative_speed(0),2) + pow(relative_speed(1),2) + pow(relative_speed(2),2));
    //relative_speed_unitvector = relative_speed/obstacle_speed_factor;
   // Eigen::Vector3d RepulsiveVector = relative_speed_unitvector;

    double RepulsiveForce  = Vmax_rep / (1 + exp((dis_repuslive*Survelliance_rep-1)*ShapingFactor_rep));
    Eigen::Vector3d RepulsiveVelocity  = RepulsiveForce *RepulsiveVector;

    std::vector<double> CurrentPosition = {q[0], q[1], q[2], q[3], q[4], q[5]}; 
    std::vector<double> repulsive_qd(6);
    std::vector<double> EFF_Velocity = {RepulsiveVelocity(0), RepulsiveVelocity(1), RepulsiveVelocity(2), 0,0,0}; 
    succeed = tm_jacobian::GetQdfromInverseJacobian(CurrentPosition,EFF_Velocity,repulsive_qd);   
    PotentialField_qd = repulsive_qd;
/*

    cout << "================================================"<<endl;
    //cout << "[ INFO ] Obstacle2Eff : " ;
    //tm_jacobian::printVector(Obstacle2Eff) ;
    printf("[ INFO ] dis_repuslive = [ %lf ] \n",dis_repuslive );
    cout << "[ INFO ] RepulsiveVector   : ";
    tm_jacobian::printVector(RepulsiveVector);
    printf("[ INFO ] RepulsiveForce  = [ %lf]  \n",RepulsiveForce );
    cout << "[ INFO ] RepulsiveVelocity  :";
    tm_jacobian::printVector(RepulsiveVelocity);
    cout << "[ INFO ] PotentialField_qd  :";
    tm_jacobian::printVector(repulsive_qd);
    cout << "================================================"<<endl;
*/
    return succeed;

}

double DistanceUpdate(double *T)
{
    double distance;
    double obstacle_x =  0.426;
    double obstacle_y = -0.122;
    double obstacle_z =  0.473;

    double diff_x = T[3] - obstacle_x;
    double diff_y = T[7] - obstacle_y;
    double diff_z = T[11] - obstacle_z;

    distance = sqrt( pow(diff_x,2) + pow(diff_y,2) + pow(diff_z,2));
    return distance;
}

bool ReflexxesVelocityInterrupt(    TmDriver& TR,
                                    RMLVelocityInputParameters &InputState, 
                                    std::vector<double> TargetVelocity, 
                                    double SynTime)
{
    double time_s, blend = 0;
    std::vector <double> VelocityCommand(6);

    ReflexxesAPI *RML = NULL;
    RMLVelocityInputParameters  *IP = NULL;
    RMLVelocityOutputParameters *OP = NULL;
    RMLVelocityFlags Flags;
    int ResultValue = 0;
    bool pass = true;

    RML = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
    IP = new RMLVelocityInputParameters(NUMBER_OF_DOFS);
    OP = new RMLVelocityOutputParameters(NUMBER_OF_DOFS);
    *IP = InputState;

    double *q = new double [6];
    double *T = new double [16];
    double distance;


    // ********************************************************************/
    // Creating all relevant objects of the Type II Reflexxes Motion Library
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP->MaxJerkVector->VecData[i] = 100;
        IP->MaxAccelerationVector->VecData[i] = 0.5*40;
        IP->TargetVelocityVector->VecData[i] = TargetVelocity[i];

        if(IP->CurrentVelocityVector->VecData[i] != TargetVelocity[i])    
            IP->SelectionVector->VecData[i] = true;
        else
            IP->SelectionVector->VecData[i] = false;
    }
    IP->MinimumSynchronizationTime = SynTime;

    // ********************************************************************


    if (IP->CheckForValidity())
    {
        //printf("Input values are valid!\n");
    }
    else
    {
        printf("Input values are INVALID!\n");
    }
    Flags.SynchronizationBehavior = RMLFlags::ONLY_TIME_SYNCHRONIZATION;
    


    struct timeval tm1,tm2, tm3, tm4;
    double cycle_iteration = 1.0;

    gettimeofday(&tm3, NULL);

    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        //********************************************************
        // The area execution in 25ms real time sharp

        gettimeofday(&tm1, NULL); 

        ResultValue =  RML->RMLVelocity(*IP, OP, Flags );

        if (ResultValue < 0)
        {
            printf("An error occurred (%d).\n", ResultValue );
            break;
        }
        VelocityCommand = { OP->NewVelocityVector->VecData[0],
                            OP->NewVelocityVector->VecData[1],
                            OP->NewVelocityVector->VecData[2],
                            OP->NewVelocityVector->VecData[3],
                            OP->NewVelocityVector->VecData[4],
                            OP->NewVelocityVector->VecData[5]};

        TR.setMoveJointSpeedabs(VelocityCommand, blend);

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            q[i] = OP->NewPositionVector->VecData[i];

        tm_kinematics::forward(q,T);
        //distance = DistanceUpdate(T);
        distance = g_distance;

        //***************************************************************
        // Print out commands

        time_s = TR.interface->stateRT->getTime();
        //printf("[ %lf ] XYZ_pos:  ",time_s);
        ROS_INFO("XYZ_pos: %10.4lf %10.4lf %10.4lf  [ %lf ] ", T[3], T[7], T[11], distance);

        //printf(" | spd: ");
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            //printf("%10.4lf ", OP->NewVelocityVector->VecData[i]);
        //printf("\n");

        //***************************************************************
        *IP->CurrentPositionVector =  *OP->NewPositionVector;
        *IP->CurrentVelocityVector =  *OP->NewVelocityVector;
        *IP->CurrentAccelerationVector = *OP->NewAccelerationVector;

        gettimeofday(&tm2, NULL);
        long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);            
        usleep(24940 - time_compensation);  

        //********************************************************
        // The area execution in 25ms real time sharp
    }

    gettimeofday(&tm4, NULL);
    long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);
/*
    if(pass)
    {
        std::vector<double> FinalPosition(6);
        time_s = TR.interface->stateRT->getQAct(FinalPosition);
        printf("=============== Final state velocity based ReflexxesVelocityInterrupt =========================\n");
        printf("[ %lf ]  ", time_s);

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            printf(" %10.4lf ",FinalPosition[i]);
        printf("\n");
        print_info("Finished in %llu us", tt);
    }
*/
    InputState = *IP;
    delete  RML;
    delete  IP;
    delete  OP;

    return pass;
}

bool ReflexxesPositionSafetyRun(    TmDriver& TR,
                                    RMLPositionInputParameters &InputState, 
                                    std::vector<double> TargetPosition,
                                    std::vector<double> TargetVelocity, 
                                    double SynTime)
{
    double time_s, blend = 0;
    std::vector<double> FinalPosition(6), CurrentPosition(6), VelocityCommand(6);
    bool pass = true;
    bool AlreadyAccese = false;

    ReflexxesAPI *RML = NULL    ;
    RMLPositionInputParameters  *IP           = NULL;
    RMLPositionOutputParameters *OP           = NULL;
    RMLVelocityInputParameters  *IP_interrupt = NULL; 
    RML          = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
    IP           = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    OP           = new RMLPositionOutputParameters(NUMBER_OF_DOFS);
    IP_interrupt = new RMLVelocityInputParameters(NUMBER_OF_DOFS);
    RMLPositionFlags Flags;
    int ResultValue = 0;

    std::vector<double> PotentialField_qd(6), qd_now(6);
    double *GoalPoint     = new double [3];
    double *ObstaclePoint = new double [3];
    double *q             = new double [6];
    double *T             = new double [16];
    double distance;

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        q[i] = TargetPosition[i];
    }
    tm_kinematics::forward(q,T);
    GoalPoint[0] = T[3];
    GoalPoint[1] = T[7];
    GoalPoint[2] = T[11];

    //  ********************************************************************/
    //  Assigning all RMLPositionInputParameters : 
    //  Current POS, VEL, ACC : set before call ReflexxesPositionRun
    //  Target POS, VEL       : set before call ReflexxesPositionRun
    //  Max VEL, ACC          : set after call ReflexxesPositionRun
    //  SelectionVector       : set after call ReflexxesPositionRun
    //  ********************************************************************
    *IP = InputState;
    time_s = TR.interface->stateRT->getQAct(CurrentPosition);
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP->CurrentPositionVector->VecData[i] = CurrentPosition[i];   
        IP->MaxVelocityVector->VecData[i] = MAX_VELOCITY; //0.3247
        IP->MaxAccelerationVector->VecData[i] = MAX_ACC;
        IP->TargetPositionVector->VecData[i] = TargetPosition[i]; 
        IP->TargetVelocityVector->VecData[i] = TargetVelocity[i];
        IP->SelectionVector->VecData[i] = true;
    }
    IP->MinimumSynchronizationTime = SynTime;


    if (IP->CheckForValidity())
    {
        //printf("Input values are valid!\n");
    }
    else
        printf("Input values are INVALID!\n");


    struct timeval tm1,tm2, tm3, tm4;
    double cycle_iteration = 1.0;

    gettimeofday(&tm3, NULL);

    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        //********************************************************
        // The area execution in 25ms real time sharp

        gettimeofday(&tm1, NULL); 

        ResultValue =  RML->RMLPosition(*IP, OP, Flags );

        if (ResultValue < 0)
        {
            printf("An error occurred (%d).\n", ResultValue );
            break;
        }
        VelocityCommand = { OP->NewVelocityVector->VecData[0],
                            OP->NewVelocityVector->VecData[1],
                            OP->NewVelocityVector->VecData[2],
                            OP->NewVelocityVector->VecData[3],
                            OP->NewVelocityVector->VecData[4],
                            OP->NewVelocityVector->VecData[5]};

        TR.setMoveJointSpeedabs(VelocityCommand, blend);

            
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            q[i] = OP->NewPositionVector->VecData[i];

        tm_kinematics::forward(q,T);

        //***************************************************************
        // Print out commands

        time_s = TR.interface->stateRT->getTime();
        //printf("[ %lf ] XYZ_pos:  ",time_s);
        ROS_INFO("XYZ_pos: %10.4lf %10.4lf %10.4lf  [ %lf ] ", T[3], T[7], T[11], g_distance);
        //printf(" | spd: ");
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            //printf("%10.4lf ", OP->NewVelocityVector->VecData[i]);
            qd_now[i] = OP->NewVelocityVector->VecData[i];
        }
        //printf("\n");

        /*if(g_distance < DANGEROUS_ZONE)
        {
            for (int i = 0; i < 3; ++i)
                ObstaclePoint[i] = g_obstacle_position[i];
            OnlinePotentialForceGeneration(PotentialField_qd, GoalPoint, ObstaclePoint, T, q, g_distance);
        }*/
        

        //***************************************************************
        // Collision avoidance control activate
        AlreadyAccese = false;
        while (g_distance < DANGEROUS_ZONE)
        {
            for (int i = 0; i < 3; ++i)
                ObstaclePoint[i] = g_obstacle_position[i];

            print_info("Safety control Activate...");
            if(!AlreadyAccese)
            {
                *IP_interrupt->CurrentPositionVector     = *IP->CurrentPositionVector;
                *IP_interrupt->CurrentVelocityVector     = *IP->CurrentVelocityVector;
                *IP_interrupt->CurrentAccelerationVector = *IP->CurrentAccelerationVector;
                AlreadyAccese = true;
            }

            if(OnlineRepulsiveForceGeneration(PotentialField_qd,ObstaclePoint,T, q, g_distance))
            {
                for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                {
                    PotentialField_qd[i] += qd_now[i];
                }

                if(CheckVelocityLimit(PotentialField_qd))
                {
                    ReflexxesVelocityInterrupt(TR, *IP_interrupt,PotentialField_qd,0.2);
                    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                    {
                        q[i]      = IP_interrupt->CurrentPositionVector->VecData[i];
                        qd_now[i] = IP_interrupt->CurrentVelocityVector->VecData[i];
                    }
                    
                    tm_kinematics::forward(q,T);
                }
                else
                {
                    print_warning("Joint velocity out of range!! Smooth stop activate");
                    tm_reflexxes::ReflexxesSmoothStop(TR, *IP_interrupt, 0.1);
                    break;
                }
            }
            else
            {
                print_warning("Joint velocity out of range!! Smooth stop activate");
                tm_reflexxes::ReflexxesSmoothStop(TR, *IP_interrupt, 0.1);
                *IP->CurrentPositionVector     = *IP_interrupt->CurrentPositionVector;
                *IP->CurrentVelocityVector     = *IP_interrupt->CurrentVelocityVector;
                *IP->CurrentAccelerationVector = *IP_interrupt->CurrentAccelerationVector;
                pass = false;
                break;
            }

            if(g_distance > DANGEROUS_ZONE)
            {
                *IP->CurrentPositionVector     = *IP_interrupt->CurrentPositionVector;
                *IP->CurrentVelocityVector     = *IP_interrupt->CurrentVelocityVector;
                *IP->CurrentAccelerationVector = *IP_interrupt->CurrentAccelerationVector;
                TargetPosition = {0.6727, -0.0318,  1.6024,  -1.5706,  0.8981,   0 };
                TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
                ReflexxesPositionSafetyRun(TR, *IP, TargetPosition, TargetVelocity, 10.0);
                pass = false;
                break;
            }
        }
        if(AlreadyAccese)
            break;
        // Collision avoidance control activate
        //***************************************************************

        *IP->CurrentPositionVector =  *OP->NewPositionVector;
        *IP->CurrentVelocityVector =  *OP->NewVelocityVector;

        gettimeofday(&tm2, NULL);
        long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);            
        usleep(24940 - time_compensation);  

        //********************************************************
        // The area execution in 25ms real time sharp
    }

    gettimeofday(&tm4, NULL);
    long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

    if(pass)
    {
        printf("=============== Final state ReflexxesPositionSafetyRun =========================\n");
        printf("[ %lf ]  ", time_s);
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            printf("%10.4lf ", IP->CurrentVelocityVector->VecData[i]);

        printf("\n");
        print_info("Finished in %llu us", tt);
    }
    InputState = *IP;

    delete  RML;
    delete  IP;
    delete  OP;
    delete  IP_interrupt;

    delete [] T;
    delete [] q;
    delete [] ObstaclePoint;
    delete [] GoalPoint;

    return pass;
}

void ReflexxesStart(TmDriver& TM5)
{
    TM5.setJointSpdModeON();
    print_info("joint velocity control mode ON...");
    
    bool run_succeed = true;
    double SynchronousTime = 10.0;
    std::vector<double> TargetPosition, TargetVelocity, CurrentPosition;

    RMLPositionInputParameters  *IP_position = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    RMLVelocityInputParameters  *IP_velocity = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

    TM5.interface->stateRT->getQAct(CurrentPosition);

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_position->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_position->CurrentVelocityVector->VecData[i] = 0.0;
        IP_position->CurrentAccelerationVector->VecData[i] = 0.0;

        IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_velocity->CurrentVelocityVector->VecData[i] = 0.0;
        IP_velocity->CurrentAccelerationVector->VecData[i] = 0.0;
    }

    TargetPosition = {0.6727, -0.0318, 1.6024, -1.5706, 0.8981, 0};
    TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
    ReflexxesPositionSafetyRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);

/*
    while(run_succeed)
    {
        if (run_succeed)
        {
            printf(" -> 1 \n");
            TargetPosition = {-0.5531,  0.4107,  1.0725,  -1.4832,  2.1239,  -0.0000};
            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = tm_reflexxes::ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        }
        else
            break;

        if (run_succeed)
        {
            printf(" -> 2 \n");
            TargetPosition = {0.0006,  0.0516,  1.1879,  -1.2394,  1.5702,   0};
            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = tm_reflexxes::ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        }
        else
            break;

        if (run_succeed)
        {
            printf(" -> 3 \n");
            TargetPosition = {0.6727, -0.0318, 1.6024, -1.5706, 0.8981, 0};
            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = tm_reflexxes::ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        }
        else
            break;

        if (run_succeed)
        {
            printf(" -> 4 \n");
            TargetPosition = {0.0006, 0.0509, 1.8490, -1.8999, 1.5702, 0};
            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = tm_reflexxes::ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        }
        else
            break;
    }
*/
    delete IP_position;
    delete IP_velocity;
}

void StaticRobotAvoidnace(TmDriver& TM5)
{
    TM5.setJointSpdModeON();
    print_info("joint velocity control mode ON...");
    
    bool run_succeed = true;
    bool AlreadyAccese = false;
    bool Stand_by_state = false;
    double SynchronousTime = 5.0;
    std::vector<double> TargetPosition(6), TargetVelocity(6), CurrentPosition(6), CurrentVelocity(6);

    RMLPositionInputParameters  *IP_position  = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    RMLVelocityInputParameters  *IP_velocity  = new RMLVelocityInputParameters(NUMBER_OF_DOFS);
    RMLVelocityInputParameters  *IP_interrupt = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

    TM5.interface->stateRT->getQAct(CurrentPosition);

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_position->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_position->CurrentVelocityVector->VecData[i] = 0.0;
        IP_position->CurrentAccelerationVector->VecData[i] = 0.0;

        IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_velocity->CurrentVelocityVector->VecData[i] = 0.0;
        IP_velocity->CurrentAccelerationVector->VecData[i] = 0.0;
    }

    //TargetPosition = {0.0, 0.2349, 1.8352, -2.08465, 1.57133, 0.0};
    TargetPosition = {0.0, 0.0, 1.57, -1.57, 1.57, 0.0};
    TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
    tm_reflexxes::ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);
    ROS_INFO("robot stabd-by");
    Stand_by_state = true;


    std::vector<double> PotentialField_qd(6), qd_now(6), tool_velocity(6);
    double *ObstaclePoint = new double [3];
    double *q             = new double [6];
    double *T             = new double [16];
    double *TCP_velocity  = new double [6];

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        q[i] = TargetPosition[i];
    }
    tm_kinematics::forward(q,T);

    print_info("press enter to start");
    getchar();
    print_info("saftey mode start");

    *IP_interrupt->CurrentPositionVector     = *IP_position->CurrentPositionVector;
    *IP_interrupt->CurrentVelocityVector     = *IP_position->CurrentVelocityVector;
    *IP_interrupt->CurrentAccelerationVector = *IP_position->CurrentAccelerationVector;

    tm_reflexxes::initTermios(1);
    while(1)
    {
        if (g_distance < DANGEROUS_ZONE)
        {
            print_info("Safety control Activate...");
            Stand_by_state = false;

            //TM5.interface->stateRT->getTool0VelAct(tool_velocity);
            for (int i = 0; i < 3; ++i)
            {
                ObstaclePoint[i] = g_obstacle_position[i];
                //TCP_velocity[i]  = tool_velocity[i];
            }

            /*OnlineRepulsiveForceGeneration(PotentialField_qd,ObstaclePoint,T, q, g_distance);
            CheckVelocityLimit(PotentialField_qd);}*/

            if(OnlineRepulsiveForceGeneration(PotentialField_qd,ObstaclePoint,T, q, g_distance))
            {
                if(CheckVelocityLimit(PotentialField_qd))
                {
                    ReflexxesVelocityInterrupt(TM5, *IP_interrupt,PotentialField_qd,0.2);
                    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                    {
                        q[i] = IP_interrupt->CurrentPositionVector->VecData[i];
                    }
                    
                    tm_kinematics::forward(q,T);
                }
                else
                {
                    print_warning("Joint velocity out of range!! Smooth stop activate");
                    tm_reflexxes::ReflexxesSmoothStop(TM5, *IP_interrupt, 0.1);
                    Stand_by_state = true;
                    break;
                }
            }
            else
            {
                print_warning("Joint velocity out of range!! Smooth stop activate");
                tm_reflexxes::ReflexxesSmoothStop(TM5, *IP_interrupt, 0.1);
                Stand_by_state = true;
                break;
            }

            if(g_distance > DANGEROUS_ZONE)
            {
                *IP_position->CurrentPositionVector     = *IP_interrupt->CurrentPositionVector;
                *IP_position->CurrentVelocityVector     = *IP_interrupt->CurrentVelocityVector;
                *IP_position->CurrentAccelerationVector = *IP_interrupt->CurrentAccelerationVector;
                tm_reflexxes::ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, 2);

                *IP_interrupt->CurrentPositionVector     = *IP_position->CurrentPositionVector;
                *IP_interrupt->CurrentVelocityVector     = *IP_position->CurrentVelocityVector;
                *IP_interrupt->CurrentAccelerationVector = *IP_position->CurrentAccelerationVector;
                
                Stand_by_state = true;
                ROS_INFO("robot stabd-by inside");
            }
        }
        else if(!Stand_by_state)
        {
            tm_reflexxes::ReflexxesSmoothStop(TM5, *IP_interrupt, 0.1);
            *IP_position->CurrentPositionVector     = *IP_interrupt->CurrentPositionVector;
            *IP_position->CurrentVelocityVector     = *IP_interrupt->CurrentVelocityVector;
            *IP_position->CurrentAccelerationVector = *IP_interrupt->CurrentAccelerationVector;
            tm_reflexxes::ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, 2);

            *IP_interrupt->CurrentPositionVector     = *IP_position->CurrentPositionVector;
            *IP_interrupt->CurrentVelocityVector     = *IP_position->CurrentVelocityVector;
            *IP_interrupt->CurrentAccelerationVector = *IP_position->CurrentAccelerationVector;

            Stand_by_state = true;
            ROS_INFO("robot stabd-by outside");
        }

        if (tm_reflexxes::kbhit())
        {
            char c = getchar();
            if (c == 'q' || c == 'Q')
            {
                break;
            }
        }
    }

    tm_reflexxes::resetTermios();
    delete IP_position;
    delete IP_velocity;
    delete IP_interrupt;
    delete [] ObstaclePoint;
    delete [] q;
    delete [] T;
    delete [] TCP_velocity;
}


std::vector<double> parse_cmd(char* cstr, const char* delim, double& res)
{
    std::vector<double> ret;
    char* pch;
    char* pch_save;
    pch = strtok_r(cstr, delim, &pch_save);
    if (pch != NULL)
    {
        while ((pch = strtok_r(NULL, delim, &pch_save)) != NULL)
        {
            //count++;
            if (ret.size() < 6)
            {
                ret.push_back(atof(pch));
            }
            else
            {
                res = atof(pch);
                break;
            }
        }
    }

    return ret; 
}

void distance_callback(const std_msgs::Float32::ConstPtr& distance)
{
    g_distance = distance->data;
    //ROS_INFO("minimum_distance = %10.3f",distance->data);
}

void position_callback(const geometry_msgs::PointStamped::ConstPtr& closest_pt)
{
    g_obstacle_position[0] = closest_pt->point.x;
    g_obstacle_position[1] = closest_pt->point.y;
    g_obstacle_position[2] = closest_pt->point.z;
    //ROS_INFO("x =  %10.3f, y =  %10.3f, z =  %10.3f", closest_pt->point.x, closest_pt->point.y, closest_pt->point.z);
}

void velocity_callback(const geometry_msgs::Twist::ConstPtr& velocity)
{
    g_obstacle_velocity[0] = velocity->linear.x;
    g_obstacle_velocity[1] = velocity->linear.y;
    g_obstacle_velocity[2] = velocity->linear.z;
    //ROS_INFO("vx = %10.3f, vy = %10.3f, vz = %10.3f", velocity->linear.x, velocity->linear.y, velocity->linear.z);
}


int main(int argc, char **argv)
{
    std::string host;

    #ifdef USE_BOOST
        boost::condition_variable data_cv;
        boost::condition_variable data_cv_rt;
    #else
        std::condition_variable data_cv;
        std::condition_variable data_cv_rt;
    #endif

    ros::init(argc, argv, "tm_action");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(3);

    ros::Subscriber distance_sub = node_handle.subscribe("/kinect_merge/minimum_distance",    1,distance_callback);
    ros::Subscriber velocity_sub = node_handle.subscribe("/kinect_merge/closest_vel_tracking",1,velocity_callback);
    ros::Subscriber position_sub = node_handle.subscribe("/kinect_merge/closest_pt_tracking", 1,position_callback);

    spinner.start();

    if (!(ros::param::get("~robot_ip", host)))
    {
        if (argc > 1)
            host = argv[1];
        else
            exit(1);
    }

    const int STDIN = 0;
    int sockfd = -1;
    bool fgRun = false;

    for (int i = 0; i < argc; i++)
    {
        printf("[DEBUG] arg%d:= %s\n", i, argv[i]);
    }
    host = argv[1];
    printf("[ INFO] host: %s\n", host.c_str());


    TmDriver TmRobot(data_cv, data_cv_rt, host, 0);

    char cstr[512];
    char delim[] = " ,;\t";
    char c;
    while (ros::ok())
    {
        memset(cstr, 0, 512);
        fgets(cstr, 512, stdin);
        int n = (int)strlen(cstr);
        if (n > 0)
        {
            if (cstr[n - 1] == '\n')
                cstr[n - 1] = '\0';
        }
        if (strncmp(cstr, "quit", 4) == 0)
        {
            TmRobot.interface->halt();
            fgRun = false;
            print_info("quit");
            boost::this_thread::sleep_for(boost::chrono::milliseconds(25));
            break;
        }
        else if (strncmp(cstr, "start", 5) == 0)
        {
            if (!fgRun)
            {
                print_info("start...");
                fgRun = TmRobot.interface->start();
            }
        }
        else if (strncmp(cstr, "halt", 4) == 0)
        {
            print_info("halt");
            TmRobot.interface->halt();
            fgRun = false;
        }
        else if(strncmp(cstr, "home", 4) == 0)
        {
            double blend = 0;
            std::vector<double> vec1 = {0,0,0,0,0,0};
            TmRobot.setMoveJabs(vec1, blend);
            print_info("Back to home");
        }
        else if(strncmp(cstr, "ready", 5) == 0)
        {
            double blend = 0;
            std::vector<double> vec1 = {0,0,1.5705,-1.5705,1.5705,0};
            TmRobot.setMoveJabs(vec1, blend);
            print_info("Back to ready position");
        }
        else if(strncmp(cstr, "1", 1) == 0)
        {
            double blend = 0;
            std::vector<double> vec1 = {-0.5531,  0.4107,  1.0725,  -1.4832,  2.1239,  0.0000};
            TmRobot.setMoveJabs(vec1, blend);
            print_info("Back to position 1");
        }
        else if (strncmp(cstr, "clear", 5) == 0)
        {
            system("clear");
        }        
        else if (strncmp(cstr, "gotest", 6) == 0)
        {
            ReflexxesStart(TmRobot);
            //StaticRobotAvoidnace(TmRobot);

            TmRobot.setJointSpdModeoOFF();
            print_info("joint vlocity control mode OFF...");
        }
        else if (strncmp(cstr, "spdmodeoff", 10) == 0)
        {
            TmRobot.setJointSpdModeoOFF();
            print_info("joint vlocity control mode OFF...");
        }
        else if (strncmp(cstr, "movjabs", 7) == 0)
        {
            double blend = 0;
            std::vector<double> vec = parse_cmd(cstr, delim, blend);
            TmRobot.setMoveJabs(vec, blend);
        }
        else
        {
            print_info("send cmd...");
            std::string msg(cstr);
            TmRobot.setCommandMsg(msg);
        }
    }
    //ros::waitForShutdown();

    printf("[ info] TM_ROS: shutdown\n");

    return 0;
}
