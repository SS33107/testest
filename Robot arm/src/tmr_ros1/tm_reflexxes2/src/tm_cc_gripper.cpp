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

#include "tm_driver/tm_print.h"
#include "tm_driver/tm_driver.h"
#include "tm_driver/tm_communication.h"
#include "tm_driver/tm_robot_state_rt.h"
#include "tm_driver/tm_hardware_interface.h"

#include "tm_reflexxes/tm_reflexxes.h"
#include "tm_kinematics/tm_kin.h"
#include "tm_reflexxes/tm_otg.h"

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

#include "tm_driver/tm_driver.h"
#include "tm_reflexxes/tm_reflexxes.h"
#include "tm_kinematics/tm_kin.h"

#include "ReflexxesAPI.h"
#include "RMLPositionFlags.h"
#include "RMLPositionInputParameters.h"
#include "RMLPositionOutputParameters.h"

#include "RMLVelocityFlags.h"
#include "RMLVelocityInputParameters.h"
#include "RMLVelocityOutputParameters.h"

#define DEG2RAD 0.01745329252
#define RAD2DEG 57.29577951
#define REPEATABILITY 0.01//0.00005

#define DANGEROUS_ZONE 0.3

#define VMAX_ATT_GPR 0.05
#define VMAX_CC_GPR  0.03
#define OBSTACLE_GPR true

#define STOP 0
#define PASS 1
#define REACH 2

using namespace std;

double last_point_x = 0.0;
double last_point_y = 0.0;
double last_point_z = 0.0;

bool   gParam_vision = true;

double g_distance;
std::vector<double> g_obstacle_position(3);
std::vector<double> g_obstacle_velocity(3);

bool ReflexxesPositionRun(  TmDriver& TR,
                            RMLPositionInputParameters &InputState, 
                            std::vector<double> TargetPosition,
                            std::vector<double> TargetVelocity, 
                            double SynTime);

void ReflexxesSmoothStop(   TmDriver& TR,
                            RMLVelocityInputParameters &InputState, 
                            double SynTime)
{
    double blend = 0, time_s;
    std::vector<double> vec;

    ReflexxesAPI *RML = NULL;
    RMLVelocityInputParameters  *IP = NULL;
    RMLVelocityOutputParameters *OP = NULL;
    RMLVelocityFlags Flags;
    int ResultValue = 0;
    bool pass = true;
    struct timeval tm1,tm2, tm3, tm4;

    RML = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
    IP = new RMLVelocityInputParameters(NUMBER_OF_DOFS);
    OP = new RMLVelocityOutputParameters(NUMBER_OF_DOFS);
    *IP = InputState;


    // ********************************************************************/
    // Creating all relevant objects of the Type II Reflexxes Motion Library*/

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP->MaxJerkVector->VecData[i] = 100; //RMLTypeII not using, needed for validity
        IP->MaxAccelerationVector->VecData[i] = 0.5*40;
        IP->TargetVelocityVector->VecData[i] = 0.0;
        if(IP->CurrentVelocityVector->VecData[i] != 0.0)    
            IP->SelectionVector->VecData[i] = true;
        else
            IP->SelectionVector->VecData[i] = false;

    }
    IP->MinimumSynchronizationTime = SynTime;

    // ********************************************************************


    if (IP->CheckForValidity())
        printf("Input values are valid!\n");
    else
        printf("Input values are INVALID!\n");

    Flags.SynchronizationBehavior = RMLFlags::ONLY_TIME_SYNCHRONIZATION;

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
        vec = { OP->NewVelocityVector->VecData[0],
                OP->NewVelocityVector->VecData[1],
                OP->NewVelocityVector->VecData[2],
                OP->NewVelocityVector->VecData[3],
                OP->NewVelocityVector->VecData[4],
                OP->NewVelocityVector->VecData[5]};

        TR.setMoveJointSpeedabs(vec, blend);

        //**********************
        // Print out commands

        time_s = TR.interface->stateRT->getTime();
        printf("[ %lf ] pos:  ",time_s );

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            printf("%10.4lf ", OP->NewPositionVector->VecData[i]);

        printf(" | spd: ");

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            printf("%10.4lf ", OP->NewVelocityVector->VecData[i]);
        
        printf("\n");

        //**********************

        *IP->CurrentPositionVector      =   *OP->NewPositionVector      ;
        *IP->CurrentVelocityVector      =   *OP->NewVelocityVector      ;
        *IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;

        gettimeofday(&tm2, NULL);
        long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);            
        usleep(24940 - time_compensation);  

        // The area execution in 25ms real time sharp
        //********************************************************
    }

    gettimeofday(&tm4, NULL);
    long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

    std::vector<double> FinalPosition;
    time_s = TR.interface->stateRT->getQAct(FinalPosition);
    printf("=============== Final state of Smooth Stop =========================\n");
    printf("[ %lf ]  ", time_s);

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        printf(" %10.4lf ",FinalPosition[i]);
    printf("\n");
    print_info("Smooth stop finish in %llu us", tt);
    InputState = *IP;

    delete  RML;
    delete  IP;
    delete  OP;
}


//  ********************************************************************/
//  fn        : ReflexxesVelocityRun()  -- For p2p motion
//  brief     : Use RML API to execute given velocity in simulation.  
//  param[in] : &InputState, Current State of robot under RML.
//  param[in] : TargetVelocity, The velocity when reach target position.
//  param[in] : SynTime, The time for execute the trajectory.
//  param[out]: 1:pass, 2:reach goal position, 0:smooth stop
//  ********************************************************************
int ReflexxesVelocityRun(   TmDriver& TR,
                            RMLVelocityInputParameters &InputState, 
                            std::vector<double> TargetVelocity, 
                            std::vector<double> TargetPosition,
                            std::vector<double> EffVelocity,
                            double SynTime)
{
    ReflexxesAPI *RML = NULL;
    RMLVelocityInputParameters  *IP = NULL;
    RMLVelocityOutputParameters *OP = NULL;
    RMLVelocityFlags Flags;
    int ResultValue = 0;
    int pass = PASS;
    double DistanceToGoal[3];
    std::vector<double> VelocityCommand(6);
    double blend = 0;
    
    double *T = new double [16];
    double *q = new double [6];

    tm_reflexxes::initTermios(1);

    RML = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
    IP  = new RMLVelocityInputParameters(NUMBER_OF_DOFS);
    OP  = new RMLVelocityOutputParameters(NUMBER_OF_DOFS);
    *IP = InputState;


    // ********************************************************************/
    // Creating all relevant objects of the Type II Reflexxes Motion Library
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP->MaxJerkVector->VecData[i]         = 100; //RMLTypeII not using, needed for validity
        IP->MaxAccelerationVector->VecData[i] = 0.5*40;
        IP->TargetVelocityVector->VecData[i]  = TargetVelocity[i];

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

        //***************************************************************
        // Print out commands

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            q[i] = OP->NewPositionVector->VecData[i];
        
        tm_jacobian::Forward_Kinematics_gripper(q,T,0.235);
        //ROS_INFO("tool0 XYZ_pos: %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);

        //***************************************************************

        for (int i = 0; i < 3; ++i)
        {
            if(abs(EffVelocity[i]) == 0)
                DistanceToGoal[i] = 0;
            else
                DistanceToGoal[i] = abs(T[i*4+3] -TargetPosition[i]);
        }

        if( (DistanceToGoal[0] < REPEATABILITY) && (DistanceToGoal[1] < REPEATABILITY) && (DistanceToGoal[2] < REPEATABILITY))
        {
            pass = REACH;
            break;
        }

        if(!tm_otg::CheckJointLimit(q))
        {
            pass = STOP;
            break; 
        }

        if (tm_reflexxes::kbhit())
        {
            char c = getchar();
            if (c == 'q' || c == 'Q')
            {
                ROS_WARN("Smooth Stop Activate...");
                tm_reflexxes::ReflexxesSmoothStop(TR, *IP, 0.25);
                pass = STOP;
                break;
            }
        }

        *IP->CurrentPositionVector     =  *OP->NewPositionVector;
        *IP->CurrentVelocityVector     =  *OP->NewVelocityVector;
        *IP->CurrentAccelerationVector =  *OP->NewAccelerationVector;

        gettimeofday(&tm2, NULL);
        long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);            
        usleep(24940 - time_compensation);  

        //********************************************************
        // The area execution in 25ms real time sharp
    }

    gettimeofday(&tm4, NULL);
    long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

    ROS_INFO("========== Final state velocity based position =============");
    ROS_INFO("pass = %d  tool0 : %10.4lf %10.4lf %10.4lf", pass, T[3], T[7], T[11]);
    ROS_INFO("Joint position  : %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", q[0]*RAD2DEG,q[1]*RAD2DEG,q[2]*RAD2DEG,q[3]*RAD2DEG,q[4]*RAD2DEG,q[5]*RAD2DEG);
    ROS_INFO("Finished in %llu us", tt);

    tm_reflexxes::resetTermios();

    InputState = *IP;
    
    delete  RML;
    delete  IP;
    delete  OP;
    delete [] T;
    delete [] q;

    return pass;
}

bool ReflexxesPositionRun(  TmDriver& TR,
                            RMLPositionInputParameters &InputState, 
                            std::vector<double> TargetPosition,
                            std::vector<double> TargetVelocity, 
                            double SynTime)
{
    double time_s;
    std::vector<double> FinalPosition;
    bool pass = true;
    std::vector<double> VelocityCommand(6);
    double blend = 0;

    ReflexxesAPI *RML = NULL    ;
    RMLPositionInputParameters  *IP = NULL;
    RMLPositionOutputParameters *OP = NULL;
    RMLPositionFlags Flags;
    int ResultValue = 0;

    double *T = new double [16];
    double *q = new double [6];

    RML = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
    IP  = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    OP  = new RMLPositionOutputParameters(NUMBER_OF_DOFS);

    *IP = InputState;

    tm_reflexxes::initTermios(1);

    //  ********************************************************************/
    //  Assigning all RMLPositionInputParameters : 
    //  Current POS, VEL, ACC : set before call ReflexxesPositionRun
    //  Target POS, VEL       : set before call ReflexxesPositionRun
    //  Max VEL, ACC          : set after call ReflexxesPositionRun
    //  SelectionVector       : set after call ReflexxesPositionRun
    //  ********************************************************************
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP->MaxJerkVector->VecData[i]         = 100;           //RMLTypeII not using, needed for validity
        IP->MaxVelocityVector->VecData[i]     = 3.14; //0.3247
        IP->MaxAccelerationVector->VecData[i] = 3.14;
        IP->TargetPositionVector->VecData[i]  = TargetPosition[i]; 
        IP->TargetVelocityVector->VecData[i]  = TargetVelocity[i];
        IP->SelectionVector->VecData[i]       = true;
    }
    IP->MinimumSynchronizationTime = SynTime;


    if (IP->CheckForValidity())
        printf("Input values are valid!\n");
    else
    {
        printf("Input values are INVALID!\n");
        pass = false;
    }


    struct timeval tm1,tm2, tm3, tm4;
    double cycle_iteration = 1.0;


    gettimeofday(&tm3, NULL);

    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED && ros::ok())
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

        //***************************************************************
        // Print out commands

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            q[i] = OP->NewPositionVector->VecData[i];

        //tm_jacobian::Forward_Kinematics_3(q,T);
        tm_jacobian::Forward_Kinematics_gripper(q,T,0.235);
        ROS_INFO("gripper XYZ_pos: %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);

        //***************************************************************

        if (tm_reflexxes::kbhit())
        {
            char c = getchar();
            if (c == 'q' || c == 'Q')
            {
                ROS_WARN("Smooth Stop Activate...");
                tm_reflexxes::ReflexxesSmoothStop(TR, IP, 0.25);
                pass = false;
                break;
            }
        }

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

    ROS_INFO("========== Final state position based position =============");
    ROS_INFO("XYZ_pos : %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);
    print_info("Finished in %llu us", tt);

    tm_reflexxes::resetTermios();
    
    if(pass)
        InputState = *IP;

    delete  RML;
    delete  IP;
    delete  OP;
    delete [] T;
    delete [] q;

    return pass;
}

//cout << "A least-squares solution of pinv(J)*x = qd is:" << endl << svd.solve(TaskVelocity) << endl;
bool pinv_SVD(const Eigen::MatrixXd &J, Eigen::MatrixXd &invJ)//, Eigen::Matrix<double,3,1> TaskVelocity)
{
    Eigen::VectorXd sigma;  //vector of singular values
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);

    sigma = svd.singularValues();
    int m = sigma.rows();

    for (int i = 0; i < m ; ++i)
    {
        if(sigma(i) > EPS)
            sigma(i) = 1.0/ sigma(i);
        else
            sigma(i) = 0.0;
    }

    invJ = svd.matrixV() * sigma.asDiagonal() * svd.matrixU().transpose();

    return true;
}

bool OnlineVFCGeneration(   Eigen::Vector3d &TaskVelocity,
                            std::vector<double> ConstrainPoint, 
                            double *q) 
{
    double Vmax_cc = 1;
    double Survelliance_cc = 2/0.27;  //dangerous zone = 0.27m
    double ShapingFactor_cc = 8;
    double *T3 = new double [16];
    double ConstrainedPlane_z  = 0.0;    //table
    double dis_constrain = 0;
    double CartesianInfluence;
    std::vector<double> ConstrainedPoint = { 0.3277,    -0.0951,     0.1648}; //object 0.359, 0.1115, 0.1646


    if(gParam_vision)
        ConstrainedPoint = g_obstacle_position;
    else
        ConstrainedPoint = { 0.3277,    -0.0951,     0.1648};

    tm_jacobian::Forward_Kinematics_3(q,T3,-0.15); //-0.15
    Eigen::Vector3d Constrain2TaskJoint;

    Constrain2TaskJoint <<  T3[3] -ConstrainedPoint[0],
                            T3[7] -ConstrainedPoint[1],
                            T3[11]-ConstrainedPoint[2];    // for z plane : table
    
    double Distance2ConstrainedPoint = sqrt(Constrain2TaskJoint.dot(Constrain2TaskJoint));

    if(Distance2ConstrainedPoint > T3[11]) //table
    {
        dis_constrain = T3[11];
        CartesianInfluence  = Vmax_cc / (1 + exp((dis_constrain*Survelliance_cc-1)*ShapingFactor_cc));
        TaskVelocity << 0., 0., CartesianInfluence;
        ROS_WARN("Constrain : table");
    }
    else 
    {
        Survelliance_cc = 2/0.37;
        dis_constrain = Distance2ConstrainedPoint;
        CartesianInfluence  = Vmax_cc / (1 + exp((dis_constrain*Survelliance_cc-1)*ShapingFactor_cc));
        TaskVelocity = CartesianInfluence*(Constrain2TaskJoint/dis_constrain);
        ROS_WARN("Constrain : point");
    }

    ROS_INFO("Distance table     : %10.4lf",T3[11]);
    ROS_INFO("Distance point     : %10.4lf",Distance2ConstrainedPoint);
    ROS_INFO("joint 3 position   : %10.4lf %10.4lf %10.4lf [%10.4lf]",T3[3],T3[7],T3[11], CartesianInfluence);
    ROS_INFO("VirtualForce       : %10.4lf %10.4lf %10.4lf ",TaskVelocity(0),TaskVelocity(1),TaskVelocity(2));

    delete [] T3;
    return true;
}

bool GetQdfromVirtualForceConstrain(std::vector<double> CurrentPosition,
                                    std::vector<double> EFF_Velocity, 
                                    std::vector<double>& qd)
{
    Eigen::Matrix<float , 6, 1> home,q;
    Eigen::Matrix<double , 6, 1> JointSpeed;
    Eigen::Matrix<double, 3, 1> EFFSpeed;

    home     <<                  0,            -PI*0.5,                  0,             PI*0.5,                  0,                  0;
    q        << CurrentPosition[0], CurrentPosition[1], CurrentPosition[2], CurrentPosition[3], CurrentPosition[4], CurrentPosition[5];
    EFFSpeed <<    EFF_Velocity[0],    EFF_Velocity[1],    EFF_Velocity[2];
    q += home;

    Eigen::Matrix<double, 6, 6> Geometry_Jacobian = tm_jacobian::Forward_Jacobian_gripper(q);// Forward_Jacobian_d(q);
    Eigen::Matrix<double, 3, 6> Jacobian_123456   = Geometry_Jacobian.block<3,6>(0,0);

    Eigen::Vector3d VirtualForce;
    Eigen::Vector3d Constrained_Velocity;

    Eigen::Vector2d qd12_original;
    std::vector<double> ConstrainedPoint(3);
    double *q_now = new double [6];
    
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        q_now[i] = CurrentPosition[i];

    Eigen::MatrixXd jacobian_123456_inv;
    pinv_SVD(Jacobian_123456,jacobian_123456_inv);
    JointSpeed = jacobian_123456_inv * EFFSpeed;
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        qd[i] = JointSpeed(i);

    qd12_original(0) = qd[0];
    qd12_original(1) = qd[1];

    if(OnlineVFCGeneration(VirtualForce,ConstrainedPoint,q_now) && OBSTACLE_GPR) //constrain generated
    {
        Eigen::Matrix<double, 3, 4> Jacobian_3456     = Jacobian_123456.block<3,4>(0,2);
        Eigen::Matrix<double, 3, 2> Jacobian_12       = Jacobian_123456.block<3,2>(0,0);
        Eigen::Matrix<double, 3, 2> TaskJacobian_3    = tm_jacobian::Forward_Linear_Jacobian_3(q);
        Eigen::Vector3d OriginalForce = TaskJacobian_3* qd12_original;

        Eigen::MatrixXd TaskJacobian_3_inv;
        if(!pinv_SVD(TaskJacobian_3, TaskJacobian_3_inv))
            return false;

        Eigen::Vector3d temp;
        temp << abs(OriginalForce(0))*VirtualForce(0),
                abs(OriginalForce(1))*VirtualForce(1),
                abs(OriginalForce(2))*VirtualForce(2);

        for (int i = 0; i < 3; ++i)
        {
            if(temp(i)*OriginalForce(i) < 0)
                Constrained_Velocity(i) = OriginalForce(i) + temp(i);
            else
                Constrained_Velocity(i) = OriginalForce(i);
        }
        //Eigen::Vector3d Constrained_Velocity = OriginalForce + temp;

        ROS_INFO("OriginalForce      : %10.4lf %10.4lf %10.4lf ",OriginalForce(0),OriginalForce(1),OriginalForce(2));
        ROS_INFO("Constrained_Velocty: %10.4lf %10.4lf %10.4lf ",Constrained_Velocity(0),Constrained_Velocity(1),Constrained_Velocity(2));

        Eigen::VectorXd qd_12 = TaskJacobian_3_inv * Constrained_Velocity;
        Eigen::Matrix<double, 3, 1> linear_velocity_temp = EFFSpeed - Jacobian_12*qd_12;
        Eigen::MatrixXd jacobian_3456_inv;
        if(!pinv_SVD(Jacobian_3456,jacobian_3456_inv))
            return false;
        Eigen::VectorXd qd_3456 = jacobian_3456_inv*linear_velocity_temp;

        qd[0] = qd_12(0);
        qd[1] = qd_12(1);
        qd[2] = qd_3456(0);
        qd[3] = qd_3456(1);
        qd[4] = qd_3456(2);
        qd[5] = qd_3456(3);
    
        ROS_WARN("qd_12   :     %10.4lf %10.4lf",qd_12(0)*RAD2DEG,qd_12(1)*RAD2DEG);
        ROS_WARN("qd_3456 :     %10.4lf %10.4lf %10.4lf %10.4lf",qd_3456(0)*RAD2DEG,qd_3456(1)*RAD2DEG,qd_3456(2)*RAD2DEG,qd_3456(3)*RAD2DEG);
    }
    
    ROS_INFO("Generatd qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);

    double ScalingFactor;
    if(tm_otg::CheckVelocityLimit(qd,ScalingFactor))
        return true;
    else
    {
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            qd[i] = qd[i]/ScalingFactor;
        }
        ROS_WARN("Scaled qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);
        return tm_otg::CheckVelocityLimit(qd,ScalingFactor);
    }
}

bool OnlineCartesianConstrainGeneration(Eigen::Vector3d &TaskVelocity,
                                        std::vector<double> ConstrainPoint, 
                                        double *q) 
{
    double Vmax_cc = 0.1;
    double Survelliance_cc = 2/0.45;  //dangerous zone = 0.2m
    double ShapingFactor_cc = 8;
    double *T3 = new double [16];

    tm_jacobian::Forward_Kinematics_3(q,T3,-0.15);

    Eigen::Vector3d Constrain2TaskJoint_vector, TaskJoint_vector, Normal_vector, OrthogonalAvoid_vector;
    Constrain2TaskJoint_vector << ConstrainPoint[0]-T3[3], ConstrainPoint[1]-T3[7], ConstrainPoint[2]-T3[11];
    TaskJoint_vector    << T3[3]-last_point_x, T3[7]-last_point_y, T3[11]-last_point_z;
    
    double dis_constrain = sqrt(pow(Constrain2TaskJoint_vector(0),2) + pow(Constrain2TaskJoint_vector(1),2) + pow(Constrain2TaskJoint_vector(2),2));
    double CartesianInfluence  = Vmax_cc / (1 + exp((dis_constrain*Survelliance_cc-1)*ShapingFactor_cc));
    ROS_INFO("Constrain distance : %10.4lf",dis_constrain);
    ROS_INFO("CartesianInfluence : %10.4lf ",CartesianInfluence);

    if(sqrt(TaskJoint_vector.dot(TaskJoint_vector)) < EPS)
    {
        TaskVelocity << 0.0, 0.0, 0.0;
    }
    else
    {
        Normal_vector = Constrain2TaskJoint_vector.cross(TaskJoint_vector);
        OrthogonalAvoid_vector = Constrain2TaskJoint_vector.cross(Normal_vector);
        double unit =  sqrt(pow(OrthogonalAvoid_vector(0),2)+pow(OrthogonalAvoid_vector(1),2)+pow(OrthogonalAvoid_vector(2),2));
        OrthogonalAvoid_vector = OrthogonalAvoid_vector/unit;
        TaskVelocity = CartesianInfluence*OrthogonalAvoid_vector;

        if(TaskVelocity.dot(TaskJoint_vector) < 0)
            TaskVelocity = -TaskVelocity;
        ROS_INFO("Task vector        : %10.4lf %10.4lf %10.4lf",OrthogonalAvoid_vector(0),OrthogonalAvoid_vector(1),OrthogonalAvoid_vector(2));
    }

    //ROS_INFO("TaskVelocity       : %10.4lf %10.4lf %10.4lf",TaskVelocity(0),TaskVelocity(1),TaskVelocity(2));
    ROS_WARN("joint 3 position   : %10.4lf %10.4lf %10.4lf",T3[3],T3[7],T3[11]);
    last_point_x = T3[3];
    last_point_y = T3[7];
    last_point_z = T3[11];

    delete [] T3;

    if(abs(CartesianInfluence) < 0.08)//Vmax_cc-0.01)
        return 0;
    else
        return 1;
}

bool OnlineCartesianConstrainGeneration_wall(Eigen::Vector3d &TaskVelocity,
                                             std::vector<double> ConstrainPoint, 
                                             double *q) 
{
    double Vmax_cc = VMAX_CC_GPR;
    double Survelliance_cc = 2/0.27;  //dangerous zone = 0.2m
    double ShapingFactor_cc = 8;
    double *T3 = new double [16];

    tm_jacobian::Forward_Kinematics_3(q,T3,-0.15);
    Eigen::Vector3d Constrain2TaskJoint;

    //Constrain2TaskJoint << T3[3]-ConstrainPoint[0], 0, 0; // for x plane
    Constrain2TaskJoint << 0,0,T3[11]-ConstrainPoint[2];    // for z plane
    
    double dis_constrain = sqrt(Constrain2TaskJoint.dot(Constrain2TaskJoint));
    double CartesianInfluence  = Vmax_cc / (1 + exp((dis_constrain*Survelliance_cc-1)*ShapingFactor_cc));

    if((T3[7] - last_point_y) >= 0)
        TaskVelocity << 0., CartesianInfluence, 0.;
    else
        TaskVelocity << 0., -CartesianInfluence, 0.;

    ROS_INFO("CartesianInfluence : %10.4lf ",CartesianInfluence);
    ROS_WARN("joint 3 position   : %10.4lf %10.4lf %10.4lf",T3[3],T3[7],T3[11]);

    last_point_y = T3[7];
    delete [] T3;

    if(abs(CartesianInfluence) < 0.001)//Vmax_cc-0.01)
        return 0;
    else
        return 1;
}


bool GetQdfromCartesianConstrain(std::vector<double> CurrentPosition,
                                 std::vector<double> EFF_Velocity, 
                                 std::vector<double>& qd)
{
    Eigen::Matrix<float , 6, 1> home,q;
    Eigen::Matrix<double , 6, 1> JointSpeed;
    Eigen::Matrix<double, 3, 1> EFFSpeed;

    home     <<                  0,            -PI*0.5,                  0,             PI*0.5,                  0,                  0;
    q        << CurrentPosition[0], CurrentPosition[1], CurrentPosition[2], CurrentPosition[3], CurrentPosition[4], CurrentPosition[5];
    EFFSpeed <<    EFF_Velocity[0],    EFF_Velocity[1],    EFF_Velocity[2];
    q += home;

    Eigen::Matrix<double, 6, 6> Geometry_Jacobian = tm_jacobian::Forward_Jacobian_gripper(q);// Forward_Jacobian_d(q);
    Eigen::Matrix<double, 3, 6> Jacobian_123456   = Geometry_Jacobian.block<3,6>(0,0);

    Eigen::Vector3d TaskVelocity_3;
    std::vector<double> ConstrainedPoint(3);
    double *q_now = new double [6];
    
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        q_now[i] = CurrentPosition[i];

    if(!OBSTACLE_GPR)
        ConstrainedPoint = {-10.3, 0.1, -10.0};
    else
        ConstrainedPoint = {-0.3, 0.1, 0.0};


    Eigen::MatrixXd jacobian_123456_inv;
    pinv_SVD(Jacobian_123456,jacobian_123456_inv);
    JointSpeed = jacobian_123456_inv * EFFSpeed;
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        qd[i] = JointSpeed(i);

    if(OnlineCartesianConstrainGeneration_wall(TaskVelocity_3,ConstrainedPoint,q_now) && OBSTACLE_GPR) //constrain generated
    {
        Eigen::Matrix<double, 3, 4> Jacobian_3456     = Jacobian_123456.block<3,4>(0,2);
        Eigen::Matrix<double, 3, 2> Jacobian_12       = Jacobian_123456.block<3,2>(0,0);
        Eigen::Matrix<double, 3, 2> TaskJacobian_3    = tm_jacobian::Forward_Linear_Jacobian_3(q);

        double EFFScalar  = sqrt(EFFSpeed.dot(EFFSpeed));
        double TaskScalar = sqrt(TaskVelocity_3.dot(TaskVelocity_3));
        if(EFFScalar < 0.01)
        {
            if(TaskScalar > EPS)
                TaskVelocity_3 = TaskVelocity_3*(EFFScalar/TaskScalar);
        }
        
        Eigen::MatrixXd TaskJacobian_3_inv;
        if(!pinv_SVD(TaskJacobian_3, TaskJacobian_3_inv))
            return false;

        Eigen::VectorXd qd_12 = TaskJacobian_3_inv * TaskVelocity_3;
        Eigen::Matrix<double, 3, 1> linear_velocity_temp = EFFSpeed - Jacobian_12*qd_12;
        Eigen::MatrixXd jacobian_3456_inv;
        if(!pinv_SVD(Jacobian_3456,jacobian_3456_inv))
            return false;
        Eigen::VectorXd qd_3456 = jacobian_3456_inv*linear_velocity_temp;

        qd[0] += qd_12(0);
        qd[1] += qd_12(1);
        qd[2] += qd_3456(0);
        qd[3] += qd_3456(1);
        qd[4] += qd_3456(2);
        qd[5] += qd_3456(3);
    
        ROS_WARN("qd_12   :     %10.4lf %10.4lf",qd_12(0)*RAD2DEG,qd_12(1)*RAD2DEG);
        ROS_WARN("qd_3456 :     %10.4lf %10.4lf %10.4lf %10.4lf",qd_3456(0)*RAD2DEG,qd_3456(1)*RAD2DEG,qd_3456(2)*RAD2DEG,qd_3456(3)*RAD2DEG);
    }
    
    ROS_INFO("Generatd qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);

    double ScalingFactor;
    if(tm_otg::CheckVelocityLimit(qd,ScalingFactor))
        return true;
    else
    {
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            qd[i] = qd[i]/ScalingFactor;
        }
        ROS_WARN("Scaled qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);
        return tm_otg::CheckVelocityLimit(qd,ScalingFactor);
    }
}

bool GetQdfromLinearJacobian(std::vector<double> CurrentPosition,std::vector<double> EFF_Velocity, std::vector<double>& qd)
{
    Eigen::Matrix<float , 6, 1> home,q;
    Eigen::Matrix<double, 3, 1> EFFSpeed;
    Eigen::Matrix<double, 6, 1> JointSpeed, TaskVecolity;

    home     <<                  0,            -PI*0.5,                  0,             PI*0.5,                  0,                  0;
    q        << CurrentPosition[0], CurrentPosition[1], CurrentPosition[2], CurrentPosition[3], CurrentPosition[4], CurrentPosition[5];
    EFFSpeed <<    EFF_Velocity[0],    EFF_Velocity[1],    EFF_Velocity[2];
    q += home;

    Eigen::Matrix<double, 6, 6> Geometry_Jacobian = tm_jacobian::Forward_Jacobian_d(q);
    Eigen::Matrix<double, 3, 6> Jacobian_123456   = Geometry_Jacobian.block<3,6>(0,0);

    //for (int i = 0; i < 4; ++i)
    //    Jacobian_123456.row(i) = Geometry_Jacobian.row(i);
    //Jacobian_123456.row(4) = Geometry_Jacobian.row(5);
    
    Eigen::MatrixXd jacobian_123456_inv;
    pinv_SVD(Jacobian_123456,jacobian_123456_inv);
    JointSpeed = jacobian_123456_inv * EFFSpeed;

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        qd[i] = JointSpeed(i);

    double ScalingFactor;

    //double *T3 = new double [16];
    //tm_jacobian::Forward_Kinematics_3(q,T3);
    //ROS_INFO("joint 3 position   : %10.4lf %10.4lf %10.4lf",T3[3],T3[7],T3[11]);

    return tm_otg::CheckVelocityLimit(qd,ScalingFactor);
}

bool OnlineAttractiveForceGeneration(   std::vector<double>& action_qd,
                                        double *TCP_position,             // robot TCP position       : for min_distance
                                        double *q,                        // robot joint position     : for jacobian
                                        std::vector<double> GoalPoint,    // eff to obstacle goal     : for att
                                        std::vector<double> &EFF_Velocity) // eff velocity             : for att
{
    bool   succeed = false;
    double Vmax_att = VMAX_ATT_GPR;
    double Survelliance_att = 0.15;
    double ShapingFactor_att = 5;

    Eigen::Vector3d Eff2Goal;
    Eff2Goal << GoalPoint[0]-TCP_position[3], GoalPoint[1]-TCP_position[7], GoalPoint[2]-TCP_position[11];
    double dis_goal = sqrt( pow(Eff2Goal(0),2) + pow(Eff2Goal(1),2) + pow(Eff2Goal(2),2)     );

    Eigen::Vector3d AttractiveVecotor = Eff2Goal/dis_goal;

    double AttractiveForce = Vmax_att - Vmax_att*exp(-(dis_goal*ShapingFactor_att)/Survelliance_att); 

    Eigen::Vector3d AttractiveVelocity = AttractiveForce*AttractiveVecotor; 

    std::vector<double> CurrentPosition = {q[0], q[1], q[2], q[3], q[4], q[5]}; 
    EFF_Velocity = {AttractiveVelocity(0), AttractiveVelocity(1), AttractiveVelocity(2), 0,0,0}; 
    double AttractiveScalar = sqrt(AttractiveVelocity.dot(AttractiveVelocity));
    ROS_INFO("Attractive Velocity: %10.4lf ", AttractiveScalar);
    //succeed = tm_jacobian::GetQdfromInverseJacobian(CurrentPosition,EFF_Velocity,action_qd);
    //succeed = GetQdfromLinearJacobian(CurrentPosition,EFF_Velocity,action_qd);  
    //succeed = GetQdfromCartesianConstrain(CurrentPosition,EFF_Velocity,action_qd);
    succeed = GetQdfromVirtualForceConstrain(CurrentPosition,EFF_Velocity,action_qd);

    return succeed;
}


bool U_shape_Demo(TmDriver& TM5)
{
    TM5.setJointSpdModeON();
    print_info("joint velocity control mode ON...");

    bool run_succeed = true;
    int  pass = 0;
    double SynchronousTime = 5;
    std::vector<double> TargetPosition, TargetVelocity, CurrentPosition(6), qd(6);
    double *T = new double [16];
    double *q = new double [6];

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

    std::vector<double> InitialPosition = {-9.2368*DEG2RAD, 0.7082*DEG2RAD, 83.6619*DEG2RAD, -130.3923*DEG2RAD, 61.4890*DEG2RAD, 0};
    TargetVelocity = {0, 0, 0, 0, 0, 0};
    if(!ReflexxesPositionRun(TM5, *IP_position, InitialPosition, TargetVelocity, SynchronousTime))
    {
        ROS_WARN("Smooth stop activate...");
        return 0;
    }

    TM5.interface->stateRT->getQAct(CurrentPosition);
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_velocity->CurrentPositionVector->VecData[i]     = CurrentPosition[i];
        IP_velocity->CurrentVelocityVector->VecData[i]     = IP_position->CurrentVelocityVector->VecData[i];
        IP_velocity->CurrentAccelerationVector->VecData[i] = IP_position->CurrentAccelerationVector->VecData[i];
    }

    //Test for joint 3
    int state = 1;
    double back_x  = -0.2772;
    double right_y = -0.2516;
    double height_z = 0.8;
    while(1)
    {
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            q[i] = IP_velocity->CurrentPositionVector->VecData[i];
        }

        if(state == 1)      //s1
        {
            TargetPosition = {back_x, right_y, height_z, 0, 0, 0};
        }
        else if(state == 2) //s2
        {
            TargetPosition = {back_x, 0.1778, height_z, 0, 0, 0};
        }
        else if(state == 3) //s3
        {
            TargetPosition = {0.4087, 0.1778, height_z, 0, 0, 0};
        }
        else if(state == 4) //s2
        {
            TargetPosition = {back_x, 0.1778, height_z, 0, 0, 0};
        }
        else if(state == 5) //s1
        {
            TargetPosition = {back_x, right_y, height_z, 0, 0, 0};
        }
        else if(state == 6) //start
        {
            TargetPosition = {0.4087, -0.3592, height_z, 0, 0, 0};//0.5321
        }
        else{}
        
        if(state != 7)  //point-to-point motion
        {
            //tm_kinematics::forward(q,T);
            tm_jacobian::Forward_Kinematics_gripper(q,T,0.235);

            if(OnlineAttractiveForceGeneration(qd, T, q, TargetPosition, TargetVelocity))
            {
                pass = ReflexxesVelocityRun(TM5,*IP_velocity, qd, TargetPosition, TargetVelocity ,0.2);

                if(pass == REACH)
                    state++;
                else if(pass == STOP)
                    break;
            }
            else
            {
                ROS_WARN("Attrative force generate fail, smooth stop activate...");
                tm_reflexxes::ReflexxesSmoothStop(TM5,*IP_velocity,0.25);
                break;
            }
        }
        else    // initialization
        {
            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            {
                IP_position->CurrentPositionVector->VecData[i]     = IP_velocity->CurrentPositionVector->VecData[i];
                IP_position->CurrentVelocityVector->VecData[i]     = IP_velocity->CurrentVelocityVector->VecData[i];
                IP_position->CurrentAccelerationVector->VecData[i] = IP_velocity->CurrentAccelerationVector->VecData[i];
            }
            ReflexxesPositionRun(TM5,*IP_position, InitialPosition, TargetVelocity, 1);
            
            TM5.interface->stateRT->getQAct(CurrentPosition);
            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            {
                IP_velocity->CurrentPositionVector->VecData[i]     = CurrentPosition[i];
                IP_velocity->CurrentVelocityVector->VecData[i]     = IP_position->CurrentVelocityVector->VecData[i];
                IP_velocity->CurrentAccelerationVector->VecData[i] = IP_position->CurrentAccelerationVector->VecData[i];
            }
            state = 1;
        }        
    }

    ROS_WARN("U_shape_Demo shutdown");
    delete IP_position;
    delete IP_velocity;
    delete [] T;
    delete [] q;

    return 0;
}

bool Hexagon_Demo(TmDriver& TM5)
{
    TM5.setJointSpdModeON();
    print_info("joint velocity control mode ON...");

    bool run_succeed = true;
    int  pass = 0;
    double SynchronousTime = 5;
    std::vector<double> TargetPosition, TargetVelocity, CurrentPosition(6), qd(6);
    double *T = new double [16];
    double *q = new double [6];

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

    std::vector<double> InitialPosition = {0.1169,     1.1010,    -0.8864,    -0.8015,     1.6174,     0.0000};
    TargetVelocity = {0, 0, 0, 0, 0, 0};
    if(!ReflexxesPositionRun(TM5, *IP_position, InitialPosition, TargetVelocity, SynchronousTime))
    {
        ROS_WARN("Smooth stop activate...");
        return 0;
    }

    TM5.interface->stateRT->getQAct(CurrentPosition);
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_velocity->CurrentPositionVector->VecData[i]     = CurrentPosition[i];
        IP_velocity->CurrentVelocityVector->VecData[i]     = IP_position->CurrentVelocityVector->VecData[i];
        IP_velocity->CurrentAccelerationVector->VecData[i] = IP_position->CurrentAccelerationVector->VecData[i];
    }

    //Test for joint 3
    int state = 1;
    double x_plane  = 0.6;

    double left_y  = -0.3360;
    double mid_y    = -0.036;
    double right_y   = 0.264;

    double upp_z     = 0.88;
    double up_z     = 0.755;

    double down_z   = 0.505;
    double downn_z   = 0.38;
    while(1)
    {
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            q[i] = IP_velocity->CurrentPositionVector->VecData[i];
        }

        if(state == 1)      //s1
        {
            TargetPosition = {x_plane, mid_y, upp_z, 0, 0, 0};
        }
        else if(state == 2) //s2
        {
            TargetPosition = {x_plane, right_y, up_z, 0, 0, 0};
        }
        else if(state == 3) //s3
        {
            TargetPosition = {x_plane, right_y, down_z, 0, 0, 0};
        }
        else if(state == 4) //s4
        {
            TargetPosition = {x_plane, mid_y, downn_z, 0, 0, 0};
        }
        else if(state == 5) //s2
        {
            TargetPosition = {x_plane, left_y, down_z, 0, 0, 0};
        }
        else if(state == 6) //s2
        {
            TargetPosition = {x_plane, left_y, up_z, 0, 0, 0};
        }
        else if(state == 7) //s2
        {
            TargetPosition = {x_plane, mid_y, upp_z, 0, 0, 0};
        }
        else{}
        
        if(state != 8)  //point-to-point motion
        {
            //tm_kinematics::forward(q,T);
            tm_jacobian::Forward_Kinematics_gripper(q,T,0.235);

            if(OnlineAttractiveForceGeneration(qd, T, q, TargetPosition, TargetVelocity))
            {
                pass = ReflexxesVelocityRun(TM5,*IP_velocity, qd, TargetPosition, TargetVelocity ,0.2);

                if(pass == REACH)
                    state++;
                else if(pass == STOP)
                    break;
            }
            else
            {
                ROS_WARN("Attrative force generate fail, smooth stop activate...");
                tm_reflexxes::ReflexxesSmoothStop(TM5,*IP_velocity,0.25);
                break;
            }
        }
        else    // initialization
        {
            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            {
                IP_position->CurrentPositionVector->VecData[i]     = IP_velocity->CurrentPositionVector->VecData[i];
                IP_position->CurrentVelocityVector->VecData[i]     = IP_velocity->CurrentVelocityVector->VecData[i];
                IP_position->CurrentAccelerationVector->VecData[i] = IP_velocity->CurrentAccelerationVector->VecData[i];
            }
            ReflexxesPositionRun(TM5,*IP_position, InitialPosition, TargetVelocity, 1);
            
            TM5.interface->stateRT->getQAct(CurrentPosition);
            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            {
                IP_velocity->CurrentPositionVector->VecData[i]     = CurrentPosition[i];
                IP_velocity->CurrentVelocityVector->VecData[i]     = IP_position->CurrentVelocityVector->VecData[i];
                IP_velocity->CurrentAccelerationVector->VecData[i] = IP_position->CurrentAccelerationVector->VecData[i];
            }
            state = 1;
        }        
    }

    ROS_WARN("Hexagon_Demo shutdown");
    delete IP_position;
    delete IP_velocity;
    delete [] T;
    delete [] q;

    return 0;
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

    ros::init(argc, argv, "tm_cc");
    ros::NodeHandle node_handle;

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
        else if(strncmp(cstr, "gopen", 6) == 0)
        {
            unsigned char ch = 0;

            TmRobot.setDigitalOutputEE(ch,false);
        }
        else if(strncmp(cstr, "gclose", 7) == 0)
        {
            unsigned char ch = 0;

            TmRobot.setDigitalOutputEE(ch,true);
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
            //bool pass = U_shape_Demo(TmRobot);
            bool pass = Hexagon_Demo(TmRobot);
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

    ROS_WARN("TM_ROS: shutdown\n");

    return 0;
}
