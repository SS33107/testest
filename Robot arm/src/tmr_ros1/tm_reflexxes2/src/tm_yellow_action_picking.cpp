/*********************************************************************
 * tm_action_picking.cpp
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

#include "tm_msgs/RobotStateMsgRT.h"
#include "tm_msgs/SetIO.h"

#include <eigen3/Eigen/Dense>

#include <stdio.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

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
#define REPEATABILITY 0.01//0.00005

#define GRIPPER_LENGTH 0.240  //0.235

#define VMAX_ATT_GPR 0.08
#define VMAX_REP     0.2
#define VMAX_CC_GPR  0.03

#define OBSTACLE_GPR false
#define V_TRAVEL 0.05

#define DANGEROUS_ZONE 0.3
#define JOINTLIMIT_SPD_123 150
#define JOINTLIMIT_SPD_456 200

#define STOP 0
#define PASS 1
#define REACH 2

#define IMAGE_CAPTURE_POSE 1
#define GRASP_READY_POSE   2
#define OBJECT_POSITION    3
#define GRASP_AFTER_POSE   4
#define PLACING_POSE_ABOVE 5
#define PLACING_POSE       6
#define MID_POSE           7

typedef float Scalar;
const float EPS = 1e-6;
const float LAMBDA_MAX = 0.3;
const float EPSQ = 1e-15;

double last_point_x = 0.0;
double last_point_y = 0.0;
double last_point_z = 0.0;

int g_objectclass;
double g_distance;
std::vector<double> g_obstacle_position(3);
std::vector<double> g_obstacle_velocity(3);
std::vector<double> g_constraint_position(3);

double g_objposition[7];
bool position_fill = false;
bool rotation_fill = false;

FILE *f_traj;
FILE *f_obstacle;

ros::Publisher RobotState;


using namespace std;


bool ReflexxesPositionRun(  TmDriver& TR,
                            RMLPositionInputParameters &InputState, 
                            std::vector<double> TargetPosition,
                            std::vector<double> TargetVelocity, 
                            double SynTime);


bool CheckJointLimit(double *q)
{
    bool valid = true;

    if(abs(q[0]) > 265*DEG2RAD)
    {
        ROS_WARN("[Position] 1st joint position out of limit (270) : %lf",q[0]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[1]) > 175*DEG2RAD)
    {
        ROS_WARN("[Position] 2nd joint position out of limit (180): %lf",q[1]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[2]) > 148*DEG2RAD)
    {
        ROS_WARN("[Position] 3rd joint position out of limit (155): %lf",q[2]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[3]) > 175*DEG2RAD)
    {
        ROS_WARN("[Position] 4th joint position out of limit (180): %lf",q[3]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[4]) > 175*DEG2RAD)
    {
        ROS_WARN("[Position] 5th joint position out of limit (180): %lf",q[4]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[5]) > 265*DEG2RAD)
    {
        ROS_WARN("[Position] 6th joint position out of limit (180): %lf",q[5]*RAD2DEG );
        valid = false;
    }
    else
        valid = true;

    return valid;
}

bool CheckVelocityLimit(std::vector<double> qd, double &MaxScalingFactor)
{
    bool valid = true;
    double ExceedRatio[6] = {0,0,0,0,0,0};
    double MaxExceed = 1;
    short  ExceedJoint;

    for (int i = 0; i < 3; ++i)
    {
        if (abs(qd[i])/JOINTLIMIT_SPD_123 > 1.0)
        {
            ExceedRatio[i] = abs(qd[i])/JOINTLIMIT_SPD_123;
            ROS_WARN("[Velocity] %dth joint velocity exceed limit(150): %10.4lf",i+1,qd[i]*RAD2DEG);
            valid = false;
        }
        else
            ExceedRatio[i] = 0.0;
    }

    for (int i = 3; i < 6; ++i)
    {
        if (abs(qd[i])/JOINTLIMIT_SPD_456 > 1.0)
        {
            ExceedRatio[i] = abs(qd[i])/JOINTLIMIT_SPD_456;
            ROS_WARN("[Velocity] %dth joint velocity exceed limit(200): %10.4lf",i+1,qd[i]*RAD2DEG);
            valid = false;
        }
        else
            ExceedRatio[i] = 0.0;
    }

    if(!valid)
    {
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            if(ExceedRatio[i] > MaxExceed)
            {
                MaxExceed = ExceedRatio[i];
                ExceedJoint = i;
            }
        }
        MaxScalingFactor = MaxExceed;
        ROS_WARN("[Velocity] THE MOST EXCEED JOINT IS %dth joint, NEED SCALING %10.4lf",ExceedJoint+1,MaxScalingFactor);
    }

    return valid;
}

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
        
        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);
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

        if(!CheckJointLimit(q))
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

//    ROS_INFO("========== Final state velocity based position =============");
//    ROS_INFO("pass = %d  tool0 : %10.4lf %10.4lf %10.4lf", pass, T[3], T[7], T[11]);
//    ROS_INFO("Joint position  : %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", q[0]*RAD2DEG,q[1]*RAD2DEG,q[2]*RAD2DEG,q[3]*RAD2DEG,q[4]*RAD2DEG,q[5]*RAD2DEG);
//    ROS_INFO("Finished in %llu us", tt);

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
        IP->MaxAccelerationVector->VecData[i] = 0.0375*40;
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

    fprintf(f_traj, "===ReflexxesPositionRun===\n");

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
        tm_jacobian::Forward_Kinematics_gripper(q,T,0); //(q, T, GRIPPER_LENGTH)
        //ROS_INFO("gripper XYZ_pos: %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);
        ROS_INFO("gripper_XYZ: %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);
        fprintf(f_traj, "gripper_XYZ: %10.4lf %10.4lf %10.4lf\n", T[3], T[7], T[11]);

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

    fprintf(f_traj, "========== Final state ReflexxesPositionRun =============\n");
    fprintf(f_traj, "gripper_XYZ: %10.4lf %10.4lf %10.4lf\n", T[3], T[7], T[11]);

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

bool pinv_SVD(const Eigen::MatrixXd &J, Eigen::MatrixXd &invJ)
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
    std::vector<double> ConstrainedPoint(3); //object 0.359, 0.1115, 0.1646

    ConstrainedPoint = g_constraint_position;//g_obstacle_position;

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
//        ROS_WARN("Constrain : table");
    }
    else 
    {
        Survelliance_cc = 2/0.37;
        dis_constrain = Distance2ConstrainedPoint;
        CartesianInfluence  = Vmax_cc / (1 + exp((dis_constrain*Survelliance_cc-1)*ShapingFactor_cc));
        TaskVelocity = CartesianInfluence*(Constrain2TaskJoint/dis_constrain);
//        ROS_WARN("Constrain : point");
    }

//    ROS_INFO("Distance table     : %10.4lf",T3[11]);
//    ROS_INFO("Distance point     : %10.4lf",Distance2ConstrainedPoint);
//    ROS_INFO("joint 3 position   : %10.4lf %10.4lf %10.4lf [%10.4lf]",T3[3],T3[7],T3[11], CartesianInfluence);
//    ROS_INFO("VirtualForce       : %10.4lf %10.4lf %10.4lf ",TaskVelocity(0),TaskVelocity(1),TaskVelocity(2));

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

    if(OnlineVFCGeneration(VirtualForce,ConstrainedPoint,q_now)) //constrain generated
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

//        ROS_INFO("OriginalForce      : %10.4lf %10.4lf %10.4lf ",OriginalForce(0),OriginalForce(1),OriginalForce(2));
//        ROS_INFO("Constrained_Velocty: %10.4lf %10.4lf %10.4lf ",Constrained_Velocity(0),Constrained_Velocity(1),Constrained_Velocity(2));

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
    
//        ROS_WARN("qd_12   :     %10.4lf %10.4lf",qd_12(0)*RAD2DEG,qd_12(1)*RAD2DEG);
//        ROS_WARN("qd_3456 :     %10.4lf %10.4lf %10.4lf %10.4lf",qd_3456(0)*RAD2DEG,qd_3456(1)*RAD2DEG,qd_3456(2)*RAD2DEG,qd_3456(3)*RAD2DEG);
    }
    
//    ROS_INFO("Generatd qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);

    double ScalingFactor;
    return CheckVelocityLimit(qd,ScalingFactor);

    /*if(CheckVelocityLimit(qd,ScalingFactor))
        return true;
    else
    {
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            qd[i] = qd[i]/ScalingFactor;
        }
        ROS_WARN("Scaled qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);
        return CheckVelocityLimit(qd,ScalingFactor);
    }*/
}

bool GetQdfromLinearJacobian(   std::vector<double> CurrentPosition,
                                std::vector<double> EFF_Velocity, 
                                std::vector<double>& qd)
{
    Eigen::Matrix<float , 6, 1> home,q;
    Eigen::Matrix<double , 6,1> JointSpeed;
    Eigen::Matrix<double, 3, 1> EFFSpeed;

    home     <<                  0,            -PI*0.5,                  0,             PI*0.5,                  0,                  0;
    q        << CurrentPosition[0], CurrentPosition[1], CurrentPosition[2], CurrentPosition[3], CurrentPosition[4], CurrentPosition[5];
    EFFSpeed <<    EFF_Velocity[0],    EFF_Velocity[1],    EFF_Velocity[2];
    q += home;

    Eigen::Matrix<double, 6, 6> Geometry_Jacobian = tm_jacobian::Forward_Jacobian_gripper(q, GRIPPER_LENGTH);// Forward_Jacobian_d(q);
    Eigen::Matrix<double, 3, 6> Jacobian_123456   = Geometry_Jacobian.block<3,6>(0,0);

    Eigen::MatrixXd jacobian_123456_inv;
    pinv_SVD(Jacobian_123456,jacobian_123456_inv);
    JointSpeed = jacobian_123456_inv * EFFSpeed;
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        qd[i] = JointSpeed(i);

    //ROS_INFO("Generatd qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);

    double ScalingFactor;
    return CheckVelocityLimit(qd,ScalingFactor);
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
//    ROS_INFO("Attractive Velocity: %10.4lf ", AttractiveScalar);
    //succeed = tm_jacobian::GetQdfromInverseJacobian(CurrentPosition,EFF_Velocity,action_qd);
    //succeed = GetQdfromLinearJacobian(CurrentPosition,EFF_Velocity,action_qd);  
    //succeed = GetQdfromCartesianConstrain(CurrentPosition,EFF_Velocity,action_qd);
    succeed = GetQdfromVirtualForceConstrain(CurrentPosition,EFF_Velocity,action_qd);

    return succeed;
}

bool OnlineRepulsiveForceGeneration(    std::vector<double>& Repulsive_qd,
                                        double *TCP_position,                   // robot TCP position       : for min_distance
                                        double *q,                              // robot joint position     : for jacobian
                                        std::vector<double> &EFF_Velocity)      // eff velocity             : for att
{
    bool succeed = false;
    double Vmax_rep = VMAX_REP;
    double Survelliance_rep = 2/DANGEROUS_ZONE;//0.2;
    double ShapingFactor_rep = 8;
    std::vector<double> ObstaclePoint(3); 
    
    ObstaclePoint = g_obstacle_position;

    Eigen::Vector3d Obstacle2Eff;
    Obstacle2Eff << TCP_position[3]-ObstaclePoint[0], TCP_position[7]-ObstaclePoint[1], TCP_position[11]-ObstaclePoint[2];
    double dis_repuslive = sqrt(Obstacle2Eff.dot(Obstacle2Eff));
    Eigen::Vector3d RepulsiveVector   = Obstacle2Eff/dis_repuslive;

    double RepulsiveForce  = Vmax_rep / (1 + exp((dis_repuslive*Survelliance_rep-1)*ShapingFactor_rep));
    Eigen::Vector3d RepulsiveVelocity  = RepulsiveForce *RepulsiveVector;

    std::vector<double> CurrentPosition = {q[0], q[1], q[2], q[3], q[4], q[5]}; 
    EFF_Velocity = {RepulsiveVelocity(0), RepulsiveVelocity(1), RepulsiveVelocity(2), 0,0,0}; 
    //OS_INFO("ObstaclePoint      : %10.4lf %10.4lf %10.4lf",ObstaclePoint[0],ObstaclePoint[1],ObstaclePoint[2]);
//    ROS_WARN("Repulsive  Velocity: %10.4lf ", sqrt(RepulsiveVelocity.dot(RepulsiveVelocity)));
    
    //succeed = tm_jacobian::GetQdfromInverseJacobian(CurrentPosition,EFF_Velocity,Repulsive_qd);   
    //succeed = GetQdfromLinearJacobian(CurrentPosition,EFF_Velocity,Repulsive_qd);   
    succeed = GetQdfromVirtualForceConstrain(CurrentPosition,EFF_Velocity,Repulsive_qd);   

    return succeed;

}

bool BinPicking_Demo(TmDriver& TM5)
{
    TM5.setJointSpdModeON();
    print_info("joint velocity control mode ON...");

    bool run_succeed = true;
    int  pass = 0;
    double SynchronousTime = 7;
    std::vector<double> TargetPosition(6), TargetVelocity, CurrentPosition(6), qd(6), Attractive_qd(6), Repulsive_qd(6), RepulsiveVelocity(6);
    double *T = new double [16];
    double *q = new double [6];
    std_msgs::Int32 robot_motion_states;

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
    std::vector<double> RecoverPosition = {0.1169,     1.1010,    -0.8864,    -0.8015,     1.6174,     0.0000};
   
   
    /* 2018.10.04 updated by YuHsien
     * START POSITION without Container (lower) 
     * ImageCapture {0.0691,    -0.6283,     1.9241,     0.2848,     1.5714,     0.0632};
     * ImageCapture position (XYZ) = [    0.2192082 -0.1072567 0.3804391](m)
     * ImageCapture rotation (XYZ) = [  -3.131790885 0 1.576672](rad)
     * ImageCapture rotation (XYZ) = [-179.4384 0 90.3367](deg)
     */

    /* 2019.05.19 updated by YiHung
     * START POSITION above yellow Container (20CM) 
     * ImageCapture = {1.5750, -0.1025, 1.3483, 0.3276, 1.5750, 0.0000};
     * ImageCapture position (X,Y,Z,Rx,Ry,Rz)(ZYX) = [   3.1415925 0 3.1415925]
     */
    std::vector<double> ImageCapture = {1.5750, -0.1025, 1.3483, 0.3276, 1.5750, 0.0000};
    

    TargetVelocity = {0, 0, 0, 0, 0, 0};
    /*if(!ReflexxesPositionRun(TM5, *IP_position, ImageCapture, TargetVelocity, SynchronousTime))
    {
        ROS_WARN("Smooth stop activate...");
        return 0;
    }
    else
    {
        robot_motion_states.data = IMAGE_CAPTURE_POSE;
        RobotState.publish(robot_motion_states);
    }

    TM5.interface->stateRT->getQAct(CurrentPosition);
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_velocity->CurrentPositionVector->VecData[i]     = CurrentPosition[i];
        IP_velocity->CurrentVelocityVector->VecData[i]     = IP_position->CurrentVelocityVector->VecData[i];
        IP_velocity->CurrentAccelerationVector->VecData[i] = IP_position->CurrentAccelerationVector->VecData[i];
    }
*/
    g_objposition[0] = 2.0;
    robot_motion_states.data = 1;

    while(ros::ok())
    {
        if(g_objposition[0] == 1.0 && position_fill && rotation_fill)
        {
            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                TargetPosition[i] = g_objposition[i+1];

            

            if(robot_motion_states.data == 1)
            {
                //above
                SynchronousTime = 3;//1.3; //3;
                robot_motion_states.data = 2;
                if(!ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;

                cout << "=== In tm_action_picking ===" << endl;
                cout << "ObjectClass = " << g_objectclass << endl;
                cout << "=== Move to TargetPosition_above ===" << endl;

                if(g_objectclass == 3 || g_objectclass == 4 || g_objectclass == 5)
                {
                    cout << "=== Gripper OPEN ===" << endl;
                    // TM5.setDigitalOutputEE(0,false);//gripper open
                    TM5.setDigitalOutputMB(0,true);
                    TM5.setDigitalOutputMB(1,false);
                }
                else if(g_objectclass == 1 || g_objectclass == 2)
                {
                    cout << "=== Gripper CLOSE ===" << endl;
                    //TM5.setDigitalOutputEE(0,true);//gripper close
                    TM5.setDigitalOutputMB(0,false);
                    TM5.setDigitalOutputMB(1,true);
                }  

                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);

                // ROS_INFO("Enter to CONTINUE");
                // getchar();
            }
            else if(robot_motion_states.data == 2)
            {   
                //pose
                SynchronousTime = 3;//1.5;//3; //2;
                robot_motion_states.data = 3;
                if(!ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;  

                cout << "=== In tm_action_picking ===" << endl;
                cout << "ObjectClass = " << g_objectclass << endl;
                cout << "=== Move to TargetPosition ===" << endl;

                if(g_objectclass == 3 || g_objectclass == 4 || g_objectclass == 5)
                {
                    cout << "=== Gripper CLOSE ===" << endl;
                    //TM5.setDigitalOutputEE(0,true);//gripper close
                    TM5.setDigitalOutputMB(0,false);
                    TM5.setDigitalOutputMB(1,true);
                }
                else if(g_objectclass == 1 || g_objectclass == 2)
                {
                    cout << "=== Gripper OPEN ===" << endl;
                    // TM5.setDigitalOutputEE(0,false);//gripper open
                    TM5.setDigitalOutputMB(0,true);
                    TM5.setDigitalOutputMB(1,false);
                }  

                // For picking test of OBJECT1
                //TM5.setDigitalOutputEE(0,true);//gripper close

                //For picking strategy of OBJECT1
                //TM5.setDigitalOutputEE(0,false);//gripper open
                
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);

                // ROS_INFO("Enter to CONTINUE");
                // getchar();
            }
            else if(robot_motion_states.data == 3)
            {
                //box_prepare
                SynchronousTime = 3;//1.3;//3; //2;
                robot_motion_states.data = 4;
                if(!ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;  
                
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);

                //ROS_INFO("Enter to CONTINUE");
                //getchar();
            }
            else if(robot_motion_states.data == 4)
            {
                //table_prepare
                SynchronousTime = 4;//1.3;//3; //2;
                robot_motion_states.data = 5;
                if(!ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;  
                
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);

                //ROS_INFO("Enter to CONTINUE");
                //getchar();
            }
            else if(robot_motion_states.data == 5)
            {
                //put
                SynchronousTime = 3;//2;//3;
                robot_motion_states.data = 6;
                if(!ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;  

                cout << "=== In tm_action_picking ===" << endl;
                cout << "ObjectClass = " << g_objectclass << endl;
                cout << "=== Move to FinalPosition ===" << endl;

                if(g_objectclass == 3 || g_objectclass == 4 || g_objectclass == 5)
                {
                    cout << "=== Gripper OPEN ===" << endl;
                    // TM5.setDigitalOutputEE(0,false);//gripper open
                    TM5.setDigitalOutputMB(0,true);
                    TM5.setDigitalOutputMB(1,false);
                }
                else if(g_objectclass == 1 || g_objectclass == 2)
                {
                    cout << "=== Gripper CLOSE ===" << endl;
                    // TM5.setDigitalOutputEE(0,true);//gripper close
                    TM5.setDigitalOutputMB(0,false);
                    TM5.setDigitalOutputMB(1,true);
                }  
                //TM5.setDigitalOutputEE(0,false);//gripper open

                //For picking strategy of OBJECT1
                //TM5.setDigitalOutputEE(0,true);//gripper close
                
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
            }
            else if(robot_motion_states.data == 6)
            {
                //table_prepare
                SynchronousTime = 4;//1.3;//3; //2;
                robot_motion_states.data = 7;
                if(!ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;  
                
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
                robot_motion_states.data = 1;
                //ROS_INFO("Enter to CONTINUE");
                //getchar();
            }
            else{}

            g_objposition[0] = 0.0;
            position_fill = false;
            rotation_fill = false;
            sleep(1);
        }
        else if(g_objposition[0] == 2.0)
        {
            SynchronousTime = 4;//3
            if(!ReflexxesPositionRun(TM5, *IP_position, ImageCapture, TargetVelocity, SynchronousTime))
                    break;
            robot_motion_states.data = IMAGE_CAPTURE_POSE;
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
            RobotState.publish(robot_motion_states);
            g_objposition[0] = 0.0;
        }
        else{}
    }



    ROS_WARN("Bin Picking Demo shutdown");
    fclose(f_traj);
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

void ObjPosition_callback(const geometry_msgs::Vector3::ConstPtr& array)
{
    g_objposition[1] = array->x;
    g_objposition[2] = array->y;
    g_objposition[3] = array->z;
    position_fill = true;
    //ROS_INFO("g_objposition[1] = %f",g_objposition[1] );
    //ROS_INFO("g_objposition[2] = %f",g_objposition[2] );
    //ROS_INFO("g_objposition[3] = %f",g_objposition[3] );
}

void ObjRotation_callback(const geometry_msgs::Vector3::ConstPtr& array)
{
    g_objposition[4] = array->x;
    g_objposition[5] = array->y;
    g_objposition[6] = array->z;
    rotation_fill = true;
    //ROS_INFO("g_objposition[4] = %f",g_objposition[4] );
    //ROS_INFO("g_objposition[5] = %f",g_objposition[5] );
    //ROS_INFO("g_objposition[6] = %f",g_objposition[6] );
}

void MotionLock_callback(const std_msgs::Int32::ConstPtr& cmd)
{
    g_objposition[0] = cmd->data;
    //ROS_INFO("g_objposition[0] = %f",g_objposition[0] );
}

void ObjectClass_callback(const std_msgs::Int32::ConstPtr &object_class)
{
    g_objectclass = object_class->data;
    //ROS_INFO("g_objectclass = %d", g_objectclass);
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

void constraint_callback(const geometry_msgs::PointStamped::ConstPtr& closest_bpt)
{
    g_constraint_position[0] = closest_bpt->point.x;
    g_constraint_position[1] = closest_bpt->point.y;
    g_constraint_position[2] = closest_bpt->point.z;
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

    ros::init(argc, argv, "tm_action_picking");
    ros::NodeHandle node_handle;

    RobotState = node_handle.advertise<std_msgs::Int32>("/robot_motion_states",10);

    ros::AsyncSpinner spinner(7);
    ros::Subscriber position_sub    = node_handle.subscribe("/eff/kinect_merge/closest_pt_tracking", 1,position_callback);
    ros::Subscriber constraint_sub  = node_handle.subscribe("/body/kinect_merge/closest_pt_tracking", 1,constraint_callback);
    ros::Subscriber ObjPosition_sub = node_handle.subscribe("/object_position", 1,ObjPosition_callback);
    ros::Subscriber ObjRotation_sub = node_handle.subscribe("/object_rotation", 1,ObjRotation_callback);
    ros::Subscriber MotionLock_sub  = node_handle.subscribe("/lock_motion", 1,MotionLock_callback);
    ros::Subscriber ObjectClass_sub = node_handle.subscribe("/object_class", 1, ObjectClass_callback);
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
                //gripper initialization 00 10
                TmRobot.setDigitalOutputMB(0,false);
                TmRobot.setDigitalOutputMB(1,false);

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
        else if(strncmp(cstr, "gopen", 5) == 0)
        {
            unsigned char ch = 0;
            //TmRobot.setDigitalOutputEE(ch,false);

            //01 10
            TmRobot.setDigitalOutputMB(0,true);
            TmRobot.setDigitalOutputMB(1,false);
            
        
        }
        else if(strncmp(cstr, "gclose", 6) == 0)
        {
            unsigned char ch = 0;
            //TmRobot.setDigitalOutputEE(ch,true);

            //00 11
            TmRobot.setDigitalOutputMB(0,false);
            TmRobot.setDigitalOutputMB(1,true);
            
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
            unsigned char ch = 0;
            //TmRobot.setDigitalOutputEE(ch,true);//gripper close
            TmRobot.setDigitalOutputMB(0,false);
            TmRobot.setDigitalOutputMB(1,true);
            //TmRobot.setDigitalOutputEE(ch,false);//gripper open
            f_traj = fopen("trajectory.txt", "w");
            f_obstacle = fopen("dis_obstacle.txt", "w");
            BinPicking_Demo(TmRobot);
            //StaticRobotAvoidnace(TmRobot);

            TmRobot.setJointSpdModeoOFF();
            print_info("joint vlocity control mode OFF...");
        }
        else if (strncmp(cstr, "photo",5 ) == 0)
        {
            double blend = 0;
            std::vector<double> vec = {-0.4948,    -0.4284,     1.5968,     0.4711,     1.7698,    -0.0367};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 1 and Take Photo");
            getchar();

            vec = {-0.1992,    -0.6827,     1.7571,     0.4322,     1.5972,    -0.2123};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 2 and Take Photo");            
            getchar();

            vec = {0.1597,    -0.6732,     1.7284,     0.5055,     1.5251,     0.1569};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 3 and Take Photo");            
            getchar();

            vec = { 0.4655,    -0.5940,     1.6912,     0.6066,     1.4034,     0.1574};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 4 and Take Photo");            
            getchar();

            vec = { 0.9068,    -0.6078,     1.7296,     0.7680,     1.3011,     0.1582};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 5 and Take Photo");            
            getchar();                       

            vec = { 0.9378,    -0.4332,     1.8255,     0.4915,     1.3684,     0.9997};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 6 and Take Photo");            
            getchar(); 
        

            vec = { 0.5770,    -0.3954,     1.8248,     0.4096,     1.4479,     0.8374};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 7 and Take Photo");            
            getchar();     

            vec = { 0.1850,    -0.5621,     1.8933,     0.2881,     1.5785,     0.2281};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 8 and Take Photo");            
            getchar(); 

            vec = { -0.2196,    -0.4297,     1.8767,     0.1490,     1.7484,     0.0226};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 9 and Take Photo");            
            getchar();  

            vec = { -0.4177,    -0.2774,     1.7962,     0.1823,     1.8680,    -0.0825};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 10 and Take Photo");            
            getchar();

            vec = { -0.3250,     0.0564,     1.4581,     0.4164,     1.7952,    -0.0836};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 11 and Take Photo");            
            getchar();

            vec = { -0.0413,     0.0269,     1.4581,     0.4445,     1.5706,    -0.0808};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 12 and Take Photo");            
            getchar();

            vec = { 0.1894,     0.0047,     1.4124,     0.5925,     1.5752,     0.1762};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 13 and Take Photo");            
            getchar();

            vec = { 0.3582,     0.0051,     1.4129,     0.6326,     1.4186,     0.1778};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 14 and Take Photo");            
            getchar();

            vec = { 0.5973,    -0.0176,     1.4117,     0.6047,     1.4594,     0.7072};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 15 and Take Photo");                                       
        }
        else if (strncmp(cstr, "binpicking",10 ) == 0)
        {
            double blend = 0;
            //std::vector<double> vec = {0.0691,    -0.6283,     1.9241,     0.2848,     1.5714,     0.0632};

            //std::vector<double> vec = {0.0658,    -0.6082,     1.8497,     0.3387,     1.5775,     0.0598};
            //std::vector<double> vec = {0.0658,    -0.6272,     1.9238,     0.2836,     1.5775,     0.0598};
            //std::vector<double> vec = {0.0657,    -0.6474,     2.0168,     0.2109,     1.5776,     0.0599};
            //TmRobot.setMoveJabs(vec, blend);
            //print_info("Back to position 1");
            //TmRobot.setMoveJabs(vec, blend);
            
            std::vector<double> vec = {1.1291,     0.2681,     1.3371,     0.0303,     1.5440,    -0.4280};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 1 and PUT");
            getchar();

            vec = {1.3276,     0.0901,     1.5472,     0.0021,     1.5573,    -0.2299};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 2 and PUT");            
            getchar();

            vec = {1.5975,    -0.0828,     1.7381,    -0.0689,     1.5541,     0.0398};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 3 and PUT");            
            getchar();

            vec = {1.5868,     0.2600,     1.3457,     0.0349,     1.5754,     0.0287};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 4 and PUT");            
            getchar();

            vec = {1.3740,     0.3923,     1.1689,     0.0789,     1.5605,    -0.1837};
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 5 and PUT");        
            getchar();  

            vec = {1.2074,     0.5594,     0.9217,     0.1562,     1.5492,    -0.3498};

            //vec = {1.1923,     0.1323,     1.5689,    -0.0856,     1.5296,    -0.3731}; //OBJECT1 and OBJECT9 experiment: OBJECT1 Position1
            //vec = {1.2414,     0.3645,     1.2773,    -0.0304,     1.5547,    -0.3378}; //OBJECT1 and OBJECT9 experiment: OBJECT1 Position2
            //vec = {1.2909,     0.6425,     0.8584,     0.1093,     1.5532,    -0.2829}; //OBJECT1 and OBJECT9 experiment: OBJECT1 Position3
            
            //vec = {1.5928,     0.0331,     1.8205,    -0.2445,     1.5771,     0.0617};//OBJECT1 and OBJECT9 experiment: OBJECT9 Position1
            //vec = {1.9709,    -0.0064,     1.8624,    -0.2519,     1.5908,     0.4395};//OBJECT1 and OBJECT9 experiment: OBJECT9 Position2
            //vec = {1.8034,     0.0193,     1.8354,    -0.2474,     1.5845,     0.2161};//OBJECT1 and OBJECT9 experiment: OBJECT9 Position3
            TmRobot.setMoveJabs(vec, blend);
            print_info("Position 6 and PUT");
                     
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
