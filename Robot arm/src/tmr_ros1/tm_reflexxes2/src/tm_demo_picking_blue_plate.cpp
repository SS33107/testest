/*********************************************************************
 * tm_action_picking_safe.cpp
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


// [2]   Picking_above_pose (computational result)
// [3]   Object_pose (computational result)
// [4]   After_picking_pose
// [5]   Prepare_to_place_pose + placing_pose



#include "tm_driver/tm_print.h"
#include "tm_driver/tm_driver.h"
#include "tm_driver/tm_communication.h"
#include "tm_driver/tm_robot_state.h"
#include "tm_driver/tm_ros_node.h"

#include <my_msg_pkg/pose_estimation_data.h>

#include "tm_reflexxes/tm_reflexxes.h"
#include "tm_kinematics/tm_kin.h"
#include "robotiq_controller/Robotiq2FGripper_robot_output.h"

#include "tm_msgs/SetIO.h"
#include "tm_msgs/SetVelocity.h"
#include "tm_msgs/SendScript.h"
#include "tm_msgs/FeedbackState.h"
#include "tm_msgs/SetEvent.h"
#include "tm_msgs/SetPositions.h"

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

#define GRIPPER_LENGTH 0.22

#define VMAX_ATT 0.08
#define VMAX_REP 0.15
#define VMAX_CC_GPR  0.03

#define DANGEROUS_ZONE 0.2
#define JOINTLIMIT_SPD_123 150
#define JOINTLIMIT_SPD_456 200

#define STOP 0
#define PASS 1
#define REACH 2

#define IMAGE_CAPTURE_POSE 1
#define GRASP_READY_POSE   2
#define OBJECT_POSITION    3
#define GRASP_AFTER_POSE   4
#define PLACING_POSE       5
#define PLACING_POSE       5
#define PLACING_POSE_DOWN  6
#define MID_POSE           7
#define CHECK_POSE         8
#define IMAGE_CAPTURE_MOVING 100

#define SMOOTHSTOP 0
#define REACHPOINT 1
#define INTERRUPT  2

#define OBSTACLE_GPR false
#define V_TRAVEL 0.10

using namespace std;

typedef float Scalar;
const float EPS = 1e-6;
const float LAMBDA_MAX = 0.3;
const float EPSQ = 1e-15;

double last_point_x = 0.0;
double last_point_y = 0.0;
double last_point_z = 0.0;

double g_distance;
std::vector<double> g_obstacle_position(3);
std::vector<double> g_obstacle_velocity(3);
std::vector<double> g_constraint_position(3);
std::vector<double> g_robot_joint_angle(6);

double g_objposition[7];
bool position_fill = false;
bool rotation_fill = false;


ros::Publisher RobotState;



using namespace std;

ros::ServiceClient vel_client;
ros::ServiceClient script_client;
ros::ServiceClient event_client;
ros::ServiceClient pos_client;

ros::Publisher pub_ctrl;
robotiq_controller::Robotiq2FGripper_robot_output gripper_command;

std::string exit_cmd = "ScriptExit()";
std::string VStart_cmd = "ContinueVJog()";
std::string VStop_cmd = "StopContinueVmode()";


bool ReflexxesPositionRun(  RMLPositionInputParameters &InputState, 
                            std::vector<double> TargetPosition,
                            std::vector<double> TargetVelocity, 
                            double SynTime);

void reset_gripper()
{
    gripper_command.rACT = 0;
    gripper_command.rGTO = 0;
    gripper_command.rSP  = 0;
    gripper_command.rFR  = 0;   
}

void init_gripper()
{
    gripper_command.rACT = 1;
    gripper_command.rGTO = 1;
    gripper_command.rSP  = 200;
    gripper_command.rFR  = 0;
}

void set_gripper(int mode)
{
    if(mode == 0)   //Open
    {
        gripper_command.rACT = 1;
        gripper_command.rGTO = 1;
        gripper_command.rSP  = 200;
        gripper_command.rFR  = 0;
        gripper_command.rPR = 170;
    }
    if(mode == 1)   //Close
    {
        gripper_command.rACT = 1;
        gripper_command.rGTO = 1;
        gripper_command.rSP  = 200;
        gripper_command.rFR  = 0;
        gripper_command.rPR = 230;
    }
}

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

void ReflexxesSmoothStop(   RMLVelocityInputParameters &InputState, 
                            double SynTime)
{
    tm_msgs::SetVelocity vel_srv;

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

        // TR.setMoveJointSpeedabs(vec, blend);
        vel_srv.request.motion_type = 1;
        vel_srv.request.velocity = vec;
        vel_client.call(vel_srv);

        //**********************
        // Print out commands

        // time_s = TR.interface->stateRT->getTime();
        // printf("[ %lf ] pos:  ",time_s );

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

    std::vector<double> FinalPosition(6);
    // time_s = TR.interface->stateRT->getQAct(FinalPosition);
    printf("=============== Final state of Smooth Stop =========================\n");
    // printf("[ %lf ]  ", time_s);

    FinalPosition[0] = g_robot_joint_angle[0];
    FinalPosition[1] = g_robot_joint_angle[1];
    FinalPosition[2] = g_robot_joint_angle[2];
    FinalPosition[3] = g_robot_joint_angle[3];
    FinalPosition[4] = g_robot_joint_angle[4];
    FinalPosition[5] = g_robot_joint_angle[5];
    for (int i = 0; i < NUMBER_OF_DOFS; ++i){
        printf(" %10.4lf ",FinalPosition[i]);
    }
    printf("\n");
    print_info("Smooth stop finish in %llu us", tt);
    InputState = *IP;

    delete  RML;
    delete  IP;
    delete  OP;
}

void ReflexxesPositionSmoothStop(   RMLPositionInputParameters *InputState, 
                                    double SynTime)
{
    tm_msgs::SetVelocity vel_srv;

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
    IP  = new RMLVelocityInputParameters(NUMBER_OF_DOFS);
    OP  = new RMLVelocityOutputParameters(NUMBER_OF_DOFS);

    *IP->CurrentPositionVector     = *InputState->CurrentPositionVector;
    *IP->CurrentVelocityVector     = *InputState->CurrentVelocityVector;
    *IP->CurrentAccelerationVector = *InputState->CurrentAccelerationVector;


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

        // TR.setMoveJointSpeedabs(vec, blend);
        vel_srv.request.motion_type = 1;
        vel_srv.request.velocity = vec;
        vel_client.call(vel_srv);

        //**********************
        // Print out commands

        // time_s = TR.interface->stateRT->getTime();
        // printf("[ %lf ] pos:  ",time_s );

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

    // std::vector<double> FinalPosition;
    // time_s = TR.interface->stateRT->getQAct(FinalPosition);
    // printf("=============== Final state of Smooth Stop =========================\n");
    // printf("[ %lf ]  ", time_s);

    // for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    //     printf(" %10.4lf ",FinalPosition[i]);
    // printf("\n");
    print_info("Smooth stop finish in %llu us", tt);

    *InputState->CurrentPositionVector     = *IP->CurrentPositionVector;
    *InputState->CurrentVelocityVector     = *IP->CurrentVelocityVector;
    *InputState->CurrentAccelerationVector = *IP->CurrentAccelerationVector;

    delete  RML;
    delete  IP;
    delete  OP;
}

bool ReflexxesPositionRun(  RMLPositionInputParameters &InputState, 
                            std::vector<double> TargetPosition,
                            std::vector<double> TargetVelocity, 
                            double SynTime)
{
    tm_msgs::SetVelocity vel_srv;

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

        // TR.setMoveJointSpeedabs(vec, blend);
        vel_srv.request.motion_type = 1;
        vel_srv.request.velocity = VelocityCommand;
        // vel_client.call(vel_srv);
        if (vel_client.call(vel_srv))                             
        {
            if (vel_srv.response.ok) {
                // ROS_INFO_STREAM("SetPositions to robot");
            }
            else 
                ROS_WARN_STREAM("SetPositions to robot , but response not yet ok ");
        }
        else
        {
            ROS_ERROR_STREAM("Error SetPositions to robot");
            // return 1;
        }
        // usleep(100*1000);

        //***************************************************************
        // Print out commands

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            q[i] = OP->NewPositionVector->VecData[i];

        //tm_jacobian::Forward_Kinematics_3(q,T);
        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);
        ROS_INFO("gripper XYZ_pos: %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);

        //***************************************************************

        if (tm_reflexxes::kbhit())
        {
            char c = getchar();
            if (c == 'q' || c == 'Q')
            {
                ROS_WARN("Smooth Stop Activate...");
                ReflexxesPositionSmoothStop(IP, 0.25);
                pass = false;
                break;
            }
        }

        *IP->CurrentPositionVector =  *OP->NewPositionVector;
        *IP->CurrentVelocityVector =  *OP->NewVelocityVector;

        gettimeofday(&tm2, NULL);
        long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);            
        // usleep(24940 - time_compensation);
        if((24940 - time_compensation) < 1)
            usleep(100);
        else
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
    double Vmax_att = VMAX_ATT;
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
    ROS_WARN("Attractive Velocity: %10.4lf ", AttractiveScalar);

    //succeed = tm_jacobian::GetQdfromInverseJacobian(CurrentPosition,EFF_Velocity,action_qd);
    succeed = GetQdfromLinearJacobian(CurrentPosition,EFF_Velocity,action_qd);  

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
//    ROS_WARN("Repulsive  Velocity: %10.4lf ", sqrt(RepulsiveVelocity.dot(RepulsiveVelocity)));
    
    //succeed = tm_jacobian::GetQdfromInverseJacobian(CurrentPosition,EFF_Velocity,Repulsive_qd);   
    succeed = GetQdfromLinearJacobian(CurrentPosition,EFF_Velocity,Repulsive_qd);   
    //succeed = GetQdfromVirtualForceConstrain(CurrentPosition,EFF_Velocity,Repulsive_qd);   

    return succeed;

}

bool ReflexxesVelocityInterrupt(RMLVelocityInputParameters &InputState, 
                                std::vector<double> TargetVelocity, 
                                double SynTime)
{
    tm_msgs::SetVelocity vel_srv;
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

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            q[i] = OP->NewPositionVector->VecData[i];

        if(!CheckJointLimit(q))
        {
            ROS_WARN("Joint position execced limit!!");
            ReflexxesSmoothStop(*IP, 0.25);
            pass = false;
            break;
        }
        else
        {
            // TR.setMoveJointSpeedabs(vec, blend);
            vel_srv.request.motion_type = 1;
            vel_srv.request.velocity = VelocityCommand;
            // vel_client.call(vel_srv);
            if (vel_client.call(vel_srv))                             
            {
                if (vel_srv.response.ok) {
                    // ROS_INFO_STREAM("SetPositions to robot");
                }
                else 
                    ROS_WARN_STREAM("SetPositions to robot , but response not yet ok ");
            }
            else
            {
                ROS_ERROR_STREAM("Error SetPositions to robot");
                // return 1;
            }
        }

        //***************************************************************
        // Print out commands

        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);

        ROS_WARN("tool XYZ_pos: %10.4lf %10.4lf  [ %lf ] ", T[3], T[7], T[11]);

        //***************************************************************

        *IP->CurrentPositionVector =  *OP->NewPositionVector;
        *IP->CurrentVelocityVector =  *OP->NewVelocityVector;
        *IP->CurrentAccelerationVector = *OP->NewAccelerationVector;

        gettimeofday(&tm2, NULL);
        long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);            
        if((24940 - time_compensation) < 1)
            usleep(100);
        else
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

int ReflexxesPositionSafetyRun( RMLPositionInputParameters &InputState, 
                                std::vector<double> TargetPosition,
                                std::vector<double> TargetVelocity, 
                                double SynTime)
{
    tm_msgs::SetVelocity vel_srv;
    double time_s, blend = 0;
    std::vector<double> FinalPosition(6), CurrentPosition(6), VelocityCommand(6);
    int pass = REACHPOINT;
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

    std::vector<double> PotentialField_qd(6), qd_now(6),RepulsiveVelocity(6);
    std::vector<double> Attractive_qd(6), Attractive_velocity(6), TargetPoint(3);

    double *GoalPoint     = new double [3];
    double *ObstaclePoint = new double [3];
    double *q             = new double [6];
    double *T             = new double [16];
    double distance;
    double ScalingFactor;

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        q[i] = TargetPosition[i];
    }
    tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);
    GoalPoint[0] = T[3];
    GoalPoint[1] = T[7];
    GoalPoint[2] = T[11];
    TargetPoint[0] = T[3];
    TargetPoint[1] = T[7];
    TargetPoint[2] = T[11];

    g_distance = sqrt(pow((g_obstacle_position[0] - T[3]),2)+pow((g_obstacle_position[1] - T[7]),2)+pow((g_obstacle_position[2] - T[11]),2));

    //  ********************************************************************/
    //  Assigning all RMLPositionInputParameters : 
    //  Current POS, VEL, ACC : set before call ReflexxesPositionRun
    //  Target POS, VEL       : set before call ReflexxesPositionRun
    //  Max VEL, ACC          : set after call ReflexxesPositionRun
    //  SelectionVector       : set after call ReflexxesPositionRun
    //  ********************************************************************
    *IP = InputState;
    // time_s = TR.interface->stateRT->getQAct(CurrentPosition);
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        CurrentPosition[i] = g_robot_joint_angle[i];
    }
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP->MaxVelocityVector->VecData[i]     = 0.5*40; //0.3247
        IP->MaxAccelerationVector->VecData[i] = 0.0375*40;;
        IP->TargetPositionVector->VecData[i]  = TargetPosition[i]; 
        IP->TargetVelocityVector->VecData[i]  = TargetVelocity[i];
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
        
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            q[i] = OP->NewPositionVector->VecData[i];                            
        
        if(!CheckJointLimit(q))
        {
            ROS_WARN("Joint position execced limit!!");
            ReflexxesPositionSmoothStop(IP, 0.25);
            pass = SMOOTHSTOP;
            break;
        }
        else
        {
            // TR.setMoveJointSpeedabs(vec, blend);
            vel_srv.request.motion_type = 1;
            vel_srv.request.velocity = VelocityCommand;
            // vel_client.call(vel_srv);
            if (vel_client.call(vel_srv))                             
            {
                if (vel_srv.response.ok) {
                    // ROS_INFO_STREAM("SetPositions to robot");
                }
                else 
                    ROS_WARN_STREAM("SetPositions to robot , but response not yet ok ");
            }
            else
            {
                ROS_ERROR_STREAM("Error SetPositions to robot");
                // return 1;
            }
        }                    

        //***************************************************************
        // Print out commands

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            q[i] = OP->NewPositionVector->VecData[i];
        }
        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);
        
        if(!CheckJointLimit(q))
        {
            ROS_WARN("Joint position execced limit!!");
            ReflexxesPositionSmoothStop(IP, 0.25);
            pass = SMOOTHSTOP;
            break;
        }

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            qd_now[i] = OP->NewVelocityVector->VecData[i];

        //***************************************************************
        // Collision avoidance control activate

        AlreadyAccese = false;

        g_distance = sqrt(pow((g_obstacle_position[0] - T[3]),2)+pow((g_obstacle_position[1] - T[7]),2)+pow((g_obstacle_position[2] - T[11]),2));

        while (g_distance < DANGEROUS_ZONE)
        {
            for (int i = 0; i < 3; ++i)
                ObstaclePoint[i] = g_obstacle_position[i];

            ROS_WARN("Safety control Activate...");
            if(!AlreadyAccese)
            {
                *IP_interrupt->CurrentPositionVector     = *IP->CurrentPositionVector;
                *IP_interrupt->CurrentVelocityVector     = *IP->CurrentVelocityVector;
                *IP_interrupt->CurrentAccelerationVector = *IP->CurrentAccelerationVector;
                AlreadyAccese = true;
            }

            OnlineAttractiveForceGeneration(Attractive_qd, T, q, TargetPoint, Attractive_velocity);
            if(OnlineRepulsiveForceGeneration(PotentialField_qd,T, q,RepulsiveVelocity))
            {
                for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                {
                    PotentialField_qd[i] += Attractive_qd[i];
                }

                if(CheckVelocityLimit(PotentialField_qd,ScalingFactor))
                {
                    ReflexxesVelocityInterrupt(*IP_interrupt,PotentialField_qd,0.2);
                    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                    {
                        q[i]      = IP_interrupt->CurrentPositionVector->VecData[i];
                        qd_now[i] = IP_interrupt->CurrentVelocityVector->VecData[i];
                    }
                    tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);
                }
                else
                {
                    ROS_WARN("Joint velocity out of range!! Smooth stop activate");
                    ReflexxesSmoothStop(*IP_interrupt, 0.1);
                    break;
                }
            }
            else
            {
                ROS_WARN("Joint velocity out of range!! Smooth stop activate");
                ReflexxesSmoothStop(*IP_interrupt, 0.1);
                *IP->CurrentPositionVector     = *IP_interrupt->CurrentPositionVector;
                *IP->CurrentVelocityVector     = *IP_interrupt->CurrentVelocityVector;
                *IP->CurrentAccelerationVector = *IP_interrupt->CurrentAccelerationVector;
                pass = SMOOTHSTOP;
                break;
            }

            g_distance = sqrt(pow((g_obstacle_position[0] - T[3]),2)+pow((g_obstacle_position[1] - T[7]),2)+pow((g_obstacle_position[2] - T[11]),2));

            if(g_distance > DANGEROUS_ZONE)
            {
                *IP->CurrentPositionVector     = *IP_interrupt->CurrentPositionVector;
                *IP->CurrentVelocityVector     = *IP_interrupt->CurrentVelocityVector;
                *IP->CurrentAccelerationVector = *IP_interrupt->CurrentAccelerationVector;
                double RecoverRoute = sqrt(pow((T[3]-GoalPoint[0]),2)+pow((T[7]-GoalPoint[1]),2)+pow((T[11]-GoalPoint[2]),2));
                double RecoverTime = RecoverRoute/V_TRAVEL;

                TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
                ReflexxesPositionSafetyRun(*IP, TargetPosition, TargetVelocity, RecoverTime);
                pass = INTERRUPT;
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
        if((24940 - time_compensation) < 1)
            usleep(100);
        else
            usleep(24940 - time_compensation); 

        //********************************************************
        // The area execution in 25ms real time sharp
    }

    gettimeofday(&tm4, NULL);
    long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

    if(pass == REACHPOINT)
    {
        printf("=============== Final state ReflexxesPositionSafetyRun =========================\n");
        // printf("[ %lf ]  ", time_s);
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            printf("%10.4lf ", IP->CurrentPositionVector->VecData[i]);

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


bool BinPicking_Demo()
{
    // TM5.setJointSpdModeON();
    // print_info("joint velocity control mode ON...");

    bool run_succeed = true;
    bool table_to_box = false;
    bool skip_flag = false;
    int  pass = 0;
    int finalpose_flag = 1;
    std::vector<double>  final_position(6);
    double SynchronousTime = 7;
    std::vector<double> TargetPosition(6), TargetVelocity, CurrentPosition(6), qd(6), Attractive_qd(6), Repulsive_qd(6), RepulsiveVelocity(6);
    double *T = new double [16];
    double *q = new double [6];
    std_msgs::Int32 robot_motion_states;

    RMLPositionInputParameters  *IP_position = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    RMLVelocityInputParameters  *IP_velocity = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

    // TM5.interface->stateRT->getQAct(CurrentPosition);
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        CurrentPosition[i] = g_robot_joint_angle[i];
    }

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_position->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_position->CurrentVelocityVector->VecData[i] = 0.0;
        IP_position->CurrentAccelerationVector->VecData[i] = 0.0;

        IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_velocity->CurrentVelocityVector->VecData[i] = 0.0;
        IP_velocity->CurrentAccelerationVector->VecData[i] = 0.0;
    }
    
    //0523 箱子image_pose
    std::vector<double> ImageCapture1    = {1.1517,    0.1876,    1.8279,    -0.4105,    1.5724,    -0.4133};
    //0524 桌角image_pose
    std::vector<double> ImageCapture2    = {0.6208,    -0.5978,    1.7682,    0.4171,    1.5659,    0.6273};

    //0524中間檢查點
    std::vector<double> RecoverPosition = {1.0773,    -0.0594,    1.7915,    -0.1471,    1.571,    -0.4834};

    // std::vector<double> BoxAbovePosition = {1.0383,    0.623,    0.3981,    0.5452,    1.5768,    -0.4699};  //0523 above box


    TargetVelocity = {0, 0, 0, 0, 0, 0};

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
                // Picking_above_pose (computational result)
                ROS_WARN("HeyHey");
                skip_flag = false;
                // SynchronousTime = 5;//1.5;
                SynchronousTime = 3;
                robot_motion_states.data = 2;
                if(!ReflexxesPositionRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
            }
            else if(robot_motion_states.data == 2) 
            {
                // Object_pose (computational result)
                // SynchronousTime = 5;//2;
                ROS_WARN("HoHo");
                SynchronousTime = 3;
                robot_motion_states.data = 3;
                if(!ReflexxesPositionRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;  
                // getchar();
                set_gripper(0); 
                pub_ctrl.publish(gripper_command);

                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
            }
            else if(robot_motion_states.data == 3)
            {
                // After_picking_pose
                // If move from box to table : Box_above_pose
                // If move from table to box : Picking_above_pose
                robot_motion_states.data = 4;

                if (!table_to_box)
                {
                    // SynchronousTime = 0.7;
                    SynchronousTime = 2;
                    // if(!ReflexxesPositionRun(TM5, *IP_position, BoxAbovePosition, TargetVelocity, SynchronousTime))
                    //     break;

                    if(!ReflexxesPositionRun(*IP_position, ImageCapture1, TargetVelocity, SynchronousTime))
                        break;
                    

                }
                else{
                    
                    // SynchronousTime = 0.8;
                    SynchronousTime = 2;
                    if(!ReflexxesPositionRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                        break;
                    if(!ReflexxesPositionRun(*IP_position, ImageCapture2, TargetVelocity, SynchronousTime))
                        break;
                }
                

                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
                // ROS_INFO("Enter to CONTINUE");
                // getchar();

            }
            else if(robot_motion_states.data == 4)
            {
                // Prepare_to_place_pose + placing_pose
                // If move from box to table : Table_above_pose + Final_pose
                // If move from table to box : RecoverPosition + Box_above_pose + Final_pose

                SynchronousTime = 9;
                robot_motion_states.data = 5;

                if(!table_to_box)
                {
                    // ReflexxesPositionSafetyRun
                    if(ReflexxesPositionSafetyRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime) == SMOOTHSTOP)
                        break;

                    //table五筒
                    
                    if(finalpose_flag == 1)
                        final_position = {0.1688,    -0.4218,    2.1167,    -0.1115,    1.5567,    0.1737};

                    else if(finalpose_flag == 2)
                        final_position = {0.1335,    -0.1536,    1.9131,    -0.1803,    1.5542,    0.1408};

                    else if(finalpose_flag == 3)
                        final_position = {0.4253,    -0.3756,    2.0115,    -0.0559,    1.554,    0.4321};

                    else if(finalpose_flag == 4)
                        final_position = {0.3342,    -0.1651,    1.8837,    -0.1463,    1.5516,    0.3391};

                    else if(finalpose_flag == 5)
                        final_position = {0.2489,    -0.3009,    1.9305,    -0.0562,    1.5516,    0.2546};

                    // SynchronousTime = 0.8;
                    SynchronousTime = 5;
                    if(!ReflexxesPositionRun(*IP_position, final_position, TargetVelocity, SynchronousTime))
                        break;
                    finalpose_flag = finalpose_flag + 1;
                    //change mode
                    if(finalpose_flag >= 6)
                    {
                        table_to_box = true;
                        skip_flag = true;
                        finalpose_flag = 1;
                    }
                    

                }
                else
                {
                    // SynchronousTime = 1.5;
                    SynchronousTime = 4;
                    
                    if(!ReflexxesPositionRun(*IP_position, RecoverPosition, TargetVelocity, SynchronousTime))
                        break;

                    // SynchronousTime = 0.5;
                    SynchronousTime = 5;
                    if(!ReflexxesPositionRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                        break;

                    //箱子內彷初始排法
                    if(finalpose_flag == 1)
                        final_position = {1.1699,    0.2217,    1.764,    -0.3986,    1.5584,    -0.4213};


                    else if(finalpose_flag == 2)
                        final_position = {1.0263,    0.3543,    1.5631,    -0.3327,    1.5567,    -0.5779};


                    else if(finalpose_flag == 3)
                        final_position = {1.2296,    0.414,    1.4451,    -0.2681,    1.557,    -0.3595};


                    else if(finalpose_flag == 4)
                        final_position = {1.0934,    0.5824,    1.1938,    -0.1855,    1.5565,    -0.4913};


                    else if(finalpose_flag == 5)
                        final_position = {1.11,    0.3693,    1.453,    -0.2227,    1.5567,    -0.4753};


                    // SynchronousTime = 0.5;
                    SynchronousTime = 5.5;
                    if(!ReflexxesPositionRun(*IP_position, final_position, TargetVelocity, SynchronousTime))
                        break;
                    
                    finalpose_flag = finalpose_flag + 1;

                    //中止program
                    if(finalpose_flag >= 6)
                    {
                        table_to_box = false;
                        finalpose_flag = 1;
                        set_gripper(1); 
                        pub_ctrl.publish(gripper_command);
                        sleep(1);
                        // SynchronousTime = 0.5;
                        SynchronousTime = 1.5;
                        if(!ReflexxesPositionRun(*IP_position, ImageCapture1, TargetVelocity, SynchronousTime))
                            break;
                        SynchronousTime = 1.5;
                        if(!ReflexxesPositionRun(*IP_position, ImageCapture2, TargetVelocity, SynchronousTime))
                            break;
                        return(0);

                    }

                }



                //close gripper
                set_gripper(1); 
                pub_ctrl.publish(gripper_command);

               
                

                
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);

                
                // ROS_INFO("Enter to CONTINUE");
                // getchar();
                
            }
            else if(robot_motion_states.data == 5)
            {
                // Mid_point
                // If move from box to table : Image1
                // If move from table to box : Image1
                robot_motion_states.data = 8;

                if (!table_to_box)
                {
                    // SynchronousTime = 0.7;
                    SynchronousTime = 5;
                    if(!ReflexxesPositionRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                        break;

                }
                else
                {
                    
                    // SynchronousTime = 0.8;
                    SynchronousTime = 5;
                    if(!ReflexxesPositionRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;
                }
                

                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
                // ROS_INFO("Enter to CONTINUE");
                // getchar();
                robot_motion_states.data = 1;
            }
            

            else{}

            g_objposition[0] = 0.0;
            position_fill = false;
            rotation_fill = false;
            sleep(1);
        }
        else if(g_objposition[0] == 2.0)
        {

            robot_motion_states.data = IMAGE_CAPTURE_MOVING;
            RobotState.publish(robot_motion_states);



            // SynchronousTime = 1.5;
            SynchronousTime = 5;
            if(!table_to_box)
            {
                if(!ReflexxesPositionRun(*IP_position, ImageCapture1, TargetVelocity, SynchronousTime))
                    break;
            }
            else
            {
                
                if(!skip_flag)
                {

                    if(!ReflexxesPositionRun(*IP_position, ImageCapture1, TargetVelocity, SynchronousTime))
                            break;
                }
                else
                {
                    skip_flag = false;
                }

                if(!ReflexxesPositionRun(*IP_position, ImageCapture2, TargetVelocity, SynchronousTime))
                    break;
            }
            robot_motion_states.data = IMAGE_CAPTURE_POSE;
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
            RobotState.publish(robot_motion_states);
            g_objposition[0] = 0.0;
            skip_flag = true;
        }
    }



    ROS_WARN("Bin Picking Demo shutdown");
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
    ROS_INFO("g_objposition[1] = %f",g_objposition[1] );
    ROS_INFO("g_objposition[2] = %f",g_objposition[2] );
    ROS_INFO("g_objposition[3] = %f",g_objposition[3] );
}

void ObjRotation_callback(const geometry_msgs::Vector3::ConstPtr& array)
{
    g_objposition[4] = array->x;
    g_objposition[5] = array->y;
    g_objposition[6] = array->z;
    rotation_fill = true;
    ROS_INFO("g_objposition[4] = %f",g_objposition[4] );
    ROS_INFO("g_objposition[5] = %f",g_objposition[5] );
    ROS_INFO("g_objposition[6] = %f",g_objposition[6] );
}

void MotionLock_callback(const std_msgs::Int32::ConstPtr& cmd)
{
    g_objposition[0] = cmd->data;
    ROS_INFO("g_objposition[0] = %f",g_objposition[0] );
}
void distance_callback(const std_msgs::Float32::ConstPtr& distance)
{
    g_distance = distance->data;
    ROS_ERROR("minimum_distance = %10.3f",distance->data);
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

void TMmsgCallback(const tm_msgs::FeedbackState::ConstPtr& msg)
{
  if(msg->joint_pos.size() == 6){
    g_robot_joint_angle[0] = msg->joint_pos[0];
    g_robot_joint_angle[1] = msg->joint_pos[1];
    g_robot_joint_angle[2] = msg->joint_pos[2];
    g_robot_joint_angle[3] = msg->joint_pos[3];
    g_robot_joint_angle[4] = msg->joint_pos[4];
    g_robot_joint_angle[5] = msg->joint_pos[5];
  }
}


int main(int argc, char **argv)
{
    // std::string host;

    // #ifdef USE_BOOST
    //     boost::condition_variable data_cv;
    //     boost::condition_variable data_cv_rt;
    // #else
    //     std::condition_variable data_cv;
    //     std::condition_variable data_cv_rt;
    // #endif

    ros::init(argc, argv, "tm_action_picking");
    ros::NodeHandle node_handle;

    RobotState = node_handle.advertise<std_msgs::Int32>("/robot_motion_states",10);
    

    ros::AsyncSpinner spinner(7);
    ros::Subscriber position_sub    = node_handle.subscribe("/eff/kinect_merge/closest_pt_tracking", 1,position_callback);
    ros::Subscriber constraint_sub  = node_handle.subscribe("/body/kinect_merge/closest_pt_tracking", 1,constraint_callback);
    ros::Subscriber ObjPosition_sub = node_handle.subscribe("/object_position", 1,ObjPosition_callback);
    ros::Subscriber ObjRotation_sub = node_handle.subscribe("/object_rotation", 1,ObjRotation_callback);
    ros::Subscriber MotionLock_sub  = node_handle.subscribe("/lock_motion", 1,MotionLock_callback);
    ros::Subscriber robot_status_sub = node_handle.subscribe("feedback_states", 1000,TMmsgCallback);

    vel_client = node_handle.serviceClient<tm_msgs::SetVelocity>("tm_driver/set_velocity");
    script_client = node_handle.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
    event_client = node_handle.serviceClient<tm_msgs::SetEvent>("tm_driver/set_event");
    pos_client = node_handle.serviceClient<tm_msgs::SetPositions>("tm_driver/set_positions");
    pub_ctrl = node_handle.advertise<robotiq_controller::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput",10);

    tm_msgs::SetPositions pos_srv;
    tm_msgs::SetEvent event_srv;
    tm_msgs::SendScript script_srv;

    spinner.start();

    // if (!(ros::param::get("~robot_ip", host)))
    // {
    //     if (argc > 1)
    //         host = argv[1];
    //     else
    //         exit(1);
    // }

    // const int STDIN = 0;
    // int sockfd = -1;
    bool fgRun = false;

    // for (int i = 0; i < argc; i++)
    // {
    //     printf("[DEBUG] arg%d:= %s\n", i, argv[i]);
    // }
    // host = argv[1];
    // printf("[ INFO] host: %s\n", host.c_str());


    // TmDriver TmRobot(data_cv, data_cv_rt, host, 0);

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
            script_srv.request.id = "quit";
            script_srv.request.script = exit_cmd;
            
            script_client.call(script_srv);
            fgRun = false;

            print_info("quit");
            break;
        }
        else if (strncmp(cstr, "start", 5) == 0)
        {
            ROS_INFO("Activating gripper");
            reset_gripper();
            pub_ctrl.publish(gripper_command);
            sleep(1);
            init_gripper();
            pub_ctrl.publish(gripper_command);
            sleep(1);
            set_gripper(1); 
            pub_ctrl.publish(gripper_command);
            sleep(1);
        }
        else if (strncmp(cstr, "halt", 4) == 0)
        {
            print_info("halt");
            break;
        }
        else if(strncmp(cstr, "home", 4) == 0)
        {
            std::vector<double> vec1 = {0,0,0,0,0,0};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("Back to home");
        }
        else if(strncmp(cstr, "ready", 5) == 0)
        {
            std::vector<double> vec1 = {0.0,    -0.4777,    1.9319,    -1.4537,    1.5708,    0.0};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("Back to ready position");
        }
        else if(strncmp(cstr, "gopen", 6) == 0)
        {
            set_gripper(0); 
            pub_ctrl.publish(gripper_command);
        }
        else if(strncmp(cstr, "gclose", 7) == 0)
        {
            set_gripper(1); 
            pub_ctrl.publish(gripper_command);
        }
        else if(strncmp(cstr, "gopen0", 7) == 0)
        {
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);
            
        }
        else if(strncmp(cstr, "1", 1) == 0)
        {
            std::vector<double> vec1 = {0.9945,    0.7611,    0.4229,    0.3955,    1.5723,    -0.5669};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("Back to position 1");
        }
        else if(strncmp(cstr, "2", 1) == 0)
        {
            std::vector<double> vec1 = {1.0334,    0.429,    0.9594,    0.1907,    1.5728,    -0.5289};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("Back to position 2");
        }
        else if(strncmp(cstr, "3", 1) == 0)
        {
            std::vector<double> vec1 = {1.1081,    0.7451,    0.4516,    0.3822,    1.5728,    -0.4536};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("Back to position 3");
        }
        else if(strncmp(cstr, "4", 1) == 0)
        {
            std::vector<double> vec1 = {1.174,    0.4032,    0.9925,    0.1781,    1.5692,    -0.3844};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("Back to position 4");
        }
        else if(strncmp(cstr, "5", 1) == 0)
        {
            std::vector<double> vec1 = {1.0418,    0.3586,    0.6946,    0.5091,    1.5725,    -0.5205};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("Back to position 5");
        }
    
        else if(strncmp(cstr, "image1", 6) == 0)
        {
            std::vector<double> vec1 = {1.1517,    0.1876,    1.8279,    -0.4105,    1.5724,    -0.4133};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("Back to position 5");
        }
        else if(strncmp(cstr, "image2", 6) == 0)
        {
            std::vector<double> vec1 = {0.6208,    -0.5978,    1.7682,    0.4171,    1.5659,    0.6273};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("Back to position 5");
        }

    
        else if (strncmp(cstr, "gotest", 6) == 0)
        {
            // TmRobot.setDigitalOutputEE(0,true);
            set_gripper(1); 
            pub_ctrl.publish(gripper_command);

            script_srv.request.id = "Vstart";
            script_srv.request.script = VStart_cmd;
            script_client.call(script_srv);
            print_info("joint velocity control mode ON...");

            BinPicking_Demo();
            //StaticRobotAvoidnace(TmRobot);
            script_srv.request.id = "spdmodeoff";
            script_srv.request.script = VStop_cmd;
            script_client.call(script_srv);
            
            print_info("joint vlocity control mode OFF...");
            


        }
        else if (strncmp(cstr, "spdmodeoff", 10) == 0)
        {
            script_srv.request.id = "spdmodeoff";
            script_srv.request.script = VStop_cmd;

            script_client.call(script_srv);
            print_info("joint vlocity control mode OFF...");
        }
        else
        {
            std::string cmd{ cstr };
            std::cout << "send cmd: " << cmd << "\n";

            script_srv.request.id = "Parse_cmd";
            script_srv.request.script = cmd;
            script_client.call(script_srv);

            print_info("send cmd...");
        }
    }
    //ros::waitForShutdown();

    printf("[ info] TM_ROS: shutdown\n");

    return 0;
}
