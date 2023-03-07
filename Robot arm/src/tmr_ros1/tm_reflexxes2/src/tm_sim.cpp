/*********************************************************************
 * tm_sim.cpp
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

#include <stdio.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>

#include "tm_reflexxes/tm_reflexxes.h"
#include "tm_kinematics/tm_kin.h"


#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>



#define DEG2RAD 0.01745329252
#define RAD2DEG 57.29577951
#define REPEATABILITY 0.01//0.00005

#define CYCLE_TIME_IN_SECONDS                   0.025
#define NUMBER_OF_DOFS                          6

#define MAX_ACC 0.0375*40 // 0.0375 : acc in 25ms
#define DANGEROUS_ZONE 0.3
#define JOINTLIMIT_SPD_123 90
#define JOINTLIMIT_SPD_456 150

#define STOP 0
#define PASS 1
#define REACH 2

#define VMAX_ATT 0.05
#define VMAX_CC  0.05

typedef float Scalar;
const float EPS = 1e-6;
const float LAMBDA_MAX = 0.3;
const float EPSQ = 1e-15;

std::vector<double> jointposition(6), jointvelocity(6), jointeffort(6);
double last_point_x = 0.0;
double last_point_y = 0.0;
double last_point_z = 0.0;
bool g_obstacle;

using namespace std;

void publishMsgRT();
bool ReflexxesPositionRun_sim(  RMLPositionInputParameters &InputState, 
                                std::vector<double> TargetPosition,
                                std::vector<double> TargetVelocity, 
                                double SynTime);


bool CheckJointLimit(double *q)
{
    bool valid = true;

    if(abs(q[0]) > 270*DEG2RAD)
    {
        ROS_WARN("[Position] 1st joint position out of limit (270) : %lf",q[0]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[1]) > 180*DEG2RAD)
    {
        ROS_WARN("[Position] 2nd joint position out of limit (180): %lf",q[1]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[2]) > 155*DEG2RAD)
    {
        ROS_WARN("[Position] 3rd joint position out of limit (155): %lf",q[2]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[3]) > 180*DEG2RAD)
    {
        ROS_WARN("[Position] 4th joint position out of limit (180): %lf",q[3]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[4]) > 180*DEG2RAD)
    {
        ROS_WARN("[Position] 5th joint position out of limit (180): %lf",q[4]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[5]) > 270*DEG2RAD)
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
        if (abs(qd[i])/(150*DEG2RAD) > 1.0)
        {
            ExceedRatio[i] = abs(qd[i])/M_PI;
            ROS_WARN("[Velocity] %dth joint velocity exceed limit(180): %10.4lf",i+1,qd[i]*RAD2DEG);
            valid = false;
        }
        else
            ExceedRatio[i] = 0.0;
    }

    for (int i = 3; i < 6; ++i)
    {
        if (abs(qd[i])/(200*DEG2RAD) > 1.0)
        {
            ExceedRatio[i] = abs(qd[i])/(255*DEG2RAD);
            ROS_WARN("[Velocity] %dth joint velocity exceed limit(180): %10.4lf",i+1,qd[i]*RAD2DEG);
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

//  ********************************************************************/
//  fn        : ReflexxesVelocityRun_sim() -- Original
//  beirf     : Use RML API to execute given velocity in simulation.  
//  param[in] : &InputState, Current State of robot under RML.
//  param[in] : TargetVelocity, The velocity when reach target position.
//  param[in] : SynTime, The time for execute the trajectory.
//  ********************************************************************
bool ReflexxesVelocityRun_sim(  RMLVelocityInputParameters &InputState, 
                                std::vector<double> TargetVelocity, 
                                double SynTime)
{
    ReflexxesAPI *RML = NULL;
    RMLVelocityInputParameters  *IP = NULL;
    RMLVelocityOutputParameters *OP = NULL;
    RMLVelocityFlags Flags;
    int ResultValue = 0;
    bool pass = true;
    
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

        //***************************************************************
        // Print out commands
        //ROS_INFO("position: ");
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            q[i] = OP->NewPositionVector->VecData[i];
            jointposition[i] = OP->NewPositionVector->VecData[i];
        }

        tm_kinematics::forward(q,T);
        ROS_INFO("tool0 XYZ_pos: %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);

        //ROS_INFO("velocity: ");

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            //ROS_INFO("%10.4lf ", OP->NewVelocityVector->VecData[i]);
            jointvelocity[i] = OP->NewVelocityVector->VecData[i];
            jointeffort[i] = OP->NewAccelerationVector->VecData[i];
        }
        //***************************************************************

        if (tm_reflexxes::kbhit())
        {
            char c = getchar();
            if (c == 'q' || c == 'Q')
            {
                pass = false;
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

    ROS_INFO("=============== Final state velocity based =========================\n");

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        printf(" %10.4lf ",IP->CurrentVelocityVector->VecData[i]);
    printf("\n");
    print_info("Finished in %llu us", tt);

    tm_reflexxes::resetTermios();

    InputState = *IP;
    delete  RML;
    delete  IP;
    delete  OP;

    delete [] T;
    delete [] q;

    return pass;
}

//  ********************************************************************/
//  fn        : ReflexxesVelocityRun_sim()  -- For p2p motion
//  brief     : Use RML API to execute given velocity in simulation.  
//  param[in] : &InputState, Current State of robot under RML.
//  param[in] : TargetVelocity, The velocity when reach target position.
//  param[in] : SynTime, The time for execute the trajectory.
//  param[out]: 1:pass, 2:reach goal position, 0:smooth stop
//  ********************************************************************
int ReflexxesVelocityRun_sim(   RMLVelocityInputParameters &InputState, 
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

        //***************************************************************
        // Print out commands
        //ROS_INFO("position: ");
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            q[i] = OP->NewPositionVector->VecData[i];
            jointposition[i] = OP->NewPositionVector->VecData[i];
        }

        tm_kinematics::forward(q,T);
        //ROS_INFO("tool0 XYZ_pos: %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);

        //ROS_INFO("velocity: ");

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            //ROS_INFO("%10.4lf ", OP->NewVelocityVector->VecData[i]);
            jointvelocity[i] = OP->NewVelocityVector->VecData[i];
            jointeffort[i] = OP->NewAccelerationVector->VecData[i];
        }
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
    print_info("Finished in %llu us", tt);

    tm_reflexxes::resetTermios();

    InputState = *IP;
    delete  RML;
    delete  IP;
    delete  OP;

    delete [] T;
    delete [] q;

    return pass;
}

bool ReflexxesPositionRun_sim(  RMLPositionInputParameters &InputState, 
                                std::vector<double> TargetPosition,
                                std::vector<double> TargetVelocity, 
                                double SynTime)
{
    double time_s;
    std::vector<double> FinalPosition;
    bool pass = true;
    tm_reflexxes::initTermios(1);

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

        //***************************************************************
        // Print out commands
        //ROS_INFO("position: ");
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            //ROS_INFO("%10.4lf ", OP->NewPositionVector->VecData[i]);
            q[i] = OP->NewPositionVector->VecData[i];
            jointposition[i] = OP->NewPositionVector->VecData[i];
        }

        //tm_jacobian::Forward_Kinematics_3(q,T);
        tm_kinematics::forward(q,T);
        ROS_INFO("joint6 XYZ_pos: %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);

        //ROS_INFO("velocity: ");

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            //ROS_INFO("%10.4lf ", OP->NewVelocityVector->VecData[i]);
            jointvelocity[i] = OP->NewVelocityVector->VecData[i];
            jointeffort[i] = OP->NewAccelerationVector->VecData[i];
        }
        //***************************************************************

        if (tm_reflexxes::kbhit())
        {
            char c = getchar();
            if (c == 'q' || c == 'Q')
            {
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
    InputState = *IP;

    delete  RML;
    delete  IP;
    delete  OP;
    delete [] T;
    delete [] q;

    return pass;
}

void publishMsgRT()
{
    ros::NodeHandle nh_;
    std::vector<double> joint_offsets;
    std::string base_frame;
    std::string tool_frame;
    static tf::TransformBroadcaster tf_bc;
    sensor_msgs::JointState joint_msg;
    std::vector<std::string> joint_names;

    joint_offsets.assign(6, 0.0);
    std::string joint_prefix = "";

    joint_names.push_back(joint_prefix + "shoulder_1_joint");
    joint_names.push_back(joint_prefix + "shoulder_2_joint");
    joint_names.push_back(joint_prefix + "elbow_1_joint");
    joint_names.push_back(joint_prefix + "wrist_1_joint");
    joint_names.push_back(joint_prefix + "wrist_2_joint");
    joint_names.push_back(joint_prefix + "wrist_3_joint");
    base_frame = joint_prefix + "base_link";
    tool_frame = joint_prefix + "tool0";

    ros::Publisher joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 40);
    joint_msg.name = joint_names;
    ros::Rate loop_rate(40);

    while (ros::ok())
    {

        double robot_state_time;
        std::vector<double> robot_state_vec;

        //Publish JointState
        joint_msg.header.stamp = ros::Time::now();
        
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            joint_msg.position.push_back(jointposition[i]);
            joint_msg.velocity.push_back(jointvelocity[i]);
            joint_msg.effort.push_back(jointeffort[i]);   
        }
        for (unsigned int i = 0; i < joint_msg.position.size(); i++)
        {
            joint_msg.position[i] += joint_offsets[i];
        }

        joint_pub.publish(joint_msg);

        loop_rate.sleep();
        joint_msg.position.clear();
        joint_msg.velocity.clear();
        joint_msg.effort.clear();
/*
        //Broadcast transform (tool0)
        robot_state_time = rros::Time::now();
        quat.setRPY(robot_state_vec[3], robot_state_vec[4], robot_state_vec[5]);
        tool_pose_msg.header.frame_id = base_frame;
        tool_pose_msg.header.stamp = joint_msg.header.stamp;
        tool_pose_msg.pose.position.x = robot_state_vec[0];
        tool_pose_msg.pose.position.y = robot_state_vec[1];
        tool_pose_msg.pose.position.z = robot_state_vec[2];
        tool_pose_msg.pose.orientation.x = quat.x();
        tool_pose_msg.pose.orientation.y = quat.y();
        tool_pose_msg.pose.orientation.z = quat.z();
        tool_pose_msg.pose.orientation.w = quat.w();
        tool_pos_pub.publish(tool_pose_msg);
        transform.setOrigin(tf::Vector3(
                                        robot_state_vec[0],
                                        robot_state_vec[1],
                                        robot_state_vec[2]));
        transform.setRotation(quat);
        tf_bc.sendTransform(tf::StampedTransform(transform, joint_msg.header.stamp, base_frame, tool_frame));
*/
    }
}

bool pinv_QR(const Eigen::MatrixXd &A, Eigen::MatrixXd &invA, Scalar eps)
{
    Eigen::MatrixXd At = A.transpose();
    Eigen::HouseholderQR < Eigen::MatrixXd > qr = At.householderQr();
    int m = A.rows();
    //int n = A.cols();

    Eigen::MatrixXd Rt = Eigen::MatrixXd::Zero(m, m);
    bool invertible;

   Eigen:: MatrixXd hR = (Eigen::MatrixXd) qr.matrixQR();
    Eigen::MatrixXd Y = ((Eigen::MatrixXd) qr.householderQ()).leftCols(m);

    //take the useful part of R
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j <= i; j++)
            Rt(i, j) = hR(j, i);
    }
    Eigen::FullPivLU < Eigen::MatrixXd > invRt(Rt);

    invertible = fabs(invRt.determinant()) > eps;

    if (invertible)
    {
        invA = Y * invRt.inverse();
        return true;
    }
    else
    {
        return false;
    }

}

//A (m x n) usually comes from a redundant task jacobian, therfore we consider m<n
bool pinv(const Eigen::MatrixXd &A, Eigen::MatrixXd &invA, Scalar eps)
{
    int m = A.rows() - 1;
    Eigen::VectorXd sigma;  //vector of singular values

    Eigen::JacobiSVD<Eigen::MatrixXd> svd_A(A.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);
    sigma = svd_A.singularValues();
    if (((m > 0) && (sigma(m) > eps)) || ((m == 0) && (A.array().abs() > eps).any()))
    {
        for (int i = 0; i <= m; i++)
        {
            sigma(i) = 1.0 / sigma(i);
        }
        invA = svd_A.matrixU() * sigma.asDiagonal() * svd_A.matrixV().transpose();
        return true;
    }
    else
    {
        return false;
    }
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
    ROS_INFO("joint 3 position   : %10.4lf %10.4lf %10.4lf",T3[3],T3[7],T3[11]);
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
    double Vmax_cc = VMAX_CC;
    double Survelliance_cc = 2/0.35;  //dangerous zone = 0.2m
    double ShapingFactor_cc = 8;
    double *T3 = new double [16];

    tm_jacobian::Forward_Kinematics_3(q,T3,-0.15);
    Eigen::Vector3d Constrain2TaskJoint;
    Constrain2TaskJoint << T3[3]-ConstrainPoint[0], 0, 0;
    
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

    Eigen::Matrix<double, 6, 6> Geometry_Jacobian = tm_jacobian::Forward_Jacobian_d(q);
    Eigen::Matrix<double, 3, 6> Jacobian_123456   = Geometry_Jacobian.block<3,6>(0,0);



    Eigen::Vector3d TaskVelocity_3;
    std::vector<double> ConstrainedPoint(3);
    double *q_now = new double [6];
    
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        q_now[i] = CurrentPosition[i];

    if(g_obstacle)
        ConstrainedPoint = {-0.3, 0.1, 0.234};
    else
        ConstrainedPoint = {-10.3, 0.1, 0.234};

    Eigen::MatrixXd jacobian_123456_inv;
    pinv_SVD(Jacobian_123456,jacobian_123456_inv);
    JointSpeed = jacobian_123456_inv * EFFSpeed;
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        qd[i] = JointSpeed(i);

    if(OnlineCartesianConstrainGeneration_wall(TaskVelocity_3,ConstrainedPoint,q_now) && g_obstacle) //constrain generated
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
    if(CheckVelocityLimit(qd,ScalingFactor))
        return true;
    else
    {
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            qd[i] = qd[i]/ScalingFactor;
        }
        ROS_WARN("Scaled qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);
        return CheckVelocityLimit(qd,ScalingFactor);
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

    ROS_INFO("Attractive Velocity: %10.4lf ", AttractiveScalar);
    //succeed = tm_jacobian::GetQdfromInverseJacobian(CurrentPosition,EFF_Velocity,action_qd);
    //succeed = GetQdfromLinearJacobian(CurrentPosition,EFF_Velocity,action_qd);  
    succeed = GetQdfromCartesianConstrain(CurrentPosition,EFF_Velocity,action_qd);     

    return succeed;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"tm_sim");
    ros::NodeHandle nh_priv("~");

    bool params_loaded = true;
    double height_z; 

    bool run_succeed = true;
    int  pass = 0;
    double SynchronousTime = 2;
    std::vector<double> TargetPosition, TargetVelocity, CurrentPosition(6), qd(6);
    boost::thread state_publisher;
    double *T = new double [16];
    double *q = new double [6];

    params_loaded *= nh_priv.getParam("obstacle",g_obstacle);
    params_loaded *= nh_priv.getParam("height_z",height_z);
    if(!params_loaded)
    {
        ROS_ERROR("Couldn't find all the required parameters. CLosing...");
        return -1;
    }

    RMLPositionInputParameters  *IP_position = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    RMLVelocityInputParameters  *IP_velocity = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_position->CurrentPositionVector->VecData[i] = 0.0;
        IP_position->CurrentVelocityVector->VecData[i] = 0.0;
        IP_position->CurrentAccelerationVector->VecData[i] = 0.0;

        IP_velocity->CurrentPositionVector->VecData[i] = 0.0;
        IP_velocity->CurrentVelocityVector->VecData[i] = 0.0;
        IP_velocity->CurrentAccelerationVector->VecData[i] = 0.0;
    }

    state_publisher = boost::thread(publishMsgRT);

    std::vector<double> InitialPosition = {0,0.1137,1.9517,-2.0654,1.5708,0}; 
    //std::vector<double> InitialPosition = {-2.4836*DEG2RAD,11.2613*DEG2RAD,90.8002*DEG2RAD,-129.1609*DEG2RAD,99.8453*DEG2RAD,0.0000}; 
    TargetVelocity = {0, 0, 0, 0, 0, 0};
    if(!ReflexxesPositionRun_sim(*IP_position, InitialPosition, TargetVelocity, SynchronousTime))
        return -1;

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_velocity->CurrentPositionVector->VecData[i]     = IP_position->CurrentPositionVector->VecData[i];
        IP_velocity->CurrentVelocityVector->VecData[i]     = IP_position->CurrentVelocityVector->VecData[i];
        IP_velocity->CurrentAccelerationVector->VecData[i] = IP_position->CurrentAccelerationVector->VecData[i];
    }
/*
    double alpha = 5.0*DEG2RAD;
    double theta = 0.0;
    double time_interval = 0;
    double radius = 0.5;

    while(ros::ok())
    {
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            q[i] = IP_velocity->CurrentPositionVector->VecData[i];
            CurrentPosition[i] = IP_velocity->CurrentPositionVector->VecData[i];
        }

        theta += alpha*time_interval;
        TargetVelocity={radius*alpha*cos(theta),radius*alpha*sin(theta),0};

        time_interval += 0.2;
        if(theta > 360*DEG2RAD)
            theta -= 360*DEG2RAD;

        if(GetQdfromLinearJacobian(CurrentPosition,TargetVelocity,qd))
        {
            pass = ReflexxesVelocityRun_sim(*IP_velocity, qd ,0.2);
            if(pass == STOP)
                break;
        }
        else
            break;
    }
    
    */

    //Test for joint 3
    //height_z = 0.55 default
    int state = 1;
    double left_y = 0.16;


    while(ros::ok())
    {
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            q[i] = IP_velocity->CurrentPositionVector->VecData[i];
        }

        if(state == 1)
        {
            TargetPosition = {-0.1372, -0.1222, height_z, 0, 0, 0}; //0.5801
        }
        else if(state == 2)
        {
            TargetPosition = {-0.1372, left_y, height_z, 0, 0, 0};
        }
        else if(state == 3)
        {
            TargetPosition = {0.4259, left_y, height_z, 0, 0, 0};
        }
        else if(state == 4)
        {
            TargetPosition = {-0.1372, left_y, height_z, 0, 0, 0};
        }
        else if(state == 5)
        {
            TargetPosition = {-0.1372, -0.1222, height_z, 0, 0, 0};
        }
        else if(state == 6)
        {
            TargetPosition = {0.4259, -0.1222, height_z, 0, 0, 0};
        }
        else{}
        
        if(state != 7)  //point-to-point motion
        {
            tm_kinematics::forward(q,T);

            if(OnlineAttractiveForceGeneration(qd, T, q, TargetPosition, TargetVelocity))
            {
                pass = ReflexxesVelocityRun_sim(*IP_velocity, qd, TargetPosition, TargetVelocity ,0.2);

                if(pass == REACH)
                    state++;
                else if(pass == STOP)
                    break;
            }
            else
                break;
        }
        else    // initialization
        {
            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            {
                IP_position->CurrentPositionVector->VecData[i]     = IP_velocity->CurrentPositionVector->VecData[i];
                IP_position->CurrentVelocityVector->VecData[i]     = IP_velocity->CurrentVelocityVector->VecData[i];
                IP_position->CurrentAccelerationVector->VecData[i] = IP_velocity->CurrentAccelerationVector->VecData[i];
            }
            ReflexxesPositionRun_sim(*IP_position, InitialPosition, TargetVelocity, 0.7);
            
            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            {
                IP_velocity->CurrentPositionVector->VecData[i]     = IP_position->CurrentPositionVector->VecData[i];
                IP_velocity->CurrentVelocityVector->VecData[i]     = IP_position->CurrentVelocityVector->VecData[i];
                IP_velocity->CurrentAccelerationVector->VecData[i] = IP_position->CurrentAccelerationVector->VecData[i];
            }
            state = 1;
        }        
    }

/*  box testing
    int state = 1;
    while(ros::ok())
    {
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            q[i] = IP_velocity->CurrentPositionVector->VecData[i];
        }
        
        if(state == 1)
            TargetPosition = {0.45, -0.4055, 0.20, 0, 0, 0};
        else if(state == 2)
            TargetPosition = {0.45, 0, 0.20, 0, 0, 0};
        else if(state == 3)
            TargetPosition = {0.45, 0.4055, 0.20, 0, 0, 0};
        else if(state == 4)
            TargetPosition = {0.45, 0, 0.20, 0, 0, 0};
        else if(state == 5)
            TargetPosition = {0.45, -0.4055, 0.20, 0, 0, 0};
        else{}

        if(state != 6)
        {
            tm_kinematics::forward(q,T);
            if(OnlineAttractiveForceGeneration(qd, T, q, TargetPosition, TargetVelocity))
            {
                pass = ReflexxesVelocityRun_sim(*IP_velocity, qd, TargetPosition, TargetVelocity ,0.2);

                if(pass == REACH)
                    state++;
                else if(pass == STOP)
                    break;
            }
            else
                break;
        }
        else
        {
            state = 1;
        }

    }
    */
/*
    while(ros::ok())
    {
        TargetPosition = {0.0000,    -1.4887,     1.7361,    -0.2474,     1.5708,     0.0000};
        if(!ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
            break;

        TargetPosition = {   -1.0231,    -1.5074,     1.6541,    -0.1467,     2.5939,    -0.0000};
        if(!ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
            break;
        
        TargetPosition = {0,0,1.5708,-1.5708,1.5708,0};
        if(!ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
            break;

        //TargetPosition = {-0.0006,    -0.0172,    -1.6890,     1.7062,     1.5714,    -0.0000};
        //run_succeed = ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        //getchar();
    }
    */
    state_publisher.join();

    delete IP_position;
    delete IP_velocity;
    delete [] T;
    delete [] q;

    return 0;
}
