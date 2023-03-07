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

#include "tm_reflexxes/tm_reflexxes.h"
#include "tm_kinematics/tm_kin.h"

#include <eigen3/Eigen/Dense>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>



#define DEG2RAD 0.01745329252
#define RAD2DEG 57.29577951

#define CYCLE_TIME_IN_SECONDS                   0.025
#define NUMBER_OF_DOFS                          6

#define MAX_ACC 0.0375*40 // 0.0375 : acc in 25ms
#define DANGEROUS_ZONE 0.3
#define JOINTLIMIT_SPD_123 90
#define JOINTLIMIT_SPD_456 150

using namespace std;

std::vector<double> jointposition(6), jointvelocity(6), jointeffort(6);

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
//  fn        : ReflexxesVelocityRun_sim()
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
        printf("Input values are valid!\n");
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
        ROS_INFO("position: ");
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            ROS_INFO("%10.4lf ", OP->NewPositionVector->VecData[i]);
            jointposition[i] = OP->NewPositionVector->VecData[i];
        }

        ROS_INFO("velocity: ");

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            ROS_INFO("%10.4lf ", OP->NewVelocityVector->VecData[i]);
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
        IP->MaxVelocityVector->VecData[i]     = 0.3247; //0.3247
        IP->MaxAccelerationVector->VecData[i] = MAX_ACC;
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
        ROS_INFO("position: ");
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            ROS_INFO("%10.4lf ", OP->NewPositionVector->VecData[i]);
            jointposition[i] = OP->NewPositionVector->VecData[i];
        }

        ROS_INFO("velocity: ");

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            ROS_INFO("%10.4lf ", OP->NewVelocityVector->VecData[i]);
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

    printf("=============== Final state position =========================\n");

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        ROS_INFO("%10.4lf ", IP->CurrentPositionVector->VecData[i]);

    ROS_INFO("Finished in %llu us", tt);
    tm_reflexxes::resetTermios();
    InputState = *IP;

    delete  RML;
    delete  IP;
    delete  OP;

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

int main(int argc, char **argv)
{
    ros::init(argc, argv,"tm_otg");

    bool run_succeed = true;
    double SynchronousTime = 0.5;
    std::vector<double> TargetPosition, TargetVelocity;
    boost::thread state_publisher;

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

    ROS_INFO("Press enter to start robot motion");
    getchar();

    while(ros::ok())
    {
        if (run_succeed)
        {
            //TargetPosition = {0,0,1.57,-1.57,1.57,0};
            TargetPosition = {3.8994*DEG2RAD,-36.3668*DEG2RAD,114.6547*DEG2RAD,15.1389*DEG2RAD,89.0517*DEG2RAD,2.3467*DEG2RAD};
            //TargetPosition = {0,0,90*DEG2RAD,0,90*DEG2RAD,0};
            //TargetPosition ={ 2.3343,     1.4075,     2.1695,     0.9385,     1.4980,    -0.8227};

            TargetVelocity = {0, 0, 0, 0, 0, 0};
            run_succeed = ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        }
        else
            break;

        if (run_succeed)
        {
            TargetPosition = {0.0316,     0.4700,     0.8690,     0.6110,     1.6354,     1.1903};

            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        }
        else
            break;

        if (run_succeed)
        {
            TargetPosition = {0.0480,     0.3052,     1.4847,     0.1591,     1.6415,     1.2056};
            
            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        }
        else
            break;
    }

    state_publisher.join();

    delete IP_position;
    delete IP_velocity;

    return 0;
}
