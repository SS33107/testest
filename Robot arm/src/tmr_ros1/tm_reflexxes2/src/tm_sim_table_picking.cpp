/*********************************************************************
 * tm_sim_picking.cpp
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


#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>

#include "tm_reflexxes/tm_reflexxes.h"
#include "tm_kinematics/tm_kin.h"


#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>



#define DEG2RAD 0.01745329252
#define RAD2DEG 57.29577951
#define REPEATABILITY 0.00005//0.01
//#define REPEATABILITY 0.01


#define CYCLE_TIME_IN_SECONDS                   0.025
#define NUMBER_OF_DOFS                          6

#define MAX_ACC 0.0375*40 // 0.0375 : acc in 25ms
#define DANGEROUS_ZONE 0.3
#define JOINTLIMIT_SPD_123 150
#define JOINTLIMIT_SPD_456 200

#define GRIPPER_LENGTH 0.235

#define STOP 0
#define PASS 1
#define REACH 2

#define VMAX_ATT 0.08
#define VMAX_REP 0.2
#define VMAX_CC  0.03

#define IMAGE_CAPTURE_POSE 1
#define GRASP_READY_POSE   2
#define OBJECT_POSITION    3
#define GRASP_AFTER_POSE   4
#define PLACING_POSE_ABOVE 5
#define PLACING_POSE       6
#define MID_POSE           7

using namespace std;

typedef float Scalar;
const float EPS = 1e-6;
const float LAMBDA_MAX = 0.3;
const float EPSQ = 1e-15;

double last_point_x = 0.0;
double last_point_y = 0.0;
double last_point_z = 0.0;
bool   gParam_obstacle;
bool   gParam_vision;

double g_distance;
std::vector<double> g_obstacle_velocity(3);

std::vector<double> g_obstacle_position(3);
std::vector<double> g_constraint_position(3);

std::vector<double> jointposition(6), jointvelocity(6), jointeffort(6), toolposition(3);
double g_objposition[7];
bool position_fill = false;
bool rotation_fill = false;

void publishMsgRT();

/*
void distance_callback(const std_msgs::Float32::ConstPtr& distance)
{
    g_distance = distance->data;
    //ROS_INFO("minimum_distance = %10.3f",distance->data);
}
void velocity_callback(const geometry_msgs::Twist::ConstPtr& velocity)
{
    g_obstacle_velocity[0] = velocity->linear.x;
    g_obstacle_velocity[1] = velocity->linear.y;
    g_obstacle_velocity[2] = velocity->linear.z;
    //ROS_INFO("vx = %10.3f, vy = %10.3f, vz = %10.3f", velocity->linear.x, velocity->linear.y, velocity->linear.z);
}
*/

void position_callback(const geometry_msgs::PointStamped::ConstPtr& closest_pt)
{
    g_obstacle_position[0] = closest_pt->point.x;
    g_obstacle_position[1] = closest_pt->point.y;
    g_obstacle_position[2] = closest_pt->point.z;
    //ROS_INFO("x =  %10.3f, y =  %10.3f, z =  %10.3f", closest_pt->point.x, closest_pt->point.y, closest_pt->point.z);
}

void constraint_callback(const geometry_msgs::PointStamped::ConstPtr& closest_bpt)
{
    g_constraint_position[0] = closest_bpt->point.x;
    g_constraint_position[1] = closest_bpt->point.y;
    g_constraint_position[2] = closest_bpt->point.z;
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

        //tm_kinematics::forward(q,T);
        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);
        toolposition[0] = T[3];
        toolposition[1] = T[7];
        toolposition[2] = T[11];
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

//    ROS_INFO("========== Final state velocity based position =============");
//    ROS_INFO("pass = %d  tool0 : %10.4lf %10.4lf %10.4lf", pass, T[3], T[7], T[11]);
//    ROS_INFO("Joint position  : %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", q[0]*RAD2DEG,q[1]*RAD2DEG,q[2]*RAD2DEG,q[3]*RAD2DEG,q[4]*RAD2DEG,q[5]*RAD2DEG);
    //print_info("Finished in %llu us", tt);

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
        //tm_kinematics::forward(q,T);
        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);
        //ROS_INFO("gripper XYZ_pos: %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);
        toolposition[0] = T[3];
        toolposition[1] = T[7];
        toolposition[2] = T[11];

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
    std::vector<std::string> joint_names;
    std::string base_frame;
    std::string tool_frame;
    std::string joint3_frame;
    sensor_msgs::JointState joint_msg;
    geometry_msgs::Point tool_msg;
    geometry_msgs::Point joint3_msg;

    static tf::TransformBroadcaster tf_bc;
    geometry_msgs::PoseStamped joint3_pose_msg;
    tf::Quaternion quat;
    tf::Transform transform;

    double *q  = new double [6];
    double *T3 = new double [16];

    joint_offsets.assign(6, 0.0);
    std::string joint_prefix = "";

    joint_names.push_back(joint_prefix + "shoulder_1_joint");
    joint_names.push_back(joint_prefix + "shoulder_2_joint");
    joint_names.push_back(joint_prefix + "elbow_1_joint");
    joint_names.push_back(joint_prefix + "wrist_1_joint");
    joint_names.push_back(joint_prefix + "wrist_2_joint");
    joint_names.push_back(joint_prefix + "wrist_3_joint");
    base_frame   = joint_prefix + "base_link";
    tool_frame   = joint_prefix + "tool0";
    joint3_frame = joint_prefix + "joint3_link";

    ros::Publisher joint_pub   = nh_.advertise<sensor_msgs::JointState>("joint_states", 40);
    ros::Publisher tool_pub    = nh_.advertise<geometry_msgs::Point>("tool_states", 40);
    ros::Publisher joint3_pub  = nh_.advertise<geometry_msgs::Point>("joint3_xyz", 40);

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
            q[i] = jointposition[i];
            joint_msg.position.push_back(jointposition[i]);
            joint_msg.velocity.push_back(jointvelocity[i]);
            joint_msg.effort.push_back(jointeffort[i]);   
        }
        for (unsigned int i = 0; i < joint_msg.position.size(); i++)
        {
            joint_msg.position[i] += joint_offsets[i];
        }
        tm_jacobian::Forward_Kinematics_3(q,T3,-0.15);

        tool_msg.x = toolposition[0];
        tool_msg.y = toolposition[1];
        tool_msg.z = toolposition[2];

        joint3_msg.x = T3[3];
        joint3_msg.y = T3[7];
        joint3_msg.z = T3[11];

        joint_pub.publish(joint_msg);
        tool_pub.publish(tool_msg);
        joint3_pub.publish(joint3_msg);

        loop_rate.sleep();
        joint_msg.position.clear();
        joint_msg.velocity.clear();
        joint_msg.effort.clear();

        //Broadcast transform (joint 3)
        transform.setOrigin(tf::Vector3(T3[3], T3[7], T3[11]));
        quat.setRPY(0, 0, 0);
        transform.setRotation(quat);
        tf_bc.sendTransform(tf::StampedTransform(transform, joint_msg.header.stamp, base_frame, joint3_frame));

    }

    delete [] q;
    delete [] T3;
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

double OnlineCartesianConstrainGeneration_SNS(std::vector<double> ConstrainPoint, 
                                             double *q) 
{
    double Vmax_cc = 1;
    double Survelliance_cc = 2/0.27;  //dangerous zone = 0.2m
    double ShapingFactor_cc = 8;
    double *T3 = new double [16];

    tm_jacobian::Forward_Kinematics_3(q,T3,-0.15);
    Eigen::Vector3d Constrain2TaskJoint;
    //Constrain2TaskJoint << 0,0,T3[11]-ConstrainPoint[2];    // for z plane
    Constrain2TaskJoint << 0,0,T3[3]-ConstrainPoint[0];    // for x plane
    
    double dis_constrain = sqrt(Constrain2TaskJoint.dot(Constrain2TaskJoint));
    double CartesianInfluence  = Vmax_cc / (1 + exp((dis_constrain*Survelliance_cc-1)*ShapingFactor_cc));

    ROS_INFO("CartesianInfluence : %10.4lf ",CartesianInfluence);
    ROS_WARN("joint 3 position   : %10.4lf %10.4lf %10.4lf",T3[3],T3[7],T3[11]);

    delete [] T3;
    return (1-CartesianInfluence);
}

bool GetQdfromCartesianConstrain_SNS(   std::vector<double> CurrentPosition,
                                        std::vector<double> EFF_Velocity, 
                                        std::vector<double>& qd)
{
    Eigen::Matrix<float , 6, 1> home,q;
    Eigen::Matrix<double , 6, 1> JointSpeed;
    Eigen::Matrix<double, 3, 1> EFFSpeed;
    double RepulsiveInfluence;

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

    if(!gParam_obstacle)
        ConstrainedPoint = {-10.3, 0.1, -10.0};
    else
        ConstrainedPoint = {-0.3, 0.1, 0.0}; //-0.3, 0.1, 0.234

    Eigen::MatrixXd jacobian_123456_inv;
    pinv_SVD(Jacobian_123456,jacobian_123456_inv);
    JointSpeed = jacobian_123456_inv * EFFSpeed;
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        qd[i] = JointSpeed(i);

    RepulsiveInfluence = OnlineCartesianConstrainGeneration_SNS(ConstrainedPoint,q_now);

    Eigen::Matrix<double, 3, 5> Jacobian_13456 = Jacobian_123456.block<3,5>(0,1);
    Eigen::Matrix<double, 3, 1> Jacobian_1     = Jacobian_123456.block<3,1>(0,0);
    Eigen::Matrix<double, 3, 1> Jacobian_2     = Jacobian_123456.block<3,1>(0,1);

    Jacobian_13456(0,0) = Jacobian_123456(0,0);
    Jacobian_13456(1,0) = Jacobian_123456(1,0);
    Jacobian_13456(2,0) = Jacobian_123456(2,0);


    double qd_2 = qd[1]*RepulsiveInfluence;

    Eigen::Matrix<double, 3, 1> linear_velocity_temp = EFFSpeed - Jacobian_2*qd_2;

    Eigen::MatrixXd Jacobian_13456_inv;
    if(!pinv_SVD(Jacobian_13456,Jacobian_13456_inv))
        return false;
    Eigen::VectorXd qd_13456 = Jacobian_13456_inv*linear_velocity_temp;

    qd[0] = qd_13456(0);
    qd[1] = qd_2;
    qd[2] = qd_13456(1);
    qd[3] = qd_13456(2);
    qd[4] = qd_13456(3);
    qd[5] = qd_13456(4);

    ROS_WARN("qd_2   :      %10.4lf",qd_2*RAD2DEG);
    ROS_WARN("qd_3456 :     %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf",qd_13456(0)*RAD2DEG,qd_13456(1)*RAD2DEG,qd_13456(2)*RAD2DEG,qd_13456(3)*RAD2DEG,qd_13456(4)*RAD2DEG);
    
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

bool OnlineVFCGeneration( Eigen::Vector3d &TaskVelocity, double *q) 
{
    double Vmax_cc = 1;
    double Survelliance_cc = 2/0.27;  //dangerous zone = 0.27m
    double ShapingFactor_cc = 8;
    double *T3 = new double [16];
    double ConstrainedPlane_z  = 0.0;    //table
    double dis_constrain = 0;
    double CartesianInfluence;
    std::vector<double> ConstrainedPoint(3);


    if(gParam_vision)
        ConstrainedPoint = g_constraint_position;
    else
        ConstrainedPoint = { 100,    100,     100};

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
    double *T3 = new double [16];
    
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        q_now[i] = CurrentPosition[i];

    Eigen::MatrixXd jacobian_123456_inv;
    pinv_SVD(Jacobian_123456,jacobian_123456_inv);
    JointSpeed = jacobian_123456_inv * EFFSpeed;
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        qd[i] = JointSpeed(i);

    qd12_original(0) = qd[0];
    qd12_original(1) = qd[1];

    if(OnlineVFCGeneration(VirtualForce,q_now) && gParam_obstacle) //constrain generated
    {
        Eigen::Matrix<double, 3, 4> Jacobian_3456     = Jacobian_123456.block<3,4>(0,2);
        Eigen::Matrix<double, 3, 2> Jacobian_12       = Jacobian_123456.block<3,2>(0,0);
        Eigen::Matrix<double, 3, 2> TaskJacobian_3    = tm_jacobian::Forward_Linear_Jacobian_3(q);
        Eigen::Vector3d OriginalForce = TaskJacobian_3* qd12_original;

        Eigen::MatrixXd TaskJacobian_3_inv;
        if(!pinv_SVD(TaskJacobian_3, TaskJacobian_3_inv))
            return false;

        //double ConstrainedForce = OriginalForce.dot(VirtualForce);
        //VirtualForce(2) = ConstrainedForce;
        //Eigen::Vector3d Constrained_Velocity = OriginalForce + VirtualForce;
//        ROS_INFO("OriginalForce      : %10.4lf %10.4lf %10.4lf ",OriginalForce(0),OriginalForce(1),OriginalForce(2));

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
        tm_jacobian::Forward_Kinematics_3(q_now,T3,-0.15);
        //ROS_INFO("j3_pos : %10.4lf %10.4lf %10.4lf VFC : %10.4lf %10.4lf %10.4lf",T3[3],T3[7],T3[11],temp(0),temp(1),temp(2));

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
    
        //ROS_WARN("qd_12   :     %10.4lf %10.4lf",qd_12(0)*RAD2DEG,qd_12(1)*RAD2DEG);
        //ROS_WARN("qd_3456 :     %10.4lf %10.4lf %10.4lf %10.4lf",qd_3456(0)*RAD2DEG,qd_3456(1)*RAD2DEG,qd_3456(2)*RAD2DEG,qd_3456(3)*RAD2DEG);
    }
    
    //ROS_INFO("Generatd qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);

    double ScalingFactor;
    if(CheckVelocityLimit(qd,ScalingFactor))
        return true;
    else
    {
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            qd[i] = qd[i]/ScalingFactor;
        }
        //ROS_WARN("Scaled qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);
        return CheckVelocityLimit(qd,ScalingFactor);
    }
}



bool OnlineCartesianConstrainGeneration_wall(Eigen::Vector3d &TaskVelocity,
                                             std::vector<double> ConstrainPoint, 
                                             double *q) 
{
    double Vmax_cc = VMAX_CC;
    double Survelliance_cc = 2/0.27;  //dangerous zone = 0.2m
    double ShapingFactor_cc = 8;
    double *T3 = new double [16];

    tm_jacobian::Forward_Kinematics_3(q,T3,-0.15);
    Eigen::Vector3d Constrain2TaskJoint;
    Constrain2TaskJoint << T3[3]-ConstrainPoint[0], 0, 0; // for x plane : wall
    //Constrain2TaskJoint << 0,0,T3[11]-ConstrainPoint[2];    // for z plane : table
    
    double dis_constrain = sqrt(Constrain2TaskJoint.dot(Constrain2TaskJoint));
    double CartesianInfluence  = Vmax_cc / (1 + exp((dis_constrain*Survelliance_cc-1)*ShapingFactor_cc));

    if((T3[7] - last_point_y) >= 0)
        TaskVelocity << 0., CartesianInfluence, 0.;
    else
        TaskVelocity << 0., -CartesianInfluence, 0.;

    ROS_WARN("joint 3 position   : %10.4lf %10.4lf %10.4lf",T3[3],T3[7],T3[11]);
    //ROS_INFO("TaskVelocity       : %10.4lf %10.4lf %10.4lf",TaskVelocity(0),TaskVelocity(1),TaskVelocity(2));

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

    if(!gParam_obstacle)
        ConstrainedPoint = {-10.3, 0.1, -10.0};
    else
        ConstrainedPoint = {-0.3, 0.1, 0.0}; //-0.3, 0.1, 0.234

    Eigen::MatrixXd jacobian_123456_inv;
    pinv_SVD(Jacobian_123456,jacobian_123456_inv);
    JointSpeed = jacobian_123456_inv * EFFSpeed;
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        qd[i] = JointSpeed(i);

    if(OnlineCartesianConstrainGeneration_wall(TaskVelocity_3,ConstrainedPoint,q_now) && gParam_obstacle) //constrain generated
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
//    ROS_WARN("Attractive Velocity: %10.4lf ", AttractiveScalar);

    //succeed = tm_jacobian::GetQdfromInverseJacobian(CurrentPosition,EFF_Velocity,action_qd);
    //succeed = GetQdfromLinearJacobian(CurrentPosition,EFF_Velocity,action_qd);  
    
    //succeed = GetQdfromCartesianConstrain(CurrentPosition,EFF_Velocity,action_qd); 
    
    //succeed = GetQdfromCartesianConstrain_SNS(CurrentPosition,EFF_Velocity,action_qd);
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

    if(gParam_vision)
        ObstaclePoint = g_obstacle_position;
    else
        ObstaclePoint = {100, 100, 100}; //0.65 -0.186 0.28

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
    //succeed = GetQdfromLinearJacobian(CurrentPosition,EFF_Velocity,Repulsive_qd);   
    succeed = GetQdfromVirtualForceConstrain(CurrentPosition,EFF_Velocity,Repulsive_qd);   

    return succeed;

}

bool OnlinePotentialForceGeneration(std::vector<double>& Potential_qd,
                                    std::vector<double>& PotentialVelocity,
                                    std::vector<double> AttractiveVelocity,
                                    std::vector<double> RepulsiveVelocity,
                                    double *q)
{
    std::vector<double> CurrentPosition(6);

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        PotentialVelocity[i] = AttractiveVelocity[i] + RepulsiveVelocity[i];
        CurrentPosition[i] = q[i];
    }

    return GetQdfromVirtualForceConstrain(CurrentPosition,PotentialVelocity,Potential_qd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"tm_sim_gripper");
    ros::NodeHandle nh_priv("~");

    bool params_loaded = true;
    bool run_succeed = true;
    double height_z;
    
    int  pass = 0;
    double SynchronousTime = 5;
    double Speed = 0.321/0.18;
    std::vector<double> TargetPosition(6), TargetVelocity, CurrentPosition(6), qd(6), Attractive_qd(6), Repulsive_qd(6), RepulsiveVelocity(6),PotentialVelocity(6);
    boost::thread state_publisher;
    double *T = new double [16];
    double *q = new double [6];
    std_msgs::Int32 robot_motion_states;


    params_loaded *= nh_priv.getParam("height_z",height_z);
    params_loaded *= nh_priv.getParam("obstacle",gParam_obstacle);
    params_loaded *= nh_priv.getParam("vision"  , gParam_vision);

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
    ros::Publisher RobotState = nh_priv.advertise<std_msgs::Int32>("/robot_motion_states",10);

    ros::AsyncSpinner spinner(6);
    //ros::Subscriber distance_sub = nh_priv.subscribe("/kinect_merge/minimum_distance",    1,distance_callback);
    //ros::Subscriber velocity_sub = nh_priv.subscribe("/kinect_merge/closest_vel_tracking",1,velocity_callback);
    ros::Subscriber position_sub    = nh_priv.subscribe("/eff/kinect_merge/closest_pt_tracking", 1,position_callback);
    ros::Subscriber constraint_sub  = nh_priv.subscribe("/body/kinect_merge/closest_pt_tracking", 1,constraint_callback);
    ros::Subscriber ObjPosition_sub = nh_priv.subscribe("/object_position", 1,ObjPosition_callback);
    ros::Subscriber ObjRotation_sub = nh_priv.subscribe("/object_rotation", 1,ObjRotation_callback);
    ros::Subscriber MotionLock_sub  = nh_priv.subscribe("/lock_motion", 1,MotionLock_callback);
    spinner.start();

    //* ImageCpature(lower)  {0.0615,    -0.6537,     2.0253,     0.2087,     1.5775,     0.0556};
    //* ImageCpature position (XYZ) = [   0.2172 -0.1083  0.3525](m)
    //* ImageCpature rotation (XYZ) = [  -3.1317 -0.0062  1.5766](rad)
    //* ImageCpature rotation (XYZ) = [-179.4327 -0.3552 90.3323](deg)

    //* ImageCpature(higher) {0.0614,    -0.5497,     1.6462,     0.4838,     1.5775,     0.0556};
    //* ImageCpature position (XYZ) = [   0.2172 -0.1083  0.4525](m)
    //* ImageCpature rotation (XYZ) = [  -3.1317 -0.0062  1.5766](rad)
    //* ImageCpature rotation (XYZ) = [-179.4327 -0.3552 90.3323](deg)
    std::vector<double> InitialPosition = {0.1169,     1.1010,    -0.8864,    -0.8015,     1.6174,     0.0000}; //for hexagon2
    std::vector<double> RecoverPosition = {0.1169,     1.1010,    -0.8864,    -0.8015,     1.6174,     0.0000}; //for hexagon2
 
 //fix  above yellow box
    std::vector<double> ImageCpature = {0.0691,    -0.6283,     1.9241,     0.2848,     1.5714,     0.0632};


    TargetVelocity = {0, 0, 0, 0, 0, 0};
    /*if(!ReflexxesPositionRun_sim(*IP_position, ImageCpature, TargetVelocity, SynchronousTime))
        return -1;
    else
    {
        robot_motion_states.data = IMAGE_CAPTURE_POSE;
        RobotState.publish(robot_motion_states);
    }*/

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
                SynchronousTime = 1;
                robot_motion_states.data = 2;
                if(!ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
            }
            else if(robot_motion_states.data == 2)
            {
                //pose
                SynchronousTime = 2;
                robot_motion_states.data = 3;
                if(!ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;  
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
            }
            else if(robot_motion_states.data == 3)
            {
                //table prepare
                SynchronousTime = 1.3;
                robot_motion_states.data = 4;
                if(!ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;  
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
            }
            else if(robot_motion_states.data == 4)
            {
                //box prepare
                SynchronousTime = 3;
                robot_motion_states.data = 5;
                
                if(!ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;  
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
            }
            else if(robot_motion_states.data == 5)
            {
                //put
                SynchronousTime = 1;
                robot_motion_states.data = 6;
                if(!ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;  
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
                
            }
            else if(robot_motion_states.data == 6)
            {
                //mid
                SynchronousTime = 1;
                robot_motion_states.data = 7;
                if(!ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;  
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
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
            SynchronousTime = 3;
            if(!ReflexxesPositionRun_sim(*IP_position, ImageCpature, TargetVelocity, SynchronousTime))
                    break;
                robot_motion_states.data = IMAGE_CAPTURE_POSE;
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
                RobotState.publish(robot_motion_states);
                g_objposition[0] = 0.0;
        }
        else{}
    }

    state_publisher.join();

    delete IP_position;
    delete IP_velocity;
    delete [] T;
    delete [] q;

    return 0;
}
