/*********************************************************************
 * tm_sim_picking_safe.cpp
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
#define DANGEROUS_ZONE 0.15
#define JOINTLIMIT_SPD_123 150
#define JOINTLIMIT_SPD_456 200

#define GRIPPER_LENGTH 0.245

#define STOP 0
#define PASS 1
#define REACH 2

#define VMAX_ATT 0.08
#define VMAX_REP 0.3   //0.4
#define VMAX_CC  0.03

#define IMAGE_CAPTURE_POSE 1
#define GRASP_READY_POSE   2
#define OBJECT_POSITION    3
#define GRASP_AFTER_POSE   4
#define PLACING_POSE       5
#define PLACING_POSE_DOWN  6
#define MID_POSE           7

#define SMOOTHSTOP 0
#define REACHPOINT 1
#define INTERRUPT  2

#define OBSTACLE_GPR false
#define V_TRAVEL 0.1

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
double min_distance;
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
    
    //succeed = GetQdfromCartesianConstrain(CurrentPosition,EFF_Velocity,action_qd); 
    
    //succeed = GetQdfromCartesianConstrain_SNS(CurrentPosition,EFF_Velocity,action_qd);
    //succeed = GetQdfromVirtualForceConstrain(CurrentPosition,EFF_Velocity,action_qd);

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
    
    if(RepulsiveForce < 0.001)
        RepulsiveForce = 0.0;

    Eigen::Vector3d RepulsiveVelocity  = RepulsiveForce *RepulsiveVector;

    //ROS_WARN("Force: %lf , dis :%10.4lf  ?????????????????????????",RepulsiveForce, dis_repuslive);

    std::vector<double> CurrentPosition = {q[0], q[1], q[2], q[3], q[4], q[5]}; 
    EFF_Velocity = {RepulsiveVelocity(0), RepulsiveVelocity(1), RepulsiveVelocity(2), 0,0,0}; 
    ROS_WARN("Repulsive  Velocity: %10.4lf ", sqrt(RepulsiveVelocity.dot(RepulsiveVelocity)));
    
    //succeed = tm_jacobian::GetQdfromInverseJacobian(CurrentPosition,EFF_Velocity,Repulsive_qd);   
    succeed = GetQdfromLinearJacobian(CurrentPosition,EFF_Velocity,Repulsive_qd);   
    //succeed = GetQdfromVirtualForceConstrain(CurrentPosition,EFF_Velocity,Repulsive_qd);   

    return succeed;

}

bool ReflexxesVelocityInterrupt_sim(RMLVelocityInputParameters &InputState, 
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

        //***************************************************************
        // Print out commands

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            q[i] = OP->NewPositionVector->VecData[i];
            jointposition[i] = OP->NewPositionVector->VecData[i];
        }
        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);
        toolposition[0] = T[3];
        toolposition[1] = T[7];
        toolposition[2] = T[11];

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            jointvelocity[i] = OP->NewVelocityVector->VecData[i];
            jointeffort[i] = OP->NewAccelerationVector->VecData[i];
        }
//fix
        //ROS_WARN("tool XYZ_pos: %10.4lf %10.4lf  [ %lf ] ", T[3], T[7], T[11]);

        //***************************************************************
        
        if(!CheckJointLimit(q))
        {
            ROS_WARN("Joint position execced limit!!");
            //tm_reflexxes::ReflexxesSmoothStop_sim(*IP, 0.25);
            pass = false;
            break;
        }

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

int ReflexxesPositionSafetyRun_sim( RMLPositionInputParameters &InputState, 
                                    std::vector<double> TargetPosition,
                                    std::vector<double> TargetVelocity, 
                                    double SynTime)
{
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

    std::vector<double> PotentialField_qd(6), qd_now(6),RepulsiveVelocity(6), Attractive_qd(6), Attractive_velocity(6), TargetPoint(3);
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


    if(gParam_vision)
    {
        g_distance = sqrt(pow((g_obstacle_position[0] - T[3]),2)+pow((g_obstacle_position[1] - T[7]),2)+pow((g_obstacle_position[2] - T[11]),2));
    }
    else
        g_distance = 100;

    //  ********************************************************************/
    //  Assigning all RMLPositionInputParameters : 
    //  Current POS, VEL, ACC : set before call ReflexxesPositionRun
    //  Target POS, VEL       : set before call ReflexxesPositionRun
    //  Max VEL, ACC          : set after call ReflexxesPositionRun
    //  SelectionVector       : set after call ReflexxesPositionRun
    //  ********************************************************************
    *IP = InputState;
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP->MaxVelocityVector->VecData[i]     = 0.5*40; //0.3247
        IP->MaxAccelerationVector->VecData[i] = 0.0375*40;;
        IP->TargetPositionVector->VecData[i]  = TargetPosition[i]; 
        IP->TargetVelocityVector->VecData[i]  = TargetVelocity[i];
        IP->SelectionVector->VecData[i] = true;
    }
//fix
    ROS_WARN("Distance : %lf", g_distance);
    if(g_distance < 0.5)
        SynTime*=2;
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

        //***************************************************************
        // Print out commands

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            q[i] = OP->NewPositionVector->VecData[i];
            jointposition[i] = OP->NewPositionVector->VecData[i];
        }
        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);
        toolposition[0] = T[3];
        toolposition[1] = T[7];
        toolposition[2] = T[11];

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            jointvelocity[i] = OP->NewVelocityVector->VecData[i];
            jointeffort[i] = OP->NewAccelerationVector->VecData[i];
        }
        
        if(!CheckJointLimit(q))
        {
            ROS_WARN("Joint position execced limit!!");
            //tm_reflexxes::ReflexxesSmoothStop_sim(IP, 0.25);
            pass = SMOOTHSTOP;
            break;
        }

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
            if(OnlineRepulsiveForceGeneration(PotentialField_qd,T, q, RepulsiveVelocity))
            {
                for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                {
                    PotentialField_qd[i] += Attractive_qd[i];
                }

                if(CheckVelocityLimit(PotentialField_qd,ScalingFactor))
                {
                    ReflexxesVelocityInterrupt_sim(*IP_interrupt,PotentialField_qd,0.2);
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
                    //tm_reflexxes::ReflexxesSmoothStop_sim(*IP_interrupt, 0.1);
                    break;
                }
            }
            else
            {
                ROS_WARN("Joint velocity out of range!! Smooth stop activate");
                //tm_reflexxes::ReflexxesSmoothStop_sim(*IP_interrupt, 0.1);
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
                ReflexxesPositionSafetyRun_sim(*IP, TargetPosition, TargetVelocity, RecoverTime);
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
        usleep(24940 - time_compensation);  

        //********************************************************
        // The area execution in 25ms real time sharp
    }

    gettimeofday(&tm4, NULL);
    long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

    if(pass == REACHPOINT)
    {
        printf("=============== Final state ReflexxesPositionSafetyRun =========================\n");
        printf("[ %lf ]  ", time_s);
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
    double *Target_q = new double [6];
    double *Target_T = new double [16];
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
    std::vector<double> ImageCpature    = {0.0479,    -0.3793,     1.5644,     0.3953,     1.5774,     0.0420}; //for bin picking


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
                SynchronousTime = 1; // 3
                robot_motion_states.data = 2;
                if(ReflexxesPositionSafetyRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime)==SMOOTHSTOP)
                    break;
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete, time = %8.4lf",robot_motion_states.data,SynchronousTime);
            }
            else if(robot_motion_states.data == 2)
            {
                //pose
                SynchronousTime = 1.5;  //3
                robot_motion_states.data = 3;
                if(!ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;  
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
            }
            else if(robot_motion_states.data == 3)
            {
                //above
                SynchronousTime = 1.3;   //3
                robot_motion_states.data = 4;               
                if(!ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;  
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
            }
            else if(robot_motion_states.data == 4)
            {
                //put
                SynchronousTime = 3;   //3 
                robot_motion_states.data = 5;
                if(ReflexxesPositionSafetyRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime)==SMOOTHSTOP)
                    break;  
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete, time = %8.4lf",robot_motion_states.data,SynchronousTime);
//fix                robot_motion_states.data = 1;
            }
//fix            
            else if(robot_motion_states.data == 5)
            {
                //putdown
                SynchronousTime = 1.5;   //3 
                robot_motion_states.data = 6;
                if(ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime)==SMOOTHSTOP)
                    break;  
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete, time = %8.4lf",robot_motion_states.data,SynchronousTime);
                
            }
            else if(robot_motion_states.data == 6)
            {
                //putup
                SynchronousTime = 1.5;   //3 
                robot_motion_states.data = 7;
                if(ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime)==SMOOTHSTOP)
                    break;  
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete, time = %8.4lf",robot_motion_states.data,SynchronousTime);
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
            if(ReflexxesPositionSafetyRun_sim(*IP_position, ImageCpature, TargetVelocity, SynchronousTime)==SMOOTHSTOP)
                break;
            robot_motion_states.data = IMAGE_CAPTURE_POSE;
            ROS_WARN("Robot State = %d Complete, time = %8.4lf",robot_motion_states.data,SynchronousTime);
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
    delete [] Target_T;
    delete [] Target_q;

    return 0;
}
