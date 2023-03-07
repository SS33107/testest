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

#define GRIPPER_LENGTH 0.240

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

#define SMOOTHSTOP 0
#define REACHPOINT 1
#define INTERRUPT  2

#define OBSTACLE_GPR false
#define V_TRAVEL 0.05

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

double g_objposition[7];
bool position_fill = false;
bool rotation_fill = false;

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
        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);
        //ROS_INFO("gripper XYZ_pos: %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);

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

bool ReflexxesVelocityInterrupt(TmDriver& TR,
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

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            q[i] = OP->NewPositionVector->VecData[i];

        if(!CheckJointLimit(q))
        {
            ROS_WARN("Joint position execced limit!!");
            tm_reflexxes::ReflexxesSmoothStop(TR, *IP, 0.25);
            pass = false;
            break;
        }
        else
        {
            TR.setMoveJointSpeedabs(VelocityCommand, blend);
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

int ReflexxesPositionSafetyRun( TmDriver& TR,
                                RMLPositionInputParameters &InputState, 
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
    time_s = TR.interface->stateRT->getQAct(CurrentPosition);
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
            tm_reflexxes::ReflexxesSmoothStop(TR, IP, 0.25);
            pass = SMOOTHSTOP;
            break;
        }
        else
        {
            TR.setMoveJointSpeedabs(VelocityCommand, blend);
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
            tm_reflexxes::ReflexxesSmoothStop(TR,IP, 0.25);
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
                    ReflexxesVelocityInterrupt(TR,*IP_interrupt,PotentialField_qd,0.2);
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
                    tm_reflexxes::ReflexxesSmoothStop(TR,*IP_interrupt, 0.1);
                    break;
                }
            }
            else
            {
                ROS_WARN("Joint velocity out of range!! Smooth stop activate");
                tm_reflexxes::ReflexxesSmoothStop(TR,*IP_interrupt, 0.1);
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
                ReflexxesPositionSafetyRun(TR,*IP, TargetPosition, TargetVelocity, RecoverTime);
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
    
    //0523 箱子image_pose
    std::vector<double> ImageCpature    = {1.0076,    0.2228,    0.9947,    0.3476,    1.5761,    -0.5633};


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
                //above
                SynchronousTime = 2.5;
                robot_motion_states.data = 2;
                if(ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime) == SMOOTHSTOP)
                    break;
                
                //close gripper
                TM5.setDigitalOutputMB(0,false);
                TM5.setDigitalOutputMB(1,true);

                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
            }
            else if(robot_motion_states.data == 2)
            {
                //pose
                SynchronousTime = 2.5;
                robot_motion_states.data = 3;
                if(!ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;  

                // TM5.setDigitalOutputEE(0,true);
                TM5.setDigitalOutputMB(0,true);
                TM5.setDigitalOutputMB(1,false);
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
            }
            else if(robot_motion_states.data == 3)
            {
                //move_prepare
                SynchronousTime = 2;
                robot_motion_states.data = 4;
                if(!ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;  
                
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
                // ROS_INFO("Enter to CONTINUE");
                // getchar();

            }
            else if(robot_motion_states.data == 4)
            {
                // put_prepare
                SynchronousTime = 5;
                robot_motion_states.data = 5;
                if(ReflexxesPositionSafetyRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime) == SMOOTHSTOP)
                    break; 
                
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
                // ROS_INFO("Enter to CONTINUE");
                // getchar();
                
            }
            else if(robot_motion_states.data == 5)
            {
                // put
                SynchronousTime = 2;
                robot_motion_states.data = 6;
                if(ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime) == SMOOTHSTOP)
                    break; 
                
                // TM5.setDigitalOutputEE(0,false);
                TM5.setDigitalOutputMB(0,false);
                TM5.setDigitalOutputMB(1,true);
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
                
            }
            else if(robot_motion_states.data == 6)
            {
                // put_prepare
                SynchronousTime = 2;
                robot_motion_states.data = 7;
                if(ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime) == SMOOTHSTOP)
                    break; 
                
                
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
                
            }
            else{}

            g_objposition[0] = 0.0;
            position_fill = false;
            rotation_fill = false;
            sleep(1);
        }
        else if(g_objposition[0] == 2.0)
        {
            SynchronousTime = 4;
            if(ReflexxesPositionSafetyRun(TM5, *IP_position, ImageCpature, TargetVelocity, SynchronousTime) == SMOOTHSTOP)
                break;
            robot_motion_states.data = IMAGE_CAPTURE_POSE;
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
            RobotState.publish(robot_motion_states);
            g_objposition[0] = 0.0;
        }
        else{}
    }



/*  hexagon demo
    //Test for hexagon
    int state = 2;
    double x_plane  = 0.6;

    double left_y   = -0.3360;
    double mid_y    = -0.036;
    double right_y  = 0.264;

    double upp_z    = 0.88;
    double up_z     = 0.755;

    double down_z   = 0.505;
    double downn_z  = 0.38;
    while(ros::ok())
    {
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            q[i] = IP_velocity->CurrentPositionVector->VecData[i];
        }

        if(state == 1)      //s1
        {
            TargetPosition = {x_plane, mid_y, upp_z, 0, 0, 0};
            RecoverPosition = {0.1169,     1.1010,    -0.8864,    -0.8015,     1.6174,     0.0000};
        }
        else if(state == 2) //s2
        {
            TargetPosition = {x_plane, right_y, up_z, 0, 0, 0};
            //RecoverPosition = {0.41259109677946276, 1.3382505958113926, -1.0653804439208756, -0.7086224323063918, 1.9323666292427222, 0.0};
        }
        else if(state == 3) //s3
        {
            TargetPosition = {x_plane, right_y, down_z, 0, 0, 0};
            //RecoverPosition = {0.2063322871662687, 1.6709624267540586, -1.4541148688084005, -0.1403339408076056, 2.3882786575698276, 0.0};
        }
        else if(state == 4) //s4
        {
            TargetPosition = {x_plane, mid_y, downn_z, 0, 0, 0};
            //RecoverPosition = {-0.33407676058144914, 1.7135984133721467, -1.6289372123621142, 0.5236655288083464, 2.523253120302068, 0.0};
        }
        else if(state == 5) //s5
        {
            TargetPosition = {x_plane, left_y, down_z, 0, 0, 0};
            //RecoverPosition = {-0.6145421913076022, 1.4903758367836897, -1.3370331686576946, 0.11311336082476366, 2.1578939652557807, 0.0};
        }
        else if(state == 6) //s6
        {
            TargetPosition = {x_plane, left_y, up_z, 0, 0, 0};
            //RecoverPosition = {-0.4298089744563622, 1.1784058722438542, -0.9467886339222736, -0.4660887787781818, 1.7741078379046684, 0.0};
        }
        else{}
        
        if(state != 7)  //point-to-point motion
        {
            //tm_kinematics::forward(q,T);
            tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);

            if(OnlineAttractiveForceGeneration(Attractive_qd, T, q, TargetPosition, TargetVelocity))
            {
                if(OnlineRepulsiveForceGeneration(Repulsive_qd, T, q, RepulsiveVelocity))
                {
                    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                        qd[i] = Attractive_qd[i] + Repulsive_qd[i];

                    ROS_INFO("gripper : %10.4lf %10.4lf %10.4lf Atrractive: %10.4lf %10.4lf %10.4lf Repulsive: %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11],TargetVelocity[0],TargetVelocity[1],TargetVelocity[2],RepulsiveVelocity[0],RepulsiveVelocity[1],RepulsiveVelocity[2] );
                    pass = ReflexxesVelocityRun(TM5,*IP_velocity, qd, TargetPosition, TargetVelocity ,0.2);

                    if(pass == REACH)
                    {
                        if(state == 1)
                        {
                            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                            {
                                IP_position->CurrentPositionVector->VecData[i]     = IP_velocity->CurrentPositionVector->VecData[i];
                                IP_position->CurrentVelocityVector->VecData[i]     = IP_velocity->CurrentVelocityVector->VecData[i];
                                IP_position->CurrentAccelerationVector->VecData[i] = IP_velocity->CurrentAccelerationVector->VecData[i];
                            }

                            //TargetVelocity = {0,0,0,0,0,0};
                            if(!ReflexxesPositionRun(TM5,*IP_position, RecoverPosition, qd, 1))
                                break;
                            
                            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                            {
                                IP_velocity->CurrentPositionVector->VecData[i]     = IP_position->CurrentPositionVector->VecData[i];
                                IP_velocity->CurrentVelocityVector->VecData[i]     = IP_position->CurrentVelocityVector->VecData[i];
                                IP_velocity->CurrentAccelerationVector->VecData[i] = IP_position->CurrentAccelerationVector->VecData[i];
                            }
                        }
                        state++;
                    }
                    else if(pass == STOP)
                        break;
                    else{}
                }
                else
                {
                    ROS_WARN("Repulsive force generate fail, smooth stop activate...");
                    tm_reflexxes::ReflexxesSmoothStop(TM5,*IP_velocity,0.25);
                    break;
                }
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
            state = 1;
        }        
    }
*/
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
    ROS_INFO("minimum_distance = %10.3f ????????????????",distance->data);
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
        else if(strncmp(cstr, "gopen", 6) == 0)
        {
            unsigned char ch = 0;

            // TmRobot.setDigitalOutputEE(ch,false);
            TmRobot.setDigitalOutputMB(0,true);
            TmRobot.setDigitalOutputMB(1,false);
        }
        else if(strncmp(cstr, "gclose", 7) == 0)
        {
            unsigned char ch = 0;

            // TmRobot.setDigitalOutputEE(ch,true);
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
            TmRobot.setDigitalOutputMB(0,false);
            TmRobot.setDigitalOutputMB(1,true);
            BinPicking_Demo(TmRobot);
            //StaticRobotAvoidnace(TmRobot);

            TmRobot.setJointSpdModeoOFF();
            print_info("joint vlocity control mode OFF...");
            


        }
        else if (strncmp(cstr, "binpicking",10 ) == 0)
        {
            double blend = 0;
            std::vector<double> vec = {0.0479,    -0.3793,     1.5644,     0.3953,     1.5774,     0.0420};
            TmRobot.setMoveJabs(vec, blend);
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
