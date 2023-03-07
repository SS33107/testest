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
#include "tm_reflexxes/tm_otg.h"

#include "tm_msgs/RobotStateMsgRT.h"
#include "tm_msgs/SetIO.h"

#include <Eigen/Dense>

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

#define GRIPPER_LENGTH 0.235
#define VMAX_CC_GPR 0.1

#define OBSTACLE_GPR false
#define V_TRAVEL 0.05

#define DANGEROUS_ZONE 0.3
#define JOINTLIMIT_SPD_123 90
#define JOINTLIMIT_SPD_456 150

#define SMOOTHSTOP 0
#define REACHPOINT 1
#define INTERRUPT  2

using namespace std;

double last_point_x = 0.0;
double last_point_y = 0.0;
double last_point_z = 0.0;

double g_distance;
std::vector<double> g_obstacle_position(3);
std::vector<double> g_obstacle_velocity(3);

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

    Eigen::Matrix<double, 6, 6> Geometry_Jacobian = tm_jacobian::Forward_Jacobian_gripper(q);// Forward_Jacobian_d(q);
    Eigen::Matrix<double, 3, 6> Jacobian_123456   = Geometry_Jacobian.block<3,6>(0,0);

    Eigen::Vector3d TaskVelocity_3;
    std::vector<double> ConstrainedPoint(3);
    double *q_now = new double [6];
    
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        q_now[i] = CurrentPosition[i];

    if(!OBSTACLE_GPR)
        ConstrainedPoint = {-10.3, 0.1, 0.234};
    else
        ConstrainedPoint = {-0.3, 0.1, 0.234};


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
    return tm_otg::CheckVelocityLimit(qd,ScalingFactor);
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
    return tm_otg::CheckVelocityLimit(qd,ScalingFactor);
}


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


bool OnlineRepulsiveForceGeneration(vector<double>& PotentialField_qd,
                                    double *ObstaclePoint,  // obstacle position        : for min_distance
                                    double *TCP_position,   // robot TCP position       : for min_distance
                                    /*double *TCP_velocity,*/   // robot TCP velocity       : for repulsive vector
                                    double *q,              // robot joint position     : for jacobian
                                    double dis_repuslive)   // eff to obstacle distance : for repulsive
{
    bool succeed = false;
    double Vmax_rep = 0.5;//1.0
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
    
    //succeed = tm_jacobian::GetQdfromInverseJacobian(CurrentPosition,EFF_Velocity,repulsive_qd);
    succeed = GetQdfromLinearJacobian(CurrentPosition,EFF_Velocity,repulsive_qd);   
    //succeed = GetQdfromCartesianConstrain(CurrentPosition,EFF_Velocity,repulsive_qd);   
    
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

bool OnlineAttractiveForceGeneration(   std::vector<double>& action_qd,
                                        double *TCP_position,             // robot TCP position       : for min_distance
                                        double *q,                        // robot joint position     : for jacobian
                                        std::vector<double> GoalPoint,    // eff to obstacle goal     : for att
                                        std::vector<double> &EFF_Velocity) // eff velocity             : for att
{
    bool   succeed = false;
    double Vmax_att = 0.1;
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
    succeed = GetQdfromLinearJacobian(CurrentPosition,EFF_Velocity,action_qd);  
    //succeed = GetQdfromCartesianConstrain(CurrentPosition,EFF_Velocity,action_qd);     

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

        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);

        //***************************************************************
        // Print out commands

        //ROS_INFO("XYZ_pos: %10.4lf %10.4lf %10.4lf  [ %lf ] ", T[3], T[7], T[11], g_distance);
        ROS_WARN("tool XYZ_pos: %10.4lf %10.4lf %10.4lf  [ %lf ] ", T[3], T[7], T[11], g_distance);

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

int ReflexxesPositionSafetyRun(    TmDriver& TR,
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

    std::vector<double> PotentialField_qd(6), qd_now(6);
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

        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);

        //***************************************************************
        // Print out commands

        time_s = TR.interface->stateRT->getTime();
        ROS_INFO("tool XYZ_pos: %10.4lf %10.4lf %10.4lf [ %lf ]", T[3], T[7], T[11], g_distance);

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            qd_now[i] = OP->NewVelocityVector->VecData[i];
        

        //***************************************************************
        // Collision avoidance control activate

        AlreadyAccese = false;
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

            if(OnlineRepulsiveForceGeneration(PotentialField_qd,ObstaclePoint,T, q, g_distance))
            {
                for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                {
                    PotentialField_qd[i] += qd_now[i];
                }

                if(tm_otg::CheckVelocityLimit(PotentialField_qd,ScalingFactor))
                {
                    ReflexxesVelocityInterrupt(TR, *IP_interrupt,PotentialField_qd,0.2);
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
                    tm_reflexxes::ReflexxesSmoothStop(TR, *IP_interrupt, 0.1);
                    break;
                }
            }
            else
            {
                ROS_WARN("Joint velocity out of range!! Smooth stop activate");
                tm_reflexxes::ReflexxesSmoothStop(TR, *IP_interrupt, 0.1);
                *IP->CurrentPositionVector     = *IP_interrupt->CurrentPositionVector;
                *IP->CurrentVelocityVector     = *IP_interrupt->CurrentVelocityVector;
                *IP->CurrentAccelerationVector = *IP_interrupt->CurrentAccelerationVector;
                pass = SMOOTHSTOP;
                break;
            }

            if(g_distance > DANGEROUS_ZONE)
            {
                *IP->CurrentPositionVector     = *IP_interrupt->CurrentPositionVector;
                *IP->CurrentVelocityVector     = *IP_interrupt->CurrentVelocityVector;
                *IP->CurrentAccelerationVector = *IP_interrupt->CurrentAccelerationVector;
                double RecoverRoute = sqrt(pow((T[3]-GoalPoint[0]),2)+pow((T[7]-GoalPoint[1]),2)+pow((T[11]-GoalPoint[2]),2));
                double RecoverTime = RecoverRoute/V_TRAVEL;

                TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
                ReflexxesPositionSafetyRun(TR, *IP, TargetPosition, TargetVelocity, RecoverTime);
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
    double SynchronousTime = 0.361/V_TRAVEL;
    double TravelTime1 = 0.325/V_TRAVEL;
    double TravelTime2 = 0.25/V_TRAVEL;
    std::vector<double> TargetPosition, TargetVelocity, CurrentPosition;

    RMLPositionInputParameters  *IP_position = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    RMLVelocityInputParameters  *IP_velocity = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

    TargetVelocity = {0,0,0,0,0,0};
    TargetPosition = { 0.1169254713601322, 0.24056347047181537, 0.8864105946782809, -1.7138184387436786, 1.6173753874427401, 0.0};
    ReflexxesPositionSafetyRun(TM5, *IP_position, TargetPosition, TargetVelocity, 4.0);

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

    for (int i = 0; i < 2; ++i)
    {
        TargetPosition={0.509742817159809, 0.18215274857762845, 1.1535638188189503, -1.6727925586943782, 1.742363612714968, 0.0};
        if(ReflexxesPositionSafetyRun(TM5, *IP_position, TargetPosition, TargetVelocity, TravelTime1) == SMOOTHSTOP)
            break;

        TargetPosition={0.4094159924987242, 0.02926865517847024, 1.65510898513015, -1.5655062483029938, 1.9383001764906427, 0.0};
        if(ReflexxesPositionSafetyRun(TM5, *IP_position, TargetPosition, TargetVelocity, TravelTime2) == SMOOTHSTOP)
            break;

        TargetPosition={-0.03190698245300169, -0.1544246744467823, 1.970883025381777, -1.467243375906543, 1.8775456252699205, 0.0};
        if(ReflexxesPositionSafetyRun(TM5, *IP_position, TargetPosition, TargetVelocity, TravelTime1) == SMOOTHSTOP)
            break;

        TargetPosition = {-0.3395576889848834, 0.027776309613829576, 1.657594283442536, -1.5741335701594608, 1.5861482893618837, 0.0};
        if(ReflexxesPositionSafetyRun(TM5, *IP_position, TargetPosition, TargetVelocity, TravelTime1) == SMOOTHSTOP)
            break;

        TargetPosition = {-0.2828951682576394, 0.23525749551557457, 1.1084140085838625, -1.696789006181191, 1.4763182358449376, 0.0};
        if(ReflexxesPositionSafetyRun(TM5, *IP_position, TargetPosition, TargetVelocity, TravelTime2) == SMOOTHSTOP)
            break;    
        
        TargetPosition = { 0.1169254713601322, 0.24056347047181537, 0.8864105946782809, -1.7138184387436786, 1.6173753874427401, 0.0};
        if(ReflexxesPositionSafetyRun(TM5, *IP_position, TargetPosition, TargetVelocity, TravelTime1) == SMOOTHSTOP)
            break;
    }


/*******************************    
//Diamond trajectory demo
    TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
    
    TargetPosition = { -0.3372, 0.1492, 1.6678, -2.2725, 1.27453, 0};
    ReflexxesPositionSafetyRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);

    TargetPosition = { 0.16253776798427813, -0.022740146593122228, 1.6244115260354994, -2.350812400256508, 1.5868641867807138, 0.0};
    ReflexxesPositionSafetyRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);

    TargetPosition = { 0.5059623478529205, 0.043527554029996925, 1.9255053401669715, -2.3699878201834443, 1.8930778946641738, 0.0};
    ReflexxesPositionSafetyRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);

    TargetPosition = { -0.024032669476945386, -0.11670655319874364, 2.500689289199614, -2.4711506615164165, 1.8542916851098756, 0.0};
    ReflexxesPositionSafetyRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);

    TargetPosition = { -0.3372, 0.1492, 1.6678, -2.2725, 1.27453, 0};
    ReflexxesPositionSafetyRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);
**********************************/

/*  Line trajectory demo
    TargetPosition = {0.6727, -0.0318, 1.6024, -1.5706, 0.8981, 0};
    TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
    ReflexxesPositionSafetyRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);

    TargetPosition = {-0.5531,  0.4107,  1.0725,  -1.4832,  2.1239,  0.0000};
    ReflexxesPositionSafetyRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);

    TargetPosition = {0.6727, -0.0318, 1.6024, -1.5706, 0.8981, 0};
    TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
    ReflexxesPositionSafetyRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);

    TargetPosition = {-0.5531,  0.4107,  1.0725,  -1.4832,  2.1239,  0.0000};
    ReflexxesPositionSafetyRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);
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
    double ScalingFactor;
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
    tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);

    ROS_INFO("press enter to start");
    getchar();
    ROS_INFO("saftey mode start");

    *IP_interrupt->CurrentPositionVector     = *IP_position->CurrentPositionVector;
    *IP_interrupt->CurrentVelocityVector     = *IP_position->CurrentVelocityVector;
    *IP_interrupt->CurrentAccelerationVector = *IP_position->CurrentAccelerationVector;

    tm_reflexxes::initTermios(1);
    while(1)
    {
        if (g_distance < DANGEROUS_ZONE)
        {
            ROS_WARN("Safety control Activate...");
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
                if(tm_otg::CheckVelocityLimit(PotentialField_qd,ScalingFactor))
                {
                    ReflexxesVelocityInterrupt(TM5, *IP_interrupt,PotentialField_qd,0.2);
                    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                    {
                        q[i] = IP_interrupt->CurrentPositionVector->VecData[i];
                    }

                    tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);
                }
                else
                {
                    ROS_WARN("Joint velocity out of range!! Smooth stop activate");
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
