/*********************************************************************
 *                      Apache License
 *                 Version 2.0, January 2004
 *               http://www.apache.org/licenses/
 *
 * tm_otg.cpp
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
 **********************************************************************/

#include "tm_reflexxes/tm_reflexxes.h"
#include "tm_kinematics/tm_kin.h"
#include <eigen3/Eigen/Geometry> 

#include <ros/ros.h>

//*************************************************************************
// defines

#define CYCLE_TIME_IN_SECONDS                   0.025
#define NUMBER_OF_DOFS                          6
#define DEG2RAD 0.01745329252
#define RAD2DEG 57.29577951

#define qd_limit_123 150
#define qd_limit_456 200

using namespace std;


bool CheckJointLimit(double *q)
{
    bool valid = true;

    if(abs(q[0]) > 270*DEG2RAD)
    {
        printf("[WARN] the 1th joint : %lf\n",q[0] );
        valid = false;
    }
    else if(abs(q[1]) > 1.57)
    {
        printf("[WARN] the 2th joint : %lf\n",q[1] );
        valid = false;
    }
    else if(abs(q[2]) > 155*DEG2RAD)
    {
        printf("[WARN] the 3th joint : %lf\n",q[2] );
        valid = false;
    }
    else if(abs(q[3]) > 180*DEG2RAD)
    {
        printf("[WARN] the 4th joint : %lf\n",q[3] );
        valid = false;
    }
    else if(abs(q[4]) > 180*DEG2RAD)
    {
        printf("[WARN] the 5th joint : %lf\n",q[4] );
        valid = false;
    }
    else if(abs(q[5]) > 270*DEG2RAD)
    {
        printf("[WARN] the 6th joint : %lf\n",q[5] );
        valid = false;
    }
    else
        valid = true;

    return valid;
}

bool CheckVelocityLimit(std::vector<double> qd)
{
    bool valid = true;

    if(abs(qd[0]) > qd_limit_123*DEG2RAD || abs(qd[1]) > qd_limit_123*DEG2RAD || abs(qd[2]) > qd_limit_123*DEG2RAD)
    {
        printf("[WARN] the 1th~3th joint : %10.4lf %10.4lf %10.4lf\n",qd[0],qd[3],qd[2] );
        valid = false;
    }
    else if(abs(qd[3]) > qd_limit_456*DEG2RAD || abs(qd[4]) > qd_limit_456*DEG2RAD || abs(qd[5]) > qd_limit_456*DEG2RAD)
    {
        printf("[WARN] the 4th~6th joint : %10.4lf %10.4lf %10.4lf\n",qd[3],qd[4],qd[5] );
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

// x:T[3] 
// y:T[7] 
// z:T[11]
bool OnlinePotentialForceGeneration(vector<double>& PotentialField_qd,
                                    double *GoalPoint,
                                    double *ObstaclePoint,
                                    double *T,
                                    double *q)
{
    double Vmax_rep = 1.5;
    double Vmax_att = 0.1;
    double Survelliance_rep = 2/0.2;
    double Survelliance_att = 0.4;
    double ShapingFactor_rep = 8;
    double ShapingFactor_att = 5;

    Eigen::Vector3d Obstacle2Eff,Eff2Goal;
    Eff2Goal     <<     0,     GoalPoint[1]-T[7],     0;
    Obstacle2Eff << 0, T[7]-ObstaclePoint[1], T[11]-ObstaclePoint[2];
    Eff2Goal     <<     GoalPoint[0]-T[3],     GoalPoint[1]-T[7],     GoalPoint[2]-T[11];
    Obstacle2Eff << T[3]-ObstaclePoint[0], T[7]-ObstaclePoint[1], T[11]-ObstaclePoint[2];

    //double dis_goal      = Eff2Goal(1);
    //double dis_repuslive = sqrt( pow(Obstacle2Eff(1),2) + pow(Obstacle2Eff(2),2) );
    double dis_goal      = sqrt( pow(Eff2Goal(0),2)     + pow(Eff2Goal(1),2)     + pow(Eff2Goal(2),2)     );
    double dis_repuslive = sqrt( pow(Obstacle2Eff(0),2) + pow(Obstacle2Eff(1),2) + pow(Obstacle2Eff(2),2) );

    Eigen::Vector3d AttractiveVecotor = Eff2Goal/dis_goal;
    Eigen::Vector3d RepulsiveVector   = Obstacle2Eff/dis_repuslive;

    double AttractiveForce = Vmax_att - Vmax_att*exp(-(dis_goal*ShapingFactor_att)/Survelliance_att); 
    double RepulsiveForce  = Vmax_rep / (1 + exp((dis_repuslive*Survelliance_rep-1)*ShapingFactor_rep));

    Eigen::Vector3d AttractiveVelocity = AttractiveForce*AttractiveVecotor; 
    Eigen::Vector3d RepulsiveVelocity  = RepulsiveForce *RepulsiveVector;
    Eigen::Vector3d PotentialVelocity = /*AttractiveVelocity +*/ RepulsiveVelocity;

    std::vector<double> CurrentPosition = {q[0], q[1], q[2], q[3], q[4], q[5]}; 
    std::vector<double> repulsive_qd(6);
    std::vector<double> EFF_Velocity = {PotentialVelocity(0), PotentialVelocity(1), PotentialVelocity(2), 0,0,0}; 
    bool succeed = GetQdfromInverseJacobian(CurrentPosition,EFF_Velocity,repulsive_qd);   
    PotentialField_qd = repulsive_qd;

/*
    cout << "==============================================================="<<endl;
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
    cout << "[ INFO ] joint speed        :";
    tm_jacobian::printVector(repulsive_qd);
    cout << "[ INFO ] PotentialField_qd        :";
    tm_jacobian::printVector(repulsive_qd);
    cout << "==============================================================="<<endl;
*/
    return succeed;
    
}

double DistanceMeasurement(double *T)
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

bool ReflexxesVelocityInterrupt_sim(      RMLVelocityInputParameters &InputState, 
                                          std::vector<double> TargetVelocity, 
                                          double SynTime)
{
    double time_s;

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
        printf("Input values are valid!\n");
    else
        printf("Input values are INVALID!\n");

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

        time_s = cycle_iteration*0.025;
        cycle_iteration++;

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            q[i] = OP->NewPositionVector->VecData[i];
        }

        tm_kinematics::forward(q,T);
        distance = DistanceMeasurement(T);

        printf("[ %lf ] XYZ_pos:  ",time_s);

        printf("%10.4lf %10.4lf %10.4lf  [ %lf ] ", T[3], T[7], T[11], distance);

        printf(" | spd: ");

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            printf("%10.4lf ", OP->NewVelocityVector->VecData[i]);
        }
        printf("\n");

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

    if(pass)
    {
        printf("=============== Final state velocity based ReflexxesVelocityRun_sim =========================\n");
        printf("[ %lf ]  ", time_s);

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            printf(" %10.4lf ",IP->CurrentVelocityVector->VecData[i]);
        printf("\n");
        print_info("Finished in %llu us", tt);
    }

    InputState = *IP;
    delete  RML;
    delete  IP;
    delete  OP;

    return pass;
}

bool ReflexxesPositionSafetyRun_sim(  RMLPositionInputParameters &InputState, 
                                      std::vector<double> TargetPosition,
                                      std::vector<double> TargetVelocity, 
                                      double SynTime)
{
    double time_s;
    std::vector<double> FinalPosition;
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
    *IP = InputState;

    std::vector<double> PotentialField_qd(6), qd_now(6);
    double *GoalPoint = new double [3];
    double *ObstaclePoint = new double [3];
    GoalPoint[0] = 0.426;
    GoalPoint[1] = 0.092;
    GoalPoint[2] = 0.580;
    ObstaclePoint[0] = 0.426;
    ObstaclePoint[1] = -0.122;
    ObstaclePoint[2] = 0.473;

    double *q = new double [6];
    double *T = new double [16];
    double distance;

    //  ********************************************************************/
    //  Assigning all RMLPositionInputParameters : 
    //  Current POS, VEL, ACC : set before call ReflexxesPositionRun
    //  Target POS, VEL       : set before call ReflexxesPositionRun
    //  Max VEL, ACC          : set after call ReflexxesPositionRun
    //  SelectionVector       : set after call ReflexxesPositionRun
    //  ********************************************************************
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP->MaxVelocityVector->VecData[i] = MAX_VELOCITY; //0.3247
        IP->MaxAccelerationVector->VecData[i] = MAX_ACC;
        IP->TargetPositionVector->VecData[i] = TargetPosition[i]; 
        IP->TargetVelocityVector->VecData[i] = TargetVelocity[i];
        IP->SelectionVector->VecData[i] = true;
    }
    IP->MinimumSynchronizationTime = SynTime;


    if (IP->CheckForValidity())
        printf("Input values are valid!\n");
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

        time_s = cycle_iteration*0.025;
        cycle_iteration++;

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            q[i] = OP->NewPositionVector->VecData[i];
        }

        tm_kinematics::forward(q,T);
        distance = DistanceMeasurement(T);

        printf("[ %lf ] XYZ_pos:  ",time_s);

        printf("%10.4lf %10.4lf %10.4lf  [ %lf ] ", T[3], T[7], T[11], distance);

        printf(" | spd: ");

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            printf("%10.4lf ", OP->NewVelocityVector->VecData[i]);
            qd_now[i] = OP->NewVelocityVector->VecData[i];
        }
        printf("\n");

        //***************************************************************
        // Collision avoidance control activate
        AlreadyAccese = false;
        while (distance < 0.15)
        {
            print_info("Safety control Activate...");
            if(!AlreadyAccese)
            {
                *IP_interrupt->CurrentPositionVector     = *IP->CurrentPositionVector;
                *IP_interrupt->CurrentVelocityVector     = *IP->CurrentVelocityVector;
                *IP_interrupt->CurrentAccelerationVector = *IP->CurrentAccelerationVector;
            }
            if(OnlinePotentialForceGeneration(PotentialField_qd, GoalPoint, ObstaclePoint, T,q ))        
            {
                for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                {
                    if(AlreadyAccese)
                    {
                        qd_now[i] = IP_interrupt->CurrentVelocityVector->VecData[i];
                    }
                    
                    PotentialField_qd[i] += qd_now[i];
                }

                ReflexxesVelocityInterrupt_sim(*IP_interrupt,PotentialField_qd,0.2);

                for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                    q[i] = IP_interrupt->CurrentPositionVector->VecData[i];

                tm_kinematics::forward(q,T);
                distance = DistanceMeasurement(T);
            }
            else
            {
                tm_reflexxes::ReflexxesSmoothStop_sim(IP_interrupt, 0.2);
                break;
            }
            AlreadyAccese = true;
            
            if(distance > 0.15)
            {
                *IP->CurrentPositionVector = *IP_interrupt->CurrentPositionVector;
                *IP->CurrentVelocityVector = *IP_interrupt->CurrentVelocityVector;
                *IP->CurrentAccelerationVector = *IP_interrupt->CurrentAccelerationVector;
                TargetPosition = {0.6727, -0.0318,  1.6024,  -1.5706,  0.8981,   0 };
                TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
                ReflexxesPositionSafetyRun_sim(*IP, TargetPosition, TargetVelocity, 2);
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
        printf("=============== Final state ReflexxesPositionSafetyRun_sim =========================\n");
        printf("[ %lf ]  ", time_s);
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            printf("%10.4lf ", IP->CurrentVelocityVector->VecData[i]);

        printf("\n");
        print_info("Finished in %llu us", tt);
    }
    //tm_reflexxes::resetTermios();
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

//**************************************************************************************
// point1 : (0.426, -0.336, 0.580), [ -0.5531  0.4107  1.0725  -1.4832  2.1239   0     ]
// point2 : (0.426, -0.122, 0.681), [  0.0006  0.0516  1.1879  -1.2394  1.5702   0     ] 
// point3 : (0.426, 0.092, 0.580),  [  0.6727 -0.0318  1.6024  -1.5706  0.8981   0     ]
// point4 : (0.426, -0.122, 0.479), [  0.0006  0.0509  1.8490  -1.8999  1.5702   0     ]
//**************************************************************************************
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tm_traj");
    bool run_succeed = true;
    double SynchronousTime = 2.0;
    std::vector<double> TargetPosition(6), TargetVelocity(6), CurrentPosition(6), JointVelocity(6);

    RMLPositionInputParameters  *IP_position = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    RMLVelocityInputParameters  *IP_velocity = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

    double *T, *q_inv, *q_ref;
    q_inv = new double [60];
    q_ref = new double [6];
    T     = new double [16];

    std::vector<double> CartesianPosition_1 = {0.426, -0.335, 0.58, 90*DEG2RAD, 0, 90*DEG2RAD};
    std::vector<double> CartesianPosition_2 = {0.425, -0.122, 0.681, 90*DEG2RAD, 0, 90*DEG2RAD};
    std::vector<double> CartesianPosition_3 = {0.426, 0.092, 0.580, 90*DEG2RAD, 0, 90*DEG2RAD};
    std::vector<double> CartesianPosition_4 = {0.426, -0.122, 0.479, 90*DEG2RAD, 0, 90*DEG2RAD};

    CurrentPosition = {-0.5531,  0.4107,  1.0725,  -1.4832,  2.1239,   0 };
    std::vector<double>effspd = {0,0.2378,0,0,0,0};

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_position->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_position->CurrentVelocityVector->VecData[i] = 0.0;
        IP_position->CurrentAccelerationVector->VecData[i] = 0.0;

        IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_velocity->CurrentVelocityVector->VecData[i] = 0.0;
        IP_velocity->CurrentAccelerationVector->VecData[i] = 0.0;
    }


    TargetPosition = {0.6727, -0.0318,  1.6024,  -1.5706,  0.8981,   0 };
    TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
    run_succeed = ReflexxesPositionSafetyRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime);

    delete IP_position;
    delete IP_velocity;
    delete [] T;
    delete [] q_inv;
    delete [] q_ref;
    return 0;
}




