#include "tm_driver/tm_print.h"
#include "tm_driver/tm_driver.h"
#include "tm_driver/tm_communication.h"
#include "tm_driver/tm_robot_state.h"
#include "tm_driver/tm_ros_node.h"

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

#include <cstdlib>
#include <iostream>
#include <string>
#include <cstring>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>


#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Header.h"
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

#define DEG2RAD M_PI/180.0
#define RAD2DEG 180.0/M_PI
#define REPEATABILITY 0.01//0.00005

#define GRIPPER_LENGTH 0.22

#define VMAX_ATT_GPR 0.08
#define VMAX_ATT 0.08
#define VMAX_REP 0.15
#define VMAX_ROT 20
#define VMAX_CC_GPR  0.03

#define DANGEROUS_ZONE 0.3
#define JOINTLIMIT_SPD_123 150
// #define JOINTLIMIT_SPD_456 200
#define JOINTLIMIT_SPD_456 150

#define STOP 0
#define PASS 1
#define REACH 2

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


std::vector<double> g_robot_joint_angle(6);
std::vector<double> g_robot_pose(6), EFF_Velocity_auo(6);
double g_objposition[7];
int tm_target_type=0; // 1: random, 2: drilldown, 3: drillup
static double tm_target_p[6];
static double tm_target_j[6]={0,0,0,0,0,0};
bool pose_fill = false;
static double SynchronousTime = 3; 
ros::Publisher RobotState;

ros::ServiceClient vel_client;
ros::ServiceClient script_client;
ros::ServiceClient event_client;
ros::ServiceClient pos_client;
tm_msgs::SetPositions target_pose, target_pose_line;
tm_msgs::SetEvent event_srv;
tm_msgs::SendScript script_srv;

ros::Publisher pub_ctrl;
robotiq_controller::Robotiq2FGripper_robot_output gripper_command;

std::string exit_cmd = "ScriptExit()";
std::string VStart_cmd = "ContinueVJog()";
std::string VStop_cmd = "StopContinueVmode()";
ros::Time Arm_time;
int Mode = 3; // 0 for PTP control, 1 for vel control, 3 for Idle

void handler(int signum) {
    exit(1);
}
void arm_pos_time_callback(const std_msgs::Header &msg){
	Arm_time = msg.stamp;
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
bool CheckVelocityLimit(std::vector<double> qd, double &MaxScalingFactor)
{
	bool valid = true;
	double ExceedRatio[6] = { 0,0,0,0,0,0 };
	double MaxExceed = 1;
	short  ExceedJoint;

	for (int i = 0; i < 3; ++i)
	{
		if (abs(qd[i]) / JOINTLIMIT_SPD_123 > 1.0)
		{
			ExceedRatio[i] = abs(qd[i]) / JOINTLIMIT_SPD_123;
			ROS_WARN("[Velocity] %dth joint velocity exceed limit(150): %10.4lf", i + 1, qd[i] * RAD2DEG);
			valid = false;
		}
		else
			ExceedRatio[i] = 0.0;
	}

	for (int i = 3; i < 6; ++i)
	{
		if (abs(qd[i]) / JOINTLIMIT_SPD_456 > 1.0)
		{
			ExceedRatio[i] = abs(qd[i]) / JOINTLIMIT_SPD_456;
			ROS_WARN("[Velocity] %dth joint velocity exceed limit(200): %10.4lf", i + 1, qd[i] * RAD2DEG);
			valid = false;
		}
		else
			ExceedRatio[i] = 0.0;
	}

	if (!valid)
	{
		for (int i = 0; i < NUMBER_OF_DOFS; ++i)
		{
			if (ExceedRatio[i] > MaxExceed)
			{
				MaxExceed = ExceedRatio[i];
				ExceedJoint = i;
			}
		}
		MaxScalingFactor = MaxExceed;
		ROS_WARN("[Velocity] THE MOST EXCEED JOINT IS %dth joint, NEED SCALING %10.4lf", ExceedJoint + 1, MaxScalingFactor);
	}

	return valid;
}

bool GetQdfromLinearJacobian(   std::vector<double> CurrentPosition,
                                std::vector<double> EFF_Velocity, 
                                std::vector<double>& qd)
{
    Eigen::Matrix<float , 6, 1> home,q;
    Eigen::Matrix<double , 6,1> JointSpeed;
    Eigen::Matrix<double, 6, 1> EFFSpeed;


    home     <<                  0,            -PI*0.5,                  0,             PI*0.5,                  0,                  0;
    q        << CurrentPosition[0], CurrentPosition[1], CurrentPosition[2], CurrentPosition[3], CurrentPosition[4], CurrentPosition[5];
    EFFSpeed <<    EFF_Velocity[0],    EFF_Velocity[1],    EFF_Velocity[2],    EFF_Velocity[3],    EFF_Velocity[4],    EFF_Velocity[5];
    q += home;

    Eigen::Matrix<double, 6, 6> Geometry_Jacobian = tm_jacobian::Forward_Jacobian_gripper(q, GRIPPER_LENGTH);// Forward_Jacobian_d(q);
    Eigen::Matrix<double, 3, 6> Jacobian_123456   = Geometry_Jacobian.block<3,6>(0,0);

    Eigen::MatrixXd Geometry_Jacobian_inv;
    pinv_SVD(Geometry_Jacobian,Geometry_Jacobian_inv);
    JointSpeed = Geometry_Jacobian_inv * EFFSpeed;
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        qd[i] = JointSpeed(i);

    //ROS_INFO("Generatd qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);

    double ScalingFactor;
    return CheckVelocityLimit(qd,ScalingFactor);
}



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

bool ReflexxesPositionRun(RMLPositionInputParameters &InputState,
	std::vector<double> TargetPosition,
	std::vector<double> TargetVelocity,
	double SynTime)
{
	
	ros::Time move_time =  ros::Time::now();
	tm_msgs::SetVelocity vel_srv;

	double time_s;
	std::vector<double> FinalPosition;
	bool pass = true;
	std::vector<double> VelocityCommand(6);
	double blend = 0;

	ReflexxesAPI *RML = NULL;
	RMLPositionInputParameters  *IP = NULL;
	RMLPositionOutputParameters *OP = NULL;
	RMLPositionFlags Flags;
	int ResultValue = 0;

	// double *T = new double[16];
	// double *q = new double[6];

	RML = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
	IP = new RMLPositionInputParameters(NUMBER_OF_DOFS);
	OP = new RMLPositionOutputParameters(NUMBER_OF_DOFS);

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
		IP->MaxJerkVector->VecData[i] = 100;           //RMLTypeII not using, needed for validity
		IP->MaxVelocityVector->VecData[i] = 3.14; //0.3247
		IP->MaxAccelerationVector->VecData[i] = 3.14;
		IP->TargetPositionVector->VecData[i] = TargetPosition[i];
		IP->TargetVelocityVector->VecData[i] = TargetVelocity[i];
		IP->SelectionVector->VecData[i] = true;
	}
	IP->MinimumSynchronizationTime = SynTime;


	if (IP->CheckForValidity()){
		printf("Input values are valid!\n");
	}
	else{
		printf("Input values are INVALID!\n");
		pass = false;
	}

	struct timeval tm1, tm2, tm3, tm4;
	double cycle_iteration = 1.0;

	gettimeofday(&tm3, NULL);
	ROS_INFO("ReflexxesPositionRun time :%f\n",ros::Time::now().toSec() - move_time.toSec());
	ROS_INFO("auoooooo arm move time pass :%f",ros::Time::now().toSec() - Arm_time.toSec());
	while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED && ros::ok())
	{
		//********************************************************
		// The area execution in 25ms real time sharp

		gettimeofday(&tm1, NULL);

		ResultValue = RML->RMLPosition(*IP, OP, Flags);
		
		if (ResultValue < 0)
		{
			printf("An error occurred (%d).\n", ResultValue);
			break;
		}

		VelocityCommand = { OP->NewVelocityVector->VecData[0],
							OP->NewVelocityVector->VecData[1],
							OP->NewVelocityVector->VecData[2],
							OP->NewVelocityVector->VecData[3],
							OP->NewVelocityVector->VecData[4],
							OP->NewVelocityVector->VecData[5] };

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
			return false;
		}
		// usleep(100*1000);

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

		*IP->CurrentPositionVector = *OP->NewPositionVector;
		*IP->CurrentVelocityVector = *OP->NewVelocityVector;

		if(pose_fill){
			for (int i = 0; i < NUMBER_OF_DOFS; ++i)
			{
				IP->TargetPositionVector->VecData[i] = tm_target_j[i];
			}
			double dt=sqrt(pow(tm_target_p[0]-g_robot_pose[0],2)+pow(tm_target_p[1]-g_robot_pose[1],2)+pow(tm_target_p[2]-g_robot_pose[2],2))/0.1;
			if(dt<0.1)dt=0.1;
			IP->MinimumSynchronizationTime=dt;
			pose_fill=false;
		}

		gettimeofday(&tm2, NULL);
		long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);
		// usleep(24940 - time_compensation);
		if ((24940 - time_compensation) < 1)
			usleep(100);
		else
			usleep(24940 - time_compensation);

		//********************************************************
		// The area execution in 25ms real time sharp
	}
	// sleep(0.3);
	gettimeofday(&tm4, NULL);
	long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

	ROS_INFO("========== Final state position based position =============");
	// ROS_INFO("XYZ_pos : %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);
	print_info("Finished in %llu us", tt);

	tm_reflexxes::resetTermios();

	if (pass)
		InputState = *IP;

	delete  RML;
	delete  IP;
	delete  OP;
	// delete[] T;
	// delete[] q;

	return pass;
}

bool CheckJointLimit(double *q)
{
	bool valid = true;

	if (abs(q[0]) > 265 * DEG2RAD)
	{
		ROS_WARN("[Position] 1st joint position out of limit (270) : %lf", q[0] * RAD2DEG);
		valid = false;
	}
	else if (abs(q[1]) > 175 * DEG2RAD)
	{
		ROS_WARN("[Position] 2nd joint position out of limit (180): %lf", q[1] * RAD2DEG);
		valid = false;
	}
	else if (abs(q[2]) > 148 * DEG2RAD)
	{
		ROS_WARN("[Position] 3rd joint position out of limit (155): %lf", q[2] * RAD2DEG);
		valid = false;
	}
	else if (abs(q[3]) > 175 * DEG2RAD)
	{
		ROS_WARN("[Position] 4th joint position out of limit (180): %lf", q[3] * RAD2DEG);
		valid = false;
	}
	else if (abs(q[4]) > 175 * DEG2RAD)
	{
		ROS_WARN("[Position] 5th joint position out of limit (180): %lf", q[4] * RAD2DEG);
		valid = false;
	}
	else if (abs(q[5]) > 265 * DEG2RAD)
	{
		ROS_WARN("[Position] 6th joint position out of limit (180): %lf", q[5] * RAD2DEG);
		valid = false;
	}
	else
		valid = true;

	return valid;
}

bool GetQfromInverseKinematics(double* CartesianPosition, double *q_inv)
{
	Eigen::Matrix<float, 4, 4> T_;
	Eigen::AngleAxisf yawAngle(CartesianPosition[5], Eigen::Vector3f::UnitZ());
	Eigen::AngleAxisf pitchAngle(CartesianPosition[4], Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf rollAngle(CartesianPosition[3], Eigen::Vector3f::UnitX());
	Eigen::Quaternion<float> q = yawAngle * pitchAngle *rollAngle;
	Eigen::Matrix<float, 3, 3> RotationMatrix = q.matrix();
	double *T = new double[16];


	T_ << 0., 0., 0., CartesianPosition[0],
		0., 0., 0., CartesianPosition[1],
		0., 0., 0., CartesianPosition[2],
		0., 0., 0., 1.;

	T_.block<3, 3>(0, 0) = RotationMatrix.block<3, 3>(0, 0);

	tm_jacobian::Matrix2DoubleArray(T_, T);
	int num_sol = tm_kinematics::inverse(T, q_inv);

	delete[] T;
	return CheckJointLimit(q_inv);
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
void AUO_Demo_pos(){ 
    ros::Time move_time =  ros::Time::now();
    std::vector<double> TargetPosition(6), TargetVelocity(6,0), CurrentPosition(6);
    // target_p[0] = 0.0483;
    // target_p[1] = -0.5762;
    // target_p[2] = 0.2732;
    // target_p[3] = -3.1047;
    // target_p[4] = 0.0784;
    // target_p[5] = 0.2608;
    
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        TargetPosition[i] = tm_target_j[i];
    RMLPositionInputParameters  *IP_position = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    // RMLVelocityInputParameters  *IP_velocity = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

    // TM5.interface->stateRT->getQAct(CurrentPosition);
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        std::cout<<TargetPosition[i]<<std::endl;


    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        CurrentPosition[i] = g_robot_joint_angle[i];
    }

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_position->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_position->CurrentVelocityVector->VecData[i] = 0.0;
        IP_position->CurrentAccelerationVector->VecData[i] = 0.0;

        // IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        // IP_velocity->CurrentVelocityVector->VecData[i] = 0.0;
        // IP_velocity->CurrentAccelerationVector->VecData[i] = 0.0;
    }
	pose_fill = false;
	ROS_INFO("AUO_Demo_pos time :%f\n",ros::Time::now().toSec() - move_time.toSec());
    move_time =  ros::Time::now();
	if(!ReflexxesPositionRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
    {
        ROS_WARN("Smooth stop activate...");
        return;
    }
	ROS_INFO("AUO_Demo_pos move_time :%f\n",ros::Time::now().toSec() - move_time.toSec());
    // std::cout<<TargetPosition.size()<<" "<<TargetVelocity.size()<<" "<<CurrentPosition.size()<<"\n";
    delete  IP_position;
    printf("hahaha\n");
    // ros::Duration(2).sleep();
}

void AUO_Demo_vel(){ 
    tm_msgs::SetVelocity vel_srv;
    std::vector<double>   CurrentPosition(6), qd(6), EFF_Velocity(6),action_qd(6),VelocityCommand(6);
    while(Mode == 1){
		if (tm_reflexxes::kbhit())
		{
			char c = getchar();
			if (c == 'q' || c == 'Q')
			{
				ROS_WARN("Smooth Stop Activate...");
				// ReflexxesPositionSmoothStop(IP, 0.25);
				break;
			}
		}
        for (int i = 0; i < NUMBER_OF_DOFS; ++i){
            CurrentPosition[i] = g_robot_joint_angle[i];
        }
		printf("EFF_Velocity_auo = (%f,%f,%f,%f,%f,%f)\n",EFF_Velocity_auo[0],EFF_Velocity_auo[1],EFF_Velocity_auo[2],EFF_Velocity_auo[3],EFF_Velocity_auo[4],EFF_Velocity_auo[5]);
        GetQdfromLinearJacobian(CurrentPosition,EFF_Velocity_auo,action_qd);
        
        // ReflexxesVelocityRun(*IP_velocity, qd, TargetPosition, TargetVelocity ,0.2);  // thos ;ine need to be modefied!!!
        VelocityCommand={action_qd[0],action_qd[1],action_qd[2],action_qd[3],action_qd[4],action_qd[5]};
        // VelocityCommand={EFF_Velocity[0],EFF_Velocity[1],EFF_Velocity[2],EFF_Velocity[3],EFF_Velocity[4],EFF_Velocity[5]};

        // VelocityCommand ={
        // 	EFF_Velocity[0],EFF_Velocity[1],EFF_Velocity[2],EFF_Velocity[3],EFF_Velocity[4],EFF_Velocity[5]
        // };
        vel_srv.request.velocity.clear();
        vel_srv.request.motion_type = 1;
        vel_srv.request.velocity = VelocityCommand;
        // vel_client.call(vel_srv);
        // usleep(100*1000);
        if (vel_client.call(vel_srv))                             
        {
            if (vel_srv.response.ok){

            } 
                // ROS_INFO_STREAM("SetPositions to robot");
            else ROS_WARN_STREAM("SetPositions to robot , but response not yet ok ");
        }
        else
        {
            ROS_ERROR_STREAM("Error SetPositions to robot");
            // return 1;
        }
    }
     printf("out!!!!\n");
}

void ToolPose_Callback(const tm_msgs::FeedbackState::ConstPtr& ToolPose)
{
	g_robot_pose[0] = ToolPose->tool_pose[0];
	g_robot_pose[1] = ToolPose->tool_pose[1];
	g_robot_pose[2] = ToolPose->tool_pose[2];
	g_robot_pose[3] = ToolPose->tool_pose[3];
	g_robot_pose[4] = ToolPose->tool_pose[4];
	g_robot_pose[5] = ToolPose->tool_pose[5];

	// tf::Quaternion q(
	//     ToolPose->pose.orientation.x,
	//     ToolPose->pose.orientation.y,
	//     ToolPose->pose.orientation.z,
	//     ToolPose->pose.orientation.w);
	// tf::Matrix3x3 m(q);

	// T_tm_end_effector <<   0., 0., 0., tm_end_effector_p[0],
	//         0., 0., 0., tm_end_effector_p[1],
	//         0., 0., 0., tm_end_effector_p[2],
	//         0., 0., 0., 1.;
	// Eigen::Matrix<double,3,3> rot;
	// // tf::matrixTFToEigen(m,rot);
	// // T_tm_end_effector.block<3,3>(0,0) = rot.block<3,3>(0,0);
	// //Roll: rx, Pitch: ry, Yaw: rz, unit: rad
	// m.getRPY(tm_end_effector_p[3], tm_end_effector_p[4], tm_end_effector_p[5]);

}

void ToolJoint_Callback(const tm_msgs::FeedbackState::ConstPtr& ToolPose)
{
	if (ToolPose->joint_pos.size() == 6) {
		g_robot_joint_angle[0] = ToolPose->joint_pos[0];
		g_robot_joint_angle[1] = ToolPose->joint_pos[1];
		g_robot_joint_angle[2] = ToolPose->joint_pos[2];
		g_robot_joint_angle[3] = ToolPose->joint_pos[3];
		g_robot_joint_angle[4] = ToolPose->joint_pos[4];
		g_robot_joint_angle[5] = ToolPose->joint_pos[5];
	}

	g_robot_pose[0] = ToolPose->tool_pose[0];
	g_robot_pose[1] = ToolPose->tool_pose[1];
	g_robot_pose[2] = ToolPose->tool_pose[2];
	g_robot_pose[3] = ToolPose->tool_pose[3];
	g_robot_pose[4] = ToolPose->tool_pose[4];
	g_robot_pose[5] = ToolPose->tool_pose[5];
}

void Target_Callback(const geometry_msgs::TransformStamped::ConstPtr& TargetPose)
{
	tm_target_p[0] = TargetPose->transform.translation.x;
	tm_target_p[1] = TargetPose->transform.translation.y;
	tm_target_p[2] = TargetPose->transform.translation.z;

	tf::Quaternion q(
		TargetPose->transform.rotation.x,
		TargetPose->transform.rotation.y,
		TargetPose->transform.rotation.z,
		TargetPose->transform.rotation.w);
	tf::Matrix3x3 m(q);
	//Roll: rx, Pitch: ry, Yaw: rz, unit: rad
	m.getRPY(tm_target_p[3], tm_target_p[4], tm_target_p[5]);

	
	pose_fill=true;
	// if (GetQfromInverseKinematics(tm_target_p, tm_target_j)) {
	// 	ROS_WARN("j=%lf %lf %lf %lf %lf %lf\n",tm_target_j[0],tm_target_j[1],tm_target_j[2],tm_target_j[3],tm_target_j[4],tm_target_j[5]);
	// 	pose_fill = true;
	// }
	// else {
	// 	ROS_WARN("Target error");
	// 	pose_fill = false;
	// }

}

void Target_Callback_auo(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	ros::Time move_time =  ros::Time::now();
    Mode = 3; //0 for PTP control, 1 for vel control
    tm_target_p[0] = msg->data[0];
	tm_target_p[1] = msg->data[1];
	tm_target_p[2] = msg->data[2];
	tm_target_p[3] = msg->data[3];
	tm_target_p[4] = msg->data[4];
	tm_target_p[5] = msg->data[5];
	SynchronousTime = msg->data[6];
	// tf::Quaternion q(
	// 	TargetPose->pose.orientation.x,
	// 	TargetPose->pose.orientation.y,
	// 	TargetPose->pose.orientation.z,
	// 	TargetPose->pose.orientation.w);
	// tf::Matrix3x3 m(q);
	// //Roll: rx, Pitch: ry, Yaw: rz, unit: rad
	// m.getRPY(tm_target_p[3], tm_target_p[4], tm_target_p[5]);
	printf("========================================================================\n");
	printf("goal\n");
	printf("x:%.4f   y:%.4f    z:%.4f\n",tm_target_p[0],tm_target_p[1],tm_target_p[2]);
	printf("rx:%.4f  ry:%.4f   rz:%.4f\n",tm_target_p[3],tm_target_p[4],tm_target_p[5]);
	printf("time:%f\n",SynchronousTime);
	Arm_time = ros::Time::now();

	pose_fill=true;
	if (GetQfromInverseKinematics(tm_target_p, tm_target_j)) {
		ROS_WARN("j=%lf %lf %lf %lf %lf %lf\n",tm_target_j[0],tm_target_j[1],tm_target_j[2],tm_target_j[3],tm_target_j[4],tm_target_j[5]);
		// pose_fill = true;
		Mode = 0; //0 for PTP control, 1 for vel control
	}
	else {
		ROS_WARN("Target error");
		pose_fill = false;
	}
	ROS_INFO("Target_Callback_auo cal ik time :%f\n",ros::Time::now().toSec() - move_time.toSec());
    // AUO_Demo_pos();
	// ROS_INFO("ALLLLL arm move time pass :%f",ros::Time::now().toSec() - Arm_time.toSec());
}

void Target_Velocity_Callback_auo(const geometry_msgs::Twist::ConstPtr& msg)
{   
    EFF_Velocity_auo[0] = msg->linear.x;
	EFF_Velocity_auo[1] = msg->linear.y;
	EFF_Velocity_auo[2] = msg->linear.z;
	EFF_Velocity_auo[3] = msg->angular.x;
	EFF_Velocity_auo[4] = msg->angular.y;
	EFF_Velocity_auo[5] = msg->angular.z;
    Mode = 1; //0 for PTP control, 1 for vel control
}

void AAAAUUUUUOOOO(){
    while(ros::ok())
    {
		if (tm_reflexxes::kbhit())
		{
			char c = getchar();
			if (c == 'q' || c == 'Q')
			{
				ROS_WARN("Smooth Stop Activate...");
				// ReflexxesPositionSmoothStop(IP, 0.25);
				break;
			}
		}
        if(Mode == 0){
			AUO_Demo_pos();
            printf("In pos mode\n");
			Mode = 3;
		}
            

        else if (Mode==1){
			printf("In vel mode\n");
			AUO_Demo_vel();
		}
            
        else{ // Mode == 3, Idle
			// /printf("In stop mode\n");
            continue;
        }
    }
}

void TargetJoint_Callback_auo(const geometry_msgs::PoseStamped::ConstPtr& TargetPose)
{
	tm_target_j[0] = TargetPose->pose.position.x;
	tm_target_j[1] = TargetPose->pose.position.y;
	tm_target_j[2] = TargetPose->pose.position.z;
	tm_target_j[3] = TargetPose->pose.orientation.x;
	tm_target_j[4] = TargetPose->pose.orientation.y;
	tm_target_j[5] = TargetPose->pose.orientation.z;

	pose_fill=true;
	// if (GetQfromInverseKinematics(tm_target_p, tm_target_j)) {
	// 	ROS_WARN("j=%lf %lf %lf %lf %lf %lf\n",tm_target_j[0],tm_target_j[1],tm_target_j[2],tm_target_j[3],tm_target_j[4],tm_target_j[5]);
	// 	pose_fill = true;
	// }
	// else {
	// 	ROS_WARN("Target error");
	// 	pose_fill = false;
	// }

}

void Target_Callback2(const geometry_msgs::PoseArray::ConstPtr& TargetPose)
{
	tm_target_p[0] = TargetPose->poses[0].position.x;
	tm_target_p[1] = TargetPose->poses[0].position.y;
	tm_target_p[2] = TargetPose->poses[0].position.z;
	tf::Quaternion q(
		TargetPose->poses[0].orientation.x,
		TargetPose->poses[0].orientation.y,
		TargetPose->poses[0].orientation.z,
		TargetPose->poses[0].orientation.w);
	tf::Matrix3x3 m(q);
	//Roll: rx, Pitch: ry, Yaw: rz, unit: rad
	m.getRPY(tm_target_p[3], tm_target_p[4], tm_target_p[5]);

	tm_target_j[0] = TargetPose->poses[1].position.x;
	tm_target_j[1] = TargetPose->poses[1].position.y;
	tm_target_j[2] = TargetPose->poses[1].position.z;
	tm_target_j[3] = TargetPose->poses[1].orientation.x;
	tm_target_j[4] = TargetPose->poses[1].orientation.y;
	tm_target_j[5] = TargetPose->poses[1].orientation.z;

	if (TargetPose->header.frame_id == "random"&& tm_target_type!=2 && tm_target_type !=3) {
		tm_target_type = 1;
	}
	else if (TargetPose->header.frame_id == "drilldown"&& tm_target_type !=3) {
		tm_target_type = 2;
	}
	else if (TargetPose->header.frame_id == "drillup"&& tm_target_type!=2 ) {
		tm_target_type = 3;
	}
	
	pose_fill=true;

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_tm5_reflexxes");
	ros::NodeHandle node_handle;
	RobotState = node_handle.advertise<std_msgs::Int32>("/robot_motion_states",10);

	// ROS_INFO("Subscribe /tool_position");
	// ros::CallbackQueue tool_pose;
	// ros::SubscribeOptions ops_toolpose = ros::SubscribeOptions::create<tm_msgs::FeedbackState>("/feedback_states", 10, ToolPose_Callback, ros::VoidPtr(), &tool_pose);
	// ros::Subscriber sub_toolpose = node_handle.subscribe(ops_toolpose);
	// ros::AsyncSpinner async_spinner_toolpose(1, &tool_pose);
	// async_spinner_toolpose.start();

	// ROS_INFO("Subscribe /robot_target");
	// ros::CallbackQueue robot_target;
	// ros::SubscribeOptions ops_robot_target = ros::SubscribeOptions::create<geometry_msgs::TransformStamped>("/robot_target", 10, Target_Callback, ros::VoidPtr(), &robot_target);
	// ros::Subscriber sub_robot_target = node_handle.subscribe(ops_robot_target);
	// ros::AsyncSpinner async_spinner_robot_target(1, &robot_target);
	// async_spinner_robot_target.start();

	ros::AsyncSpinner spinner(6);
	// ros::Subscriber sub_toolpose  = node_handle.subscribe("/robot_target", 10,TargetJoint_Callback);
	ros::Subscriber sub_toolpose  = node_handle.subscribe("/arm_pos_cmd", 1,Target_Callback_auo);
    ros::Subscriber sub_robot_target = node_handle.subscribe("feedback_states", 100, ToolJoint_Callback);
    ros::Subscriber sub_robot_vel = node_handle.subscribe("arm_vel_cmd", 1, Target_Velocity_Callback_auo);
	ros::Subscriber sub_time = node_handle.subscribe("arm_pos_time",1,arm_pos_time_callback);

    
	vel_client = node_handle.serviceClient<tm_msgs::SetVelocity>("tm_driver/set_velocity");
	script_client = node_handle.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
	event_client = node_handle.serviceClient<tm_msgs::SetEvent>("tm_driver/set_event");
	pos_client = node_handle.serviceClient<tm_msgs::SetPositions>("tm_driver/set_positions");
	pub_ctrl = node_handle.advertise<robotiq_controller::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput",10);
	spinner.start();


	

	target_pose.request.motion_type = tm_msgs::SetPositions::Request::PTP_J;
	target_pose.request.velocity = 2.0;//rad/s
	target_pose.request.acc_time = 0.2;
	target_pose.request.blend_percentage = 0;
	target_pose.request.fine_goal = false;

	target_pose_line.request.motion_type = tm_msgs::SetPositions::Request::LINE_T;
	target_pose_line.request.velocity = 0.3;//rad/s
	target_pose_line.request.acc_time = 0.1;
	target_pose_line.request.blend_percentage = 0;
	target_pose_line.request.fine_goal = false;

	std::vector<double> home = { 0.5134511108398437, 0.16386395263671874, 0.40, -M_PI, 0.0, M_PI / 2 };

	script_srv.request.id = "spdmodeoff";
	script_srv.request.script = VStop_cmd;
	script_client.call(script_srv);

	print_info("joint vlocity control mode OFF...");

	// ros::Publisher pub_estimation;
	ros::Rate r(30);
	int valid = 0;

	bool fgRun = false;
	
	char delim[] = " ,;\t";
	char c;
	while (ros::ok())
	{
        std::cout << "start\n";
		char cstr[512];
		fgets(cstr, 512, stdin);
        // std::cin>>cstr;
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
		else if (strncmp(cstr, "home", 4) == 0)
		{
			std::vector<double> vec1 = { 0,0,0,0,0,0 };

			target_pose.request.positions.clear();
			target_pose.request.positions = vec1;
			// pos_client.call(target_pose);
			if (pos_client.call(target_pose))                             
            {
                if (target_pose.response.ok) ROS_INFO_STREAM("SetPositions to robot");
                else ROS_WARN_STREAM("SetPositions to robot , but response not yet ok ");
            }
            else
            {
                ROS_ERROR_STREAM("Error SetPositions to robot");
                return 1;
            }

			print_info("Back to home");
		}
		else if (strncmp(cstr, "ready", 5) == 0)
		{
			std::vector<double> vec1 = { 0.0,    -0.4777,    1.9319,    0,    1.5708,    0.0 };

			target_pose.request.positions.clear();
			target_pose.request.positions = vec1;
			// pos_client.call(target_pose);
			if (pos_client.call(target_pose))                             
            {
                if (target_pose.response.ok) ROS_INFO_STREAM("SetPositions to robot");
                else ROS_WARN_STREAM("SetPositions to robot , but response not yet ok ");
            }
            else
            {
                ROS_ERROR_STREAM("Error SetPositions to robot");
                return 1;
            }

			print_info("Back to ready position");
		}
		else if (strncmp(cstr, "gstart", 6) == 0)
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
		else if (strncmp(cstr, "gopen", 6) == 0)
		{
			set_gripper(0);
			pub_ctrl.publish(gripper_command);
		}
		else if (strncmp(cstr, "gclose", 7) == 0)
		{
			set_gripper(1);
			pub_ctrl.publish(gripper_command);
		}
		else if (strncmp(cstr, "gopen0", 7) == 0)
		{
			gripper_command.rACT = 1;
			gripper_command.rGTO = 1;
			gripper_command.rSP = 200;
			gripper_command.rFR = 0;
			gripper_command.rPR = 0;
			pub_ctrl.publish(gripper_command);

		}
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				// do bin picking demo 
		else if (strncmp(cstr, "demo", 4) == 0)
		{
			// set_gripper(1); // close gripper=1 
			// pub_ctrl.publish(gripper_command);

			script_srv.request.id = "Vstart";
			script_srv.request.script = VStart_cmd;
			script_client.call(script_srv);
			print_info("joint velocity control mode ON...");
            // for (int iii = 0; iii < NUMBER_OF_DOFS; iii++)
            //     std::cout<<tm_target_j[iii]<<std::endl;
			// AUO_Demo_pos();

            AAAAUUUUUOOOO();

            // bool run_succeed = true;
			// int  pass = 0;
			// double SynchronousTime = 7;
			// std::vector<double> TargetPosition(6), TargetVelocity(6), CurrentPosition(6), qd(6), Attractive_qd(6), Repulsive_qd(6), RepulsiveVelocity(6),Rotation_qd(6);
			// double target_p[6],target_j[6];
			// target_p[0] = 0.0483;
			// target_p[1] = -0.5762;
			// target_p[2] = 0.2732;
			// target_p[3] = -3.1047;
			// target_p[4] = 0.0784;
			// target_p[5] = 0.2608;
			
			// GetQfromInverseKinematics(target_p, target_j);
			// for (int i = 0; i < NUMBER_OF_DOFS; ++i)
			// 	TargetPosition[i] = target_j[i];
			// RMLPositionInputParameters  *IP_position = new RMLPositionInputParameters(NUMBER_OF_DOFS);
			// // RMLVelocityInputParameters  *IP_velocity = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

			// // TM5.interface->stateRT->getQAct(CurrentPosition);
			// for (int i = 0; i < NUMBER_OF_DOFS; ++i)
			// {
			// 	CurrentPosition[i] = g_robot_joint_angle[i];
			// }

			// for (int i = 0; i < NUMBER_OF_DOFS; ++i)
			// {
			// 	IP_position->CurrentPositionVector->VecData[i] = CurrentPosition[i];
			// 	IP_position->CurrentVelocityVector->VecData[i] = 0.0;
			// 	IP_position->CurrentAccelerationVector->VecData[i] = 0.0;

			// 	// IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
			// 	// IP_velocity->CurrentVelocityVector->VecData[i] = 0.0;
			// 	// IP_velocity->CurrentAccelerationVector->VecData[i] = 0.0;
			// }

			// std::vector<double> InitialPosition = {0.2595,    -0.1595,    1.4565,    -1.3401,    1.2991,    -0.0};

			// TargetVelocity = {0, 0, 0, 0, 0, 0};
			// if(!ReflexxesPositionRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
			// {
			// 	ROS_WARN("Smooth stop activate...");
			// 	return 0;
			// }
            
			script_srv.request.id = "spdmodeoff";
			script_srv.request.script = VStop_cmd;
			script_client.call(script_srv);

			print_info("joint vlocity control mode OFF...");
            
		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
        std::cout << "end\n";
	}
	// ros::waitForShutdown();
	printf("[ info] TM_ROS: shutdown\n");

	return 0;
}
