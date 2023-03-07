#include <stdio.h>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "tm_msgs/FeedbackState.h"

#define D1 0.1451
#define D2 0
#define D3 0
#define D4 -0.1222
#define D5 0.106
#define D6 0.1144

#define A1 0
#define A2 0.4290
#define A3 0.4115
#define A4 0
#define A5 0
#define A6 0

#define ALPHA1 -90
#define ALPHA2 0
#define ALPHA3 0 
#define ALPHA4 90
#define ALPHA5 90
#define ALPHA6 0

#define PI 3.141592654
#define DEG2RAD 0.01745329252
#define RAD2DEG 57.29577951
double gripper_length = 0.22;
std::vector<double> g_robot_joint_angle(6);
std::vector<double> g_tool_pose(6);
bool get_data = false;

void Forward_Kinematics_gripper(const double* q, double* T) 
{
    const double _PI = M_PI;
    const double _PI_2 = 0.5 * M_PI;
    const double _2_PI = 2.0 * M_PI;
    double c1, c2, c3, c4, c5, c6, s1, s2, s3, s4, s5, s6;
    double cp, sp;
    double Length = gripper_length;
    c1 = cos(q[0]); s1 = sin(q[0]);
    c2 = cos(q[1] - _PI_2); s2 = sin(q[1] - _PI_2);
    c3 = cos(q[2]); s3 = sin(q[2]);
    c4 = cos(q[3] + _PI_2); s4 = sin(q[3] + _PI_2);
    c5 = cos(q[4]); s5 = sin(q[4]);
    c6 = cos(q[5]); s6 = sin(q[5]);
    cp = cos(q[1] + q[2] + q[3]);
    sp = sin(q[1] + q[2] + q[3]);

    T[0]  = c1*sp*s6 - s1*s5*c6 + c1*cp*c5*c6;  T[1]  = c1*sp*c6 + s1*s5*s6 - c1*cp*c5*s6;  T[2]  = c1*cp*s5 + s1*c5;

    T[3]  = D5*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3)) - D4*s1 + D6*(c5*s1 + s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2))) + Length*(c5*s1 + s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2))) + A2*c1*c2 + A3*c1*c2*c3 - A3*c1*s2*s3;

    T[4]  = s1*sp*s6 + c1*s5*c6 + s1*cp*c5*c6;  T[5]  = s1*sp*c6 - c1*s5*s6 - s1*cp*c5*s6;  T[6]  = s1*cp*s5 - c1*c5;

    T[7]  = D5*(c4*(c2*s1*s3 + c3*s1*s2) - s4*(s1*s2*s3 - c2*c3*s1)) + D4*c1 - D6*(c1*c5 + s5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2))) - Length*(c1*c5 + s5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2))) + A2*c2*s1 + A3*c2*c3*s1 - A3*s1*s2*s3;

    T[8]  = cp*s6 - sp*c5*c6;       T[9]  = cp*c6 + sp*c5*s6;       T[10] = -sp*s5;

    T[11] = D1 - A2*s2 + D5*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2)) - A3*c2*s3 - A3*c3*s2 - D6*s5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3)) - Length*s5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3));

    T[12] = 0;      T[13] = 0;      T[14] = 0;

    T[15] = 1;
}

void Forward_Kinematics_3(const double* q, double* T, double y_offset) 
{
    const double _PI = M_PI;
    const double _PI_2 = 0.5 * M_PI;
    const double _2_PI = 2.0 * M_PI;
    double d3 = y_offset;
    double c1, c2, s1, s2;
    
    c1 = cos(q[0]); 
    s1 = sin(q[0]);
    c2 = cos(q[1] - _PI_2); 
    s2 = sin(q[1] - _PI_2);

    T[0] = c1*c2;    T[1]  = -c1*s2;    T[2]  = -s1;    T[3]  = A2*c1*c2-d3*s1;

    T[4] = c2*s1;    T[5]  = -s1*s2;    T[6]  =  c1;    T[7]  = d3*c1+A2*c2*s1;

    T[8] = -s2;      T[9]  = -c2;       T[10] =   0;    T[11] = D1 - A2*s2; 

    T[12] = 0;       T[13] = 0;         T[14] =   0;    T[15] = 1;
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
  if(msg->tool_pose.size() == 6){
    g_tool_pose[0] = msg->tool_pose[0];
    g_tool_pose[1] = msg->tool_pose[1];
    g_tool_pose[2] = msg->tool_pose[2];
    g_tool_pose[3] = msg->tool_pose[3];
    g_tool_pose[4] = msg->tool_pose[4];
    g_tool_pose[5] = msg->tool_pose[5];
  }
  get_data = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tm_state_publisher");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(2);
    ros::Subscriber joint_sub = n.subscribe("feedback_states", 1000, TMmsgCallback);
    
    ros::Publisher gripper_pos_pub = n.advertise<geometry_msgs::Point>("gripper_position",40);

    geometry_msgs::Point gripper_pose_msg;

    static tf::TransformBroadcaster tf_bc;
    tf::Quaternion quat;
    tf::Transform transform;

    tf::Quaternion quat_gripper;
    tf::Quaternion quat_joint3;
    tf::Transform transform_gripper;
    tf::Transform transform_joint3;

    ros::Rate loop_rate(40);

    spinner.start();

    std::string joint_prefix = "";
    std::string gripper_frame;
    std::string joint3_frame;
    std::string base_frame;
    gripper_frame = joint_prefix + "gripper_link";
    joint3_frame  = joint_prefix + "joint3_link";
    base_frame    = joint_prefix + "base";

    double *q = new double [6];
    double *T = new double [16];
    double *T3 = new double [16];

    while(ros::ok()){
        if(get_data){
            for(int i = 0; i < 6; i++){
                q[i] = g_robot_joint_angle[i];
            }

            //Publish position (gripper)
            Forward_Kinematics_gripper(q,T);
            gripper_pose_msg.x = T[3];
            gripper_pose_msg.y = T[7];
            gripper_pose_msg.z = T[11];
            gripper_pos_pub.publish(gripper_pose_msg);

            //Broadcast transform (gripper)
            quat.setRPY(g_tool_pose[3], g_tool_pose[4], g_tool_pose[5]);
            transform_gripper.setOrigin(tf::Vector3(T[3],T[7],T[11]));
            transform_gripper.setRotation(quat);
            tf_bc.sendTransform(tf::StampedTransform(transform_gripper, ros::Time::now(), base_frame, gripper_frame));

            //Broadcast transfor (joint3)
            Forward_Kinematics_3(q,T3, -0.15);
            transform_joint3.setOrigin(tf::Vector3(T3[3], T3[7], T3[11]));
            quat_joint3.setRPY(0, 0, 0);
            transform_joint3.setRotation(quat_joint3);
            tf_bc.sendTransform(tf::StampedTransform(transform_joint3, ros::Time::now(), base_frame, joint3_frame));
        }
        loop_rate.sleep();
    }

    ros::waitForShutdown();

    delete [] q;
    delete [] T;
    delete [] T3;

    return 0;
}