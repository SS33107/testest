#include "ros/ros.h"
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <ros/package.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>

#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"

#include <eigen3/Eigen/Dense>

#include "tm_reflexxes/tm_reflexxes.h"
#include "tm_kinematics/tm_kin.h"
#include <tm_msgs/FeedbackState.h>

#include "spline.h"

using namespace std;
int set_name=0;
class Tag_Pose{
public:
	ros::NodeHandle nh;
	ofstream end_pose,joint_vel;
	ofstream pchip_cmd,joint_pos,joint_acc;
	
	ros::Subscriber sub_tm;
	ros::Publisher pub_target;
	ros::Publisher vel_gazebo;

	int if_init_tag11=0;
	int if_init_tag297=0;
	int grip_trigger=0;
	int length;
	int record_tag = 0;

	double slow;
	double grasp, open;

	double tm_target_p[6];
	double tm_target_t[6];
	double tm_now_p[6];
	double tm_target_j[6]={0,0,0,0,0,0};
	
	vector<double> smooth_t,smooth_x,smooth_y,smooth_z,smooth_rx,smooth_ry,smooth_rz;
	vector<double> spine_t,spine_x,spine_y,spine_z,spine_rx,spine_ry,spine_rz;
	vector<double> joint_t,joint_1,joint_2,joint_3,joint_4,joint_5,joint_6;

	vector<vector<double>> store;

	std_msgs::Float64MultiArray vel_cmd_gazebo;
	
	Tag_Pose(){
		sub_tm     = nh.subscribe("/tool0_pose", 1, &Tag_Pose::tm_callback,this);

		vel_gazebo = nh.advertise<std_msgs::Float64MultiArray>("joint_vel_controller/command",10);
		pub_target = nh.advertise<geometry_msgs::Transform> ("/robot_target", 1);

		end_pose.open ("/home/ss/s/tool/techman/src/auo_demo/data/end_data.txt" , std::ifstream::out);
		joint_vel.open("/home/ss/s/tool/techman/src/auo_demo/data/joint_vel.txt", std::ifstream::out);
		joint_pos.open("/home/ss/s/tool/techman/src/auo_demo/data/joint_pos.txt", std::ifstream::out);
		pchip_cmd.open("/home/ss/s/tool/techman/src/auo_demo/data/pchip_cmd.txt", std::ifstream::out);
		joint_acc.open("/home/ss/s/tool/techman/src/auo_demo/data/joint_acc.txt", std::ifstream::out);
	}
	bool CheckJointLimit(double *q){
		bool valid = true;

		if (abs(q[0]) > 265 * DEG2RAD){
			ROS_WARN("[Position] 1st joint position out of limit (270) : %lf", q[0] * RAD2DEG);
			valid = false;
		}
		else if (abs(q[1]) > 175 * DEG2RAD){
			ROS_WARN("[Position] 2nd joint position out of limit (180): %lf", q[1] * RAD2DEG);
			valid = false;
		}
		else if (abs(q[2]) > 150 * DEG2RAD){
			ROS_WARN("[Position] 3rd joint position out of limit (155): %lf", q[2] * RAD2DEG);
			valid = false;
		}
		else if (abs(q[3]) > 175 * DEG2RAD){
			ROS_WARN("[Position] 4th joint position out of limit (180): %lf", q[3] * RAD2DEG);
			valid = false;
		}
		else if (abs(q[4]) > 175 * DEG2RAD){
			ROS_WARN("[Position] 5th joint position out of limit (180): %lf", q[4] * RAD2DEG);
			valid = false;
		}
		else if (abs(q[5]) > 265 * DEG2RAD){
			ROS_WARN("[Position] 6th joint position out of limit (180): %lf", q[5] * RAD2DEG);
			valid = false;
		}
		else
			valid = true;

		return valid;
	}
	bool GetQfromInverseKinematics(double* CartesianPosition, double *q_inv){
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

	void readtxt(){ //read x y z rx ry rz information from txt
		ifstream path_read;
		path_read.open ("/home/ss/s/tool/techman/src/auo_demo/data/test_data.txt", std::ifstream::in);
		// initial point
		smooth_x.push_back(tm_now_p[0]);
		smooth_y.push_back(tm_now_p[1]);
		smooth_z.push_back(tm_now_p[2]);
		if(tm_now_p[3]<-3)
			tm_now_p[3] += 3.14;
		smooth_rx.push_back(tm_now_p[3]);
		smooth_ry.push_back(tm_now_p[4]);
		smooth_rz.push_back(tm_now_p[5]);
		smooth_t.push_back(0);

		smooth_x.push_back(tm_now_p[0]);
		smooth_y.push_back(tm_now_p[1]);
		smooth_z.push_back(tm_now_p[2]);
		if(tm_now_p[3]<-3)
			tm_now_p[3] += 3.14;
		smooth_rx.push_back(tm_now_p[3]);
		smooth_ry.push_back(tm_now_p[4]);
		smooth_rz.push_back(tm_now_p[5]);
		smooth_t.push_back(2);
		// read from txt
		string str,temp;
		std::vector<double> store_temp;
		while (getline(path_read,str)){
			stringstream divide(str);
			while(divide>>temp){
				store_temp.push_back(atof(temp.c_str()));
			}
			smooth_x.push_back(store_temp[0]);
			smooth_y.push_back(store_temp[1]);
			smooth_z.push_back(store_temp[2]);
			smooth_rx.push_back(store_temp[3]);
			smooth_ry.push_back(store_temp[4]);
			smooth_rz.push_back(store_temp[5]);
			store_temp.clear();
		} 
		length = smooth_x.size();
		path_read.close();

		// end point
		smooth_x.push_back(smooth_x[length-1]);
		smooth_y.push_back(smooth_y[length-1]);
		smooth_z.push_back(smooth_z[length-1]);
		smooth_rx.push_back(smooth_rx[length-1]);
		smooth_ry.push_back(smooth_ry[length-1]);
		smooth_rz.push_back(smooth_rz[length-1]);

		smooth_x.push_back(smooth_x[length-1]);
		smooth_y.push_back(smooth_y[length-1]);
		smooth_z.push_back(smooth_z[length-1]);
		smooth_rx.push_back(smooth_rx[length-1]);
		smooth_ry.push_back(smooth_ry[length-1]);
		smooth_rz.push_back(smooth_rz[length-1]);

		// sleep(0.5);
	}
	
	void to_trajectory(){ // give time information 
		std::cout<<"enter vel : ";
		std::cin.clear();
		string tmp_string;
		getline(cin,tmp_string);
		double set_vel  = atof(tmp_string.c_str());
		cout << "set velocity = " << set_vel << endl;
		cout << "cmd_length = " << smooth_x.size() << endl;
		length = smooth_x.size();

		double point_length, last_time = 2, now_time ,length_x ,length_y  ,length_z ;
		for(int i=2;i<length;i++){
			length_x = smooth_x[i]-smooth_x[i-1];
			length_y = smooth_y[i]-smooth_y[i-1];
			length_z = smooth_z[i]-smooth_z[i-1];
			point_length = sqrt(pow(length_x,2) + pow(length_y,2) + pow(length_z,2));
			now_time = last_time + point_length/set_vel;
			if(point_length<0.001)
				now_time = last_time + 0.5;
			cout << "length = " << point_length << "  time = " << now_time<< endl;
			smooth_t.push_back(now_time);
			last_time = now_time ;
		}
		cout << "x_length = " << smooth_x.size() << endl;
		cout << "t_length = " << smooth_t.size() << endl;

		for(int i=0;i<length;i++){
			cout << "time = " << smooth_t[i] << "  x = " << smooth_x[i]
			<< "  x = " << smooth_x[i]<< "  y = " << smooth_y[i]<< "  z = " << smooth_z[i] << endl;
		}
		tk::spline sh0(smooth_t,smooth_x,tk::spline::cspline_hermite);
		tk::spline sh1(smooth_t,smooth_y,tk::spline::cspline_hermite);
		tk::spline sh2(smooth_t,smooth_z,tk::spline::cspline_hermite);
		tk::spline sh3(smooth_t,smooth_rx,tk::spline::cspline_hermite);
		tk::spline sh4(smooth_t,smooth_ry,tk::spline::cspline_hermite);
		tk::spline sh5(smooth_t,smooth_rz,tk::spline::cspline_hermite);

		for (double i=smooth_t[0];i<smooth_t[length-1];i+=1){
			spine_t.push_back(i);
			spine_x.push_back(sh0(i));
			spine_y.push_back(sh1(i));
			spine_z.push_back(sh2(i));
			spine_rx.push_back(sh3(i));
			spine_ry.push_back(sh4(i));
			spine_rz.push_back(sh5(i));
			char outmsg_tag[200];
	    	sprintf(outmsg_tag,"%.7f %.7f %.7f %.7f %.7f %.7f %.7f",i,
	    		sh0(i),sh1(i),sh2(i),sh3(i),sh4(i),sh5(i));
	    	pchip_cmd << outmsg_tag << endl;
		}
		smooth_x.clear();
		smooth_y.clear();
		smooth_z.clear();
		smooth_rx.clear();
		smooth_ry.clear();
		smooth_rz.clear();
	}
	void change_to_joint(){
		joint_t = spine_t;
		cout << "spine_x : " << spine_x.size() << "  spine_t : " << spine_t.size()<<endl;
		spine_t.clear();
		int lll = spine_x.size();
		for(int ii=0;ii<lll;++ii){
			tm_target_p[0] = spine_x[ii];
			tm_target_p[1] = spine_y[ii];
			tm_target_p[2] = spine_z[ii];
			tm_target_p[3] = spine_rx[ii];
			tm_target_p[4] = spine_ry[ii];
			tm_target_p[5] = spine_rz[ii];
			if (GetQfromInverseKinematics(tm_target_p, tm_target_j)) {
				// ROS_WARN("j=%lf %lf %lf %lf %lf %lf\n",tm_target_j[0],tm_target_j[1],tm_target_j[2],tm_target_j[3],tm_target_j[4],tm_target_j[5]);
			}
			else {
				ROS_WARN("Target error");
			}
			joint_1.push_back(tm_target_j[0]);
			joint_2.push_back(tm_target_j[1]);
			joint_3.push_back(tm_target_j[2]);
			joint_4.push_back(tm_target_j[3]);
			joint_5.push_back(tm_target_j[4]);
			joint_6.push_back(tm_target_j[5]);
		}
		spine_x.clear();
		spine_y.clear();
		spine_z.clear();
		spine_rx.clear();
		spine_ry.clear();
		spine_rz.clear();
	}
	void spline_joint(){
		record_tag = 1;
		tk::spline s1(joint_t,joint_1);
		tk::spline s2(joint_t,joint_2);
		tk::spline s3(joint_t,joint_3);
		tk::spline s4(joint_t,joint_4);
		tk::spline s5(joint_t,joint_5);
		tk::spline s6(joint_t,joint_6);
		ros::Time ori=ros::Time::now();
		tm_msgs::SetVelocity vel_srv;
		cout <<  "spline ok" <<endl;

		double start_time = 0,end_time = joint_t [joint_t.size()-1];
		for (double i=start_time ; i<= end_time+0.1 ; i=i+0.03){
			while((ros::Time::now().toSec() - ori.toSec()) < (i-start_time)){
		      ros::Duration(0.001).sleep();
		    }
		    cout << "now time " << i ;
		    vel_cmd_gazebo.data.resize(6);
		    vel_cmd_gazebo.data[0] = (s1.deriv(1,i));
			vel_cmd_gazebo.data[1] = (s2.deriv(1,i));
			vel_cmd_gazebo.data[2] = (s3.deriv(1,i));
			vel_cmd_gazebo.data[3] = (s4.deriv(1,i));
			vel_cmd_gazebo.data[4] = (s5.deriv(1,i));
			vel_cmd_gazebo.data[5] = (s6.deriv(1,i));
			vel_gazebo.publish(vel_cmd_gazebo);
			if(record_tag){
				char outmsg_tag[200];
		    	sprintf(outmsg_tag,"%.7f %.7f %.7f %.7f %.7f %.7f %.7f",ros::Time::now().toSec()-ori.toSec(),
		    		vel_cmd_gazebo.data[0],vel_cmd_gazebo.data[1],vel_cmd_gazebo.data[2],
		    		vel_cmd_gazebo.data[3],vel_cmd_gazebo.data[4],vel_cmd_gazebo.data[5]);
		    	joint_vel << outmsg_tag << endl;
		    	sprintf(outmsg_tag,"%.7f %.7f %.7f %.7f %.7f %.7f %.7f",ros::Time::now().toSec()-ori.toSec(),
		    		s1(i),s2(i),s3(i),s4(i),s5(i),s6(i));
		    	joint_pos << outmsg_tag << endl;
		    	sprintf(outmsg_tag,"%.7f %.7f %.7f %.7f %.7f %.7f %.7f",ros::Time::now().toSec()-ori.toSec(),
		    		s1.deriv(2,i),s2.deriv(2,i),s3.deriv(2,i),s4.deriv(2,i),s5.deriv(2,i),s6.deriv(2,i));
		    	joint_acc << outmsg_tag << endl;
		    	cout << "  j1_vel : " << vel_cmd_gazebo.data[0] << endl; ;
			}
		}
		char outmsg_tag[200];
		vel_cmd_gazebo.data.resize(6);
		vel_cmd_gazebo.data[0] = (0);
		vel_cmd_gazebo.data[1] = (0);
		vel_cmd_gazebo.data[2] = (0);
		vel_cmd_gazebo.data[3] = (0);
		vel_cmd_gazebo.data[4] = (0);
		vel_cmd_gazebo.data[5] = (0);
		sprintf(outmsg_tag,"%.7f %.7f %.7f %.7f %.7f %.7f %.7f",ros::Time::now().toSec()-ori.toSec(),
		    		0.0,0.0,0.0,0.0,0.0,0.0);
		    	joint_vel << outmsg_tag << endl;
		
		vel_gazebo.publish(vel_cmd_gazebo);
	}

	void tm_callback(const geometry_msgs::Pose::ConstPtr& ToolPose)
	{	
	    tm_now_p[0] = ToolPose->position.x;
	    tm_now_p[1] = ToolPose->position.y;
	    tm_now_p[2] = ToolPose->position.z;

	     tf::Quaternion q(ToolPose->orientation.x,ToolPose->orientation.y,
	     					ToolPose->orientation.z,ToolPose->orientation.w);
	    tf::Matrix3x3 m(q);
	    // //Roll: rx, Pitch: ry, Yaw: rz, unit: rad
	    m.getRPY(tm_now_p[3], tm_now_p[4], tm_now_p[5]);
	    
		if(record_tag){
			char outmsg_tag[200];
	    	sprintf(outmsg_tag,"%.7f %.7f %.7f",tm_now_p[0],tm_now_p[1],tm_now_p[2]);
	    	end_pose << outmsg_tag << endl;
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tf_robot2target");

	Tag_Pose tag_pose;
	
	ros::AsyncSpinner spinner(4);
	spinner.start();
	while(ros::ok()){
		std::cout << "start\n";
		char cstr[512];
		fgets(cstr, 512, stdin);
		int n = (int)strlen(cstr);
		if (n > 0)
		{
			if (cstr[n - 1] == '\n')
				cstr[n - 1] = '\0';
		}
		if (strncmp(cstr, "go", 2) == 0)
		{
			printf("finish init\n");
			ROS_INFO("readtxt");
			tag_pose.readtxt();

			ROS_INFO("to_trajectory");
			tag_pose.to_trajectory();

			ROS_INFO("change_to_joint");
			tag_pose.change_to_joint();
			ROS_INFO("spline_joint");
			tag_pose.spline_joint();		
		}
		
	}
	ros::shutdown();
	return 0;
}