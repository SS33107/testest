#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include <ruckig/ruckig.hpp>
#include "tm_kinematics/tm_kin.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
// #include <Eigen/Eigen>
// #include <Eigen/Dense>



#define NUMBER_OF_DOFS 6
using std::placeholders::_1;
using namespace std;
using namespace ruckig;
using namespace std::chrono_literals;


typedef struct{
    
    // double EndPoint_joint[6];
    // double EndPoint_cartesian[6];
    // std::vector<double> robot_now_joint;
    // std::vector<double> StartPoint;
    // std::vector<double> EndPoint;
    // std::vector<std::array<double, 6>> WayPoint;
    bool WayPointControl;
    
}RuckigInput;

class gazebo_control : public rclcpp::Node
{
  public:
    gazebo_control(): Node("minimal_subscriber"){
      robot_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&gazebo_control::robot_feedback_callback, this, _1));

      joint_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "/joint_cmd", 10, std::bind(&gazebo_control::cmd_callback, this, _1));

      vel_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_velocity_controller/commands", 10);
    }
    

  private:  
    RuckigInput RuckigInput_ga;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robot_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joint_cmd_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub;

    void robot_feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        printf("123");
        // RuckigInput_ga.robot_now_joint = { msg->position[0],msg->position[1],msg->position[4],msg->position[2],msg->position[3],msg->position[5]};
        // printf("j0: %f\n", RuckigInput_ga.robot_now_joint[0]);
    }
    
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        printf("cmd_callback");
        // RuckigInput_ga.EndPoint_cartesian[0] = msg->linear.x;
        // RuckigInput_ga.EndPoint_cartesian[1] = msg->linear.y;
        // RuckigInput_ga.EndPoint_cartesian[2] = msg->linear.z;
        // RuckigInput_ga.EndPoint_cartesian[3] = msg->angular.x;
        // RuckigInput_ga.EndPoint_cartesian[4] = msg->angular.y;
        // RuckigInput_ga.EndPoint_cartesian[5] = msg->angular.z;

        // GetQfromInverseKinematics(RuckigInput_ga.EndPoint_cartesian, RuckigInput_ga.EndPoint_joint);
        // for(int i=0;i<6;i++)
        //     RuckigInput_ga.EndPoint[i] = RuckigInput_ga.EndPoint_joint[i];
        
        // RukigWayPointRun(RuckigInput_ga,0.2);
        
        
    }
    void publish_vel(std::vector<double> vel_cmd)
    {
      auto message = std_msgs::msg::Float64MultiArray();
    //   message.data = vel_cmd;
    //   vel_pub->publish(message);
    }

    // bool pinv_SVD(const Eigen::MatrixXd &J, Eigen::MatrixXd &invJ)
    // {
    //     Eigen::VectorXd sigma;  //vector of singular values
    //     Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);

    //     sigma = svd.singularValues();
    //     int m = sigma.rows();

    //     for (int i = 0; i < m ; ++i)
    //     {
    //         if(sigma(i) > EPS)
    //             sigma(i) = 1.0/ sigma(i);
    //         else
    //             sigma(i) = 0.0;
    //     }

    //     invJ = svd.matrixV() * sigma.asDiagonal() * svd.matrixU().transpose();

    //     return true;
    // }
    // bool CheckJointLimit(double *q){
    //     bool valid = true;
    //     if (abs(q[0]) > 265 * DEG2RAD){
    //         // RCLCPP_WARN_SKIPFIRST("[Position] 1st joint position out of limit (270) : %lf", q[0] * RAD2DEG);
    //         valid = false;
    //     }
    //     else if (abs(q[1]) > 175 * DEG2RAD){
    //         // RCLCPP_WARN_SKIPFIRST("[Position] 2nd joint position out of limit (180): %lf", q[1] * RAD2DEG);
    //         valid = false;
    //     }
    //     else if (abs(q[2]) > 150 * DEG2RAD){
    //         // RCLCPP_WARN_SKIPFIRST("[Position] 3rd joint position out of limit (155): %lf", q[2] * RAD2DEG);
    //         valid = false;
    //     }
    //     else if (abs(q[3]) > 175 * DEG2RAD) {
    //         // RCLCPP_WARN_SKIPFIRST("[Position] 4th joint position out of limit (180): %lf", q[3] * RAD2DEG);
    //         valid = false;
    //     }
    //     else if (abs(q[4]) > 175 * DEG2RAD){
    //         // RCLCPP_WARN_SKIPFIRST("[Position] 5th joint position out of limit (180): %lf", q[4] * RAD2DEG);
    //         valid = false;
    //     }
    //     else if (abs(q[5]) > 265 * DEG2RAD){
    //         // RCLCPP_WARN_SKIPFIRST("[Position] 6th joint position out of limit (180): %lf", q[5] * RAD2DEG);
    //         valid = false;
    //     }
    //     else
    //         valid = true;

    //     return valid;
    // }
    // bool GetQfromInverseKinematics(double* CartesianPosition, double *q_inv)
    // {
    //     Eigen::Matrix<float, 4, 4> T_;
    //     Eigen::AngleAxisf yawAngle(CartesianPosition[5], Eigen::Vector3f::UnitZ());
    //     Eigen::AngleAxisf pitchAngle(CartesianPosition[4], Eigen::Vector3f::UnitY());
    //     Eigen::AngleAxisf rollAngle(CartesianPosition[3], Eigen::Vector3f::UnitX());
    //     Eigen::Quaternion<float> q = yawAngle * pitchAngle *rollAngle;
    //     Eigen::Matrix<float, 3, 3> RotationMatrix = q.matrix();
    //     double *T = new double[16];


    //     T_ << 0., 0., 0., CartesianPosition[0],
    //         0., 0., 0., CartesianPosition[1],
    //         0., 0., 0., CartesianPosition[2],
    //         0., 0., 0., 1.;

    //     T_.block<3, 3>(0, 0) = RotationMatrix.block<3, 3>(0, 0);

    //     tm_jacobian::Matrix2DoubleArray(T_, T);
    //     int num_sol = tm_kinematics::inverse(T, q_inv);
    //     delete[] T;
    //     return CheckJointLimit(q_inv);
    // }

    // bool RukigWayPointRun(RuckigInput IP_state, double MVel){
    //     // Create instances: the Ruckig OTG as well as input and output parameters
    //     const double control_cycle {0.025};
    //     const size_t max_number_of_waypoints {10};  // for memory allocation
    //     Ruckig<NUMBER_OF_DOFS> otg {control_cycle, max_number_of_waypoints};  // control cycle
    //     InputParameter<NUMBER_OF_DOFS> IP;
    //     OutputParameter<NUMBER_OF_DOFS> OP {max_number_of_waypoints};
        
    //     std::vector<double> VelocityCommand(6);

    //     // Set input parameters
    //     for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    //     {
    //         IP.current_position[i] = IP_state.robot_now_joint[i];
    //         IP.current_velocity[i] = 0;
    //         IP.current_acceleration[i] = 0;

    //         IP.target_position[i] = IP_state.EndPoint[i];
    //         IP.target_velocity[i] = 0;
    //         IP.target_acceleration[i] = 0;
            
    //         IP.max_acceleration[i] = 1;
    //         IP.max_jerk[i] = 3;
    //         IP.max_velocity[i] = MVel;
    //     }

    //     // for(auto wp: IP_state.WayPoint)
    //     //     IP.intermediate_positions.push_back(wp);
    //     IP.interrupt_calculation_duration = 500; // [µs]

    //     // Generate the trajectory within the control loop
    //     std::cout << "t | p1 | p2 | p3" << std::endl;
    //     while (otg.update(IP, OP) == Result::Working) 
    //     {
    //         // while (!vel_client->wait_for_service(1s)) {
    //         //     if (!rclcpp::ok()) {
    //         //         RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), 
    //         //             "Interrupted while waiting for the service. Exiting.");
    //         //         return false;
    //         //     }
    //         //     RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), 
    //         //         "service not available, waiting again...");
    //         // }

    //         if (OP.new_calculation) {
    //             std::cout << "Updated the trajectory:" << std::endl;
    //             std::cout << "  Reached target position in " << OP.trajectory.get_duration() << " [s]." << std::endl;
    //             std::cout << "  Calculation in " << OP.calculation_duration << " [µs]." << std::endl;
    //         }

    //         // The area execution in 25ms real time sharp
    //         auto& p = OP.new_velocity;
    //         std::cout << OP.time << " " << p[0] << " " << p[1] << " " << p[2] 
    //             << " " << p[3]  << " " << p[4] << " " << p[5] << std::endl;

    //         VelocityCommand = { OP.new_velocity[0],
    //                             OP.new_velocity[1],
    //                             OP.new_velocity[2],
    //                             OP.new_velocity[3],
    //                             OP.new_velocity[4],
    //                             OP.new_velocity[5] };

    //         //>>>>>>>>>>>> Send Velocity to TM-Driver <<<<<<<<<<<<<<
    //         // Vrequest -> velocity = VelocityCommand;
    //         // auto Vresult = vel_client->async_send_request(Vrequest);
    //         publish_vel(VelocityCommand);

    //         // Wait for the result.
    //         // if (rclcpp::spin_until_future_complete(node_setvel, Vresult) == rclcpp::executor::FutureReturnCode::SUCCESS)
    //         // {
    //         //     //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->ok);
    //         //     if( Vresult.get()-> ok )
    //         //         RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"OK");
    //         //     else
    //         //         RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"not OK");
    //         // }
    //         // else 
    //         //     RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service");

    //         OP.pass_to_input(IP);
    //         usleep(25*1000);
    //     }

    //     std::cout << "Trajectory duration: " << OP.trajectory.get_duration() << " [s]." << std::endl;
    //     // request_mode("off");

    //     return true;
    // }
};




int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gazebo_control>());
  
    // node_sendscript = rclcpp::Node::make_shared("demo_send_script");
    // node_setvel = rclcpp::Node::make_shared("demo_set_velocity");

    //request for use spdmode
    // client_sendscript = node_sendscript->create_client<tm_msgs::srv::SendScript>("send_script");
    // //set robot velocity
    // vel_client = node_setvel->create_client<tm_msgs::srv::SetVelocity>("set_velocity");

    
    // RuckigInput A;
    // A.StartPoint = {-1.4914166826053952, 0.01331910839432627, 1.8589293177688586, -0.3845250413649439, 1.4930353519159454, 0.11209542137052625};
    // A.EndPoint = {-0.6700550586894625, -0.2906813032420315, 1.9125914811017373, -0.5967474217853418, 2.0402833974683117, 0.11214390754712251};
    // A.WayPointControl = true;

    // // std::vector<double> w_point(6);
    // Vector w_point = {-1.1216667258101565, -0.2321916943801935, 1.9128528703627286, -0.7338430265400131, 1.4930549261499575, 0.11217300258203163};
 
    // A.WayPoint.push_back(w_point);

    // RukigWayPointRun(A, 0.2);

    rclcpp::shutdown();
    return 0;
}
