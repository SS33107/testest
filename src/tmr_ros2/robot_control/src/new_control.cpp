#include <iostream>
#include <ruckig/ruckig.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/srv/set_velocity.hpp"
#include "tm_msgs/srv/send_script.hpp"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#define NUMBER_OF_DOFS 6

using namespace ruckig;
using namespace std::chrono_literals;
using Vector = std::array<double, 6>;

std::shared_ptr<rclcpp::Node> node_sendscript;
std::shared_ptr<rclcpp::Node> node_setvel;

rclcpp::Client<tm_msgs::srv::SetVelocity>::SharedPtr vel_client;
rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client_sendscript;

typedef struct{
    std::vector<double> StartPoint;
    std::vector<double> EndPoint;
    bool WayPointControl;
    std::vector<Vector> WayPoint;
}RuckigInput;

bool RukigWayPointRun(RuckigInput IP_state, double MVel);
bool request_mode(std::string RT_MODE);


bool request_mode(std::string RT_MODE)
{
    auto request = std::make_shared<tm_msgs::srv::SendScript::Request>();
    if(RT_MODE == "on"){
        request->id = "Vstart";
        request->script = "ContinueVJog()";
    }
    else if(RT_MODE == "off"){
        request->id = "spdmodeoff";
        request->script = "StopContinueVmode()";
    }
    else{
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "FALSE requset type. Exiting.");
        return false;
    }

    //wait for server
    while (!client_sendscript->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto request_result = client_sendscript->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_sendscript, request_result) ==
            rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->ok);
        if(request_result.get()->ok){
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"OK");
        } 
        else {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"not OK");
        }
    } 
    else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service");
    }

    return true;
}

// bool RukigPositionRun(RuckigInput IP_state, double SynTime)
bool RukigWayPointRun(RuckigInput IP_state, double MVel)
{
    // Create instances: the Ruckig OTG as well as input and output parameters
    const double control_cycle {0.025};
    const size_t max_number_of_waypoints {10};  // for memory allocation
    Ruckig<NUMBER_OF_DOFS> otg {control_cycle, max_number_of_waypoints};  // control cycle
    InputParameter<NUMBER_OF_DOFS> IP;
    OutputParameter<NUMBER_OF_DOFS> OP {max_number_of_waypoints};
    
    //send tm5 request
    request_mode("on");
    auto Vrequest = std::make_shared<tm_msgs::srv::SetVelocity::Request>();
    Vrequest-> motion_type = 1;
    std::vector<double> VelocityCommand(6);

    // Set input parameters
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
	{
        IP.current_position[i] = IP_state.StartPoint[i];
        IP.current_velocity[i] = 0;
        IP.current_acceleration[i] = 0;

        IP.target_position[i] = IP_state.EndPoint[i];
        IP.target_velocity[i] = 0;
        IP.target_acceleration[i] = 0;
        
        IP.max_acceleration[i] = 1;
        IP.max_jerk[i] = 3;
        IP.max_velocity[i] = MVel;
    }

    // for(auto wp: IP_state.WayPoint)
    //     IP.intermediate_positions.push_back(wp);
    IP.interrupt_calculation_duration = 500; // [µs]

    // Generate the trajectory within the control loop
    std::cout << "t | p1 | p2 | p3" << std::endl;
    while (otg.update(IP, OP) == Result::Working) 
    {
        while (!vel_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), 
                    "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), 
                "service not available, waiting again...");
        }

        if (OP.new_calculation) {
            std::cout << "Updated the trajectory:" << std::endl;
            std::cout << "  Reached target position in " << OP.trajectory.get_duration() << " [s]." << std::endl;
            std::cout << "  Calculation in " << OP.calculation_duration << " [µs]." << std::endl;
        }

        // The area execution in 25ms real time sharp
        auto& p = OP.new_velocity;
        std::cout << OP.time << " " << p[0] << " " << p[1] << " " << p[2] 
            << " " << p[3]  << " " << p[4] << " " << p[5] << std::endl;

        VelocityCommand = { OP.new_velocity[0],
							OP.new_velocity[1],
							OP.new_velocity[2],
							OP.new_velocity[3],
							OP.new_velocity[4],
							OP.new_velocity[5] };

        //>>>>>>>>>>>> Send Velocity to TM-Driver <<<<<<<<<<<<<<
        Vrequest -> velocity = VelocityCommand;
        auto Vresult = vel_client->async_send_request(Vrequest);

        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node_setvel, Vresult) == rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->ok);
            if( Vresult.get()-> ok )
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"OK");
            else
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"not OK");
        }
        else 
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service");

        OP.pass_to_input(IP);
        usleep(25*1000);
    }

    std::cout << "Trajectory duration: " << OP.trajectory.get_duration() << " [s]." << std::endl;
    request_mode("off");

    return true;
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);

    node_sendscript = rclcpp::Node::make_shared("demo_send_script");
    node_setvel = rclcpp::Node::make_shared("demo_set_velocity");

    //request for use spdmode
    client_sendscript = node_sendscript->create_client<tm_msgs::srv::SendScript>("send_script");
    //set robot velocity
    vel_client = node_setvel->create_client<tm_msgs::srv::SetVelocity>("set_velocity");

    
    RuckigInput A;
    A.StartPoint = {-1.4914166826053952, 0.01331910839432627, 1.8589293177688586, -0.3845250413649439, 1.4930353519159454, 0.11209542137052625};
    A.EndPoint = {-0.6700550586894625, -0.2906813032420315, 1.9125914811017373, -0.5967474217853418, 2.0402833974683117, 0.11214390754712251};
    A.WayPointControl = true;

    // std::vector<double> w_point(6);
    Vector w_point = {-1.1216667258101565, -0.2321916943801935, 1.9128528703627286, -0.7338430265400131, 1.4930549261499575, 0.11217300258203163};
 
    A.WayPoint.push_back(w_point);

    RukigWayPointRun(A, 0.2);

    rclcpp::shutdown();
    return 0;
}
