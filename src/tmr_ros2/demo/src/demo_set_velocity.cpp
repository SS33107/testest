#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/srv/set_velocity.hpp"
#include "tm_msgs/srv/send_script.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

bool send_cmd(std::shared_ptr<rclcpp::Node> node, rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client){
  auto request = std::make_shared<tm_msgs::srv::SendScript::Request>();
  request->id = "spdmodeoff";
  request->script = "StopContinueVmode()";

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->ok);
    if(result.get()->ok){
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"OK");
    } else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"not OK");
    }

  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }
  return true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);


  std::shared_ptr<rclcpp::Node> node_setvel = rclcpp::Node::make_shared("demo_set_velocity");
  rclcpp::Client<tm_msgs::srv::SetVelocity>::SharedPtr vel_client =
    node_setvel->create_client<tm_msgs::srv::SetVelocity>("set_velocity");

  //velocity mode on, tell service i want veclocity control
  std::shared_ptr<rclcpp::Node> node_sendscript = rclcpp::Node::make_shared("demo_send_script");
  rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client_sendscript =
    node_sendscript->create_client<tm_msgs::srv::SendScript>("send_script");

  send_cmd(node_sendscript, client_sendscript);

  //裝禮物
  auto request = std::make_shared<tm_msgs::srv::SetVelocity::Request>();
  request->motion_type = 1;
  std::vector<double> VelocityCommand(6);
  VelocityCommand={-0.1,0.0,0.0,0.0,0.0,0.0};
  request->velocity=VelocityCommand;
  rclcpp::WallRate loop_rate(40);

  while(1){
    while (!vel_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = vel_client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_setvel, result) ==
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->ok);
      if(result.get()->ok){
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"OK");
      } else{
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"not OK");
      }
    } else {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service");
    }
    loop_rate.sleep();
  }
  
  rclcpp::shutdown();
  return 0;
}
