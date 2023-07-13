#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/pos.hpp"        // CHANGE

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 2) { // CHANGE
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: Get Pos");      // CHANGE
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("send_pos_client"); // CHANGE
  rclcpp::Client<tutorial_interfaces::srv::Pos>::SharedPtr client =                        // CHANGE
    node->create_client<tutorial_interfaces::srv::Pos>("send_pos");                  // CHANGE

  auto request = std::make_shared<tutorial_interfaces::srv::Pos::Request>();               // CHANGE
  request->cnt = atoll(argv[1]);           // CHANGE

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pos: (%.3f,%.3f)", result.get()->x,result.get()->y);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service send_pos");    // CHANGE
  }

  rclcpp::shutdown();
  return 0;
}