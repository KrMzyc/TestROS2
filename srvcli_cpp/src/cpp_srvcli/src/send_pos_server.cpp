#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/pos.hpp"     // CHANGE

#include <memory>

float getrand(int Min, int Max)
{
  return float(( rand() % (Max - Min + 1) ) + Min) ;
}


void getpos(const std::shared_ptr<tutorial_interfaces::srv::Pos::Request> request,     // CHANGE
          std::shared_ptr<tutorial_interfaces::srv::Pos::Response>       response)  // CHANGE
{
  response->x=getrand(request->cnt,request->cnt+10);
  response->y=getrand(request->cnt+2,request->cnt+20);                                      // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\ncnt: %ld",request->cnt);                                          // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: (%.3f,%.3f)", response->x,response->y);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("send_pos_server");  // CHANGE

  rclcpp::Service<tutorial_interfaces::srv::Pos>::SharedPtr service =                 // CHANGE
    node->create_service<tutorial_interfaces::srv::Pos>("send_pos", &getpos);     // CHANGE

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to Send Pos.");      // CHANGE

  rclcpp::spin(node);
  rclcpp::shutdown();
}