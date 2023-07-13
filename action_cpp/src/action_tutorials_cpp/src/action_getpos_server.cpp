#include <functional>
#include <memory>
#include <thread>
#include <ctime>

#include "action_tutorials_interfaces/action/get_pos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_tutorials_cpp/visibility_control.h"

namespace action_tutorials_cpp
{
class GetPosActionServer : public rclcpp::Node
{
public:
  using GetPos = action_tutorials_interfaces::action::GetPos;
  using GoalHandleGetPos = rclcpp_action::ServerGoalHandle<GetPos>;

  ACTION_TUTORIALS_CPP_PUBLIC
  explicit GetPosActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("GetPos_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<GetPos>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "GetPos",
      std::bind(&GetPosActionServer::handle_goal, this, _1, _2),
      std::bind(&GetPosActionServer::handle_cancel, this, _1),
      std::bind(&GetPosActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<GetPos>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GetPos::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request to get (x,y) with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGetPos> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGetPos> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&GetPosActionServer::execute, this, _1), goal_handle}.detach();
  }

  int getrand(int Min, int Max)
  {
    return ( rand() % (Max - Min + 1) ) + Min ;
  }

  void execute(const std::shared_ptr<GoalHandleGetPos> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GetPos::Feedback>();
    // auto & sequence = feedback->partial_sequence;
    // sequence.push_back(0);
    // sequence.push_back(1);
    srand((unsigned)time(NULL));
    auto x=getrand(1,10);
    auto y=getrand(1,20);
    feedback->cur_x=x;
    feedback->cur_y=y;
    auto result = std::make_shared<GetPos::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      feedback->cur_x++;
      feedback->cur_y--;
      if (goal_handle->is_canceling()) {
        result->result = -1;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->result = 1;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class GetPosActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::GetPosActionServer)