#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "action_tutorials_interfaces/action/get_pos.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_tutorials_cpp
{
class GetPosActionClient : public rclcpp::Node
{
public:
  using GetPos = action_tutorials_interfaces::action::GetPos;
  using GoalHandleGetPos = rclcpp_action::ClientGoalHandle<GetPos>;

  explicit GetPosActionClient(const rclcpp::NodeOptions & options)
  : Node("GetPos_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<GetPos>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "GetPos");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&GetPosActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = GetPos::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<GetPos>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&GetPosActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&GetPosActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&GetPosActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<GetPos>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(std::shared_future<GoalHandleGetPos::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleGetPos::SharedPtr,
    const std::shared_ptr<const GetPos::Feedback> feedback)
  {
    // std::stringstream ss;
    // ss << "Next number in sequence received: ";
    // for (auto number : feedback->partial_sequence) {
    //   ss << number << " ";
    // }
    RCLCPP_INFO(this->get_logger(), "当前位置：（%d,%d）",feedback->cur_x,feedback->cur_y);
  }

  void result_callback(const GoalHandleGetPos::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    // std::stringstream ss;
    // ss << "Result received: ";
    // for (auto number : result.result->sequence) {
    //   ss << number << " ";
    // }
    RCLCPP_INFO(this->get_logger(), "get all (x,y)");
    rclcpp::shutdown();
  }
};  // class GetPosActionClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::GetPosActionClient)