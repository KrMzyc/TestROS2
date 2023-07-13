// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <vector>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/pos.hpp" 
using std::placeholders::_1;
float pre_x=0.0;
float pre_y=0.0;
class MinimalSubscriber : public rclcpp::Node
{
public:
  
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<tutorial_interfaces::msg::Pos>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  
  void topic_callback(const tutorial_interfaces::msg::Pos::SharedPtr msg) const
  {
    float current_x=msg->x;
    float current_y=msg->y;
    float delta_x=abs(current_x-pre_x);
    float delta_y=abs(current_y-pre_y);
    float dis=sqrt(pow(delta_x,2)+pow(delta_y,2));
    RCLCPP_INFO(this->get_logger(), "Pre Pos: (%.3f,%.3f) ----> Current Pos: (%.3f,%.3f), Move Distance: %.3f", pre_x,pre_y,msg->x,msg->y,dis);
    pre_x=current_x;
    pre_y=current_y;
  }
  rclcpp::Subscription<tutorial_interfaces::msg::Pos>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
