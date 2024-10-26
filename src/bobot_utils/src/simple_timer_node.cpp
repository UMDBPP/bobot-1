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

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class SimpleTimer : public rclcpp::Node
{
public:
  SimpleTimer()
  : Node("simple_timer"), count_(0)
  {
    this->declare_parameter("time_upper_bound", 60 * 60 * 60);
    publisher_ = this->create_publisher<std_msgs::msg::UInt16>("/bobot_timer/time", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&SimpleTimer::publishTime, this));
    
    client_ = this->create_client<std_srvs::srv::Trigger>("/bobot_timer/is_past_due_date");
  }

private:
  void publishTime()
  {
    auto message = std_msgs::msg::UInt16();
    message.data = this->count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%i'", message.data);
    this->publisher_->publish(message);
    long unsigned int upper_bound = (long unsigned int)(this->get_parameter("time_upper_bound").as_int());
    if (count_ > upper_bound && !notified_)
    {
      pastDueNotification();
      notified_ = true;
    }
  }

 void pastDueNotification()
  {
    // Check if the service is available
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Waiting for the service to be available...");
    }

    // Notify manager
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client_->async_send_request(request);

    RCLCPP_INFO(this->get_logger(), "Notified the manager node.");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr publisher_;
  size_t count_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;

  bool notified_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleTimer>());
  rclcpp::shutdown();
  return 0;
}
