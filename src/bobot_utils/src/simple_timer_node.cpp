// |***************************************************************|
// |* (c) Copyright 2024                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

/* todos
lifecycle node
- add lifecycle state functions
save timer count to file, and recover when reboot
- difference between a fresh start and a restart?
- append to timer on each iteration
- read in on startup
figure out what the trigger time(s) should be and add a lower bound
*/

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
