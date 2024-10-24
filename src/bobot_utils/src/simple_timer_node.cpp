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
*/

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class SimpleTimer : public rclcpp::Node
{
public:
  SimpleTimer()
  : Node("simple_timer"), count_(0)
  {
    // Lower bound: 15 min; upper bound: 65 min
    this->declare_parameter("time_lower_bound", 15 * 60);
    this->declare_parameter("time_upper_bound", 65 * 60);
    publisher_ = this->create_publisher<std_msgs::msg::UInt16>("/bobot_timer/time", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&SimpleTimer::publishTime, this));
    
    client_lower_ = this->create_client<std_srvs::srv::Trigger>("/bobot_timer/is_past_premie_date");
    client_upper_ = this->create_client<std_srvs::srv::Trigger>("/bobot_timer/is_past_due_date");
  }

private:
  void publishTime()
  {
    auto message = std_msgs::msg::UInt16();
    message.data = this->count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%i'", message.data);
    this->publisher_->publish(message);
    long unsigned int lower_bound = (long unsigned int)(this->get_parameter("time_lower_bound").as_int());
    long unsigned int upper_bound = (long unsigned int)(this->get_parameter("time_upper_bound").as_int());
    if (count_ > lower_bound && notified_ == 0) {
      pastPremieNotification();
      notified_++;
    }
    else if (count_ > upper_bound && notified_ == 1) {
      pastDueNotification();
      notified_++;
    }
  }

 void pastPremieNotification()
  {
    // Check if the service is available
    while (!client_lower_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Waiting for the service to be available...");
    }

    // Notify manager
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client_lower_->async_send_request(request);

    RCLCPP_INFO(this->get_logger(), "Notified the manager node.");
  }

 void pastDueNotification()
  {
    // Check if the service is available
    while (!client_upper_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Waiting for the service to be available...");
    }

    // Notify manager
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client_upper_->async_send_request(request);

    RCLCPP_INFO(this->get_logger(), "Notified the manager node.");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr publisher_;
  size_t count_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;

  // 0 before any notifs sent; 1 after premie notif sent; 2 after past due notif sent
  bool notified_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleTimer>());
  rclcpp::shutdown();
  return 0;
}
