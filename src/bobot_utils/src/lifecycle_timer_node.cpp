// |***************************************************************|
// |* (c) Copyright 2024                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

/* todos
lifecycle node
- make sure that the state functions do everything they need to do
- debug lifecycle state functions that I copy pasted
save timer count to file, and recover when reboot
- difference between a fresh start and a restart?
- append to timer on each iteration
- read in on startup
*/

#include <chrono>
#include <memory>

#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

class Timer : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit Timer(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode("timer",
    rclcpp:NodeOptions().use_intra_process_comms(intra_process_comms)), count_(0)
  {}

private:
  void publishTime()
  {
    // TODO: if node is not active, perhaps block publishing
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

 /// Transition callback for state configuring
  /**
   * on_configure callback is being called when the lifecycle node
   * enters the "configuring" state.
   * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
   */
 rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    // Lower bound: 15 min; upper bound: 65 min
    this->declare_parameter("time_lower_bound", 15 * 60);
    this->declare_parameter("time_upper_bound", 65 * 60);
    
    timer_ = this->create_wall_timer(1000ms, std::bind(&SimpleTimer::publishTime, this));
    publisher_ = this->create_publisher<std_msgs::msg::UInt16>("/bobot_timer/time", 10);
    client_lower_ = this->create_client<std_srvs::srv::Trigger>("/bobot_timer/is_past_premie_date");
    client_upper_ = this->create_client<std_srvs::srv::Trigger>("/bobot_timer/is_past_due_date");
    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

/// Transition callback for state activating
  /**
   * on_activate callback is being called when the lifecycle node
   * enters the "activating" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "active" state or stays
   * in "inactive".
   * TRANSITION_CALLBACK_SUCCESS transitions to "active"
   * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state)
  {
    // The parent class method automatically transition on managed entities
    // (currently, LifecyclePublisher).
    // pub_->on_activate() could also be called manually here.
    // Overriding this method is optional, a lot of times the default is enough.
    LifecycleNode::on_activate(state);

    RCLCPP_INFO(get_logger(), "on_activate() is called.");

    // TODO: do work here??

    // We return a success and hence invoke the transition to the next
    // step: "active".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "inactive" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

/// Transition callback for state deactivating
  /**
   * on_deactivate callback is being called when the lifecycle node
   * enters the "deactivating" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "inactive" state or stays
   * in "active".
   * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
   * TRANSITION_CALLBACK_FAILURE transitions to "active"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state)
  {
    // The parent class method automatically transition on managed entities
    // (currently, LifecyclePublisher).
    // pub_->on_deactivate() could also be called manually here.
    // Overriding this method is optional, a lot of times the default is enough.
    LifecycleNode::on_deactivate(state);

    RCLCPP_INFO(get_logger(), "on_deactivate() is called.");

    // We return a success and hence invoke the transition to the next
    // step: "inactive".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "active" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /// Transition callback for state cleaningup
  /**
   * on_cleanup callback is being called when the lifecycle node
   * enters the "cleaningup" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "unconfigured" state or stays
   * in "inactive".
   * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
   * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    // In our cleanup phase, we release the shared pointers to the
    // timer and publisher. These entities are no longer available
    // and our node is "clean".
    timer_.reset();
    pub_.reset();

    RCLCPP_INFO(get_logger(), "on cleanup is called.");

    // We return a success and hence invoke the transition to the next
    // step: "unconfigured".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "inactive" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /// Transition callback for state shutting down
  /**
   * on_shutdown callback is being called when the lifecycle node
   * enters the "shuttingdown" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "finalized" state or stays
   * in its current state.
   * TRANSITION_CALLBACK_SUCCESS transitions to "finalized"
   * TRANSITION_CALLBACK_FAILURE transitions to current state
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state)
  {
    // In our shutdown phase, we release the shared pointers to the
    // timer and publisher. These entities are no longer available
    // and our node is "clean".
    timer_.reset();
    pub_.reset();

    RCLCPP_INFO(get_logger(), "on shutdown is called from state %s.", state.label().c_str());

    // We return a success and hence invoke the transition to the next
    // step: "finalized".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the current state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
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
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  
  rclcpp::init(argc, argv);
  
  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<Timer> lc_node =
  std::make_shared<Timer>("timer");

  exe.add_node(lc_node->get_node_base_interface());

  exe.spin();
  
  rclcpp::shutdown();
  
  return 0;
}
