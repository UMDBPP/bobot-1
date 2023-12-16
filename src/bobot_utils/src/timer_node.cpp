// |***************************************************************|
// |* (c) Copyright 2023                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <bobot_msgs/msg/bobot_timer.hpp> // No idea why it doesnt build as "BobotTimer.hpp" but we ball I gues

// C++ specific libraries
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

using namespace std::literals::chrono_literals;
// using namespace bobot; 


class BobotTimerNode : public rclcpp_lifecycle::LifecycleNode
{

public:

    // Constructor -  needs a node name and a boolean that indicates if we are using intra_process_commms (?))
    explicit BobotTimerNode(const std::string & node_name, bool intra_process_comms = false) : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {
        // Get the current bobot's name for logging purposes
        this->declare_parameter("bobot_name", rclcpp::PARAMETER_STRING);
        bobot_name = this->get_parameter("bobot_name").as_string();
        RCLCPP_INFO(get_logger(), "Bobot timer node has succesfully launched for %s, starting state is unconfigured state", bobot_name.c_str());
    }


    void publish_time_count()
    {
        std::unique_ptr<bobot_msgs::msg::BobotTimer> timer_msg = std::make_unique<bobot_msgs::msg::BobotTimer>(); // make a unique point to our ROS message object
        time_val = time_val + 1;
        timer_msg->time = time_val;
        timer_msg->activated = timer_activated;
        timer_msg->time_limit_reached = time_limit_reached;

        if(timer_publisher_->is_activated() != true && has_started != -1) //  If we're not activated, lets count how many seconds it takes. If we've already activated and we're shut down, don't send message
        {
            RCLCPP_WARN(get_logger(), "Bobot Timer has not started yet, this should not last longer than a few seconds");
            has_started = has_started + 1;
        }
        if(has_started == 15)
        {
            RCLCPP_ERROR(get_logger(), "Bobot Timer Node has taken longer than 15 seconds to start, logging error!");
            timer_msg->took_too_long = true;
            RCLCPP_ERROR(get_logger(), "Bobot Manager should be attempting to resolve this issue, hang tight!");
            has_started = -1; // set this to -1 so that we don't spam the message. The manager node will take care of it now
        }
        timer_publisher_->publish(std::move(timer_msg));
    }

    /*
        The following are declerations to the callback functions for on_configure, on_active, on_deactivate, on_cleanup, and on_shutdown.
        There are inherited lifecycle methods from the lifecycle node class definition, and we are simply overriding them
    */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State&)
    {
        topic_name = bobot_name + "/timer";
        timer_publisher_ = this->create_publisher<bobot_msgs::msg::BobotTimer>(topic_name, 10); // Create a publisher with a namespace of bobot1 called timer, and have the queue size be 10 (since it should publish at 1 hz)
        bobotTimer_callback_ = this->create_wall_timer(loop_rate, [this]() -> void { publish_time_count(); }); // Weird sytnax, but I think this is basically creating the callback function call at the specified spin rate
        
        // Add log information, incase our logging fails
        RCLCPP_INFO(get_logger(), "Attempting to configure %s timer, please hold!", bobot_name.c_str());


        // -- FROM ROS REFERENCE -- //
        // We return a success and hence invoke the transition to the next step: "inactive".
        // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
        // would stay in the "unconfigured" state.
        // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
        // this callback, the state machine transitions to state "errorprocessing".

        // TL:DR, we update the state of the node to say that on_configure successfully called. If the on_configure function isn't successful, it should either not update the state or change it to error_processing
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State&)
    {
        // -- FROM ROS -- //
        // on_activate callback is being called when the lifecycle node enters the "activating" state.

        // Here, we are activating the node, allowing it publish messages
        timer_publisher_->on_activate(); // Call the activatio functions
        timer_activated = true; // set the timer activated to be true
        has_started = -1;
        RCLCPP_INFO(get_logger(), "%s's timer has been activated", bobot_name.c_str()); // use the ros logger incase our logging messes ups
        
        // We return a success and hence invoke the transition to the next step: "active".
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; // let the people know we've activated. In theory, this would so something important
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&)
    {
        // -- FROM ROS -- //
        // on_deactivate callback is being called when the lifecycle node enters the "deactivating" state.

        // Here, we are deactivating the node, which no longer allows its messages to go through
        timer_publisher_->on_deactivate(); // Call the activation functions
        timer_activated = false; // set the timer activated to be true
        RCLCPP_INFO(get_logger(), "%s's timer has been deactivated... This should only occur if we've reached the time limit!", bobot_name.c_str()); // use the ros logger incase our logging messes ups
        time_limit_reached = true; //publish that we've reached the time limit
        RCLCPP_WARN(get_logger(), "Bobot Timer Node confirms time limit reached and has deactivated, preparing to shut down node");

        // We return a success and hence invoke the transition to the next step: "inactive".
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; // let the people know we've activated. In theory, this would so something important
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&)
    {
        // -- FROM ROS -- //
        // on_cleanup callback is being called when the lifecycle node enters the "cleaningup" state.
        // In our cleanup phase, we release the shared pointers to the
        // timer and publisher. These entities are no longer available
        // and our node is "clean".

        bobotTimer_callback_.reset(); // release the timer first
        timer_publisher_.reset(); // release the publisher after the timer

        RCLCPP_INFO(get_logger(), "Bobot Timer Node has begun cleaning up");

        // We return a success and hence invoke the transition to the next step: "unconfigured".
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state)
    {
        // -- FROM ROS -- //
        // on_shutdown callback is being called when the lifecycle node enters the "shuttingdown" state.
        // In our shutdown phase, we release the shared pointers to the
        // timer and publisher. These entities are no longer available
        // and our node is "clean".
    
        // similar to the cleanup phase, although one is meant to cleanup the node, and the other is mean to shut down the node. We will be using the shutdown method
        bobotTimer_callback_.reset(); // release the timer first
        timer_publisher_.reset(); // release the publisher after the timer

        RCLCPP_INFO(get_logger(), "Bobot Timer Node has shut down");
        RCLCPP_INFO(get_logger(), "shutdown command was called from state %s", state.label().c_str());

        // We return a success and hence invoke the transition to the next step: "finalized".
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:

// ---- BIG INFO BLOCKS ARE DIRECTLY FROM ROS ---- //
/*
    We hold an instance of a lifecycle publisher. This lifecycle publisher can 
    be activated or deactivated regarding on which state the lifecycle node is in.
    By default, a lifecycle publisher is inactive by creation and has to be
    activated to publish messages into the ROS world.
*/
    // This is a regular ROS publisher, BUT it follows the rules of the lifecycle management, as in it won't do JACK until you set a certain state
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<bobot_msgs::msg::BobotTimer>> timer_publisher_;

/*
    We hold an instance of a timer which periodically triggers the publish function.
    As for the beta version, this is a regular timer. In a future version, a
    lifecycle timer will be created which obeys the same lifecycle management as the
    lifecycle publisher.
*/  
    /// Same as above, this is a ros timer that regulates the rate at which the publisher publishhes data. Should follow lifecycle management (someho?)
    std::shared_ptr<rclcpp::TimerBase> bobotTimer_callback_;

    std::string bobot_name;
    std::string topic_name;
    uint32_t time_val = 0; // Keep track of the timer value
    int has_started = 0; // Check if we have started yet
    bool timer_activated = false; // Let the manager know if we've activated (for logging purposes)
    bool time_limit_reached = false; // Let the manager know if we've confirmed that we've reached our time limit (for logging purposes)
    std::chrono::seconds loop_rate = 1s; // Create the loop rate variable (its 1 second (using chrono literals) because thats the time we are going for)
};


int main(int argc, char* argv[])
{
    // -- FROM ROS -- //
    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    rclcpp::init(argc, argv); // initialize ROS
    rclcpp::executors::SingleThreadedExecutor bobot_exec; // created an executor on a single thread, because thats whats recommended (still learning about this)
    std::shared_ptr<BobotTimerNode> bobot_timer_node = std::make_shared<BobotTimerNode>("BobotTimer"); // create the ros node
    bobot_exec.add_node(bobot_timer_node->get_node_base_interface()); // add the nodes base class (lifecycle stuff)
    bobot_exec.spin(); // start spinning the node
    rclcpp::shutdown(); // Shutdown cleanly when we're done (ALL OF ROS, not just the node)

    return 0;
}