// |***************************************************************|
// |* (c) Copyright 2023                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <bobot_msgs/msg/altitude_monitor.hpp> // AltitudeMonitor msg file
#include <bobot_utils/bobot_common_types.hpp> // custom library


// C++ specific libraries
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

using namespace std::literals::chrono_literals;
using namespace bobot; 


class BobotAltitudeMonitor : public rclcpp_lifecycle::LifecycleNode
{

public:

    // Constructor -  needs a node name and a boolean that indicates if we are using intra_process_commms (?))
    explicit BobotAltitudeMonitor(const std::string & node_name, bool intra_process_comms = false) : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {
        // Get the name of the current bobot for ros-logging purposes
        this->declare_parameter("bobot_name", rclcpp::PARAMETER_STRING);
        this->declare_parameter("device_type", rclcpp::PARAMETER_STRING);
        bobot_name = this->get_parameter("bobot_name").as_string();
        device_type = this->get_parameter("device_type").as_string();
        RCLCPP_INFO(get_logger(), "Getting parameters for Altitude Monitor node...");
        parameter_helper(); // get the parameters
        RCLCPP_INFO(get_logger(), "Altitude Monitor node has succesfully launched for %s, starting state is unconfigured state", bobot_name.c_str());
    }

    void publish_altitude_info()
    {
        std::unique_ptr<bobot_msgs::msg::AltitudeMonitor> altitude_info_msg = std::make_unique<bobot_msgs::msg::AltitudeMonitor>(); // make a unique point to our ROS message object

        /*
            Add code that will access the board and get a data and checks what alt we are at
        */

        altitude_info_->publish(std::move(altitude_info_msg));
    }

    /*
        The following are declerations to the callback functions for on_configure, on_active, on_deactivate, on_cleanup, and on_shutdown.
        There are inherited lifecycle methods from the lifecycle node class definition, and we are simply overriding them
    */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State&)
    {
        topic_name = bobot_name + "/altitude_monitor";
        altitude_info_ = this->create_publisher<bobot_msgs::msg::AltitudeMonitor>(topic_name, 50); // Create a publisher with a namespace of bobot1 called timer, and have the queue size be 10 (since it should publish at 1 hz)
        bobotAltitude_callback_ = this->create_wall_timer(std::chrono::milliseconds(sampling_rate), [this]() -> void { publish_altitude_info(); }); // Weird sytnax (lambda syntax), but I think this is basically creating the callback function call at the specified spin rate
        
        // Add log information, incase our logging fails
        RCLCPP_INFO(get_logger(), "Attempting to configure Altitude Monitor, please hold!");


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
        altitude_info_->on_activate(); // Call the activatio functions
        reading = true; // set the reading to be true, since we're activated
        RCLCPP_INFO(get_logger(), "Altitude Monitor has been activated has been activated"); // use the ros logger incase our logging messes ups
        
        // We return a success and hence invoke the transition to the next step: "active".
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; // let the people know we've activated. In theory, this would so something important
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&)
    {
        // -- FROM ROS -- //
        // on_deactivate callback is being called when the lifecycle node enters the "deactivating" state.

        // Here, we are deactivating the node, which no longer allows its messages to go through
        altitude_info_->on_deactivate(); // Call the activation functions
        reading = false; // set the timer activated to be false
        RCLCPP_INFO(get_logger(), "Servo jerking has been deactivated... we should be starting motion!"); // use the ros logger incase our logging messes ups

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

        bobotAltitude_callback_.reset(); // release the timer first
        altitude_info_.reset(); // release the publisher after the timer

        RCLCPP_INFO(get_logger(), "Servo Jerk Node has begun cleaning up");

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
        bobotAltitude_callback_.reset(); // release the timer first
        altitude_info_.reset(); // release the publisher after the timer

        RCLCPP_INFO(get_logger(), "Servo Jerk Node has shut down");
        RCLCPP_INFO(get_logger(), "shutdown command was called from state %s", state.label().c_str());

        // We return a success and hence invoke the transition to the next step: "finalized".
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    // function to open the serial port for arduino communication
    bool initializeComms()
    {
        // todo
    }

protected:
    // functions to help get parameters from the node's parameter server
    // this is a protected function because I don't want people to access it outside of the node if they TRIED but I also want inherited classes to use it
    void parameter_helper()
    {
        // Get the rest of the parameters that we need for this node!
        // First, we delcare the parameters
        std::string device_params = device_type + ".pins";
        this->declare_parameter(device_params, rclcpp::PARAMETER_STRING_ARRAY);
        this->declare_parameter("sample_rate", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("max_altitude", rclcpp::PARAMETER_INTEGER);

        // Then, we get the parameters
        pin_info.pins = this->get_parameter(device_params).as_string_array();
        sampling_rate = this->get_parameter("sample_rate").as_int();
        max_alt = this->get_parameter("max_altitude").as_int();
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
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<bobot_msgs::msg::AltitudeMonitor>> altitude_info_;

/*
    We hold an instance of a timer which periodically triggers the publish function.
    As for the beta version, this is a regular timer. In a future version, a
    lifecycle timer will be created which obeys the same lifecycle management as the
    lifecycle publisher.
*/  
    /// Same as above, this is a ros timer that regulates the rate at which the publisher publishhes data. Should follow lifecycle management (someho?)
    std::shared_ptr<rclcpp::TimerBase> bobotAltitude_callback_;

    std::string bobot_name; // private property for the bobot name
    std::string device_type; // the type of device we are using
    std::string topic_name; // private property for the topic name
    PinInfo pin_info; // a structure to hold our pin information
    int64_t sampling_rate; // servo jerk rate, defined in the servo_info.yaml file in the bobot_utils/hardware folder
    bool reading = false; // boolean to indicate if we are reading data
    int64_t max_alt; // the max altitude, since we want to inform the manager when we've reached it
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
    std::shared_ptr<BobotAltitudeMonitor> bobot_alt_monitor = std::make_shared<BobotAltitudeMonitor>("AltitudeMonitor"); // create the ros node
    bobot_exec.add_node(bobot_alt_monitor->get_node_base_interface()); // add the nodes base class (lifecycle stuff)
    bobot_exec.spin(); // start spinning the node
    rclcpp::shutdown(); // Shutdown cleanly when we're done (ALL OF ROS, not just the node)

    return 0;
}