// |***************************************************************|
// |* (c) Copyright 2023                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <bobot_msgs/msg/altitude_monitor.hpp> // AltitudeMonitor .msg file
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "bobot_hardware_interface/bobot_altimeter_interface.hpp"

// C++ specific libraries
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

class BobotAltitudeMonitor : public rclcpp_lifecycle::LifecycleNode
{

public:

    // Constructor -  needs a node name and a boolean that indicates if we are using intra_process_commms (?))
    explicit BobotAltitudeMonitor(const std::string & node_name, bool intra_process_comms = false) : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {
        // Get the name of the current bobot for ros-logging purposes
        this->declare_parameter("BOBOT_NAME", rclcpp::PARAMETER_STRING);
        bobot_name = this->get_parameter("BOBOT_NAME").as_string();
        this->print_debug_message("Getting parameters for Altitude Monitor node...");

        // Get the parameters
        parameter_helper(); 
        this->print_debug_message("Altitude Monitor node has succesfully launched! Starting state is [unconfigured]");
    }

    void print_debug_message(std::string message)
    {
        RCLCPP_INFO(this->get_logger(), "[USER DEBUG LOG] %s", message.c_str()); // just a helper to mroe easily print generic debug messages
        return;
    }
    void print_warning_message(std::string message)
    {
        RCLCPP_WARN(this->get_logger(), "[USER WARN LOG] %s", message.c_str()); // just another helper to more easily print generid warning messages
        return;
    }
    void print_error_message(std::string message)
    {
        RCLCPP_ERROR(this->get_logger(), "[USER ERROR LOG] %s", message.c_str()); // just another helper to more easily print generid warning messages
        return;
    }
    std::string get_current_time_for_logs()
    {
        // Stolen from Rahul
        std::ostringstream oss; // Make an ostringstream object
        auto t = std::time(nullptr); // Make a time objects
        auto tm = *std::localtime(&t); // Make a tm pointer thingy
        oss << std::put_time(&tm, "%d-%H:%M:%S"); // Do this stuff
        return oss.str(); // return da string
    }
    std::string get_current_time()
    {
        // Stolen from Rahul
        std::ostringstream oss; // Make an ostringstream object
        auto t = std::time(nullptr); // Make a time objects
        auto tm = *std::localtime(&t); // Make a tm pointer thingy
        oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S-%ms"); // Do this stuff
        return oss.str(); // return da string
    }
    double simulate_altitude_sensor()
    {
        return (15.0/this->sampling_rate);
    }

    void publish_altitude_info()
    {
        // Read the serial bus for altitude info
        this->altitude_serial_interface.request_altitude();
        this->altitude_serial_interface.read_serial();

        // Check if we've reached the max altitude
        if(this->altitude_serial_interface.altitude >= this->max_altitude && this->alt_reached == false)
        {
            this->alt_reached = true;
            // Check if the service is available
            if(!this->altitude_reached_notification->wait_for_service(std::chrono::seconds(2))) 
            {
                this->print_error_message("Unable to call altitude reached service! We're at altitude but can't do anything about it!");
            }
            else
            {
                // Notify the manager
                std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();
                this->altitude_reached_notification->async_send_request(request);
            }
        }

        std::unique_ptr<bobot_msgs::msg::AltitudeMonitor> altitude_info_msg = std::make_unique<bobot_msgs::msg::AltitudeMonitor>(); // make a unique point to our ROS message object
        
        // Add the info to the message
        altitude_info_msg->current_altitude = this->altitude_serial_interface.altitude;
        altitude_info_msg->current_altitude_centimeters = this->altitude_serial_interface.altitude_centimeters;
        altitude_info_msg->alt_reached = this->alt_reached;
        altitude_info_msg->send_time = get_current_time_for_logs();
        altitude_info_msg->ros_send_time = this->now().seconds(); /// get ros time

        this->altitude_info_->publish(std::move(altitude_info_msg));
    }
    void publish_simulated_altitude_info()
    {
        this->altitude_buffer_sim += this->simulate_altitude_sensor(); // simulate collecting the altitude from the sensor

        // Check if we've reached the max altitude
        if(this->altitude_buffer_sim >= this->max_altitude && this->alt_reached == false)
        {
            this->alt_reached = true;
            // Check if the service is available
            if(!this->altitude_reached_notification->wait_for_service(std::chrono::seconds(2))) 
            {
                this->print_error_message("Unable to call altitude reached service! We're at altitude but can't do anything about it!");
            }
            else
            {
                // Notify the manager
                std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();
                this->altitude_reached_notification->async_send_request(request);
            }
        }

        std::unique_ptr<bobot_msgs::msg::AltitudeMonitor> altitude_info_msg = std::make_unique<bobot_msgs::msg::AltitudeMonitor>(); // make a unique point to our ROS message object
        
        // Add the info to the message
        altitude_info_msg->current_altitude = this->altitude_buffer_sim;
        altitude_info_msg->current_altitude_centimeters = (int)(this->altitude_buffer_sim*30.48);
        altitude_info_msg->alt_reached = this->alt_reached;
        altitude_info_msg->send_time = get_current_time_for_logs();
        altitude_info_msg->ros_send_time = this->now().seconds(); /// get ros time

        this->altitude_info_->publish(std::move(altitude_info_msg));
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State&)
    {
        this->print_warning_message("Altitude Monitor returning to state [unconfigured]");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /*
        The following are declerations to the callback functions for on_configure, on_active, on_deactivate, on_cleanup, and on_shutdown.
        There are inherited lifecycle methods from the lifecycle node class definition, and we are simply overriding them
    */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State&)
    {
        this->print_debug_message("Attempting to configure Altitude Monitor, please hold...");

        topic_name = bobot_name + "/altitude_monitor_info";
        this->altitude_info_ = this->create_publisher<bobot_msgs::msg::AltitudeMonitor>(topic_name, 20); // Create a publisher with a namespace of bobot1 called timer, and have the queue size be 10 (since it should publish at 1 hz)
        this->altitude_reached_notification = this->create_client<std_srvs::srv::Trigger>(this->get_name() + std::string("/altitude_reached"));
        int freq_in_miliseconds = 1/this->sampling_rate * 1000; 
        
        // set up the serial connection
        if(this->is_simulated == false)
        {
            // create the callback thread for the non-simulated case
            altitude_info_callback_ = this->create_wall_timer(std::chrono::milliseconds(freq_in_miliseconds), [this]() -> void { publish_altitude_info(); }); // Weird sytnax (lambda syntax), but I think this is basically creating the callback function call at the specified spin rate

            if(altitude_serial_interface.open_serial_connection() == true)
            {
                this->print_debug_message("Altitude Monitor successfully opened the serial port");
                // TL:DR, we update the state of the node to say that on_configure successfully called. 
                // If the on_configure function isn't successful, it should either not update the state 
                // or change it to error_processing
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
            }
            else
            {
                this->print_error_message("Altitude was unable to open the serial port! This is bad!!");
                // TL:DR, we update the state of the node to say that on_configure successfully called. 
                // If the on_configure function isn't successful, it should either not update the state 
                // or change it to error_processing
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
            }
        }
        else
        {
            // create the callback thread for the simulated case
            altitude_info_callback_ = this->create_wall_timer(std::chrono::milliseconds(freq_in_miliseconds), [this]() -> void { publish_simulated_altitude_info(); }); // Weird sytnax (lambda syntax), but I think this is basically creating the callback function call at the specified spin rate
            this->print_warning_message("Running in simulation mode, so not attempting to open the serial port!");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        } 
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State&)
    {
        // -- FROM ROS -- //
        // on_activate callback is being called when the lifecycle node enters the "activating" state.

        // Get the initial altitude, and check if it's different from expected
        if(this->is_simulated == false)
        {
            this->altitude_serial_interface.request_altitude();
            this->altitude_serial_interface.read_serial();
            // This command automatically saves the altitude value in the altitude interface object
        }
        else
        {
            this->altitude_buffer_sim = 0.0; // say we're at sea level
        }

        // Here, we are activating the node, allowing it publish messages
        altitude_info_->on_activate(); // Call the activatio functions
        this->print_debug_message("Altitude Monitor has been activated has been activated");
        
        // We return a success and hence invoke the transition to the next step: "active".
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; // let the people know we've activated. In theory, this would so something important
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&)
    {
        // -- FROM ROS -- //
        // on_deactivate callback is being called when the lifecycle node enters the "deactivating" state.

        // Here, we are deactivating the node, which no longer allows its messages to go through
        altitude_info_->on_deactivate(); // Call the activation functions
        this->print_debug_message("Altitude Monitor has been deactivated... end of flight approaching!");

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
        this->print_debug_message("Altitude Monitor Node has begun cleaning up");

        altitude_info_callback_.reset(); // release the timer first
        altitude_info_.reset(); // release the publisher after the timer

        this->print_debug_message("Altitude Monitor finished cleaning up!");

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
        altitude_info_callback_.reset(); // release the timer first
        altitude_info_.reset(); // release the publisher after the timer

        this->print_debug_message("Altitude Monitor Node has shut down");
        this->print_debug_message("Shutdown command was called from state " + state.label());

        // We return a success and hence invoke the transition to the next step: "finalized".
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

protected:
    // functions to help get parameters from the node's parameter server
    // this is a protected function because I don't want people to access it outside of the node if they TRIED but I also want inherited classes to use it
    void parameter_helper()
    {
        // Get the rest of the parameters that we need for this node!
        // First, we delcare the parameters
        this->declare_parameter("SAMPLE_RATE", 10.0); // milliseconds per cycle
        this->declare_parameter("MAX_ALTITUDE", 90000); // feet
        this->declare_parameter("ALT_IS_SIMULATED", false);

        // Then, we get the parameters
        this->sampling_rate = this->get_parameter("SAMPLE_RATE").as_double();
        this->max_altitude = this->get_parameter("MAX_ALTITUDE").as_int();
        this->is_simulated = this->get_parameter("ALT_IS_SIMULATED").as_bool();
        this->print_debug_message(std::string("Found following parameters for the Altitude Monitor\n") 
                                + std::string("Sample Rate (Hz): ") + std::to_string(this->sampling_rate) + "\n"
                                + std::string("Max Altitude: ") + std::to_string(this->max_altitude) + "\n"
                                + std::string("Is Simulated: ") + std::to_string(this->is_simulated) + "\n");
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
    std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> altitude_reached_notification;

/*
    We hold an instance of a timer which periodically triggers the publish function.
    As for the beta version, this is a regular timer. In a future version, a
    lifecycle timer will be created which obeys the same lifecycle management as the
    lifecycle publisher.
*/  
    /// Same as above, this is a ros timer that regulates the rate at which the publisher publishhes data. Should follow lifecycle management (someho?)
    std::shared_ptr<rclcpp::TimerBase> altitude_info_callback_;
    
    double altitude_buffer_sim; // variable to store altitude data
    bool alt_reached = false;

    std::string bobot_name; // private property for the bobot name
    std::string topic_name; // private property for the topic name
    double sampling_rate; // servo jerk rate, defined in the servo_info.yaml file in the bobot_utils/hardware folder
    int64_t max_altitude; // the max altitude, since we want to inform the manager when we've reached it
    bool is_simulated; // boolean for if we are simulating values
    bobot_hardware::BobotAltimeterInterface altitude_serial_interface;
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
