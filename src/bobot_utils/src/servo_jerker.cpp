// |***************************************************************|
// |* (c) Copyright 2023                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <bobot_msgs/msg/servo_jerk.hpp> // servo jerk message file
#include <bobot_utils/bobot_common_types.hpp>
#include "bobot_hardware_interface/bobot_servo_interface.hpp"

// C++ specific libraries
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>


class BobotServoJerk : public rclcpp_lifecycle::LifecycleNode
{

public:

    // Constructor -  needs a node name and a boolean that indicates if we are using intra_process_commms (?))
    explicit BobotServoJerk(const std::string & node_name, bool intra_process_comms = false) : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {
        // Get the name of the current bobot for ros-logging purposes
        this->declare_parameter("BOBOT_NAME", "bobot_1");
        this->bobot_name = this->get_parameter("BOBOT_NAME").as_string();
        print_debug_message("Getting parameters for Servo Jerker node...");

        parameter_helper(); // get the parameters

        print_debug_message("Servo Jerker node has successfully launched! Starting state is {state: unconfigured}");
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
        oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S"); // Do this stuff
        return oss.str(); // return da string
    }

    void publish_jerking_info()
    {
        std::unique_ptr<bobot_msgs::msg::ServoJerk> servo_jerk_info_msg = std::make_unique<bobot_msgs::msg::ServoJerk>(); // make a unique point to our ROS message object
        std::string send_string = "Still jerking it (and by it I mean servos): ";
        for(int i=0;i<(int)this->servos_to_jerk_id.size();i+=1)
        {
            send_string = send_string + (this->servos_to_jerk_id[i]) + ", ";
        }
        servo_jerk_info_msg->jerk_msg = send_string;
        servo_jerk_info_msg->jerk_rate = this->jerk_rate;
        servo_jerk_info_msg->num_strokes = this->strokes;
        servo_jerk_info_msg->send_time = this->get_current_time_for_logs();

        this->servo_jerk_info_->publish(std::move(servo_jerk_info_msg));
    }

    void jerk_the_servos()
    {
        if(this->flip_flopper == true)
        {
            for(int i=0;i<(int)this->servos_to_jerk.size();i+=1)
            {
            
                this->bobot_serial_interface.command_position(this->servos_to_jerk[i], this->max_jerk_angle);
            }
            this->flip_flopper = false;
        }
        else if(this->flip_flopper == false)
        {
            for(int i=0;i<(int)this->servos_to_jerk.size();i+=1)
            {
                this->bobot_serial_interface.command_position(this->servos_to_jerk[i], this->min_jerk_angle);
            }
            this->flip_flopper = true;
        }
        // Dont need to mutex because I am using a single threaded executor, so each callback will be called one after the other
        this->strokes += 1;
    }
    void jerk_the_simulated_servos()
    {
        if(this->flip_flopper == true)
        {

            for(int i=0;i<(int)this->servos_to_jerk.size();i+=1)
            {
                // Do nothing
                // this->bobot_serial_interface.command_position(this->servos_to_jerk[i], this->max_jerk_angle);
            }
            this->flip_flopper = false;
        }
        else if(this->flip_flopper == false)
        {
            for(int i=0;i<(int)this->servos_to_jerk.size();i+=1)
            {
                // Do nothing
                // this->bobot_serial_interface.command_position(this->servos_to_jerk[i], this->min_jerk_angle);
            }
            this->flip_flopper = true;
        }
        // Dont need to mutex because I am using a single threaded executor, so each callback will be called one after the other
        this->strokes += 1;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State&)
    {
        this->print_warning_message("Servo Jerker returning to state [unconfigured]");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /*
        The following are declerations to the callback functions for on_configure, on_active, on_deactivate, on_cleanup, and on_shutdown.
        There are inherited lifecycle methods from the lifecycle node class definition, and we are simply overriding them
    */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State&)
    {
        this->print_debug_message("Attempting to configure Servo Jerker, please hold");

        // Get the topic name
        this->topic_name = bobot_name + "/servo_jerk_info";

        // Create a publisher to publish data to an info topic at every 10 seconds, indicating the jerking status
        this->servo_jerk_info_ = this->create_publisher<bobot_msgs::msg::ServoJerk>(topic_name, 20); 
        this->bobot_servo_jerk_info_callback_ = this->create_wall_timer(std::chrono::seconds(2), [this]() -> void { publish_jerking_info(); }); // Weird sytnax (lambda syntax), but I think this is basically creating the callback function call at the specified spin rate

        int freq_in_miliseconds = 1/this->jerk_rate * 1000;
        if(this->is_simulated == false)
        {
            this->bobot_servo_jerker_ = this->create_wall_timer(std::chrono::milliseconds(freq_in_miliseconds), [this]() -> void { jerk_the_servos(); }); // Weird sytnax (lambda syntax), but I think this is basically creating the callback function call at the specified spin rate
        }
        else
        {
            this->bobot_servo_jerker_ = this->create_wall_timer(std::chrono::milliseconds(freq_in_miliseconds), [this]() -> void { jerk_the_simulated_servos(); }); // Weird sytnax (lambda syntax), but I think this is basically creating the callback function call at the specified spin rate
        }
        
        // Add log information, incase our logging fails
        this->print_debug_message("Servo Jerker ROS stuff made! Opening the serial port");

        // set up the serial connection
        if(this->is_simulated == false)
        {
            if(bobot_serial_interface.open_serial_connection() == true)
            {
                this->print_debug_message("Servo Jerker successfully opened the serial port");
                // TL:DR, we update the state of the node to say that on_configure successfully called. 
                // If the on_configure function isn't successful, it should either not update the state 
                // or change it to error_processing
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
            }
            else
            {
                this->print_error_message("Servo Jerker was unable to open the serial port! This is bad!!");
                // TL:DR, we update the state of the node to say that on_configure successfully called. 
                // If the on_configure function isn't successful, it should either not update the state 
                // or change it to error_processing
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
            }
        }
        else
        {
            this->print_warning_message("Running in simulation mode, so not attempting to open the serial port!");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        } 
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State&)
    {
        // -- FROM ROS -- //
        // on_activate callback is being called when the lifecycle node enters the "activating" state.

        // Here, we are activating the node, allowing it publish messages
        this->servo_jerk_info_->on_activate(); // Call the activatio functions
        this->print_debug_message("Servo Jerking has been activated!");
        
        // We return a success and hence invoke the transition to the next step: "active".
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; // let the people know we've activated. In theory, this would so something important
    }


    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&)
    {
        // -- FROM ROS -- //
        // on_deactivate callback is being called when the lifecycle node enters the "deactivating" state.

        // Here, we are deactivating the node, which no longer allows its messages to go through
        this->servo_jerk_info_->on_deactivate(); // Call the standard deactivate function 
        this->print_debug_message("Servo jerking has been deactivated... we should be starting motion!");

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
        this->print_debug_message("Servo Jerk Node has begin cleaning up");

        this->bobot_servo_jerk_info_callback_.reset(); // release the wall_timer first
        this->bobot_servo_jerker_.reset();
        this->servo_jerk_info_.reset(); // release the publisher after the timer

        this->print_debug_message("Servo Jerk Node has finished cleaning up, quite the mess we made!");
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
        this->bobot_servo_jerk_info_callback_.reset(); // release the timer first
        this->servo_jerk_info_.reset(); // release the publisher after the timer

        this->print_debug_message("Servo Jerk Node has shut down!");
        this->print_debug_message("Shutdown was called from state " + state.label());

        // We return a success and hence invoke the transition to the next step: "finalized".
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

protected:
    // function to help parse the parameters from the nodes parameter servo
    // this is a protected function because I don't want people to access it outside of the node if they TRIED but I also want inherited classes to use it
    void parameter_helper()
    {
        // Get the rest of the parameters that we need for this node!
        // First, we delcare the parameters
        this->declare_parameter("JERK_RATE", 1.0);
        this->declare_parameter("SERVOS_TO_JERK_ID", std::vector<std::string>({"ID-1","ID-2"}));
        this->declare_parameter("SERVOS_TO_JERK", std::vector<int>({1, 2}));
        this->declare_parameter("SERVO_IS_SIMULATED", false);
        this->declare_parameter("MAX_ANGLE", 0);
        this->declare_parameter("MIN_ANGLE", 0);

        // Then, we get the parameters
        this->jerk_rate = this->get_parameter("JERK_RATE").as_double();
        this->servos_to_jerk_id = this->get_parameter("SERVOS_TO_JERK_ID").as_string_array();
        this->servos_to_jerk_long = this->get_parameter("SERVOS_TO_JERK").as_integer_array();
	    for(int i=0;i<(int)this->servos_to_jerk_long.size();i+=1)
        {
            this->servos_to_jerk.push_back((uint8_t)servos_to_jerk_long[i]); // typcast
            this->print_debug_message("Found servo to jerk: " + this->servos_to_jerk_id[i]);
        }
        this->is_simulated = this->get_parameter("SERVO_IS_SIMULATED").as_bool();
        this->max_jerk_angle_long = this->get_parameter("MAX_ANGLE").as_int();
        this->min_jerk_angle_long = this->get_parameter("MIN_ANGLE").as_int();
	    this->max_jerk_angle = (uint8_t)this->max_jerk_angle_long; // typecast
	    this->min_jerk_angle = (uint8_t)this->min_jerk_angle_long;
        this->print_debug_message(std::string("Found following parameters for Servo Jerker\n")
                                + std::string("Servo Jerk rate (Hz): ") + std::to_string(this->jerk_rate) + "\n"
                                + std::string("Is simulated: ") + std::to_string(this->is_simulated) + "\n"
                                + std::string("Max Angle: ") + std::to_string(this->max_jerk_angle_long) + "\n"
                                + std::string("Min Angle: ") + std::to_string(this->min_jerk_angle_long) + "\n");
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
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<bobot_msgs::msg::ServoJerk>> servo_jerk_info_;

/*
    We hold an instance of a timer which periodically triggers the publish function.
    As for the beta version, this is a regular timer. In a future version, a
    lifecycle timer will be created which obeys the same lifecycle management as the
    lifecycle publisher.
*/  
    /// Same as above, this is a ros timer that regulates the rate at which the publisher publishhes data. Should follow lifecycle management (someho?)
    std::shared_ptr<rclcpp::TimerBase> bobot_servo_jerk_info_callback_;
    std::shared_ptr<rclcpp::TimerBase> bobot_servo_jerker_;

    std::string bobot_name; // private property for the bobot name
    std::string topic_name; // private property for the topic name
    double jerk_rate; // servo jerk rate in the bobot_hardware_config_file.yaml
    std::vector<std::string> servos_to_jerk_id;
    std::vector<long int> servos_to_jerk_long;
    std::vector<uint8_t> servos_to_jerk;
    unsigned int strokes = 0; // I'm LITERALLY hilarious
    bool is_simulated = false; // by default we are not in simulation
    bool flip_flopper = true;
    int max_jerk_angle_long = 0;
    int min_jerk_angle_long = 0;
    uint8_t max_jerk_angle = 0;
    uint8_t min_jerk_angle = 0;
    bobot_hardware::BobotServoInterface bobot_serial_interface;

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
    std::shared_ptr<BobotServoJerk> bobot_servo_jerk = std::make_shared<BobotServoJerk>("ServoJerk"); // create the ros node
    bobot_exec.add_node(bobot_servo_jerk->get_node_base_interface()); // add the nodes base class (lifecycle stuff)
    bobot_exec.spin(); // start spinning the node
    rclcpp::shutdown(); // Shutdown cleanly when we're done (ALL OF ROS, not just the node)

    return 0;
}
