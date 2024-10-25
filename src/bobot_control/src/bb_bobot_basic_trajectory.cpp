// |***************************************************************|
// |* (c) Copyright 2023                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <bobot_msgs/msg/bobot_commander.hpp>
#include "bobot_hardware_interface/bobot_servo_interface.hpp"

// C++ specific libraries
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>


class BBBobotBasicTrajectoryGenerator : public rclcpp_lifecycle::LifecycleNode
{

public:

    // Constructor -  needs a node name and a boolean that indicates if we are using intra_process_commms (?))
    explicit BBBobotBasicTrajectoryGenerator(const std::string &node_name, bool intra_process_comms = false) : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {
        // Get the name of the current bobot for ros-logging purposes
        this->declare_parameter("BOBOT_NAME", "bobot_1");
        this->bobot_name = this->get_parameter("BOBOT_NAME").as_string();
        print_debug_message("Getting parameters for BB-Bobot Trajectory Generator node...");

        parameter_helper(); // get the parameters

        // establish the joint positions vector
        for(int i=0;i<this->num_joints;i+=1)
        {
            this->joint_positions.push_back(0);
        }

        // make out trajectories:
        generate_sweep_trajectory();
        generate_0_90_180_trajectory();

        print_debug_message("BB-Bobot-1 Trajectory Generator node has successfully launched! Starting state is {state: unconfigured}");
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

    // Make a sweep trajectory
    void generate_sweep_trajectory()
    {
        // Using the formula from Dan Grammer:
        // Degrees/sec * 1/Hz = Degrees / Cycle
        // Length of array should be Hz * seoncds to complete trajectory
        // For example, if we are moving for 2 seconds, are array is length 200
        double degrees_per_cycle = this->constant_joint_velocity/this->hardware_loop_rate; // the degree per cycle we need to travel at the given speed

        // Find out how many seconds it will take to get to 180 degrees:
        double period = (180/this->constant_joint_velocity)*2; // at 90 deg/sec, this should be 2 sec for a half a period (multiply by 2 to get the full period)
        
        // Total array size is period * hardware_loop_rate, so
        // To ensure this number is a int, floor the sucker
        int array_length = floor(this->hardware_loop_rate * period);
        double curr_val = 0;
        int direction = 1;
        for(int i=0;i<array_length;i++)
        {
            curr_val += degrees_per_cycle*direction; // get the current value
            if(curr_val <= 0.0) // make sure we don't go less than 0
            {
                curr_val = 0.0;
            }
            this->one_period_sweep.push_back((uint8_t)floor(curr_val));
            if(curr_val >= 180)
            {
                direction = direction * -1;
            }
            // this->print_debug_message(std::to_string(one_period_sweep[i]*1.0));
        }
    }

    void generate_0_90_180_trajectory()
    {
        // Want the servo to sit at a position for a certain amount of time
        int period = this->trajectory_time/5;
        // This is one period of the trajectory
        int array_length = floor(this->hardware_loop_rate * period);

        for(int i=0;i<array_length/5;i++)
        {
            this->one_0_90_180_traj.push_back(5);
            this->print_debug_message(std::to_string(one_0_90_180_traj[i]*1.0));
        }
        for(int i=array_length/5;i<(2*(array_length/5));i++)
        {
            this->one_0_90_180_traj.push_back(90);
            this->print_debug_message(std::to_string(one_0_90_180_traj[i]*1.0));
        }
        for(int i=(2*(array_length/5));i<(3*(array_length/5));i++)
        {
            this->one_0_90_180_traj.push_back(175);
            this->print_debug_message(std::to_string(one_0_90_180_traj[i]*1.0));
        }
        for(int i=(3*(array_length/5));i<(4*(array_length/5));i++)
        {
            this->one_0_90_180_traj.push_back(90);
            this->print_debug_message(std::to_string(one_0_90_180_traj[i]*1.0));
        }
        for(int i=(4*(array_length/5));i<(array_length);i++)
        {
            this->one_0_90_180_traj.push_back(5);
            this->print_debug_message(std::to_string(one_0_90_180_traj[i]*1.0));
        }
    }
    

    void trajectory_generator_callback()
    {
        // std::unique_ptr<bobot_msgs::msg::BobotCommander> bobot_command_msg = std::make_unique<bobot_msgs::msg::BobotCommander>(); // make a unique point to our ROS message object
        // std::string send_string = "Still jerking it (and by it, I mean servos): ";
        // for(int i=0;i<(int)this->servos_to_jerk_id.size();i+=1)
        // {
        //     send_string = send_string + (this->servos_to_jerk_id[i]) + ", ";
        // }
        // trajectory_publishermsg->jerk_msg = send_string;
        // trajectory_publishermsg->jerk_rate = this->jerk_rate;
        // trajectory_publishermsg->num_strokes = this->strokes;
        // trajectory_publishermsg->send_time = this->get_current_time_for_logs();

        // this->trajectory_publisher->publish(std::move(trajectory_publishermsg));
    }
    void simulated_trajectory_generator_callback()
    {
        // std::unique_ptr<bobot_msgs::msg::BobotCommander> bobot_command_msg = std::make_unique<bobot_msgs::msg::BobotCommander>(); // make a unique point to our ROS message object
        
        // trajectory_publishermsg->jerk_msg = send_string;
        // trajectory_publishermsg->jerk_rate = this->jerk_rate;
        // trajectory_publishermsg->num_strokes = this->strokes;
        // trajectory_publishermsg->send_time = this->get_current_time_for_logs();

        // this->trajectory_publisher->publish(std::move(trajectory_publishermsg));
    }


    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State&)
    {
        this->print_warning_message("BB-Bobot-1 Trajectory Generator returning to state [unconfigured]");
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
        this->topic_name = bobot_name + "/basic_trajectory_generator";

        // Create a publisher to publish data to an info topic at every 10 seconds, indicating the jerking status
        this->trajectory_publisher = this->create_publisher<bobot_msgs::msg::BobotCommander>(topic_name, 20); 
        int freq_in_miliseconds = 1/this->hardware_loop_rate * 1000;

        if(this->is_simulated == false)
        {
            this->trajectory_looper = this->create_wall_timer(std::chrono::milliseconds(freq_in_miliseconds), [this]() -> void { trajectory_generator_callback(); }); // Weird sytnax (lambda syntax), but I think this is basically creating the callback function call at the specified spin rate
        }
        else
        {
            this->trajectory_looper = this->create_wall_timer(std::chrono::milliseconds(freq_in_miliseconds), [this]() -> void { simulated_trajectory_generator_callback(); }); // Weird sytnax (lambda syntax), but I think this is basically creating the callback function call at the specified spin rate
        }

        // Add log information, incase our logging fails
        this->print_debug_message("BB-Bobot-1 Trajectory Generator ROS stuff made! Opening the serial port");

        // set up the serial connection
        if(this->is_simulated == false)
        {
            if(bobot_serial_interface.open_serial_connection() == true)
            {
                this->print_debug_message("BB-Bobot-1 Trajectory Generator successfully opened the serial port");
                // TL:DR, we update the state of the node to say that on_configure successfully called. 
                // If the on_configure function isn't successful, it should either not update the state 
                // or change it to error_processing
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
            }
            else
            {
                this->print_error_message("BB-Bobot-1 Trajectory Generator was unable to open the serial port! This is bad!!");
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
        this->trajectory_publisher->on_activate(); // Call the activatio functions
        this->print_debug_message("BB-Bobot-1 Trajectory Generatoring has been activated!");
        
        // We return a success and hence invoke the transition to the next step: "active".
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; // let the people know we've activated. In theory, this would so something important
    }


    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&)
    {
        // -- FROM ROS -- //
        // on_deactivate callback is being called when the lifecycle node enters the "deactivating" state.

        // Here, we are deactivating the node, which no longer allows its messages to go through
        this->trajectory_publisher->on_deactivate(); // Call the standard deactivate function 
        this->print_debug_message("BB-Bobot-1 Trajectory Generatoring has been deactivated... we should be ending motion!");

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
        this->print_debug_message("BB-Bobot-1 Trajectory Generator Node has begin cleaning up");

        this->trajectory_looper.reset(); // release the wall_timer first
        this->trajectory_publisher.reset(); // release the publisher after the timer

        this->print_debug_message("BB-Bobot-1 Trajectory Generator Node has finished cleaning up, should have ended motion!");
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
        this->trajectory_looper.reset(); // release the timer first
        this->trajectory_publisher.reset(); // release the publisher after the timer

        this->print_debug_message("BB-Bobot-1 Trajectory Generator Node has shut down!");
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
        this->declare_parameter("JOINT_IDS", std::vector<int>({1, 2}));
        this->declare_parameter("TRAJ_IS_SIMULATED", false);
        this->declare_parameter("HARDWARE_LOOP_RATE", 100.0);
        this->declare_parameter("CONSTANT_JOINT_VELOCITY", 90.0);
        this->declare_parameter("TRAJECTORY_TIME", 5);

        // Then, we get the parameters
        this->joint_IDs_long = this->get_parameter("JOINT_IDS").as_integer_array();
        this->is_simulated = this->get_parameter("TRAJ_IS_SIMULATED").as_bool();
        this->hardware_loop_rate = this->get_parameter("HARDWARE_LOOP_RATE").as_double();
        this->constant_joint_velocity = this->get_parameter("CONSTANT_JOINT_VELOCITY").as_double();
        this->trajectory_time = this->get_parameter("TRAJECTORY_TIME").as_int();
        this->num_joints = (int)this->joint_IDs_long.size();
        for(int i=0;i<this->num_joints;i+=1)
        {
            this->joint_IDs.push_back((uint8_t)this->joint_IDs_long[i]);
            this->print_debug_message("Found joint with id: " + std::to_string(this->joint_IDs_long[i]));
        }
        this->print_debug_message(std::string("Found following parameters for BB-Bobot-1's Basic Trajectory Generator\n") 
                                + std::string("Is Simulated: ") + std::to_string(this->is_simulated) + "\n"
                                + std::string("Hardware Loop Rate: ") + std::to_string(this->hardware_loop_rate) + "\n"
                                + std::string("Num Joints: ") + std::to_string(this->num_joints) + "\n"
                                + std::string("Trajectory Time: ") + std::to_string(this->trajectory_time) + "\n");
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
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<bobot_msgs::msg::BobotCommander>> trajectory_publisher;

/*
    We hold an instance of a timer which periodically triggers the publish function.
    As for the beta version, this is a regular timer. In a future version, a
    lifecycle timer will be created which obeys the same lifecycle management as the
    lifecycle publisher.
*/  
    /// Same as above, this is a ros timer that regulates the rate at which the publisher publishhes data. Should follow lifecycle management (someho?)
    std::shared_ptr<rclcpp::TimerBase> trajectory_looper;

    std::string bobot_name; // private property for the bobot name
    std::string topic_name; // private property for the topic name
    int num_joints;
    std::vector<int> joint_positions;
    std::vector<long int> joint_IDs_long;
    std::vector<uint8_t> joint_IDs;
    double constant_joint_velocity;
    int trajectory_time;
    bool is_simulated = false; // by default we are not in simulation
    double hardware_loop_rate;
    bobot_hardware::BobotServoInterface bobot_serial_interface;

    // Trajectories
    std::vector<uint8_t> one_period_sweep;
    std::vector<uint8_t> one_0_90_180_traj;

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
    std::shared_ptr<BBBobotBasicTrajectoryGenerator> bb_bobot_basic_traj_generator = std::make_shared<BBBobotBasicTrajectoryGenerator>("BBBobotBasicTrajectoryGenerator"); // create the ros node
    bobot_exec.add_node(bb_bobot_basic_traj_generator->get_node_base_interface()); // add the nodes base class (lifecycle stuff)
    bobot_exec.spin(); // start spinning the node
    rclcpp::shutdown(); // Shutdown cleanly when we're done (ALL OF ROS, not just the node)

    return 0;
}
