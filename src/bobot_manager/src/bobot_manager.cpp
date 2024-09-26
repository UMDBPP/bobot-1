// |***************************************************************|
// |* (c) Copyright 2024                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

#include <rclcpp/rclcpp.hpp>

// C++ specific libraries
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <sstream>
#include <ctime>
#include <filesystem>

/*
            BOBOT MANAGER NODE
This node is the manager for Bobot-1, a robotic
arm payload that is silly. That is all it is!
This node should manage data reading/writing to
the specified log files, as well as making sure
events are exectued by monitoring topics and 
acting as a server for clients in the system

Contributers:
- Romeo Perlstein (romeperl@umd.edu)
- Rahul Vishnoi (rvishnoi@umd.edu)

*/

// A manager node that manages stuff, and things!
class BobotManager : public rclcpp::Node
{
public:
    BobotManager(const std::string &_node_name) : rclcpp::Node(_node_name), ros_logger_name(_node_name)
    {
        // Print out some log messages for the ROS logs
        RCLCPP_INFO(this->get_logger(), "[USER DEBUG LOG] Bobot Manager starting up!");
        this->print_debug_message("Bobot Manger starting up!");

        // Check if we've started already (hopefully not)
        if(std::filesystem::exists(std::filesystem::current_path() / "bobot_recovery") == true) // nominal case
        {
            
        }

        // Get the time at the very start of the software:
        start_time = get_current_time();
        RCLCPP_INFO(this->get_logger(), "[USER DEBUG LOG] Starting time: %s", start_time.c_str());

        // Get the directories for the log files:
        this->get_directory_paths();

        // Now, check to see if the directories exist or not
        this->make_log_file_paths();

        //
    }

    // Helper function to get the current time - stolen from rahul
    std::string get_current_time()
    {
        // Stolen from Rahul
        std::ostringstream oss; // Make an ostringstream object
        auto t = std::time(nullptr); // Make a time objects
        auto tm = *std::localtime(&t); // Make a tm pointer thingy
        oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S"); // Do this stuff
        return oss.str(); // return da string
    }

    void get_directory_paths()
    {
        // STD:FILESYSTEM COMMING IN CLUUUTTTCCCHHHHH! //

        // get the path to the bobot-1/src directory
        this->bobot_bin_dir = std::filesystem::current_path() / "src/bobot_bin";
        this->print_debug_message("Got the path to bobot_bin/ directory: " + bobot_bin_dir.string());

        // Get the error log file path and get the flight log file path
        std::string bobot_error_log_file_path = "log_files/error_logs/bobot_error_log_" + this->start_time + ".txt";
        std::string bobot_flight_log_file_path = "log_files/flight_logs/bobot_flight_log_" + this->start_time + ".txt";
        this->error_log_dir = bobot_bin_dir / bobot_error_log_file_path;
        this->flight_log_dir = bobot_bin_dir / bobot_flight_log_file_path;
        this->print_debug_message("Got the path to the error log for this flight: " + error_log_dir.string());
        this->print_debug_message("Got the path to the flight log for this flight: " + flight_log_dir.string());
    }

    void make_log_file_paths()
    {
        bool built_bin_path = false; // Bool to see if we've already build the files
        if(std::filesystem::exists(this->bobot_bin_dir) == false)
        {
            this->print_warning_message("bobot_bin/ directory does not exist - taking note of this as this should not be the case!");
            std::filesystem::create_directory(this->bobot_bin_dir);
            std::filesystem::create_directory(this->error_log_dir);

        }
    }

    void print_debug_message(std::string message)
    {
        RCLCPP_INFO(this->get_logger(), "[USER DEBUG LOG] %s", message.c_str()); // just a helper to mroe easily print generic debug messages
    }
    void print_warning_message(std::string message)
    {
        RCLCPP_INFO(this->get_logger(), "[USER WARN LOG] %s", message.c_str()); // just another helper to more easily print generid warning messages
    }

private:
    // ROS VARIABLES
    std::string ros_logger_name;

    // TIME AND DATA LOGGING VARIABLES
    std::string start_time;
    std::filesystem::path bobot_bin_dir;
    std::filesystem::path error_log_dir;
    std::filesystem::path flight_log_dir;


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
    std::shared_ptr<BobotManager> bobot_manager = std::make_shared<BobotManager>("BobotManager"); // create the ros node
    bobot_exec.add_node(bobot_manager->get_node_base_interface()); // add the nodes base class (lifecycle stuff)
    bobot_exec.spin(); // start spinning the node
    rclcpp::shutdown(); // Shutdown cleanly when we're done (ALL OF ROS, not just the node)

    return 0;
}

