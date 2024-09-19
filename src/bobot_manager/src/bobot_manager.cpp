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

// A manager node that manages stuff, and things!
class BobotManager : public rclcpp::Node
{
public:
    BobotManager(const std::string &_node_name) : rclcpp::Node(_node_name), ros_logger_name(_node_name)
    {
        RCLCPP_INFO(this->get_logger(), "[USER DEBUG LOG] Bobot Manager starting up!");
        // Get the time at the very start of the software:
        start_time = get_current_time();
        RCLCPP_INFO(this->get_logger(), "[USER DEBUG LOG] Starting time: %s", start_time.c_str());

    }

    // Helper function to get the current time - stolen from rahul
    std::string get_current_time()
    {
        // Stolen from rahul
        std::ostringstream oss; // Make an ostringstream object
        auto t = std::time(nullptr); // Make a time objects
        auto tm = *std::localtime(&t); // Make a tm pointer thingy
        oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S"); // Do this stuff
        return oss.str(); // return da string
    }

private:
    // ROS VARIABLES
    std::string ros_logger_name;

    // TIME AND DATA LOGGING VARIABLES
    std::string start_time;

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

