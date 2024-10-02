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
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <vector>
#include <iomanip>


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
        this->print_debug_message("Bobot Manager starting up!");

        // Check to make sure the bobot_recovery/ directory exists, and that current_flight.txt and reset_counter.txt exist (but DONT check if they are empty)
        std::string start_time_logs = this->get_current_time_for_logs();
        std::string critical_error_msg = "[BOOTUP BEGIN " + start_time_logs + "] Checking if bobot_recovery/current_flight.txt and bobot_recovery/reset_counter.txt exist! If so attempting to read from them now"; // This string only gets used if none of the below cases get tripped
        int error_count = 0;
        std::string flight_log_msg = "[BOOTUP BEGIN " + start_time_logs + "] (SEE ERROR LOG FOR CRITICAL ISSUES) Setting up the BobotManager for flight, please hold...";

        // Initial check variables
        bool reset_counter_exists = true;
        bool current_flight_exists = true;
        std::string print_to_screen;
        if(std::filesystem::exists(std::filesystem::current_path() / "bobot_recovery") == true) // nominal case
        { 
            if(std::filesystem::exists(std::filesystem::current_path() / "bobot_recovery/current_flight.txt") == false && 
                std::filesystem::exists(std::filesystem::current_path() / "bobot_recovery/reset_counter.txt") == false)
            {
                print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "bobot_recovery/ exists, but current_flight.txt and reset_counter.txt do not exist! This is a critical error - system will assuming that we are at the beginning of the flight", error_count);
                this->print_error_message(print_to_screen);
                reset_counter_exists = false;
                current_flight_exists = false;
            }
            else if(std::filesystem::exists(std::filesystem::current_path() / "bobot_recovery/current_flight.txt") == true && 
                std::filesystem::exists(std::filesystem::current_path() / "bobot_recovery/reset_counter.txt") == false)
            {
                print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "bobot_recovery/ exists, but reset_counter.txt does not exist! This is a critical error - system will attempt to read current_flight.txt and decide if we are already in flight", error_count);
                reset_counter_exists = false;
                this->print_error_message(print_to_screen);
            }
            else if(std::filesystem::exists(std::filesystem::current_path() / "bobot_recovery/current_flight.txt") == false && 
                std::filesystem::exists(std::filesystem::current_path() / "bobot_recovery/reset_counter.txt") == true)
            {
                print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "bobot_recovery/ exists, but current_flight.txt does not exist! This is a critical error - system will attempt to read reset_counter.txt and decide if we are already in flight", error_count);
                current_flight_exists = false;
                this->print_error_message(print_to_screen);
            }
            else
            {
                // For this case, just indicate that we did not enccounter critical error 1
                error_count += 1;
            }
        }
        else
        {
            print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "bobot_recovery/ does not exist! This is a critical error - system will assuming that we are at the beginning of the flight", error_count);
            this->print_error_message(print_to_screen);
        }

        // Get the time at the very start of the software:
        start_time = get_current_time();
        RCLCPP_INFO(this->get_logger(), "[USER DEBUG LOG] Starting time: %s", start_time.c_str());
        flight_log_msg = flight_log_msg + "[Flight Logs] Starting time: " + start_time;

        // Create a temporary string to send to the current_flight.txt file
        std::string current_flight_information = "CURRENT FLIGHT DATA FOR RECOVERY\ntime:\n" + start_time + "\n\nHAPPY BOBOT-ING";
        std::string reset_count_string = std::to_string(this->reset_counter);

        // get the is_flight variable
        // NOTE:    This variable is declared by LITERALLY writing to a config file
        //          when we run the "flight_preparer.sh" or "dev_preparer.sh" bash
        //          scripts, so, don't worry about changing this variable. It was 
        //          easier to do this than try to get an environment variable working
        //          lol.
        //          Defaults to being in flight mode, so just a heads up on that
        this->declare_parameter("IS_FLIGHT", true);
        this->IS_FLIGHT = this->get_parameter("IS_FLIGHT").as_bool();
        if(this->IS_FLIGHT == true)
        {
            this->print_debug_message("We are currently in flight mode!");
        }
        else if(this->IS_FLIGHT == false)
        {
            this->print_debug_message("We are currently in development mode");
        }

        // Now, check if we are in flight mode
        // If we are in flight mode, then we should
        //      first: check to see if already have 
        //      data written current_flight.txt file,
        //      and this should be empty unless we have
        //      already started a flight (and as such,
        //      we've reset)
        if(this->IS_FLIGHT == true)
        {
            if(current_flight_exists == false)
            {
                // If the file does not exist, we are assuming something went wrong,
                // and are going to save the current data of the flight and count that
                // we've reset
                reset_counter += 1;
                std::string reset_count_string = std::to_string(this->reset_counter);
                std::ofstream current_flight_txt;
                current_flight_txt.open((std::filesystem::current_path() / "bobot_recovery/current_flight.txt").string(), std::ios::out | std::ios::app);
                if(current_flight_txt.is_open() == false)
                {
                    print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "Could not open current_flight.txt! This is a critical error - system is unable to write the captured start time and will be unable to recover", error_count);
                    this->print_error_message(print_to_screen);
                }
                current_flight_txt << current_flight_information << std::endl;
                current_flight_txt.flush();
                current_flight_txt.close();
                // done handling this case!
            }
            else if(current_flight_exists == true)
            {
                // Read in the data from current flight exists
                std::ifstream current_flight_txt;
                current_flight_txt.open((std::filesystem::current_path() / "bobot_recovery/current_flight.txt").string(), std::ios::in);
                if(current_flight_txt.is_open() == true)
                {
                    print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "Could not open current_flight.txt! This is a critical error - system is unable to write the captured start time and will be unable to recover", error_count);
                    this->print_error_message(print_to_screen);
                }
                // TODO: this
                std::string 
                std::string = std::getline(current_flight_txt, 1);
                current_flight_txt << current_flight_information << std::endl;
                current_flight_txt.flush();
                current_flight_txt.close();
                // done handling this case!
            }
        }

        // Get the directories for the log files:
        this->get_directory_paths();

        // Now, check to see if the directories exist or not
        this->make_log_file_paths();

        // end
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
    std::string get_current_time_for_logs()
    {
        // Stolen from Rahul
        std::ostringstream oss; // Make an ostringstream object
        auto t = std::time(nullptr); // Make a time objects
        auto tm = *std::localtime(&t); // Make a tm pointer thingy
        oss << std::put_time(&tm, "%d-%H:%M:%S"); // Do this stuff
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
        RCLCPP_WARN(this->get_logger(), "[USER WARN LOG] %s", message.c_str()); // just another helper to more easily print generid warning messages
    }
    void print_error_message(std::string message)
    {
        RCLCPP_ERROR(this->get_logger(), "[USER ERROR LOG] %s", message.c_str()); // just another helper to more easily print generid warning messages
    }
    
    std::string add_to_flight_log_init_msg(std::string flight_log_prev, std::string new_msg)
    {
        std::string curr_time = this->get_current_time_for_logs();
        return flight_log_prev + "\n[FLIGHT LOG :  " + curr_time + "] " + new_msg;
    }

    // Returns a string that contains the new message formatted correct, and also passes-by-reference the old message and adds the new message to it
    // Doing this because we want to print these to the terminal for the user to see, and for the ROS logs to catch
    std::string add_to_critical_error_log_init_msg(std::string &critical_error_msg, std::string new_error, int &error_count)
    {
        error_count += 1;
        std::string curr_time = this->get_current_time_for_logs();
        std::string count_string = std::to_string(error_count);
        critical_error_msg = critical_error_msg + "\n[CRITICAL STARTUP ERROR " + count_string + " : " + curr_time + "] " + new_error;
        return "[CRITICAL STARTUP ERROR " + count_string + " : " + curr_time + "] " + new_error;
    }

private:
    // ROS VARIABLES
    std::string ros_logger_name;

    // FLIGHT VS DEV VARIABLES
    bool IS_FLIGHT = false;

    // TIME AND DATA LOGGING VARIABLES
    std::string start_time;
    int reset_counter = 0;
    std::filesystem::path bobot_bin_dir;
    std::filesystem::path error_log_dir;
    std::filesystem::path flight_log_dir;

    // FILE HANDLES
    std::ofstream timer_log;
    std::ofstream altitude_log;
    std::ofstream error_log;
    std::ofstream flight_log;



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

