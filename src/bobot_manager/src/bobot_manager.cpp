// |***************************************************************|
// |* (c) Copyright 2024                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/srv/get_available_states.hpp>
#include <lifecycle_msgs/srv/get_available_transitions.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>


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
#include <functional>


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
- Rahul Vishnoi (rahul.vishnoi@nasa.gov)

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
        }
        else
        {
            print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "bobot_recovery/ does not exist! This is a critical error - system will assuming that we are at the beginning of the flight", error_count);
            this->print_error_message(print_to_screen);
        }

        // Get the time at the very start of the software:
        start_time = get_current_time();
        RCLCPP_INFO(this->get_logger(), "[USER DEBUG LOG] Starting time: %s", start_time.c_str());
        this->add_to_flight_log_init_msg(flight_log_msg, "Starting time: " + start_time);

        // get the IS_FLIGHT variable
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
////-------- THIS IS A BIG PORTION, THAT IN THEORY I COULD MOVE TO A FUNCTION, BUT HEY FUCK YOU - Romeo p.
        if(this->IS_FLIGHT == true)
        {
            if(current_flight_exists == false)
            {
                // If the file does not exist, we are assuming something went wrong,
                // and are going to save the current data of the flight and count that
                // we've reset
                print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "No current_flight.txt file, making one right now! (Counting this as an error)");
                this->print_warning_message(print_to_screen);
                reset_counter = 1; // say that we've reset once
                this->write_to_current_flight_file(critical_error_msg, error_count);
                // done handling this case!
            }
            else if(current_flight_exists == true)
            {
                // Read in the data from current flight exists
                print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Reading from current_flight.txt to see if we had already recorded data!"); 
                this->print_debug_message(print_to_screen);
                std::ifstream current_flight_txt;
                current_flight_txt.open((std::filesystem::current_path() / "bobot_recovery/current_flight.txt").string(), std::ios::in);
                if(current_flight_txt.is_open() == false)
                {
                    print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "Could not open current_flight.txt! This is a critical error - system is unable to write the captured start time and will be unable to recover", error_count);
                    this->print_error_message(print_to_screen);
                }

                // Make a vector to store the data we are reading in
                std::vector<std::string> data_from_file;
                // data file is in this form:
                /*
                    [current_flight.txt]
                    1| CURRENT FLIGHT DATA FOR RECOVERY
                    2| time:
                    3| 2024-10-01_20-27-44
                    4|
                    5| HAPPY BOBOT-ING
                    
                    [END OF FILE]
                    This, of course, does NOT need to be formatted like this, but it doesn't make
                    things too much more difficult, so lets just keep it like this heheheh
                */

                // Now, read every line and put it into the vector we made
                for(std::string line; std::getline(current_flight_txt, line, current_flight_txt.widen('\n'));)
                {
                    data_from_file.push_back(line);
                }
                current_flight_txt.close(); // close the file after we are done with it

                // Check if our vector is empty (no data)
                if(data_from_file.size() == 0)
                {
                    print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "current_flight.txt is empty, filling it with current flight data!");
                    this->print_debug_message(print_to_screen);
                    this->write_to_current_flight_file(critical_error_msg, error_count);
                    // done with this case!
                }
                else if(data_from_file.size() >= 3) // If our vector is not empty
                {
                    if(data_from_file[2] == "") // If the expected location for the file is empty, then we just have empty lines lol
                    {
                        print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "current_flight.txt is WEIRDLY empty (had multiple blank lines), filling it with current flight data!");
                        this->print_debug_message(print_to_screen);
                        this->write_to_current_flight_file(critical_error_msg, error_count);
                        // done with this case!
                    }
                    else if(data_from_file[2] != "")
                    {
                        this->start_time = data_from_file[2]; // save the new start time
                        print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Found a new time in the current_flight.txt file!");
                        this->print_debug_message(print_to_screen);
                        print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "New time: " + this->start_time);
                        this->print_debug_message(print_to_screen);
                        reset_counter = 1; // up the reset counter to be 1;
                        print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "Found a new time in current_flight.txt, indicating that we have reset! This is a considered a critical anomaly!", error_count);
                        this->print_error_message(print_to_screen);
                        // done with this case!
                    }
                }
                else
                {
                    print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "current_flight.txt is empty, at least 1 or more empty lines. Filling it with current flight data!");
                    this->print_debug_message(print_to_screen);
                    this->write_to_current_flight_file(critical_error_msg, error_count);
                    // done handling this case!
                }
            }

            // Now, worry about reset_counter.txt lol
            if(reset_counter_exists == false)
            {
                // If the file does not exist, we are assuming something went wrong,
                // and are going to save the current data of the flight and count that
                // we've reset
                // Note: we've already taken note that this file doesn't exist, so no point making another one
                print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "No reset_counter.txt file, making one right now! (Counting this as an error)");
                this->print_warning_message(print_to_screen);
                reset_counter = 1;
                this->write_to_reset_counter_file(critical_error_msg, error_count);
                // done handling this case!
            }
            else if(reset_counter_exists == true)
            {
                // Read in the data from current flight exists
                print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Reading from reset_counter.txt to see if we had already reset!"); 
                this->print_debug_message(print_to_screen);
                std::ifstream reset_counter_text;
                reset_counter_text.open((std::filesystem::current_path() / "bobot_recovery/reset_counter.txt").string(), std::ios::in);
                if(reset_counter_text.is_open() == false)
                {
                    print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "Could not open reset_coutner.txt! This is a critical error - system is unable to write the captured start time and will be unable to recover", error_count);
                    this->print_error_message(print_to_screen);
                }

                std::vector<std::string> reset_count_string;
                // file format:
                /*

                    [reset_counter.txt]
                    1| 0 <---- (or another number, depending on how many times we've reset)

                    [END OF FILE]
                
                    So in theory, we should only have ONE line of data, on line 1

                */
                // Now, read every line and put it into the vector we made
                for(std::string line; std::getline(reset_counter_text, line, reset_counter_text.widen('\n'));)
                {
                    reset_count_string.push_back(line);
                }

                // Check if our vector is empty (no data)
                if(reset_count_string.size() == 0)
                {
                    print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "reset_counter.txt is empty, assuming no prior resets! Writing current number resets to file");
                    this->print_debug_message(print_to_screen);
                    this->write_to_reset_counter_file(critical_error_msg, error_count);
                    // done with this case!
                }
                else if(reset_count_string.size() > 0) // If our vector is not empty
                {
                    if(reset_count_string[0] == "")
                    {
                        print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "reset_counter.txt is empty, assuming no prior resets! Writing current number resets to file");
                        this->print_debug_message(print_to_screen);
                        this->write_to_reset_counter_file(critical_error_msg, error_count);
                        // done with this case!
                    }
                    else if(reset_count_string[0] != "")
                    {
                        int prior_resets = std::stoi(reset_count_string[0]);
                        if(prior_resets == 0)
                        {
                            print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "reset_counter.txt is empty, assuming no prior resets! Writing current number resets to file");
                            this->print_debug_message(print_to_screen);
                            this->write_to_reset_counter_file(critical_error_msg, error_count);
                            // done with this case!
                        }
                        else if(prior_resets > 0)
                        {
                            this->reset_counter = reset_counter + prior_resets;
                            print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Found number of resets: " + std::to_string(prior_resets));
                            this->print_debug_message(print_to_screen);
                            print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Currrent number of resets: " + std::to_string(this->reset_counter));
                            this->print_debug_message(print_to_screen);
                            print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "reset_counter.txt indicates that we've reset multiple times! This is a critical error that will require investigating!", error_count);
                            this->print_error_message(print_to_screen);
                            // write the new number of reset to the file
                            this->write_to_reset_counter_file(critical_error_msg, error_count);
                            // done with this case!
                        }
                    }
                }
            }


        } // If IS_FLIGHT == false, skip this whole section tbh
////-------- SWEET, WE'RE DONE ALL DAT, SO COPE - Romeo p.

        // Get the directories for the log files:
        this->get_directory_paths();

        // Now, check to see if the directories exist or not! This will inform our error state still
        this->check_log_file_paths(flight_log_msg, critical_error_msg, error_count);

        print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Finished verifiying startup sequence, now setting up ROS infrastructure!");
        this->print_debug_message(print_to_screen);

        // Now, create our pub/subs, and our services and clients
        this->set_up_lifecycle_clients(flight_log_msg, critical_error_msg, error_count);

        // end
    }


    void receive_past_due_notification(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        response->success = true;
        if(this->begun_desired_motion == false)
        {
            response->message = "Successfully recieved past due notification! Starting trajectory";
            this->print_debug_message("Recieved past due notification from timer, starting servo commander!");


            // Make a Change State service message
            std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> change_state_req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
            lifecycle_msgs::msg::Transition transition_req; // The change state service message is a struct with another message type in it, of type Transition
            transition_req.id = 4; // transition to deactive a node
            change_state_req->transition = transition_req; // pack in the message
            this->servo_jerker_change_state->async_send_request(change_state_req);
            transition_req.id = 2; // cleanup 
            change_state_req->transition = transition_req; // pack in the message
            this->servo_jerker_change_state->async_send_request(change_state_req);transition_req.id = 4; // transition to deactive a node

            // Now tell the servo commander to turn on!
            transition_req.id = 1; // configure
            change_state_req->transition = transition_req;
            this->servo_command_change_state->async_send_request(change_state_req); // set state to active
            transition_req.id = 3; // activate
            change_state_req->transition = transition_req;
            this->servo_command_change_state->async_send_request(change_state_req); // set state to active
            this->print_debug_message("Manager is starting the servo commander!");
            this->begun_desired_motion = true;
        }
        else
        {
            response->message = "Successfully recieved past due notification, but we've already reached our altitude!";
            this->print_debug_message("Recieved past due notification from timer, but we've already reached our altitude!");
        }
    }

    void reached_altitude_notification(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        response->success = true;
        if(this->begun_desired_motion == false)
        {
            response->message = "Successfully informed that we have reached out operating altitude, starting servo commander!";
            this->print_debug_message("Successfully informed that we have reached out operating altitude, starting servo commander!");


            // Make a Change State service message
            std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> change_state_req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
            lifecycle_msgs::msg::Transition transition_req; // The change state service message is a struct with another message type in it, of type Transition
            transition_req.id = 4; // transition to deactive a node
            change_state_req->transition = transition_req; // pack in the message
            this->servo_jerker_change_state->async_send_request(change_state_req);

            // Now tell the servo commander to turn on!
            transition_req.id = 3;
            change_state_req->transition = transition_req;
            this->servo_command_change_state->async_send_request(change_state_req); // set state to active
            this->print_debug_message("Manager is starting the servo commander!");
            this->begun_desired_motion = true;
        }
        else
        {
            response->message = "Successfully informed that we have reached out operating altitude, but we've already past our time limit!";
            this->print_debug_message("Successfully informed that we have reached out operating altitude, but we've already past our time limit!");
        }
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
        this->bobot_log_files_dir = this->bobot_bin_dir / "log_files";
        this->print_debug_message("Got the path to bobot_bin/ directory: " + bobot_bin_dir.string());

        // Get the error log file path and get the flight log file path
        std::string bobot_error_log_dir = "log_files/error_logs";
        std::string bobot_flight_log_dir = "log_files/flight_logs";
        std::string bobot_error_log_file_path = "log_files/error_logs/bobot_error_log_" + this->start_time + ".txt";
        std::string bobot_flight_log_file_path = "log_files/flight_logs/bobot_flight_log_" + this->start_time + ".txt";
        this->error_log_dir = bobot_bin_dir / bobot_error_log_dir;
        this->flight_log_dir = bobot_bin_dir / bobot_flight_log_dir;
        this->error_log_file_path = bobot_bin_dir / bobot_error_log_file_path;
        this->flight_log_file_path = bobot_bin_dir / bobot_flight_log_file_path;
        this->print_debug_message("Got the path to the error log for this flight: " + error_log_dir.string());
        this->print_debug_message("Got the path to the flight log for this flight: " + flight_log_dir.string());
        return;
    }

    void check_log_file_paths(std::string &flight_log_msg, std::string &critical_error_msg, int &error_count)
    {
        std::string print_to_screen;
        if(std::filesystem::exists(this->bobot_bin_dir) == false)
        {
            print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Creating bobot_bin/ directory as it did not exist");
            this->print_warning_message(print_to_screen);
            print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "bobot_bin/ directory does not exist - taking note of this as this should not be the case!", error_count);
            this->print_error_message(print_to_screen);
            std::filesystem::create_directory(this->bobot_bin_dir); // make the bin directory

            // since bin/ doesnt' exists, error_logs/ and flight_logs/ also don't exist
            std::filesystem::create_directory(this->bobot_log_files_dir);
            std::filesystem::create_directory(this->error_log_dir);
            std::filesystem::create_directory(this->flight_log_dir);
            print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Created bobot_bin, bobot_bin/log_files/flight_logs/, bobot_bin/log_files/error_logs/");
            this->print_warning_message(print_to_screen);
            // done this case!
        }
        // If bobot_bin exists, this is good
        else if(std::filesystem::exists(this->bobot_bin_dir) == true)
        {
            if(std::filesystem::exists(this->bobot_log_files_dir) == false)
            {
                print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Creating bobot_bin/log_files/ directory as it did not exist");
                this->print_warning_message(print_to_screen);
                print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "bobot_bin/log_files/ directory does not exist - taking note of this as this should not be the case!", error_count);
                this->print_error_message(print_to_screen);
                std::filesystem::create_directory(this->bobot_log_files_dir); // make the bin directory
                // done this case!
            }
            // Now check if flight_logs/ exists
            if(std::filesystem::exists(this->flight_log_dir) == false)
            {
                print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Creating bobot_bin/log_files/flight_logs/ directory as it did not exist");
                this->print_warning_message(print_to_screen);
                print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "bobot_bin/log_files/flight_logs/ directory does not exist - taking note of this as this should not be the case!", error_count);
                this->print_error_message(print_to_screen);
                std::filesystem::create_directory(this->flight_log_dir);
                print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Created bobot_bin/log_files/flight_logs/");
                this->print_warning_message(print_to_screen);
                // done this case!
            }
            else // if it DOES exist, just log it
            {
                print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "bobot_bin/log_files/flight_logs/ directory exists! Moving on...");
                this->print_debug_message(print_to_screen);
                // done this case!
            }

            // Now check if error_logs/ exists
            if(std::filesystem::exists(this->error_log_dir) == false)
            {
                print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Creating bobot_bin/log_files/error_logs/ directory as it did not exist");
                this->print_warning_message(print_to_screen);
                print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "bobot_bin/log_files/error_logs/ directory does not exist - taking note of this as this should not be the case!", error_count);
                this->print_error_message(print_to_screen);
                std::filesystem::create_directory(this->error_log_dir);
                print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Created bobot_bin/log_files/error_logs/");
                this->print_warning_message(print_to_screen);
                // done this case!
            }
            else // if it DOES exist, just log it
            {
                print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "bobot_bin/log_files/error_logs/ directory exists! Moving on...");
                this->print_debug_message(print_to_screen);
                // done this case!
            }
        }

        // Now, check if the files already exist
        if(std::filesystem::exists(this->error_log_file_path) == false)
        {
            print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Error log file did not previously exist, creating it now!");
            this->print_debug_message(print_to_screen);
            print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "New error log file: " + this->error_log_file_path.string());
            this->print_debug_message(print_to_screen);
            // done this case!
        }
        else if(std::filesystem::exists(this->error_log_file_path) == true)
        {
            print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Error log file already exists, appending new data to it");
            this->print_debug_message(print_to_screen);
            print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Found error log file: " + this->error_log_file_path.string());
            this->print_debug_message(print_to_screen);
            print_to_screen = this->add_to_error_log_init_msg(critical_error_msg, "Found an already existing error log at our start_time, indicating we've restarted. We should have already noted this!");
            this->print_warning_message(print_to_screen);
            // done this case!
        }
        
        if(std::filesystem::exists(this->flight_log_file_path) == false)
        {
            print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Flight log file did not previously exist, creating it now!");
            this->print_debug_message(print_to_screen);
            print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "New flight log file: " + this->flight_log_file_path.string());
            this->print_debug_message(print_to_screen);
            // done this case!
        }
        else if(std::filesystem::exists(this->flight_log_file_path) == true)
        {
            print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Flight log file already exists, appending new data to it");
            this->print_debug_message(print_to_screen);
            print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Found flight log file: " + this->error_log_file_path.string());
            this->print_debug_message(print_to_screen);
            print_to_screen = this->add_to_error_log_init_msg(critical_error_msg, "Found an already existing flight log at our start_time, indicating we've restarted. We should have already noted this!");
            this->print_warning_message(print_to_screen);
            // done this case!
        }
        // Done path checking!
        return;
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
    
    std::string add_to_flight_log_init_msg(std::string &flight_log_msg, std::string new_msg)
    {
        std::string curr_time = this->get_current_time_for_logs();
        flight_log_msg = flight_log_msg + "\n[FLIGHT LOG :  " + curr_time + "] " + new_msg;
        return "[FLIGHT LOG: " + curr_time + "] " + new_msg;
    }

    std::string add_to_error_log_init_msg(std::string &critical_error_msg, std::string new_msg)
    {
        std::string curr_time = this->get_current_time_for_logs();
        critical_error_msg = critical_error_msg + "\n[ERROR LOG :  " + curr_time + "] " + new_msg;
        return "[ERROR LOG: " + curr_time + "] " + new_msg;
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

    void write_to_current_flight_file(std::string &critical_error_msg, int &error_count)
    {
        // Create a temporary string to send to the current_flight.txt file
        std::string current_flight_information = "CURRENT FLIGHT DATA FOR RECOVERY\ntime:\n" + this->start_time + "\n\nHAPPY BOBOT-ING";
        std::ofstream current_flight_txt;
        current_flight_txt.open((std::filesystem::current_path() / "bobot_recovery/current_flight.txt").string(), std::ios::out);
        if(current_flight_txt.is_open() == false)
        {
            std::string print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "Could not open current_flight.txt when trying to save start_time for the first time! This is a critical error - system is unable to write the captured start time and will be unable to recover", error_count);
            this->print_error_message(print_to_screen);
        }
        current_flight_txt << current_flight_information << std::endl;
        current_flight_txt.flush();
        current_flight_txt.close();
        return;
    }
    void write_to_reset_counter_file(std::string &critical_error_msg, int &error_count)
    {
        // Create a temporary string to send to the reset_counter.txt file
        std::string reset_counter = std::to_string(this->reset_counter);
        std::ofstream reset_counter_txt;
        reset_counter_txt.open((std::filesystem::current_path() / "bobot_recovery/reset_counter.txt").string(), std::ios::out);
        if(reset_counter_txt.is_open() == false)
        {
            std::string print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "Could not open current_flight.txt when trying to save start_time for the first time! This is a critical error - system is unable to write the captured start time and will be unable to recover", error_count);
            this->print_error_message(print_to_screen);
        }
        reset_counter_txt << reset_counter << std::endl;
        reset_counter_txt.flush();
        reset_counter_txt.close();
        return;
    }

    void set_up_lifecycle_clients(std::string &flight_log_msg, std::string &critical_error_msg, int &error_count)
    {
        // // Flight timer
        // this->flight_timer_get_state = this->create_client<lifecycle_msgs::srv::GetState>("flight_timer/get_state");
        // this->flight_timer_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>("flight_timer/change_state");

        // Servo Jerker
        this->servo_jerker_get_state = this->create_client<lifecycle_msgs::srv::GetState>("servo_jerker/get_state");
        this->servo_jerker_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>("servo_jerker/change_state");

        // Servo Command
        // This is, at this point, a trajectory generator
        this->servo_command_get_state = this->create_client<lifecycle_msgs::srv::GetState>("bobot_trajectory_generator/get_state");
        this->servo_command_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>("bobot_trajectory_generator/change_state");

        // Altitude Monitor 
        this->altitude_monitor_get_state = this->create_client<lifecycle_msgs::srv::GetState>("altitude_monitor/get_state");
        this->altitude_monitor_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>("altitude_monitor/change_state");

        // wait for each service sequentially:
        std::string print_to_screen;
        print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Attempting to connect to the flight timer's /get_state service!");
        this->print_debug_message(print_to_screen);

        // While we want to be able to control this, we don't really want to have this get caught up and cause our whole system to freeze. These
        // critical nodes will start in the "active" state anyway, and the purpose of the lifecycle is more to be able to turn these off when we 
        // no longer need them to help with processing! (At least that is my reason)
        // // Timer
        // if(!flight_timer_get_state->wait_for_service(std::chrono::seconds(5)))
        // {
        //     print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "Unable to connect to /get_state service for the flight timer! this is bad!", error_count);
        //     this->print_error_message(print_to_screen);
        // }
        // else
        // {
        //     print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Successfully connected to /get_state service for the flight timer!");
        //     this->print_debug_message(print_to_screen);
        // }
        // if(!flight_timer_change_state->wait_for_service(std::chrono::seconds(5)))
        // {
        //     print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "Unable to connect to /change_state service for the flight timer! this is bad!", error_count);
        //     this->print_error_message(print_to_screen);
        // }
        // else
        // {
        //     print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Successfully connected to /change_state service for the flight timer!");
        //     this->print_debug_message(print_to_screen);
        // }
        
        // Create over time limit server
        this->past_due_timer_service = create_service<std_srvs::srv::Trigger>("simple_timer/is_past_due", std::bind(&BobotManager::receive_past_due_notification, this, std::placeholders::_1, std::placeholders::_2));
        // Create past altitude server
        this->reached_altitude_service = create_service<std_srvs::srv::Trigger>("altitude_monitor/altitude_reached", std::bind(&BobotManager::reached_altitude_notification, this, std::placeholders::_1, std::placeholders::_2));

        // Servo Jerker
        if(!servo_jerker_get_state->wait_for_service(std::chrono::seconds(5)))
        {
            print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "Unable to connect to /get_state service for the servo jerker! this is bad!", error_count);
            this->print_error_message(print_to_screen);
        }
        else
        {
            print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Successfully connected to /get_state service for the servo jerker!");
            this->print_debug_message(print_to_screen);
        }
        if(!servo_jerker_change_state->wait_for_service(std::chrono::seconds(5)))
        {
            print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "Unable to connect to /change_state service for the servo jerker! this is bad!", error_count);
            this->print_error_message(print_to_screen);
        }
        else
        {
            print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Successfully connected to /change_state service for the servo jerker!");
            this->print_debug_message(print_to_screen);
        }

        // Servo commander
        if(!servo_command_get_state->wait_for_service(std::chrono::seconds(5)))
        {
            print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "Unable to connect to /get_state service for the servo commander! this is bad!", error_count);
            this->print_error_message(print_to_screen);
        }
        else
        {
            print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Successfully connected to /get_state service for the servo commander!");
            this->print_debug_message(print_to_screen);
        }
        if(!servo_command_change_state->wait_for_service(std::chrono::seconds(5)))
        {
            print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "Unable to connect to /change_state service for the servo commander! this is bad!", error_count);
            this->print_error_message(print_to_screen);
        }
        else
        {
            print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Successfully connected to /change_state service for the servo commander!");
            this->print_debug_message(print_to_screen);
        }

        // Altitude Monitor
        if(!altitude_monitor_get_state->wait_for_service(std::chrono::seconds(5)))
        {
            print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "Unable to connect to /get_state service for the altitude monitor! this is bad!", error_count);
            this->print_error_message(print_to_screen);
        }
        else
        {
            print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Successfully connected to /get_state service for the altitude monitor!");
            this->print_debug_message(print_to_screen);
        }
        if(!altitude_monitor_change_state->wait_for_service(std::chrono::seconds(5)))
        {
            print_to_screen = this->add_to_critical_error_log_init_msg(critical_error_msg, "Unable to connect to /change_state service for the altitude monitor! this is bad!", error_count);
            this->print_error_message(print_to_screen);
        }
        else
        {
            print_to_screen = this->add_to_flight_log_init_msg(flight_log_msg, "Successfully connected to /change_state service for the altitude monitor!");
            this->print_debug_message(print_to_screen);
        }

        // Make a Change State service message
        std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> change_state_req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        lifecycle_msgs::msg::Transition transition_req; // The change state service message is a struct with another message type in it, of type Transition
        transition_req.id = 1; // configure
        change_state_req->transition = transition_req; // pack in the message

        // Turn on each of our nodes
        servo_jerker_change_state->async_send_request(change_state_req);
        altitude_monitor_change_state->async_send_request(change_state_req);
        transition_req.id = 3; //activate
        change_state_req->transition = transition_req;
        servo_jerker_change_state->async_send_request(change_state_req);
        altitude_monitor_change_state->async_send_request(change_state_req);
    }

    void set_up_pubs_and_subs()
    {

    }

private:
    // ROS VARIABLES
    std::string ros_logger_name;

// lifecycle node services and clients
    // Flight timer services and clients
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> past_due_timer_service;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> flight_timer_get_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> flight_timer_change_state;
    // TODO, maybe -add the other clients so we can utilize the full functionality of the states (probably unnecessary for us)

//// ---- THIS WILL SOON BE CHANGE TO CONTROLLERS AND THE CONTROLLER MANAGER
    // Servo jerker services and clients (soon to be hardware interface clients)
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> servo_jerker_get_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> servo_jerker_change_state;
    // TODO, maybe -add the other clients so we can utilize the full functionality of the states (probably unnecessary for us)

    // Servo command services and clients
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> servo_command_get_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> servo_command_change_state;
    // TODO, maybe -add the other clients so we can utilize the full functionality of the states (probably unnecessary for us)
//// ---- END

    // Altitude Monitor services and clients
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> reached_altitude_service;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> altitude_monitor_get_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> altitude_monitor_change_state;
    // TODO, maybe -add the other clients so we can utilize the full functionality of the states (probably unnecessary for us)

// Manager Node publishers and subscribers
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> balls;


    // FLIGHT VS DEV VARIABLES
    bool IS_FLIGHT = false;

    // TIME AND DATA LOGGING VARIABLES
    std::string start_time;
    int reset_counter = 0;
    std::filesystem::path bobot_bin_dir;
    std::filesystem::path bobot_log_files_dir;
    std::filesystem::path error_log_dir;
    std::filesystem::path flight_log_dir;
    std::filesystem::path error_log_file_path;
    std::filesystem::path flight_log_file_path;

    // FILE HANDLES
    std::ofstream timer_log;
    std::ofstream altitude_log;
    std::ofstream error_log;
    std::ofstream flight_log;

    // boolean to check if we are beginning desired motion
    bool begun_desired_motion = false;

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

