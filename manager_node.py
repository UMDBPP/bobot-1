
# |***************************************************************|
# |* (c) Copyright 2023                                           |
# |* Balloon Payload Program, University of Maryland              |
# |* College Park, MD 20742                                       |
# |* https://bpp.umd.edu                                          |
# |***************************************************************|


import rclpy, os, datetime
from rclpy.node import Node # Need to do this because ROS is cringe

# -- BOBOT MSGS -- #
from bobot_msgs.msg import BobotTimer
from bobot_msgs.msg import BobotAbsPosition
from lifecycle_msgs.srv import GetState, ChangeState, GetAvailableStates, GetAvailableTransitions

# -- BOBOT SRVS -- #
from bobot_msgs.srv import BobotLastTimerValue

class BobotManagerNode(Node): # Create a new class that inherits the rclpy.Node Class
    def __init__(self): # Define the initialze function
        super().__init__("BobotManger") # Input the required args for the rclpy.Node object (basic input is only the node name)
        
        self.flight_start_time = datetime.datetime.now().strftime("%d-%m-%y_%H%M%S")
        # Check to see if our "bin" files are made
        src_path = os.path.join(os.getcwd(), "src", "bobot_bin") # The path to the bobot-1/src/bobot_bin directory
        logs_path = os.path.join(src_path, "log_files") # The path to the bobot-1/src/bobot_bin/log_files directory
        self.error_log_file_path = os.path.join(logs_path, "error_logs", "bobot_log_" + self.flight_start_time + ".txt") # Save the name of the error log file path
        self.log_file_path = os.path.join(logs_path, "flight_logs", "bobot_log_" + self.flight_start_time + ".txt") # Save the name of the regular log file path

        built_bin_path = False # Bool to see if we've already built the files
        if not os.path.exists(src_path): # Check to see if the src file exists
            self.get_logger().warning("Bobot bin directory does not exists - taking note of this") # Throw a warning message
            os.mkdir(src_path) # Make the bobot_bin directory
            os.mkdir(logs_path) # Make the bobot_bin/log_files directory
            os.mkdir(os.path.join(logs_path, "flight_logs")) # Create the flight_logs directory
            os.mkdir(os.path.join(logs_path, "error_logs")) # Create the error_logs directory
            errorlogfile = open(self.error_log_file_path, "w") # Open the error logs, and write error messages
            errorlogfile.write("[Error Log]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] Program started, but bobot_bin/ directory did not exist!\n")
            errorlogfile.write("[Error Log]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] creating path: src/bobot_bin/log_files/error_logs\n")
            errorlogfile.write("[Error Log]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] creating path: src/bobot_bin/log_files/flight_logs\n")
            errorlogfile.close()
            built_bin_path = True # Let the below functions know that we already built everything

        # If the previous statement did not execute (aka bobot_bin exists), double check that the other directories exist
        if not os.path.exists(logs_path) and built_bin_path == False: # If the bobot_bin/log_files directory does not exist, make it and it's subsequent directories
            self.get_logger().warning("nobody should ever see this 1") # log a warning
            os.mkdir(logs_path) # make the path to the log files directory
            os.mkdir(os.path.join(logs_path, "flight_logs")) # make the subdirectory in the log files directory for flight logs
            os.mkdir(os.path.join(logs_path, "error_logs")) # make the subdirectory in the log files directory error logs
            errorlogfile = open(self.error_log_file_path, "w") # open the file for error logging 
            errorlogfile.write("[Error Log]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] creating path: src/bobot_bin/log_files/error_logs\n")
            errorlogfile.write("[Error Log]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] creating path: src/bobot_bin/log_files/flight_logs\n")
            errorlogfile.close() # close the file so we don't seg fault
            built_bin_path = True # Let everyone know we've built everything
        if not os.path.exists(os.path.join(logs_path, "error_logs")) and built_bin_path == False:
            self.get_logger().warning("nobody should ever see this 2") # log a warning
            os.mkdir(os.path.join(logs_path, "error_logs")) # create the error log file only since flight logs exist
            errorlogfile = open(self.error_log_file_path, "w") # open the error file to write too
            errorlogfile.write("[Error Log]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] creating path: src/bobot_bin/log_files/error_logs\n")
            errorlogfile.close() # close the error log
        if not os.path.exists(os.path.join(logs_path, "flight_logs")) and built_bin_path == False:
            errorlogfile = open(self.error_log_file_path, "w") # open the the error log file
            self.get_logger().warning("nobody should ever see this 3")
            os.mkdir(os.path.join(logs_path, "flight_logs")) # make the path to the flight logs 
            errorlogfile.write("[Error Log]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] creating path: src/bobot_bin/log_files/flight_logs\n")
            errorlogfile.close() # close the error log

        self.get_logger().info("Starting up Bobot!")
        self.logfile = open(self.log_file_path, "w") # start writing to the log file
        self.logfile.write("[BobotManager]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] Starting up Bobot!\n")
        
        # Make our save state files so that we can start writing to it
        if not os.path.exists(os.path.join(src_path, "timer.txt")):
            self.get_logger().info("creating timer watcher file")
            self.logfile.write("[BobotManager]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] creating timer watcher file\n")
            open(os.path.join(src_path, "timer.txt"), "w").close() # save it
            
        if not os.path.exists(os.path.join(src_path, "abs_position.txt")):
            self.get_logger().info("creating abs_position watcher file")
            self.logfile.write("[BobotManager]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] creating abs_position watcher file\n")
            open(os.path.join(src_path, "abs_position.txt"), "w").close() # save it

        if not os.path.exists(os.path.join(src_path, "reset_counter.txt")):
            self.get_logger().info("creating reset counter file")
            self.logfile.write("[BobotManager]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] creating reset counter file\n")
            open(os.path.join(src_path, "reset_counter.txt"), "w").close() # save it

        # Put the stuff below into a function

        self.declare_parameter("bobot_name", rclpy.Parameter.Type.STRING) # declare the ros parameter for the bobot_name
        bobot_name = self.get_parameter("bobot_name") # get the parameter for the bobot_name
        self.get_logger().info("Current bobot's name: %s" % str(bobot_name.value)) 
        self.logfile.write("[BobotManager]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] Current bobot's name: " + str(bobot_name.value))
        
        # create clients to contact the ChangeState srv, GetState srv, and other state info services so that we can query the timer_node and make it start n stuff
        self.flight_timer_change_state = self.create_client(ChangeState, "FlightTimer/change_state")
        self.flight_timer_get_state = self.create_client(GetState, "FlightTimer/get_state")
        self.flight_timer_get_avail_state = self.create_client(GetAvailableStates, "FlightTimer/get_available_states")
        self.flight_timer_get_avail_transitions = self.create_client(GetAvailableTransitions, "FlightTimer/get_available_transitions")
        self.logfile.write("[BobotManager]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] Created Flight Timer Node clients for " + str(bobot_name.value) + "\n")
        self.get_logger().info("Created Flight Timer Node clients")

        # create clients to contact that ChangeState srv, GetState srv, and other state info services so that we can turn on/off the altitude monitor node and do other things
        self.alt_monitor_change_state = self.create_client(ChangeState, "AltitudeManager/change_state")
        self.alt_monitor_get_state = self.create_client(GetState, "AltitudeManager/get_state")
        self.alt_monitor_get_avail_state = self.create_client(GetAvailableStates, "AltitudeManager/get_available_states")
        self.alt_monitor_get_avail_transitions = self.create_client(GetAvailableTransitions, "AltitudeManager/get_available_transitions")
        self.logfile.write("[BobotManager]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] Created Altitude Monitor Node clients for " + str(bobot_name.value) + "\n")
        self.get_logger().info("Created Altitude Monitor Node clients")
        
        # create clients to contact that ChangeState srv, GetState srv, and other state info services so that we can turn on/off the servo jerk node and do other things
        self.servo_jerk_change_state = self.create_client(ChangeState, "ServoJerk/change_state")
        self.servo_jerk_get_state = self.create_client(GetState, "ServoJerk/get_state")
        self.servo_jerk_get_avail_state = self.create_client(GetAvailableStates, "ServoJerk/get_available_states")
        self.servo_jerk_get_avail_transitions = self.create_client(GetAvailableTransitions, "ServoJerk/get_available_transitions")
        self.logfile.write("[BobotManager]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] Created Servo Jerker Node clients for " + str(bobot_name.value) + "\n")
        self.get_logger().info("Created Servo Jerker Node clients")

        # Leaving this big block to spite Dan Grammer
        while (not self.flight_timer_change_state.wait_for_service(timeout_sec=1.0) 
                and self.flight_timer_get_state.wait_for_service(timeout_sec=1.0) 
                and self.flight_timer_get_avail_state.wait_for_service(timeout_sec=1.0) 
                and self.flight_timer_get_avail_transitions.wait_for_service(timeout_sec=1.0)
                and self.alt_monitor_change_state.wait_for_service(timeout_sec=1.0)
                and self.alt_monitor_get_state.wait_for_service(timeout_sec=1.0)
                and self.alt_monitor_get_avail_state.wait_for_service(timeout_sec=1.0)
                and self.alt_monitor_get_avail_transitions.wait_for_service(timeout_sec=1.0)
                and self.servo_jerk_change_state.wait_for_service(timeout_sec=1.0)
                and self.servo_jerk_get_state.wait_for_service(timeout_sec=1.0)
                and self.servo_jerk_get_avail_state.wait_for_service(timeout_sec=1.0)
                and self.servo_jerk_get_avail_transitions.wait_for_service(timeout_sec=1.0)):
            self.get_logger().warn("Flight Timer services not available, waiting...")
        
        # Create variables for each of the services so that we can use their respongses
        
        

        """
        TODO:

        - Need to start processes of activating lifecycle nodes - attempted to use service calls in this python node to no success, need to try again
        after submitting project!

        - incorporate the different board types for the altitude monitor and servo jerker
        - add serial port connection for the arduino board type, so that we can directly communicate with the arduino without having to use micro-ROS
        - begin saving time, and absolute position into their respective files (with redundancies)
        - upload (or send through service) last saved flight timer value to catch if we've reset
        - incorporate way to keep track of resets
        - test, test, test!

        """


    # Make a request to the flight timer to get it's state
    def request_flight_timer_state(self):
        # TODO
        pass

    # Make a request to the Altitude Monitor to get it's state
    def request_alt_monitor_state(self):
        # TODO
        pass

    def change_flight_timer_state(self, transition_id):
        # TODO
        pass
    def change_alt_monitor_state(self, transition_id):
        # TODO
        pass
    def change_servo_jerk_state(self, transition_id):
        # TODO
        pass




def main():
    rclpy.init()

    balls = BobotManagerNode()
    rclpy.spin(balls)
    balls.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()