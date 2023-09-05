
# |***************************************************************|
# |* (c) Copyright 2023                                           |
# |* Balloon Payload Program, University of Maryland              |
# |* College Park, MD 20742                                       |
# |* https://bpp.umd.edu                                          |
# |***************************************************************|


import rclpy, os, datetime
from rclpy.node import Node # Need to do this because ROS is cringe


class BobotManagerNode(Node): # Create a new class that inherits the rclpy.Node Class
    def __init__(self): # Define the initialze function
        super().__init__("BobotManger") # Input the required args for the rclpy.Node object (basic input is only the node name)
        
        self.flight_start_time = datetime.datetime.now().strftime("%d-%m-%y_%H%M%S")
        # Check to see if our "bin" files are made
        src_path = os.path.join(os.getcwd(), "src", "bobot_bin") # The path to the bobot-1/src/bobot_bin directory
        logs_path = os.path.join(src_path, "log_files") # The path to the bobot-1/src/bobot_bin/log_files directory
        error_log_file_path = os.path.join(logs_path, "error_logs", "bobot_log_" + self.flight_start_time + ".txt") # Save the name of the error log file path
        log_file_path = os.path.join(logs_path, "flight_logs", "bobot_log_" + self.flight_start_time + ".txt") # Save the name of the regular log file path

        built_bin_path = False # Bool to see if we've already built the files
        if not os.path.exists(src_path): # Check to see if the src file exists
            self.get_logger().warning("Bobot bin directory does not exists - taking note of this") # Throw a warning message
            os.mkdir(src_path) # Make the bobot_bin directory
            os.mkdir(logs_path) # Make the bobot_bin/log_files directory
            os.mkdir(os.path.join(logs_path, "flight_logs")) # Create the flight_logs directory
            os.mkdir(os.path.join(logs_path, "error_logs")) # Create the error_logs directory
            errorlogfile = open(error_log_file_path, "w") # Open the error logs, and write error messages
            errorlogfile.write("[Error Log]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] Program started, but bobot_bin/ directory did not exist!\n")
            errorlogfile.write("[Error Log]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] creating path: src/bobot_bin/log_files/error_logs\n")
            errorlogfile.write("[Error Log]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] creating path: src/bobot_bin/log_files/flight_logs\n")
            errorlogfile.close()
            built_bin_path = True # Let the below functions know that we already built everything

        # If the previous statement did not execute (aka bobot_bin exists), double check that the other directories exist
        if not os.path.exists(logs_path) and built_bin_path == False: # If the bobot_bin/log_files directory does not exist, make it and it's subsequent directories
            self.get_logger().warning("nobody should ever see this 1")
            os.mkdir(logs_path)
            os.mkdir(os.path.join(logs_path, "flight_logs"))
            os.mkdir(os.path.join(logs_path, "error_logs"))
            errorlogfile = open(error_log_file_path, "w")
            errorlogfile.write("[Error Log]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] creating path: src/bobot_bin/log_files/error_logs\n")
            errorlogfile.write("[Error Log]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] creating path: src/bobot_bin/log_files/flight_logs\n")
            errorlogfile.close()
            built_bin_path = True # Let everyone know we've built everything
        if not os.path.exists(os.path.join(logs_path, "error_logs")) and built_bin_path == False:
            self.get_logger().warning("nobody should ever see this 2")
            os.mkdir(os.path.join(logs_path, "error_logs"))
            errorlogfile = open(error_log_file_path, "w")
            errorlogfile.write("[Error Log]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] creating path: src/bobot_bin/log_files/error_logs\n")
            errorlogfile.close()
        if not os.path.exists(os.path.join(logs_path, "flight_logs")) and built_bin_path == False:
            errorlogfile = open(error_log_file_path, "w")
            self.get_logger().warning("nobody should ever see this 3")
            os.mkdir(os.path.join(logs_path, "flight_logs"))
            errorlogfile.write("[Error Log]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] creating path: src/bobot_bin/log_files/flight_logs\n")
            errorlogfile.close()

        self.get_logger().info("Starting up Bobot!")
        logfile = open(log_file_path, "w")
        logfile.write("[BobotManager]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] Starting up Bobot!\n")
        
        if not os.path.exists(os.path.join(src_path, "timer.txt")):
            self.get_logger().info("creating timer watcher file")
            logfile.write("[BobotManager]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] creating timer watcher file\n")
            open(os.path.join(src_path, "timer.txt"), "w").close()
            
        if not os.path.exists(os.path.join(src_path, "abs_position.txt")):
            self.get_logger().info("creating abs_position watcher file")
            logfile.write("[BobotManager]-[" + datetime.datetime.now().strftime("%d-%m-%y_%H%M%S") + "] creating abs_position watcher file\n")
            open(os.path.join(src_path, "abs_position.txt"), "w").close()

        # Start a subscriber to read the timer and write to the file(?)

        # Start a subscriber to read the abs_position and write to the file
        



        
        








def main():
    rclpy.init()

    balls = BobotManagerNode()
    rclpy.spin(balls)

if __name__ == "__main__":
    main()