import rclpy, os, datetime
from rclpy.node import Node # Need to do this because ROS is cringe


class BobotManagerNode(Node): # Create a new class that inherits the rclpy.Node Class
    def __init__(self): # Define the initialze function
        super().__init__("BobotManger") # Input the required args for the rclpy.Node object (basic input is only the node name)
        
        self.flight_start_time = datetime.datetime.now().strftime("%d-%m-%y_%H%M%S")
        # Check to see if our "bin" files are made
        src_path = os.path.join(os.getcwd(), "src", "bobot_bin")
        if not os.path.exists(src_path):
            self.get_logger().warning("Bobot bin directory does not exists - logging")
            os.mkdir(src_path)
            if not os.path.exists(os.path.join(src_path, "log_files")):
                os.mkdir(os.path.join(src_path, "log_files"))
            logfile = open(os.path.join(src_path, "log_files", "bobot_log_" + self.flight_start_time + ".txt"), "w")
            logfile.write("[Error Log]-[" + self.flight_start_time + "] Program started, but Bobot Bin directory did not exist!\n")








def main():
    rclpy.init()

    balls = BobotManagerNode()
    rclpy.spin(balls)

if __name__ == "__main__":
    main()