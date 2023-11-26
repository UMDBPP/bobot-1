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

# -- BOBOT SRVS -- #
from bobot_msgs.srv import BobotLastTimerValue

class BobotTimerNode(Node):
    def __init__(self):
        super().__init__("BobotTimer")

        self.get_last_time_value = self.create_client(BobotLastTimerValue, "last_timer_value")
        while not self.get_last_time_value.wait_for_service(timeout_sec=2):
            self.get_logger().warning("Waiting to get")



def main():
    rclpy.init()
    balls2 = BobotTimerNode()
    rclpy.spin(balls2)

if __name__ == "__main__":
    main()

        