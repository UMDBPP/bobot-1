import rclpy, geometry_msgs



class PubNode(rclpy.Node):
    def __init__(self):
        super().__init__("SendCommands")
    self.PublishDeez = self.create_publisher(geometry_msgs.Float64MultiArray, "topic", 10)
    

def main():
    pass


if __name__ == "__main__":
    main()