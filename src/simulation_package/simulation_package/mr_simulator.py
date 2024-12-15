import rclpy
from rclpy.node import Node

class robot_simulator(Node):
    def __init__(self):
        super().__init__("robot_simulator")

def main(args=None):
    rclpy.init(args=args)
    node = robot_simulator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()