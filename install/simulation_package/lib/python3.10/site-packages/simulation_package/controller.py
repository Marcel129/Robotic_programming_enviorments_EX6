import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class controller(Node):
    def __init__(self):
        super().__init__("robot_controller")

        self.controlPublisher = self.create_publisher(Twist, "/control", 10)
        self.timer = self.create_timer(1, self.send_control_values)

        self.counter = 0

    def send_control_values(self):
        msg = Twist()
        if self.counter < 100:
            msg.angular.x = 1.0
            msg.linear.x = 2.0
        else:
            msg.angular.x = 0.0
            msg.linear.x = 0.0

        self.counter += 1

        self.controlPublisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()