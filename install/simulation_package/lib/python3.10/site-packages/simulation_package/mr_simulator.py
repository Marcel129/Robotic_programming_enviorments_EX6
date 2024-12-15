import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist, PoseStamped
import time

class robot_simulator(Node):
    def __init__(self):
        super().__init__("robot_simulator")

        self.current_X = 0
        self.current_Y = 0
        self.current_theta = 0

        self.current_dX = 0
        self.current_dY = 0
        self.current_dtheta = 0

        self.u1 = 0
        self.u2 = 0

        self.delta = 0.1
        self.lastUpdateTime = time.time()

        self.create_subscription(Twist, "/control", self.read_control_values, 10)
        self.robotPositionPublisher = self.create_publisher(PoseStamped, "/robot_position", 10)

        self.timer = self.create_timer(0.1, self.update_robot_state)
        self.timer2 = self.create_timer(0.2, self.public_pose)

    def update_robot_state(self):

        # no control signal for time period
        if time.time() - self.lastUpdateTime > 2.0:
            self.u1 = 0.0
            self.u2 = 0.0

        self.current_dX = self.u1 * math.cos(self.current_dtheta)
        self.current_dY = self.u1 * math.sin(self.current_dtheta)
        self.current_dtheta = self.u2

        self.current_X = self.current_X + self.delta * self.u1 * math.cos(self.current_theta)
        self.current_Y = self.current_Y + self.delta * self.u1 * math.sin(self.current_theta)
        self.current_theta = self.current_theta + self.delta * self.u2

        self.get_logger().info(f"X_pos: {self.current_X}, Y_pos: {self.current_Y}, Theta: {self.current_theta}" )

    def read_control_values(self, msg: Twist):
        self.u1 = msg.linear.x
        self.u2 = msg.angular.x

        self.lastUpdateTime = time.time()

    def public_pose(self):
        pose = PoseStamped()
        pose.pose.position.x = float(self.current_X)
        pose.pose.position.y = float(self.current_Y)

        pose.pose.orientation.x = float(0)
        pose.pose.orientation.y = float(0)
        pose.pose.orientation.z = float(math.sin(self.current_theta / 2))
        pose.pose.orientation.w = float(math.cos(self.current_theta / 2))

        self.robotPositionPublisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = robot_simulator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()