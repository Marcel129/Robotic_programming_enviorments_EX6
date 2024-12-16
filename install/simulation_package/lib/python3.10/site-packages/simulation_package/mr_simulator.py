import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import time

class robot_simulator(Node):
    def __init__(self):
        super().__init__("robot_simulator")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('delta_coeff', rclpy.Parameter.Type.DOUBLE),
                ('no_signal_timeout_s', rclpy.Parameter.Type.DOUBLE)
            ]
        )

        self.delta = self.get_parameter('delta_coeff').get_parameter_value().double_value
        self.no_signal_timeout = self.get_parameter('no_signal_timeout_s').get_parameter_value().double_value

        self.current_X = 0
        self.current_Y = 0
        self.current_theta = 0
        self.current_dX = 0
        self.current_dY = 0
        self.current_dtheta = 0

        self.u1 = 0
        self.u2 = 0

        self.lastUpdateTime = time.time()

        self.create_subscription(Twist, "/control", self.read_control_values, 10)
        self.robotPositionPublisher = self.create_publisher(PoseStamped, "/robot_position", 10)

        self.timer = self.create_timer(0.1, self.update_robot_state)
        self.timer2 = self.create_timer(0.2, self.public_pose)

        self.tf_broadcaster = TransformBroadcaster(self)

    def update_robot_state(self):

        # no control signal for time period
        if time.time() - self.lastUpdateTime > self.no_signal_timeout:
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
        t = TransformStamped()

        pose.pose.position.x = t.transform.translation.x = float(self.current_X)
        pose.pose.position.y = t.transform.translation.y = float(self.current_Y)
        t.transform.translation.z = 0.0

        pose.pose.orientation.x = t.transform.rotation.x = float(0)
        pose.pose.orientation.y = t.transform.rotation.y = float(0)
        pose.pose.orientation.z = t.transform.rotation.z = float(math.sin(self.current_theta / 2))
        pose.pose.orientation.w = t.transform.rotation.w = float(math.cos(self.current_theta / 2))

        self.robotPositionPublisher.publish(pose)


        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'  # Układ globalny
        t.child_frame_id = 'base_link'  # Układ robota

        # t.transform.translation.x = self.global_x
        # t.transform.translation.y = self.global_y

        # t.transform.rotation.x = 0.0
        # t.transform.rotation.y = 0.0
        # t.transform.rotation.z = math.sin(self.global_yaw / 2.0)
        # t.transform.rotation.w = math.cos(self.global_yaw / 2.0)

        # Publikacja transformacji
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = robot_simulator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()