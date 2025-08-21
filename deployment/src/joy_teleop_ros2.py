import yaml
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

from topic_names import JOY_BUMPER_TOPIC


CONFIG_PATH = "../config/robot.yaml"
JOY_CONFIG_PATH = "../config/joystick.yaml"


class JoyTeleopNode(Node):
    def __init__(self):
        super().__init__('joy_teleop_ros2')

        with open(CONFIG_PATH, "r") as f:
            robot_config = yaml.safe_load(f)
        with open(JOY_CONFIG_PATH, "r") as f:
            joy_config = yaml.safe_load(f)

        self.max_v = 0.4
        self.max_w = 0.8
        vel_topic = robot_config.get("vel_teleop_topic", "/cmd_vel")
        # If a ROS1 mux topic is configured, fall back to /cmd_vel in ROS2
        if 'cmd_vel_mux' in vel_topic:
            self.get_logger().warn("vel_teleop_topic uses cmd_vel_mux (ROS1). Using /cmd_vel for ROS 2.")
            vel_topic = "/cmd_vel"

        self.deadman_idx = joy_config.get("deadman_switch", 0)
        self.lin_axis = joy_config.get("lin_vel_button", 1)
        self.ang_axis = joy_config.get("ang_vel_button", 0)
        self.rate_hz = 9.0

        self.vel_pub = self.create_publisher(Twist, vel_topic, 1)
        self.bumper_pub = self.create_publisher(Bool, JOY_BUMPER_TOPIC, 1)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.callback_joy, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_loop)

        self.button_pressed = False
        self.bumper = False
        self.current_cmd = Twist()

        self.get_logger().info("JoyTeleop ROS 2 node started. Hold the deadman switch to drive.")

    def callback_joy(self, msg: Joy):
        # Protect array indexing
        if self.deadman_idx < len(msg.buttons):
            self.button_pressed = bool(msg.buttons[self.deadman_idx])
        else:
            self.button_pressed = False

        bumper_idx = max(self.deadman_idx - 1, 0)
        self.bumper = bool(msg.buttons[bumper_idx]) if bumper_idx < len(msg.buttons) else False

        lin = msg.axes[self.lin_axis] if self.lin_axis < len(msg.axes) else 0.0
        ang = msg.axes[self.ang_axis] if self.ang_axis < len(msg.axes) else 0.0

        if self.button_pressed:
            self.current_cmd.linear.x = self.max_v * float(lin)
            self.current_cmd.angular.z = self.max_w * float(ang)
        else:
            self.current_cmd = Twist()

    def publish_loop(self):
        # Publish bumper state regardless
        self.bumper_pub.publish(Bool(data=self.bumper))
        if self.button_pressed:
            self.vel_pub.publish(self.current_cmd)


def main():
    rclpy.init()
    node = JoyTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
