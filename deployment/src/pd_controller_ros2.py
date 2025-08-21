import numpy as np
import yaml
from typing import Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Bool

from topic_names import WAYPOINT_TOPIC, REACHED_GOAL_TOPIC

CONFIG_PATH = "../config/robot.yaml"
with open(CONFIG_PATH, "r") as f:
    robot_config = yaml.safe_load(f)
MAX_V = robot_config["max_v"]
MAX_W = robot_config["max_w"]
VEL_TOPIC_CFG = robot_config.get("vel_navi_topic", "/cmd_vel")
DT = 1/robot_config["frame_rate"]
RATE = 9.0
EPS = 1e-8
WAYPOINT_TIMEOUT = 1.0


def clip_angle(theta: float) -> float:
    theta = np.fmod(theta, 2 * np.pi)
    if -np.pi < theta < np.pi:
        return theta
    return theta - 2 * np.pi


class PDControllerNode(Node):
    def __init__(self):
        super().__init__('pd_controller_ros2')
        vel_topic = VEL_TOPIC_CFG
        if 'cmd_vel_mux' in vel_topic:
            self.get_logger().warn("vel_navi_topic uses cmd_vel_mux (ROS1). Using /cmd_vel for ROS 2.")
            vel_topic = "/cmd_vel"

        self.vel_pub = self.create_publisher(Twist, vel_topic, 1)
        self.waypoint_sub = self.create_subscription(Float32MultiArray, WAYPOINT_TOPIC, self.callback_waypoint, 10)
        self.reached_goal_sub = self.create_subscription(Bool, REACHED_GOAL_TOPIC, self.callback_reached_goal, 10)

        self.timer = self.create_timer(1.0 / RATE, self.loop)

        self.last_waypoint_time = self.get_clock().now()
        self.waypoint = None
        self.reached_goal = False

        self.get_logger().info("PD Controller ROS 2 node started.")

    def callback_waypoint(self, msg: Float32MultiArray):
        self.waypoint = np.array(msg.data, dtype=float)
        self.last_waypoint_time = self.get_clock().now()

    def callback_reached_goal(self, msg: Bool):
        self.reached_goal = bool(msg.data)

    def pd_controller(self, waypoint: np.ndarray) -> Tuple[float, float]:
        assert len(waypoint) in (2, 4)
        if len(waypoint) == 2:
            dx, dy = waypoint
        else:
            dx, dy, hx, hy = waypoint
        if len(waypoint) == 4 and abs(dx) < EPS and abs(dy) < EPS:
            v = 0.0
            w = clip_angle(np.arctan2(hy, hx))/DT
        elif abs(dx) < EPS:
            v = 0.0
            w = np.sign(dy) * np.pi/(2*DT)
        else:
            v = dx / DT
            w = np.arctan(dy/dx) / DT
        v = float(np.clip(v, 0, MAX_V))
        w = float(np.clip(w, -MAX_W, MAX_W))
        return v, w

    def loop(self):
        cmd = Twist()
        if self.reached_goal:
            self.vel_pub.publish(cmd)
            self.get_logger().info("Reached goal! Stopping...")
            return
        # waypoint timeout check
        elapsed = (self.get_clock().now() - self.last_waypoint_time).nanoseconds / 1e9
        if self.waypoint is not None and elapsed < WAYPOINT_TIMEOUT:
            v, w = self.pd_controller(self.waypoint)
            cmd.linear.x = v
            cmd.angular.z = w
        self.vel_pub.publish(cmd)


def main():
    rclpy.init()
    node = PDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
