import os
import time
import argparse
import yaml
import numpy as np
import torch
from PIL import Image as PILImage

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray

from utils import msg_to_pil, to_numpy, transform_images, load_model
from vint_train.training.train_utils import get_action

TOPOMAP_IMAGES_DIR = "../topomaps/images"
MODEL_WEIGHTS_PATH = "../model_weights"
ROBOT_CONFIG_PATH = "../config/robot.yaml"
MODEL_CONFIG_PATH = "../config/models.yaml"

class NavigateNode(Node):
    def __init__(self, args):
        super().__init__('vint_navigation')
        self.args = args
        self.context_queue = []
        self.context_size = None
        self.subgoal = []
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Using device: {self.device}")

        # Load robot config
        with open(ROBOT_CONFIG_PATH, "r") as f:
            robot_config = yaml.safe_load(f)
        self.max_v = robot_config["max_v"]
        self.max_w = robot_config["max_w"]
        self.rate_hz = robot_config["frame_rate"]

        # Load model config
        with open(MODEL_CONFIG_PATH, "r") as f:
            model_paths = yaml.safe_load(f)
        model_config_path = model_paths[args.model]["config_path"]
        with open(model_config_path, "r") as f:
            self.model_params = yaml.safe_load(f)
        self.context_size = self.model_params["context_size"]

        # Load model weights
        ckpth_path = model_paths[args.model]["ckpt_path"]
        if os.path.exists(ckpth_path):
            self.get_logger().info(f"Loading model from {ckpth_path}")
        else:
            raise FileNotFoundError(f"Model weights not found at {ckpth_path}")
        self.model = load_model(ckpth_path, self.model_params, self.device)
        self.model = self.model.to(self.device)
        self.model.eval()

        # Load topomap
        topomap_dir = os.path.join(TOPOMAP_IMAGES_DIR, args.dir)
        topomap_filenames = sorted(os.listdir(topomap_dir), key=lambda x: int(x.split(".")[0]))
        self.topomap = [PILImage.open(os.path.join(topomap_dir, fname)) for fname in topomap_filenames]
        self.closest_node = 0
        self.goal_node = args.goal_node if args.goal_node != -1 else len(self.topomap) - 1
        assert -1 <= self.goal_node < len(self.topomap), "Invalid goal index"
        self.reached_goal = False

        # ROS 2 publishers/subscribers
        self.image_sub = self.create_subscription(Image, args.image_topic, self.callback_obs, 1)
        self.waypoint_pub = self.create_publisher(Float32MultiArray, args.waypoint_topic, 1)
        self.goal_pub = self.create_publisher(Bool, '/topoplan/reached_goal', 1)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.navigation_loop)

        self.get_logger().info("Node initialized. Waiting for image observations...")

    def callback_obs(self, msg):
        obs_img = msg_to_pil(msg)
        if self.context_size is not None:
            if len(self.context_queue) < self.context_size + 1:
                self.context_queue.append(obs_img)
            else:
                self.context_queue.pop(0)
                self.context_queue.append(obs_img)

    def navigation_loop(self):
        if self.reached_goal:
            return
        chosen_waypoint = np.zeros(4)
        if len(self.context_queue) > self.model_params["context_size"]:
            start = max(self.closest_node - self.args.radius, 0)
            end = min(self.closest_node + self.args.radius + 1, self.goal_node)
            distances = []
            waypoints = []
            batch_obs_imgs = []
            batch_goal_data = []
            for i, sg_img in enumerate(self.topomap[start: end + 1]):
                transf_obs_img = transform_images(self.context_queue, self.model_params["image_size"])
                goal_data = transform_images(sg_img, self.model_params["image_size"])
                batch_obs_imgs.append(transf_obs_img)
                batch_goal_data.append(goal_data)
            batch_obs_imgs = torch.cat(batch_obs_imgs, dim=0).to(self.device)
            batch_goal_data = torch.cat(batch_goal_data, dim=0).to(self.device)
            distances, waypoints = self.model(batch_obs_imgs, batch_goal_data)
            distances = to_numpy(distances)
            waypoints = to_numpy(waypoints)
            min_dist_idx = np.argmin(distances)
            if distances[min_dist_idx] > self.args.close_threshold:
                chosen_waypoint = waypoints[min_dist_idx][self.args.waypoint]
                self.closest_node = start + min_dist_idx
            else:
                chosen_waypoint = waypoints[min(min_dist_idx + 1, len(waypoints) - 1)][self.args.waypoint]
                self.closest_node = min(start + min_dist_idx + 1, self.goal_node)
        if self.model_params["normalize"]:
            chosen_waypoint[:2] *= (self.max_v / self.rate_hz)
        waypoint_msg = Float32MultiArray()
        waypoint_msg.data = chosen_waypoint
        self.waypoint_pub.publish(waypoint_msg)
        self.reached_goal = self.closest_node == self.goal_node
        self.goal_pub.publish(Bool(data=self.reached_goal))
        if self.reached_goal:
            self.get_logger().info("Reached goal! Stopping...")

def main():
    parser = argparse.ArgumentParser(description="ROS 2 ViNT Navigation Node")
    parser.add_argument("--model", "-m", default="vint", type=str, help="model name (default: vint)")
    parser.add_argument("--waypoint", "-w", default=2, type=int, help="index of the waypoint used for navigation")
    parser.add_argument("--dir", "-d", default="topomap", type=str, help="path to topomap images")
    parser.add_argument("--goal-node", "-g", default=-1, type=int, help="goal node index in the topomap")
    parser.add_argument("--close-threshold", "-t", default=3, type=int, help="temporal distance for localization")
    parser.add_argument("--radius", "-r", default=4, type=int, help="number of nodes to look at in the topomap")
    parser.add_argument("--image-topic", default="/camera/image_raw", type=str, help="ROS 2 image topic")
    parser.add_argument("--waypoint-topic", default="/waypoint", type=str, help="ROS 2 waypoint topic")
    args = parser.parse_args()
    rclpy.init()
    node = NavigateNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()