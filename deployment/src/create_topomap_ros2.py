import os
import time
import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from utils import msg_to_pil

TOPOMAP_IMAGES_DIR = "../topomaps/images"


class TopomapRecorder(Node):
    def __init__(self, image_topic: str, out_dir: str, dt: float):
        super().__init__('create_topomap_ros2')
        self.out_dir = os.path.join(TOPOMAP_IMAGES_DIR, out_dir)
        self.dt = float(dt)
        os.makedirs(self.out_dir, exist_ok=True)
        self.get_logger().info(f"Saving images to: {self.out_dir}")

        self.sub = self.create_subscription(Image, image_topic, self.cb_image, 10)
        self.last_img = None
        self.idx = self._init_index()
        self.last_save = 0.0
        self.timer = self.create_timer(max(0.01, self.dt/2.0), self.tick)

    def _init_index(self) -> int:
        # Continue numbering if folder already has images
        nums = []
        for f in os.listdir(self.out_dir):
            name, ext = os.path.splitext(f)
            if ext.lower() in ('.png', '.jpg', '.jpeg') and name.isdigit():
                nums.append(int(name))
        return (max(nums) + 1) if nums else 0

    def cb_image(self, msg: Image):
        try:
            self.last_img = msg_to_pil(msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")

    def tick(self):
        now = time.time()
        if self.last_img is None:
            return
        if now - self.last_save < self.dt:
            return
        out_path = os.path.join(self.out_dir, f"{self.idx}.png")
        try:
            self.last_img.save(out_path)
            self.get_logger().info(f"Saved {out_path}")
            self.idx += 1
            self.last_save = now
        except Exception as e:
            self.get_logger().error(f"Failed to save image: {e}")


def main():
    parser = argparse.ArgumentParser(description="Create a topomap by saving images from a ROS 2 camera topic")
    parser.add_argument('--dir', '-d', default='topomap', help='subfolder under ../topomaps/images')
    parser.add_argument('--dt', '-t', type=float, default=1.0, help='seconds between saved frames')
    parser.add_argument('--image-topic', default='/camera/image_raw', help='camera topic (ROS 2)')
    args = parser.parse_args()

    rclpy.init()
    node = TopomapRecorder(args.image_topic, args.dir, args.dt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
