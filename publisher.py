#!/usr/bin/env python3
import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)

# YOLO's input topic (confirmed from ros2 node info)
YOLO_IMAGE_TOPIC = "/camera/rgb/image_raw"

class ImagePublisher(Node):
    def __init__(self):
        super().__init__("image_publisher_10_images")
        self.bridge = CvBridge()

        # Match YOLO subscriber: RELIABLE, volatile, keep_last
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.publisher_ = self.create_publisher(Image, YOLO_IMAGE_TOPIC, qos)

        # Directory containing your 10 images
        self.images_dir = "/root/ros2_ws/images"

        # Load all JPG/PNG files
        self.image_paths = sorted([
            os.path.join(self.images_dir, f)
            for f in os.listdir(self.images_dir)
            if f.lower().endswith((".jpg", ".jpeg", ".png"))
        ])

        if len(self.image_paths) < 10:
            self.get_logger().error(
                f"Need at least 10 images in {self.images_dir}, found {len(self.image_paths)}"
            )
            rclpy.shutdown()
            return

        self.get_logger().info(f"Loaded {len(self.image_paths)} images.")
        self.index = 0

        # Timer → publish at 5 Hz (real-time enough for YOLO)
        self.timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        img_path = self.image_paths[self.index]
        img = cv2.imread(img_path)

        if img is None:
            self.get_logger().warn(f"Failed to load {img_path}")
            self.index = (self.index + 1) % len(self.image_paths)
            return

        # Convert BGR → RGB because YOLO expects rgb8 images
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Convert to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(img_rgb, encoding="rgb8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"

        # Publish
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {os.path.basename(img_path)}")

        # Move to next image
        self.index = (self.index + 1) % len(self.image_paths)

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()