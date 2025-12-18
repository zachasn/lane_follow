#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from cv_bridge import CvBridge
import cv2
import numpy as np


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.lane_error_pub = self.create_publisher(
            Float32,
            '/lane_error',
            10
        )

        # Publish last known error at a steady rate (5 Hz)
        self.last_error = 0.0
        self.timer = self.create_timer(0.2, self.timer_publish)

        self.get_logger().info("Vision node started (lane detection + /lane_error publishing)")

    def timer_publish(self):
        out = Float32()
        out.data = float(self.last_error)
        self.lane_error_pub.publish(out)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = image.shape

        # ROI: bottom 40%
        roi = image[int(h * 0.6):h, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # White lane threshold (tune if needed)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        M = cv2.moments(mask)

        center_x = w // 2

        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            error = (cx - center_x) / float(center_x)
            self.last_error = float(error)
            lane_found = True
        else:
            # Lane lost: keep last_error (do NOT snap to 0)
            lane_found = False

        # Debug windows (optional)
        # if lane_found:
        #     cv2.circle(roi, (cx, roi.shape[0] // 2), 5, (0, 0, 255), -1)
        # cv2.line(roi, (center_x, 0), (center_x, roi.shape[0]), (255, 0, 0), 2)
        # cv2.imshow("Vision: ROI + Lane Center", roi)
        cv2.imshow("Vision: Mask", mask)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
