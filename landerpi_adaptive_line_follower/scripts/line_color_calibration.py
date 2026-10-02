#!/usr/bin/env python3
# Copyright 2026 ldavalle
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Standalone calibration helper for landerpi_adaptive_line_follower, same
# role pose_keyboard_teleop.py plays for landerpi_perimeter_guard: a plain
# script outside the C++ node so the node never needs a GUI window of its
# own. Shows the raw camera feed; a mouse click samples that pixel as the
# line's target color via line_follower_node's ~/set_target_color service
# (must be called while the node is IDLE — see the project's CLAUDE.md,
# "Flujo de configuración de color").
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from interfaces.srv import SetPoint
from rclpy.node import Node
from sensor_msgs.msg import Image


class LineColorCalibrationNode(Node):

    def __init__(self):
        super().__init__('line_color_calibration')
        self.declare_parameter('image_topic', '/ascamera/camera_publisher/rgb0/image')
        self.declare_parameter('set_target_color_service', '/line_follower_node/set_target_color')

        image_topic = self.get_parameter('image_topic').value
        service_name = self.get_parameter('set_target_color_service').value

        self.image_width = None
        self.image_height = None
        self.latest_frame = None

        self.set_target_color_client = self.create_client(SetPoint, service_name)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 1)

        cv2.namedWindow('line_color_calibration')
        cv2.setMouseCallback('line_color_calibration', self.mouse_callback)
        self.get_logger().info(
            'Click on the line in the window to calibrate its color. Press q to quit.')

    def image_callback(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height
        self.latest_frame = np.ndarray(
            shape=(msg.height, msg.width, 3), dtype=np.uint8, buffer=msg.data)

    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or self.image_width is None:
            return

        if not self.set_target_color_client.service_is_ready():
            self.get_logger().warn('set_target_color service not available yet')
            return

        request = SetPoint.Request()
        request.data = Point(x=x / self.image_width, y=y / self.image_height, z=0.0)
        future = self.set_target_color_client.call_async(request)
        future.add_done_callback(self.set_target_color_done_callback)

    def set_target_color_done_callback(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info('calibrated: %s' % response.message)
        else:
            self.get_logger().warn('calibration failed: %s' % response.message)

    def spin_with_display(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_frame is not None:
                bgr_frame = cv2.cvtColor(self.latest_frame, cv2.COLOR_RGB2BGR)
                cv2.imshow('line_color_calibration', bgr_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


def main():
    rclpy.init()
    node = LineColorCalibrationNode()
    try:
        node.spin_with_display()
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
