#!/usr/bin/env python3
import rclpy
import cv2

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)

        camera_id  = self.get_parameter('camera_id').value
        width        = self.get_parameter('width').value
        height       = self.get_parameter('height').value
        fps          = self.get_parameter('fps').value

        self.get_logger().info(f'使用的相机设备号：{camera_id}')

        self.publisher_cam = self.create_publisher(Image, f'/camera_{camera_id}/image', 10)

        self.cam = cv2.VideoCapture(camera_id)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if not self.cam.isOpened():
            msg = f'无法打开相机设备 {camera_id}'
            self.get_logger().error(msg)
            raise RuntimeError(msg)

        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / fps, self.publish_callback)

    def publish_callback(self):
        ret, frame = self.cam.read()
        now = self.get_clock().now().to_msg()

        if ret:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            msg = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
            msg.header = Header(stamp=now, frame_id='camera_frame')
            self.publisher_cam.publish(msg)

    def destroy_node_and_release(self):
        self.cam.release()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node_and_release()
        rclpy.shutdown()
