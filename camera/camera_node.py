import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_node")

        self.declare_parameter('camera_id', '0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)

        device = self.get_parameter('camera_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value

        self.get_logger().info(f'使用 GStreamer 从 {device} 获取视频流')

        gst_pipeline = (
            f'v4l2src device={device} ! '
            f'image/jpeg, width={width}, height={height}, framerate={fps}/1 ! '
            f'jpegdec ! videoconvert ! appsink'
        )

        self.cam = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.cam.isOpened():
            msg = f'无法打开相机设备 {device}，请检查 GStreamer 管道是否正确'
            self.get_logger().error(msg)
            raise RuntimeError(msg)

        self.publisher_cam = self.create_publisher(Image, '/camera/image', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / fps, self.publish_callback)

        real_size = self.cam.get(cv2.CAP_PROP_FRAME_WIDTH), self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
        real_fps = self.cam.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f'相机实际分辨率：{real_size[0]}x{real_size[1]}')
        self.get_logger().info(f'相机实际帧率：{real_fps} fps')

        self.output_width = 224
        self.output_height = 224

    def resize_with_padding(self, image, target_width, target_height):
        original_height, original_width = image.shape[:2]
        ratio = min(target_width / original_width, target_height / original_height)
        new_width = int(original_width * ratio)
        new_height = int(original_height * ratio)

        resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)
        padded = np.zeros((target_height, target_width, 3), dtype=np.uint8)
        x_offset = (target_width - new_width) // 2
        y_offset = (target_height - new_height) // 2
        padded[y_offset:y_offset + new_height, x_offset:x_offset + new_width] = resized
        return padded

    def publish_callback(self):
        ret, frame = self.cam.read()
        now = self.get_clock().now().to_msg()

        if ret:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            processed = self.resize_with_padding(rgb, self.output_width, self.output_height)
            msg = self.bridge.cv2_to_imgmsg(processed, encoding='rgb8')
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
