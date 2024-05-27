import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import time


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")
        self.image_publisher = self.create_publisher(
            CompressedImage, "camera/image/compressed", 10
        )
        self.latency_publisher = self.create_publisher(Float64, "camera/latency", 10)
        self.timer = self.create_timer(0.016, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

        self.br = CvBridge()

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            rclpy.shutdown()

    def timer_callback(self):
        start_time = time.time()
        ret, frame = self.cap.read()
        if ret:
            compressed_image_msg = CompressedImage()
            compressed_image_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_image_msg.format = "jpeg"
            compressed_image_msg.data = cv2.imencode(".jpg", frame)[1].tostring()

            self.image_publisher.publish(compressed_image_msg)
            latency = time.time() - start_time
            self.latency_publisher.publish(Float64(data=latency))
        else:
            self.get_logger().warning("Failed to capture image")

    def __del__(self):
        self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
