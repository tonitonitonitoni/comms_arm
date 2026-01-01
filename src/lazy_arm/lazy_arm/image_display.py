import cv2
import numpy as np
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image


class WristCameraDisplay(Node):
    def __init__(self) -> None:
        super().__init__('wrist_camera_display')
        self._subscription = self.create_subscription(
            Image,
            '/wrist_camera/image_raw',
            self._image_callback,
            10,
        )

    def _image_callback(self, msg: Image) -> None:
        frame = self._to_cv_image(msg)
        if frame is None:
            return

        cv2.imshow('Wrist Camera', frame)
        # waitKey(1) keeps the window responsive without adding noticeable delay
        cv2.waitKey(1)

    def _to_cv_image(self, msg: Image):
        encoding = msg.encoding.lower()
        channels = 3 if encoding in ('bgr8', 'rgb8') else 1 if encoding in ('mono8', '8uc1') else None
        if channels is None:
            self.get_logger().warn(f'Unsupported encoding: {msg.encoding}')
            return None

        expected_size = msg.height * msg.width * channels
        if len(msg.data) < expected_size:
            self.get_logger().warn('Image data size is smaller than expected')
            return None

        frame = np.frombuffer(msg.data, dtype=np.uint8)
        frame = frame.reshape((msg.height, msg.width, channels))

        if encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        return frame


def main() -> None:
    rclpy.init()
    node = WristCameraDisplay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
