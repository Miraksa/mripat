import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pyzbar.pyzbar import decode
import numpy as np

class ComputerVision(Node):
    def __init__(self):
        super().__init__('computer_vision')
        self.processed_image_publisher_ = self.create_publisher(Image, '/vehicle/cv', 10)
        self.fpv_camera_subscriber_ = self.create_subscription(Image, '/camera', self.image_callback, 10)

        self.bridge = CvBridge()
        self.frame_width = None
        self.frame_height = None

    def image_callback(self, msg):
        try:
            pass
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            if self.frame_width is None or self.frame_height is None:
                self.frame_height, self.frame_width = frame.shape[:2]
            
            for d in decode(frame):
                frame = cv2.rectangle(frame, (d.rect.left, d.rect.top),
                                    (d.rect.left + d.rect.width, d.rect.top + d.rect.height), (255, 0, 0), 2)
                frame = cv2.polylines(frame, [np.array(d.polygon)], True, (0, 255, 0), 2)
                frame = cv2.putText(frame, d.data.decode(), (d.rect.left, d.rect.top + d.rect.height),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)

            cv2.imshow("Frame", frame)
            
        except Exception as e:
                self.get_logger().error(f'Failed to process frame: {e}')
        
def main(args=None):
    rclpy.init(args=args)
    computer_vision = ComputerVision()
    rclpy.spin(computer_vision)
    rclpy.shutdown()

if __name__ == "__main__":
    main()