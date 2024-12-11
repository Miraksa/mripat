import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Camera(Node):
    def __init__(self):
        super().__init__('camera')
        self.camera_publisher_ = self.create_publisher(Image, '/camera', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture('/dev/video4')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def timer_callback(self):
        _, frame = self.cap.read()
        
        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.camera_publisher_.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()