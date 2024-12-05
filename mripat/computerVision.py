import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2DArray
from cv_bridge import CvBridge
import cv2
import time

class ComputerVision(Node):
    def __init__(self):
        super().__init__('computer_vision')
        self.processed_image_publisher_ = self.create_publisher(Image, '/vehicle/processed_image', 10)
        self.camera_subscriber_ = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.detection_subscriber_ = self.create_subscription(BoundingBox2DArray, '/detected_objects', self.detection_callback, 10)
        
        self.bridge = CvBridge()
        self.detections = []

        self.fps = 0
        self.frame_count = 0
        self.total_fps = 0
        self.avg_fps = 0
        self.detections = []

    def detection_callback(self, msg):
        self.detections = msg.boxes

    def image_callback(self, msg):
        
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        start = time.time()

        for bbox in self.detections:
            x = int(bbox.center.position.x - bbox.size_x / 2)
            y = int(bbox.center.position.y - bbox.size_y / 2)
            w = int(bbox.size_x)
            h = int(bbox.size_y)

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
            
        end = time.time()
        self.frame_count += 1
        self.fps = 1 / (end - start)
        self.total_fps += self.fps
        self.avg_fps = self.total_fps / self.frame_count

        self.get_logger().info(f"FPS: {int(self.fps)}")


        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.processed_image_publisher_.publish(ros_image)


def main(args=None):
    rclpy.init(args=args)
    computer_vision = ComputerVision()
    rclpy.spin(computer_vision)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
