import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2DArray
from vision_msgs.msg import Point2D
from cv_bridge import CvBridge
import cv2
import time

class ComputerVision(Node):
    def __init__(self):
        super().__init__('computer_vision')
        self.processed_image_publisher_ = self.create_publisher(Image, '/vehicle/processed_image', 10)
        self.target_center_pos_publisher_ = self.create_publisher(Point2D, '/target_center_pos', 10)
        self.cropped_image_publisher_ = self.create_publisher(Image, '/vehicle/cropped_image', 10)
        self.camera_subscriber_ = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.detection_subscriber_ = self.create_subscription(BoundingBox2DArray, '/detected_objects', self.detection_callback, 10)
        
        self.bridge = CvBridge()
        self.detections = []

        self.fps = 0
        self.frame_count = 0
        self.total_fps = 0
        self.avg_fps = 0
        self.detections = []

        self.frame_segment_counter = 0
        self.frame_segment = [
            [[0, 0], [640, 640]],
            [[0, 80], [640, 720]],
            [[320, 0], [960, 640]],
            [[320, 80], [960, 720]],
            [[640, 0], [1280, 640]],
            [[640, 80], [1280, 720]]
        ]

    def detection_callback(self, msg):
        self.detections = msg.boxes

    def image_callback(self, msg):
        
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cropped_frame = frame.copy()

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


        # Crop the frame using the rectangle coordinates
        x1, y1 = self.frame_segment[self.frame_segment_counter][0] # top_left
        x2, y2 = self.frame_segment[self.frame_segment_counter][1] # bottom_right
        cropped_frame = cropped_frame[y1:y2, x1:x2]

        self.cropped_image_publisher_.publish(self.bridge.cv2_to_imgmsg(cropped_frame, "bgr8"))

        cv2.rectangle(frame, self.frame_segment[self.frame_segment_counter][0], self.frame_segment[self.frame_segment_counter][1], (0, 0, 255), 2)

        # cv2.imshow("Cropped Frame", cropped_frame)
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.processed_image_publisher_.publish(ros_image)

        self.frame_segment_counter += 1
        if self.frame_segment_counter >= len(self.frame_segment):
            self.frame_segment_counter = 0


def main(args=None):
    rclpy.init(args=args)
    computer_vision = ComputerVision()
    rclpy.spin(computer_vision)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
