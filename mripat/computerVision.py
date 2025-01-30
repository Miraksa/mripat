import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2DArray, Pose2D
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import time

class ComputerVision(Node):
    def __init__(self):
        super().__init__('computer_vision')
        self.processed_image_publisher_ = self.create_publisher(Image, '/vehicle/processed_image', 10)
        self.camera_subscriber_ = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.detection_subscriber_ = self.create_subscription(BoundingBox2DArray, '/detection/objects', self.detection_callback, 10)
        self.detection_fps_subscriber_ = self.create_subscription(Float32, '/detection/fps', self.detection_fps_callback, 10)
        self.target_frame_location_publisher_ = self.create_publisher(Pose2D, '/target_frame_location', 10)

        self.bridge = CvBridge()
        self.detection_fps = 0

        self.fps = 0
        self.frame_count = 0
        self.total_fps = 0
        self.avg_fps = 0
        self.detections = []

        self.frame_width = None
        self.frame_height = None
        self.targetHorizontal = None
        self.targetVertical = None

    def detection_callback(self, msg):
        self.detections = msg.boxes

    def detection_fps_callback(self, msg):
        self.detection_fps = msg

    def image_callback(self, msg):
        
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        if self.frame_width is None or self.frame_height is None:
            self.frame_height, self.frame_width = frame.shape[:2]
            self.targetHorizontal = self.frame_width*0.05
            self.targetVertical = self.frame_height*0.05


        start = time.time()

        if self.detections is None or len(self.detections) == 0:            
            empty_pose = Pose2D()
            empty_pose.theta = 0.0
            empty_pose.position.x = 0.0
            empty_pose.position.y = 0.0
            self.target_frame_location_publisher_.publish(empty_pose)

        else:
            for bbox in self.detections:
                x = int(bbox.center.position.x - bbox.size_x / 2)
                y = int(bbox.center.position.y - bbox.size_y / 2)
                w = int(bbox.size_x)
                h = int(bbox.size_y)

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                self.target_frame_location_publisher_.publish(bbox.center)

        end = time.time()
        self.frame_count += 1
        self.fps = 1 / (end - start)
        self.total_fps += self.fps
        self.avg_fps = self.total_fps / self.frame_count

        self.get_logger().info(f"FPS: {int(self.fps)}")
        cv2.rectangle(frame, (int(self.frame_width * 0.25), int(self.frame_height * 0.1)), 
                            (int(self.frame_width * 0.75), int(self.frame_height * 0.9)), 
                            (0, 0, 255), 3)

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
