import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class ComputerVission(Node):
    def __init__(self): 
        super().__init__('computer_vision')
        self.processed_image_publisher_ = self.create_publisher(Image, '/vehicle/cv', 10)
        self.fpv_camera_subscriber_ = self.create_subscription(Image, '/camera', self.image_callback, 10)
        
        self.bridge = CvBridge()

        self.fps = 0
        self.frame_count = 0
        self.total_fps = 0
        self.avg_fps = 0

        self.frame_width = None
        self.frame_height = None

    def image_callback(self, msg):
        try:
            start = time.time()
            
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            if self.frame_width is None or self.frame_height is None:
                self.frame_height, self.frame_width = frame.shape[:2]

            cv2.rectangle(frame, 
                          (int(self.frame_width * 0.25), int(self.frame_height * 0.1)), 
                          (int(self.frame_width * 0.75), int(self.frame_height * 0.9)), 
                          (0, 0, 255), 3)

            end = time.time()
            self.frame_count += 1
            self.fps = 1 / (end - start)
            self.total_fps += self.fps
            self.avg_fps = self.total_fps / self.frame_count

            cv2.imshow("Frame", frame)
            print("FPS: " + str(int(self.fps)))
            print("Average FPS: " + str(int(self.avg_fps)))
            print("Frame Size: " + str(self.frame_height) + "x" + str(self.frame_width) + "\n\n\n")
            cv2.waitKey(1) & 0xFF

            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.processed_image_publisher_.publish(ros_image)

        except Exception as e:
            self.get_logger().error(f'Failed to process frame: {e}')

def main(args=None):
    rclpy.init(args=args)
    computer_vision = ComputerVission()
    rclpy.spin(computer_vision)
    rclpy.shutdown()

if __name__ == "__main__": 
    main()
