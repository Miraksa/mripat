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

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.timer = self.create_timer(0.0000001, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.camera_publisher_.publish(ros_image)
            
            # cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)
        else:
            self.get_logger().warn("Failed to capture frame from camera.")

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    try:
        rclpy.spin(camera)
    except KeyboardInterrupt:
        pass
    finally:
        camera.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
