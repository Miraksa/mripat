import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ComputerVision(Node):
    def __init__(self): 
        super().__init__('computer_vision')
        self.processed_image_publisher_ = self.create_publisher(Image, '/vehicle/cv', 10)
        self.fpv_camera_subscriber_ = self.create_subscription(Image, '/camera', self.image_callback, 10)
        
        self.bridge = CvBridge()
        self.result = None
        self.frame_width = None
        self.frame_height = None

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            if self.frame_width is None or self.frame_height is None:
                # Initialize frame size and VideoWriter
                self.frame_height, self.frame_width = frame.shape[:2]
                self.result = cv2.VideoWriter(
                    '/home/ambatron/Developer/ws_fighter/src/mripat/mripat/record.mp4',
                    cv2.VideoWriter_fourcc(*'mp4v'),
                    30,  # FPS
                    (self.frame_width, self.frame_height)
                )

            if self.result is not None:
                self.result.write(frame)

            cv2.imshow('Frame', frame)
            cv2.waitKey(1)  # Necessary for the window to display properly
            
        except Exception as e:
            self.get_logger().error(f'Failed to process frame: {e}')

    def destroy_node(self):
        # Release the VideoWriter resource when the node is destroyed
        if self.result is not None:
            self.result.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    computer_vision = ComputerVision()
    try:
        rclpy.spin(computer_vision)
    except KeyboardInterrupt:
        pass
    finally:
        computer_vision.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
