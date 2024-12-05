import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from vision_msgs.msg import BoundingBox2DArray
from vision_msgs.msg import Pose2D
from vision_msgs.msg import Point2D
from cv_bridge import CvBridge
import time
from ultralytics import YOLO

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')
        self.publisher_ = self.create_publisher(BoundingBox2DArray, '/detected_objects', 10)
        self.subscriber_ = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.bridge = CvBridge()
        self.model = YOLO('/home/ambatron/Developer/ws_fighter/src/mripat/model/uav_4000dts.pt')

        self.fps = 0
        self.frame_count = 0
        self.total_fps = 0
        self.avg_fps = 0

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        try:
            start = time.time()

            results = self.model.predict(source=frame, save=False, save_txt=False, conf=0.5, verbose=False)
            boxes = results[0].boxes
            confidences = boxes.conf
            class_ids = boxes.cls.int()
            rects = boxes.xyxy.int()

            bounding_boxes = BoundingBox2DArray()

            for i in range(boxes.shape[0]):
                bbox = BoundingBox2D()
                pose = Pose2D()
                position = Point2D()

                position.x = float((rects[i][0] + rects[i][2]) / 2.0 ) # Center X
                position.y = float((rects[i][1] + rects[i][3]) / 2.0 )# Center Y

                pose.position = position

                bbox.center = pose
                bbox.size_x = float(rects[i][2] - rects[i][0]) # Width
                bbox.size_y = float(rects[i][3] - rects[i][1])  # Height

                # Append to array
                bounding_boxes.boxes.append(bbox)

            self.publisher_.publish(bounding_boxes)
            end = time.time()
            self.frame_count += 1
            self.fps = 1 / (end - start)
            self.total_fps += self.fps
            self.avg_fps = self.total_fps / self.frame_count

            self.get_logger().info(f"FPS: {int(self.fps)}")

        except Exception as e:
            self.get_logger().error(f'Error during object detection: {e}')


def main(args=None):
    rclpy.init(args=args)
    object_detection = ObjectDetection()
    rclpy.spin(object_detection)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
