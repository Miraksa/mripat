import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time
import threading

print("Importing YOLO...")
from ultralytics import YOLO
print("Done!")

start_track = None
class ComputerVision(Node):
    def __init__(self):
        super().__init__('computer_vision')
        self.processed_image_publisher_ = self.create_publisher(Image, '/vehicle/processed_image', 10)
        self.fpv_camera_subscriber_ = self.create_subscription(Image, '/camera', self.image_callback, 10)
        
        self.model = YOLO('/home/ambatron/Developer/ws_fighter/src/mripat/model/uav_4000dts.pt')
        self.bridge = CvBridge()

        self.results = None
        self.is_model_busy = False
        self.lock = threading.Lock()

        self.fps = 0
        self.model_fps = 0
        self.frame_count = 0
        self.total_fps = 0
        self.avg_fps = 0

        self.frame_width = None
        self.frame_height = None
        self.targetHorizontal = None
        self.targetVertical = None

    def image_callback(self, msg):
        global start_track

        start = time.time()
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        if self.frame_width is None or self.frame_height is None:
            self.frame_height, self.frame_width = frame.shape[:2]
            self.targetHorizontal = self.frame_width*0.05
            self.targetVertical = self.frame_height*0.05

        if not self.is_model_busy:
            threading.Thread(target=self.run_model_inference, args=(frame,)).start()

        if self.results is not None:
            self.process_results(frame)

        end = time.time()
        self.frame_count += 1
        self.fps = 1 / (end - start)
        self.total_fps += self.fps
        self.avg_fps = self.total_fps / self.frame_count

        cv2.rectangle(frame, 
                      (int(self.frame_width * 0.25), int(self.frame_height * 0.1)), 
                      (int(self.frame_width * 0.75), int(self.frame_height * 0.9)), 
                      (0, 0, 255), 3)

        cv2.imshow("Frame", frame)

        print("FPS: " + str(int(self.fps)))
        print("Model FPS: ", str(int(self.model_fps)))
        print("FPS Average: " + str(int(self.avg_fps)))
        print("Frame Size: " + str(self.frame_height) + "x" + str(self.frame_width) + "\n\n\n")
        cv2.waitKey(1) & 0xFF

        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.processed_image_publisher_.publish(ros_image)

    def run_model_inference(self, frame):
        with self.lock:
            self.is_model_busy = True
            try:
                model_start = time.time()
                self.results = self.model.predict(source=frame, save=False, save_txt=False, conf=0.5, verbose=False)
                model_end = time.time()

                model_time = model_end - model_start
                self.model_fps = 1 / model_time

            except Exception as e:
                self.get_logger().error(f'Error during model inference: {e}')
            finally:
                self.is_model_busy = False

    def process_results(self, frame):
        global start_track

        boxes = self.results[0].boxes
        names = self.model.names
        confidance, class_ids = boxes.conf, boxes.cls.int()
        rects = boxes.xyxy.int()
        object_count = 0

        for ind in range(boxes.shape[0]):
            object_count += 1
            bounding_box = rects[ind].tolist()
            confidence = confidance[ind].item()
            label = names[class_ids[ind].item()]

            xyxy = rects[ind].tolist()
            boxWidth = abs(xyxy[0] - xyxy[2])
            boxHeight = abs(xyxy[1] - xyxy[3])
            centerX = (xyxy[0] + xyxy[2]) // 2
            centerY = (xyxy[1] + xyxy[3]) // 2

            if (boxWidth >= self.targetHorizontal and boxHeight >= self.targetVertical):
                if start_track is None:
                    start_track = time.time()

            cv2.rectangle(frame, (bounding_box[0], bounding_box[1]), (bounding_box[2], bounding_box[3]), (0, 255, 255), 2)
            cv2.putText(frame, f"{label}: {confidence:.2f}", (bounding_box[0], bounding_box[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            if not (centerX >= int(self.frame_width * 0.25) and centerX <= int(self.frame_width * 0.75) and
                    centerY >= int(self.frame_height * 0.1) and centerY <= int(self.frame_height * 0.9)):
                start_track = None

            if start_track is not None:
                elapsed_time = time.time() - start_track
                if elapsed_time >= 5:
                    cv2.putText(frame, "LOCKED", (centerX, centerY), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                else:
                    cv2.putText(frame, "{:.2f}".format(elapsed_time), (centerX, centerY), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

def main(args=None):
    rclpy.init(args=args)
    computer_vision = ComputerVision()
    rclpy.spin(computer_vision)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
