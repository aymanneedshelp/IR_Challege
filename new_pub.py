import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class AnnotatedFramePublisher(Node):
    def __init__(self):
        super().__init__('annotated_frame_publisher')

        # Publisher to publish annotated frames
        self.publisher = self.create_publisher(Image, 'annotated_frames', 10)
        self.get_logger().info('ANN FRAMES')
        #self.label_publisher = self.create_publisher(String, 'labels', 10)
        self.trigger_publisher = self.create_publisher(String,'trigger',10) #this line here
        self.get_logger().info('TRIGGER')

        # Load YOLO model
        model_path = "yolo11n.pt"  # Update this with your model path
        self.model = YOLO(model_path)

        self.labels = []
        self.stop_sign = False
        #self.prev_detections = set()
        #self.current_detections = set()

        # Initialize the webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open the webcam.")
            rclpy.shutdown()
            return

        self.bridge = CvBridge()
        self.get_logger().info("Annotated Frame Publisher Node has been started.")

        # Timer to process frames
        self.timer = self.create_timer(0.1, self.publish_frame)  # Adjust to ~10 FPS
        #self.timer2 = self.create_timer(0.2, self.new_label)


        # Subscribe to a position source
        self.detected_objects = []
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.new = [] #maybe remove

        self.no_capture_zone_x = float('inf')
        self.no_capture_zone_y = float('inf')

        self.count_this_detection = False
        self.bounding_box_constraint = False
        self.width = 0
        self.height = 0

        self.current_label = str()
        self.previous_label = str()

    def odom_callback(self, msg):
        # Extract x, y from msg (Odometry)
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warning("Failed to capture frame from camera.")
            return

        # Resize frame for faster processing
        frame = cv2.resize(frame, (320, 240))

        # Perform YOLO inference
        results = self.model.predict(source=frame, save=False, conf=0.5, max_det=2, classes=[2, 47, 11, 15, 16, 56])

        # Reset current detections for this frame
        #self.current_detections = set()

        

        print("ROB X ",self.robot_x)
        print("ROB Y ",self.robot_y)
        print("NCZ X ",self.no_capture_zone_x)
        print("NCZ Y ",self.no_capture_zone_y)
        print('current ',self.current_label)
        print('prev ',self.previous_label)

        #distance_x = self.distance(self.robot_x, self.no_capture_zone_x)
        distance = ((self.robot_x - self.no_capture_zone_x)**2 + (self.robot_y - self.robot_y)**2)**0.5
        #distance_y = self.distance(self.robot_y, self.no_capture_zone_y)
        radius = 1.75

        # for box in results[0].boxes:

        #     self.width = box.xywh[0][2].item()
        #     self.height = box.xywh[0][3].item()
        #     #find a nice size and add the constraint 80x80
        #     print("WH ",self.width,self.height)

        #     self.bounding_box_constraint = self.width >= 30 or self.height >= 40
                
        # if self.bounding_box_constraint: #and distance_y > ; radius (distance > radius)
        #     for detection in results[0].boxes.data:
        #         label = self.model.names[int(detection[-1])]
        #         self.current_label = label
        #         if len(label) > 1 and (self.current_label != self.previous_label):
        #             self.previous_label = self.current_label
        #             trigger_msg = String()
        #             trigger_msg.data = str(label)
        #             self.trigger_publisher.publish(trigger_msg)
                    
            
        #             self.no_capture_zone_x = self.robot_x
        #             self.no_capture_zone_y = self.robot_y
        
        for box in results[0].boxes:
            self.width = box.xywh[0][2].item()
            self.height = box.xywh[0][3].item()
            
            self.bounding_box_constraint = (self.width > 45 and self.height > 45) or (self.width > 50 and self.height > 80)
            print("W,H ",self.width,self.height)
            print("Bounding Box: ",self.bounding_box_constraint)
            for detection in box.data:
                label = self.model.names[int(detection[-1])]
                self.current_label = label
                self.bounding_box_by_label(label)
                if self.bounding_box_constraint:
                    if distance < radius:
                        print("INSIDE RADIUS")
                        if (self.current_label != self.previous_label):
                            self.send_trigger()
                    else:
                        if self.current_label != self.previous_label:
                            self.send_trigger()
                            self.no_capture_zone_x = self.robot_x
                            self.no_capture_zone_y = self.robot_y

        # for detection in results[0].boxes.data:
        #     label = self.model.names[int(detection[-1])]
        #     self.current_detections.add(label)
        #     if label not in self.labels:
        #         self.labels.append(label)

        # if self.current_label == 'stop sign': #need to add way of killing everything after stop sign
        #     print("HELLO")
        #     self.cap.release()
        #     self.get_logger().info("Stop Sign Detected - Stopping.")
        #     super().destroy_node()

        
        # Annotate the frame
        annotated_frame = results[0].plot()

        # Convert annotated frame to ROS2 Image message
        msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')

        # Publish the annotated frame
        self.publisher.publish(msg)
        self.get_logger().info("Published an annotated frame.")

    def send_trigger(self):
        self.previous_label = self.current_label
        trigger_msg = String()
        trigger_msg.data = str(self.current_label)
        self.trigger_publisher.publish(trigger_msg)

        if self.current_label == 'stop sign':
            self.get_logger().info("Stop Sign Detected - Stopping.")
            self.destroy_node()
    
    def bounding_box_by_label(self,label):
        if label == 'dog':
            self.bounding_box_constraint = self.width > 45 and self.height > 35
        elif label == 'chair':
            self.bounding_box_constraint = self.width > 35 and self.height > 60
        elif label == 'cat':
            self.bounding_box_constraint = self.width > 35 and self.height > 40
        elif label == 'car':
            self.bounding_box_constraint = self.width > 60 and self.height > 30
        elif label == 'apple':
            self.bounding_box_constraint = self.width > 20 and self.height > 20
        

    def destroy_node(self):
        # Release the camera
        self.cap.release()
        super().destroy_node()


    # def new_label(self):
    #     if self.count_this_detection:
    #         msg = String()
    #         msg.data = str(self.current_label)
    #         self.label_publisher.publish(msg)
    #         self.get_logger().info("JUST COUNTED AN IMAGE")
    #         self.count_this_detection = False

        # if self.new != self.detected_objects:
        #     self.new = self.detected_objects
        #     msg = String()
        #     msg.data = str(self.detected_objects)
        #     self.label_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AnnotatedFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
