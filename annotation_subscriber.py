import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class AnnotatedFrameSubscriber(Node):
    def __init__(self):
        super().__init__('annotated_frame_subscriber')

        # Subscriber to the annotated frames topic
        self.subscription = self.create_subscription(
            Image,
            'annotated_frames',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("Annotated Frame Subscriber Node has been started.")

    def listener_callback(self, msg):
        # Convert ROS2 Image message to OpenCV frame
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Display the annotated frame
        cv2.imshow("Annotated Frames", frame)

        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Shutting down...")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = AnnotatedFrameSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()