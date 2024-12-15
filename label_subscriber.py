import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class LabelSubscriber(Node):
    def __init__(self):
        super().__init__('label_subscriber')
        self.subscription = self.create_subscription(
            String,
            'trigger',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.old_det = ""
        
        self.counter_dict = {}

    def listener_callback(self, msg):
        # Relay the message to the log
        
        something = msg.data
        #self.get_logger().info(f"Message {something}")
        detection = something.replace("{'","").replace("'}","")
        
        self.counter(detection=detection)
        self.get_logger().info(f'{self.counter_dict}')
        
        # if detection == self.old_det:
        #     time.sleep(5)
        # else:
        #     self.old_det = detection
        #     self.counter(detection)
        #     #self.get_logger().info(f'Received: {detection}')
        #     self.get_logger().info(f'{self.counter_dict}')
            
    def counter(self,detection):
        self.counter_dict[detection] = self.counter_dict.get(detection,0) + 1
        



def main():
    rclpy.init()
    node = LabelSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == 'main':
    main()