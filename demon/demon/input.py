'''
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import sys
sys.path.append('/root/cs50/')
from indoor import indoor


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print("hehfjd")
        msg.data = indoor(msg.data)
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
'''

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import sys
sys.path.append('/root/cs50/')
from indoor import indoor


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        # Explicitly state that we're intentionally not using the subscription variable
        # This is to prevent unused variable warnings
        self.subscription # This line is intentionally left blank

    def listener_callback(self, msg):
        # Assuming indoor function processes the message data
        processed_data = indoor(msg.data)
        # Assign the processed data back to msg.data
        msg.data = processed_data
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

