import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

led_pin = 18  
GPIO.setup(led_pin, GPIO.OUT)

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            0
        )
        self.subscription

    def listener_callback(self, msg):
        global tanx, tany
        x1, x2, y1, y2 = map(int, msg.data.split(','))
        x = (x1 + x2) / 2
        y = (y1 + y2) / 2 
        if x=< 630 and x>= 650:
            GPIO.output(led_pin, GPIO.HIGH)
            time.sleep(0.001)
            GPIO.output(led_pin, GPIO.LOW)
        #self.get_logger().info('Received message: %s' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = MySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
