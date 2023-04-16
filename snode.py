import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
from std_msgs.msg import String,  Int64MultiArray

output_pin = 18
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
             String,
            'topic',
            self.listener_callback,
             0)
        self.subscription  # prevent unused variable warning
    
        GPIO.setmode(GPIO.BCM)  
        GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.output(output_pin, GPIO.LOW)

    def listener_callback(self, msg):
        '''x1, x2, y1, y2 = map(int, msg.data.split(", "))
        listt = [x1, x2, y1, y2]
        x = (x1 + x2) / 2
        y = (y1 + y2) / 2
        print(x, y)'''
        print(msg.data)
        GPIO.output(output_pin, GPIO.HIGH)
        time.sleep(0.050)
        GPIO.output(output_pin, GPIO.LOW)
       # self.get_logger().info('I heard: "%s"' % listt)


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
