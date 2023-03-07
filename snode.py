import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math

aperturex = 47.936
aperturey = 26.964

tanx = math.tan(math.radians(aperturex))
tany = math.tan(math.radians(aperturey))

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        global tanx, tany
        x1, x2, y1, y2 = map(int, msg.data.split(','))
        x = (x1 + x2) / 2
        y = (y1 + y2) / 2 
        anglex = math.degrees(math.arctan(tanx*(x/1280)))
        angley = math.degrees(math.arctan(tany*(y/720)))
        print(anglex, angley)
        #self.get_logger().info('Received message: %s' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = MySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
