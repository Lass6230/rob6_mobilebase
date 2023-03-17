import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int8


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('vaccumControl')
        self.publisher_ = self.create_publisher(Int8, '/vaccumControl', 10)
        timer_period = 2.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int8()
        
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        if self.i == 5:
            self.i = 0


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
