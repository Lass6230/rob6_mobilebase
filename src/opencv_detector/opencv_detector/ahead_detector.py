
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32

from sensor_msgs.msg import Laserscan
class AheadDetector(Node):

    def __init__(self):
        super().__init__('ahead_detector')
        self.sub_command = self.create_subscription(
            Int32,
            'ahead_detector',
            self.aheadDetector,
            10
        )
        self.sub_command
        self.subscription = self.create_subscription(
            Laserscan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Int32, 'ahead_status', 10)

        self.status = 0
        self.search_angle = 0.0872664626 # 5 degrees
        self.distance_threshold = 1.5

    def listener_callback(self, msg):
        self.get_logger().info('I heard:')
        if self.status == 1:
            # for i in range(int(msg.angle_increment/self.search_angle))
            #     msg.ranges[i] 
            if msg.ranges[i] > self.distance_threshold or msg.ranges[i] == 0.0:
                return_status = Int32()
                return_status.data = 1
                self.publisher_.publish(return_status)
                
    
    def aheadDetector(self, msg):
        self.get_logger().info('aheadDetector callback')
        self.status = msg.data    



def main(args=None):
    rclpy.init(args=args)

    ahead_detector = AheadDetector()

    rclpy.spin(ahead_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ahead_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
