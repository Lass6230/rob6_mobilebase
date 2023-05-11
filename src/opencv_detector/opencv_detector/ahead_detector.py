
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int8

# from sensor_msgs.msg import Laserscan
from sensor_msgs.msg import LaserScan
class AheadDetector(Node):

    def __init__(self):
        super().__init__('ahead_detector')
        self.sub_command = self.create_subscription(
            Int8,
            '/ahead_detector',
            self.aheadDetector,
            10
        )
        self.sub_command
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Int8, '/ahead_status', 10)
        self.state = 0
        self.status = 1
        self.search_range = 560
        self.distance_threshold = 0.5
        self.angle_increment = 0.0029088

    def listener_callback(self, msg):
        #self.get_logger().info('I heard:')

        #print(msg.ranges[560])
        #print(self.state)
        if self.status == 1:
            if msg.ranges[self.search_range] > self.distance_threshold and self.state == 0:
                self.state = 1
                #print(self.state)
            if msg.ranges[self.search_range] < self.distance_threshold and self.state == 1:
                self.state = 2
                #print(self.state)
            if msg.ranges[self.search_range] > self.distance_threshold and self.state == 2:
                return_status = Int8()
                return_status.data = 1
                self.publisher_.publish(return_status)
                self.get_logger().info('go!')
                print(return_status)
                self.status = 0
                
                
    
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
