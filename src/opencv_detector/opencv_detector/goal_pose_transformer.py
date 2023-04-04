import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer, TransformStamped
from rclpy.duration import Duration
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose


class MyNode(Node):
    #target_frame = 'base_link'
    def __init__(self):
        super().__init__('vision_goal_publisher')
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            PoseStamped, 
            '/goal', 
            10)
        
        self.subscription = self.create_subscription(
            PoseStamped, 
            '/goal_relative', 
            self.goal_callback,
            10)
        self.subscription


    def goal_callback(self, msg):
        Node.get_logger(self).info(f'Recieved relative goal, x: {msg.pose.position.x}, y: {msg.pose.position.x}, quat_z: {msg.pose.orientation.z}')
        
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time()) 
            Node.get_logger(self).info('Transform found!')
            ts_msg = TransformStamped()
            ts_msg.header = msg.header
            ts_msg.child_frame_id = 'base_link'
            ts_msg.transform = transform.transform

            transformed_pose = do_transform_pose(msg.pose, ts_msg)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = transformed_pose  
            
            self.publisher.publish(pose_stamped)
            Node.get_logger(self).info(f'Published pose in map frame! x: {pose_stamped.pose.position.x}, y: {pose_stamped.pose.position.y}, quat_z: {pose_stamped.pose.orientation.z}')
            
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            Node.get_logger(self).warn(f'{e}')

    

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
