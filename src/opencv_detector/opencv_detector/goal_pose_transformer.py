import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer, TransformStamped
from rclpy.duration import Duration
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose


class MyNode(Node):
    target_frame = 'base_link'
    

    def __init__(self):
        super().__init__('my_node')
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PoseStamped, '/goal', 10)
        self.subscription = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.subscription


    def goal_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id, rclpy.time.Time())
            ts_msg = TransformStamped()
            ts_msg.header = msg.header
            ts_msg.child_frame_id = self.target_frame
            ts_msg.transform.translation.x = transform.transform.translation.x
            ts_msg.transform.translation.y = transform.transform.translation.y
            ts_msg.transform.translation.z = transform.transform.translation.z
            ts_msg.transform.rotation.x = transform.transform.rotation.x
            ts_msg.transform.rotation.y = transform.transform.rotation.y
            ts_msg.transform.rotation.z = transform.transform.rotation.z
            ts_msg.transform.rotation.w = transform.transform.rotation.w

            transformed_pose = do_transform_pose(msg.pose, ts_msg)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = transformed_pose
            pose_stamped.pose.position.z = float(0)
            pose_stamped.pose.orientation.x = float(0)
            pose_stamped.pose.orientation.y = float(0)

            self.publisher.publish(pose_stamped)
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            print(e)

    

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
