import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
import numpy as np
import message_filters 
from std_msgs.msg import Bool, Int8

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class ImageSubscriberNode(Node):

    search = True
    

    def __init__(self):
        super().__init__('image_subscriber_node')
        
        self.bridge = CvBridge()
        self.image_sub = message_filters.Subscriber(self, Image,'/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image,'/camera/aligned_depth_to_color/image_raw')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Bool, '/search_aruco', self.start_callback, 10)
        self.subscription 
        self.status_publisher = self.create_publisher(Int8, '/status_aruco',10)
        self.found_aruco_counter = 0
        # Syncronize topics
        ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 1)
        ts.registerCallback(self.image_callback)
  
    def start_callback(self, msg):
        self.search = msg.data
        print("searching")


    def image_callback(self, rgb_image, depth_image):
        if self.search:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_image, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
            self.process_image(cv_image, cv_depth)


    def process_image(self, cv_image, cv_depth):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        parameters =  cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(cv_image)
        #print("marker id: " + str(markerIds))
        #id = float(markerIds)
        
        
        cv2.aruco.drawDetectedMarkers(cv_image, markerCorners, markerIds, (0, 255, 0))
        
        if len(markerCorners) > 0:
            # print("1x:" + str(markerCorners[0][0][0][0]) + "y: " + str(markerCorners[0][0][0][1]))
            # print("2x:" + str(markerCorners[0][0][1][0]) + "y: " + str(markerCorners[0][0][1][1]))
            # print("3x:" + str(markerCorners[0][0][2][0]) + "y: " + str(markerCorners[0][0][2][1]))
            # print("4x:" + str(markerCorners[0][0][3][0]) + "y: " + str(markerCorners[0][0][3][1]))
        

            x = int((markerCorners[0][0][0][0] + markerCorners[0][0][2][0]) /2)
            y = int((markerCorners[0][0][0][1] + markerCorners[0][0][2][1]) /2)

            cv2.circle(cv_image, (x,y), 5, (0,0,255))
            z = cv_depth[y,x]
            #### lasse stuff #####
            range2 = 0
            sum = 0
            count = 0
            for k in range(int(markerCorners[0][0][0][0]),int(markerCorners[0][0][2][0]+1)):
                for j in range(int(markerCorners[0][0][2][1]), int(markerCorners[0][0][0][1]+1)):
                    sum += cv_depth[j,k]
                    count += 1
            if count != 0:
                range2 = sum/count
            else:
                range2 = z
            # self.get_logger().info("corner 1: %i" %int(markerCorners[0][0][0][0]))
            # self.get_logger().info("corner 2: %i" %int(markerCorners[0][0][2][0]))
            # self.get_logger().info("corner 3: %i" %int(markerCorners[0][0][0][1]))
            # self.get_logger().info("corner 4: %i" %int(markerCorners[0][0][2][1]))
            # self.get_logger().info("count: %i" %count)
            # self.get_logger().info("range aruco: %f" %range2)

            # self.get_logger().info('z aruco: "%f"' % z)
            z = range2+0.02
            ######################

            center_x, center_y = cv_image.shape[1]/2, cv_image.shape[0]/2

            #calculate angle from center
            x_angle = (center_x - x) / cv_image.shape[1] * 69
            y_angle = (center_y - y) / cv_image.shape[0] * 42
            z_angle = np.sqrt(np.square((center_x - x)) + np.square((center_y - y))) / np.sqrt(np.square(cv_image.shape[1]) + np.square(cv_image.shape[0])) * 80.78

            #calculate cartesian coordinates in m
            cartesian_x = np.sin(np.deg2rad(x_angle)) * z / 1000
            cartesian_y = np.sin(np.deg2rad(y_angle)) * z / 1000
            cartesian_z = np.cos(np.deg2rad(z_angle)) * z / 1000

            self.publish_transform(cartesian_x, cartesian_y, cartesian_z, markerIds)


        # display the image with overlayed markers
        cv2.imshow("Image", cv_image)
        cv2.waitKey(1)


    def publish_transform(self, x, y, z, id):
        tf = TransformStamped()

        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'camera_link'
        tf.child_frame_id = 'aruco'
        tf.transform.translation.x = float(z) #z
        tf.transform.translation.y = float(x) +0.025
        tf.transform.translation.z = float(y) #x
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf)
        self.found_aruco_counter += 1
        if self.found_aruco_counter == 5 and int(id) != 53:
            msg = Int8()
            msg.data = int(id)
            self.status_publisher.publish(msg)
            self.found_aruco_counter = 0







def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



