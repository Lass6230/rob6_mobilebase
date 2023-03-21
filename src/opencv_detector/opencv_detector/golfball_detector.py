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
        self.subscription = self.create_subscription(Bool, '/search_golfball', self.start_callback, 10)
        self.subscription
        self.status_publisher = self.create_publisher(Int8, '/status_golfball',10)
        self.found_ball_counter = 0
   
        # Syncronize topics
        ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 1)
        ts.registerCallback(self.image_callback)
  
        
    def start_callback(self, msg):
        self.search = msg.data
        print("searching")


    def image_callback(self, rgb_image, depth_image):
        if self.search:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_image, desired_encoding='passthrough')
            cv_depth = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
            self.process_image(cv_image, cv_depth)

    
    def post_process(self, image, color):
        kernel = np.ones((9, 9), np.uint8)
        image = cv2.erode(image, kernel)
        image = cv2.dilate(image, kernel)
        blur = cv2.medianBlur(image, 9)
        #text = "post process: " + str(color)
        #cv2.imshow(text, blur)
        return blur
       


    def process_image(self, cv_image, cv_depth):
        # # Blue
        frame_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #frame_threshold_blue = cv2.inRange(frame_HSV, (14, 148, 122), (23, 255, 255))
        frame_threshold_blue = cv2.inRange(frame_HSV, (14, 97, 77), (24, 255, 255))
        #cv2.imshow("blue", frame_threshold_blue)
         
        # Green
        frame_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        frame_threshold_green = cv2.inRange(frame_HSV, (35, 72, 89), (48, 255, 255))
        #cv2.imshow("green", frame_threshold_green)

        # # Yellow
        frame_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        frame_threshold_yellow = cv2.inRange(frame_HSV, (54, 41, 149), (94, 220, 255))
        #cv2.imshow("yellow", frame_threshold_yellow)

        frame_all = frame_threshold_green + frame_threshold_yellow + frame_threshold_blue

        blur = self.post_process(frame_all, "green")
        
        circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
                               param1=1000, param2=15, minRadius=10, maxRadius=80)
    
        #print(circles)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                x, y = center
                z = cv_depth[y,x]

                if z < 2000:
                    # circle center
                    cv2.circle(cv_image, center, 1, (0, 100, 100), 3)
                    # circle outline
                    radius = i[2]
                    cv2.circle(cv_image, center, radius, (255, 0, 255), 3)

                    self.calculate_cartesian(x, y, z, cv_image)

                else:
                    print("ball rejected: >2m")
                    # circle center
                    cv2.circle(cv_image, center, 1, (0, 255, 255), 3)
                    # circle outline
                    radius = i[2]
                    cv2.circle(cv_image, center, radius, (0, 0, 255), 3)


        cv2.imshow("detected circles", cv_image)
        cv2.waitKey(1)


    def calculate_cartesian(self, x, y, z, cv_image):
        center_x, center_y = cv_image.shape[1]/2, cv_image.shape[0]/2
                    
        #calculate angle from center
        x_angle = (center_x - x) / cv_image.shape[1] * 69
        y_angle = (center_y - y) / cv_image.shape[0] * 42
        z_angle = np.sqrt(np.square((center_x - x)) + np.square((center_y - y))) / np.sqrt(np.square(cv_image.shape[1]) + np.square(cv_image.shape[0])) * 80.78

        #calculate cartesian coordinates in m
        cartesian_x = np.sin(np.deg2rad(x_angle)) * z / 1000
        cartesian_y = np.sin(np.deg2rad(y_angle)) * z / 1000
        cartesian_z = np.cos(np.deg2rad(z_angle)) * z / 1000
        self.publish_transform(cartesian_x, cartesian_y, cartesian_z)


    def publish_transform(self, x, y, z):
        tf = TransformStamped()

        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'camera_link'
        tf.child_frame_id = 'ball'
        tf.transform.translation.x = float(z) #z
        tf.transform.translation.y = float(x) 
        tf.transform.translation.z = float(y) #x
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf)
        self.found_ball_counter += 1
        if self.found_ball_counter == 5:
            msg = Int8()
            msg.data = 1
            self.status_publisher(msg)
            self.found_ball_counter = 0
            
        



def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



