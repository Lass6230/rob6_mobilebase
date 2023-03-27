import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
import numpy as np
import message_filters 
from std_msgs.msg import Bool, Int8
import imutils

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
        self.status_publisher = self.create_publisher(Int8, '/status_ball',10)
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
        cv_image = cv2.GaussianBlur(cv_image, (11, 11), 0)
        # # Blue
        frame_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #frame_threshold_blue = cv2.inRange(frame_HSV, (14, 148, 122), (23, 255, 255))
        frame_threshold_blue = cv2.inRange(frame_HSV, (17, 156, 54), (255, 255, 255))#(14, 97, 77), (24, 255, 255)
        #cv2.imshow("blue", frame_threshold_blue)
         
        # Green
        frame_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        frame_threshold_green = cv2.inRange(frame_HSV, (0, 78, 0), (80, 255, 255)) #(35, 72, 89), (48, 255, 255)
        #cv2.imshow("green", frame_threshold_green)

        # # Yellow
        frame_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        frame_threshold_yellow = cv2.inRange(frame_HSV, (54, 41, 149), (94, 220, 255))
        #cv2.imshow("yellow", frame_threshold_yellow)

        frame_all = frame_threshold_green + frame_threshold_yellow + frame_threshold_blue

        blur = self.post_process(frame_all, "green")

        cnts = cv2.findContours(blur.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            width, height = cv_image.shape[:2]
            if radius > 10:
                cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 255), 5)
                #cv2.imwrite("circled_frame.png", cv2.resize(cv_image, (int(height / 2), int(width / 2))))
                cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
                radius_i = int(radius)
                x, y = center
                count = 0
                sum = 0
                range2 = 0
                # for k in blur[0,:]:
                #     for j in blur[:,k]:
                #         count +=1

                if radius_i + y < 480 and radius_i + x < 640:

                    for k in range(radius_i):
                        for j in range(radius_i):
                            count +=1(35, 72, 89), (48, 255, 255)
                            sum += cv_depth[y+j,x+k]
                    for k in range(-radius_i):
                        for j in range(-radius_i):
                            count +=1
                            sum += cv_depth[y+j,x+k]
                    
                    range2 = sum/count

                self.get_logger().info("range: %f" %range2)

                z = range2+0.05

                if z < 2000:
                    # circle center
                    #cv2.circle(cv_image, center, 1, (0, 100, 100), 3)
                    # circle outline
                    #radius = i[2]
                    #cv2.circle(cv_image, center, radius, (255, 0, 255), 3)

                    self.calculate_cartesian(x, y, z, cv_image,range2)

                else:
                    print("ball rejected: >2m")
                    # circle center
                    #cv2.circle(cv_image, center, 1, (0, 255, 255), 3)
                    # circle outline
                    #radius = i[2]
                    #cv2.circle(cv_image, center, radius, (0, 0, 255), 3)
        
        #circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
        #                       param1=1000, param2=15, minRadius=10, maxRadius=80)
    
        #print(circles)
        # if circles is not None:
        #     circles = np.uint16(np.around(circles))
        #     for i in circles[0, :]:
        #         center = (i[0], i[1])
        #         x, y = center
        #         z = cv_depth[y,x]
        #         radius_i = int(i[2]/2)
        #         #self.get_logger().info('z: "%f"' % z)
        #         #self.get_logger().info("pixel x: %i" %x)
        #         #self.get_logger().info("pixel y: %i" %y)
        #         #self.get_logger().info("radius: %i" %radius_i)
        #         count = 0
        #         sum = 0
        #         range2 = 0
        #         # for k in blur[0,:]:
        #         #     for j in blur[:,k]:
        #         #         count +=1

        #         if radius_i + y < 640 or radius_i + x < 640:

        #             for k in range(radius_i):
        #                 for j in range(radius_i):
        #                     count +=1
        #                     sum += cv_depth[y+j,x+k]
        #             for k in range(-radius_i):
        #                 for j in range(-radius_i):
        #                     count +=1
        #                     sum += cv_depth[y+j,x+k]
                
                
                
                # #self.get_logger().info("count: %i" %count)
                # #self.get_logger().info("sum: %f" %sum)
                # range2 = sum/count

                # self.get_logger().info("range: %f" %range2)

                # z = range2+0.05
                # # radius_i =i[2]
                # # sum = 0
                # # count = 0
                # # for k in range(radius_i):
                # #     for j in range(radius_i):
                # #         sum += cv_depth[y+k,x+j]
                # #         count += 1
                # # for k in range(-radius_i):
                # #     for j in range(-radius_i):
                # #         sum += cv_depth[y+k,x+j]
                # #         count += 1
                # # print(str(sum/count))

                # if z < 2000:
                #     # circle center
                #     cv2.circle(cv_image, center, 1, (0, 100, 100), 3)
                #     # circle outline
                #     radius = i[2]
                #     cv2.circle(cv_image, center, radius, (255, 0, 255), 3)

                #     self.calculate_cartesian(x, y, z, cv_image,range2)

                # else:
                #     print("ball rejected: >2m")
                #     # circle center
                #     cv2.circle(cv_image, center, 1, (0, 255, 255), 3)
                #     # circle outline
                #     radius = i[2]
                #     cv2.circle(cv_image, center, radius, (0, 0, 255), 3)


        cv2.imshow("detected circles", cv_image)
        cv2.waitKey(1)


    def calculate_cartesian(self, x, y, z, cv_image, range2):
        center_x, center_y = cv_image.shape[1]/2, cv_image.shape[0]/2
                    
        #calculate angle from center
        x_angle = (center_x - x) / cv_image.shape[1] * 69
        y_angle = (center_y - y) / cv_image.shape[0] * 42
        z_angle = (np.sqrt(np.square((center_x - x)) + np.square((center_y - y))) / np.sqrt(np.square(cv_image.shape[1]) + np.square(cv_image.shape[0]))) *40.39#* 77#42#80.78
        self.get_logger().info('angle x: "%f"' % x_angle)
        self.get_logger().info('angle y: "%f"' % y_angle)
        self.get_logger().info('angle z: "%f"' % z_angle)
        #calculate cartesian coordinates in m
        cartesian_x = np.sin(np.deg2rad(x_angle)) * z / 1000
        cartesian_y = np.sin(np.deg2rad(y_angle)) * z / 1000
        cartesian_z = np.cos(np.deg2rad(z_angle)) * z / 1000
        #new_z = np.cos(np.deg2rad(z_angle)) * range2 / 1000
        #self.get_logger().info('cartesian z: "%f"' % cartesian_z)
        #self.get_logger().info('new cartesian z: "%f"' % new_z)
        self.publish_transform(cartesian_x, cartesian_y, cartesian_z)


    def publish_transform(self, x, y, z):
        tf = TransformStamped()

        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'camera_link'
        tf.child_frame_id = 'ball'
        tf.transform.translation.x = float(z) # +0.02#z
        tf.transform.translation.y = float(x)#+0.03
        tf.transform.translation.z = float(y) #+0.02#x
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf)
        self.found_ball_counter += 1
        if self.found_ball_counter == 5:
            msg = Int8()
            msg.data = 1
            self.status_publisher.publish(msg)
            self.found_ball_counter = 0
            
        



def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



