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
    search_ball = 0

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


        self.subscription_ball = self.create_subscription(Int8, '/search_golfball', self.start_callback_ball, 10)
        self.subscription_ball
        self.status_publisher_ball = self.create_publisher(Int8, '/status_ball',10)
        self.found_ball_counter = 0
        # Syncronize topics
        ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 1)
        ts.registerCallback(self.image_callback)
  
    def start_callback(self, msg):
        self.search = msg.data
        print("searching")
    
    def start_callback_ball(self, msg):
        self.search_ball = msg.data
        print("searching")


    def image_callback(self, rgb_image, depth_image):
        if self.search:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_image, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")

            cv2.waitKey(1)
            self.process_image(cv_image, cv_depth)
        elif self.search_ball == 1 or self.search_ball == 2:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_image, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
            
            #time.sleep(1)
            #cv2.imshow("lol",cv_image)
            # cv2.waitKey(1)
            self.process_image_ball(cv_image, cv_depth)




    def process_image_ball(self, cv_image, cv_depth):

        if self.search_ball == 1:
            colors_to_detect = [
                #(Lhue, Lsat, Lval, Hhue, Hsat, Hval, color, color in BGR)
                (17, 120, 90, 31, 255, 255, "blue", (255,0,0)), #blue grupperum
                (108, 87, 132, 138, 255, 255, "red", (0, 0, 255)), # Red works well
                (81, 3, 189, 103, 255, 255, "yellow", (0, 255, 255)), # Yellow good
                (30, 69, 117, 54, 255, 255, "green", (0, 255, 0)) #green good? (32, 62, 88, 94, 186, 255, "green", (0, 255, 0))
                ]
            status, mask, contour, color, x, y, = self.detector(cv_image, colors_to_detect)
    
        elif self.search_ball == 2:
            colors_to_detect = [
                #(Lhue, Lsat, Lval, Hhue, Hsat, Hval, color, color in BGR)

                # (0, 0, 168, 156, 159, 255, "pink", (147, 112, 219)), # Pink high
                # (117, 78, 33, 156, 207, 255, "pink", (147, 112, 219)), #pink low
                # (88, 49, 20, 119, 255, 255, "orange", (0, 165, 255)) # Orange DTU light (wide)#(110, 110, 114, 119, 195, 255, "orange", (0, 165, 255)), # Orange decent
                (17, 120, 90, 31, 255, 255, "blue", (255,0,0)), #blue grupperum
                (108, 87, 132, 138, 255, 255, "red", (0, 0, 255)), # Red works well
                (81, 3, 189, 103, 255, 255, "yellow", (0, 255, 255)), # Yellow good
                (30, 69, 117, 54, 255, 255, "green", (0, 255, 0)) #green good? (32, 62, 88, 94, 186, 255, "green", (0, 255, 0))
                ]
            status, mask, contour, color, x, y = self.detector(cv_image, colors_to_detect)
        
        
        if not status:
            mask = np.zeros(cv_depth.shape, np.uint8)
            cv2.drawContours(mask, [contour], 0, 255, -1)
            
            # Calculate average value of gray image within mask
            avg = cv2.mean(cv_depth, mask=mask)[0]
            z = avg + 0.0215
            #cv2.imshow("depthmask",mask)
            #print(z)
            self.get_logger().info(f"{color} ball seen at depth: {z}")
            self.calculate_cartesian(x, y, z, cv_image)



    def calculate_cartesian(self, x, y, z, cv_image):
        center_x, center_y = cv_image.shape[1]/2, cv_image.shape[0]/2
                    
        #calculate angle from center
        x_angle = (center_x - x) / cv_image.shape[1] * 69
        y_angle = (center_y - y) / cv_image.shape[0] * 42
        z_angle = (np.sqrt(np.square((center_x - x)) + np.square((center_y - y))) / np.sqrt(np.square(cv_image.shape[1]) + np.square(cv_image.shape[0]))) *40.39#* 77#42#80.78
        #self.get_logger().info('angle x: "%f"' % x_angle)
        #self.get_logger().info('angle y: "%f"' % y_angle)
        #self.get_logger().info('angle z: "%f"' % z_angle)
        
        #calculate cartesian coordinates in m
        cartesian_x = np.sin(np.deg2rad(x_angle)) * z / 1000
        cartesian_y = np.sin(np.deg2rad(y_angle)) * z / 1000
        cartesian_z = np.cos(np.deg2rad(z_angle)) * z / 1000
        #new_z = np.cos(np.deg2rad(z_angle)) * range2 / 1000
        #self.get_logger().info('cartesian z: "%f"' % cartesian_z)
        #self.get_logger().info('new cartesian z: "%f"' % new_z)
        self.publish_transform_ball(cartesian_x, cartesian_y, cartesian_z)
    
    def publish_transform_ball(self, x, y, z):
        tf = TransformStamped()

        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'camera_link'
        tf.child_frame_id = 'ball'
        tf.transform.translation.x = float(z) # +0.02#z
        tf.transform.translation.y = float(x)+0.01
        tf.transform.translation.z = float(y) #+0.02#x
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf)
        self.found_ball_counter += 1
        if self.found_ball_counter >= 5:
            self.get_logger().info('ball found and transform sent!')
            msg = Int8()
            msg.data = 1
            self.status_publisher_ball.publish(msg)
            self.found_ball_counter = 0
    


    def detector(self, frame, colors_to_detect):
        
        

        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        # loop through each color to detect
        for color in colors_to_detect:
            #print(f"looking for {color[6]}")
            # define the range of colors in HSV color space
            lower_color = np.array([color[0], color[1], color[2]])
            upper_color = np.array([color[3], color[4], color[5]])
        
            # create a mask for the color range
            mask = cv2.inRange(hsv, lower_color, upper_color)

            # close the found
            kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask_closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
            
            # find the contours of the golf balls in the mask
            contours, hierarchy = cv2.findContours(mask_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            #cv2.imshow("bruh", frame)
            #self.publisher_.publish(self.bridge.cv2_to_imgmsg(mask_closed))
            #cv2.imshow("mask", mask)
            #averages = []
            # loop through each contour
            for contour in contours:
                # find the center and radius of the circle that encloses the contour
                (x,y), radius = cv2.minEnclosingCircle(contour)
                r = radius
                center = (int(x),int(y))
                radius = int(radius)

            
            
                if radius > 5:
                    # compute the circularity of the circle
                    mask_circle = np.zeros_like(mask_closed)
                    cv2.circle(mask_circle, center, radius, 255, -1)
                
                    area_color = cv2.countNonZero(cv2.bitwise_and(mask_closed, mask_circle))
                    
                    # compute the area of a perfect circle with the same radius
                    area_circle = np.pi * r * r

                    # compute the circularity ratio
                    circularity = area_color / area_circle

                    if circularity > 0.8:
                        #circle found!
                        cv2.circle(frame, center, radius, color[7], 2)
                        #save
                        #cv2.imshow("ball", frame)

                        return 0, mask, contour, color[6], x, y 


        return 1 ,0,0,0,0,0


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

            self.get_logger().info("range: %f" %range2)
            ######################

            center_x, center_y = cv_image.shape[1]/2, cv_image.shape[0]/2

            #calculate angle from center
            x_angle = (center_x - x) / cv_image.shape[1] * 69
            y_angle = (center_y - y) / cv_image.shape[0] * 42
            z_angle = np.sqrt(np.square((center_x - x)) + np.square((center_y - y))) / np.sqrt(np.square(cv_image.shape[1]) + np.square(cv_image.shape[0])) *40.39#* 80.78

            #calculate cartesian coordinates in m
            cartesian_x = np.sin(np.deg2rad(x_angle)) * z / 1000
            cartesian_y = np.sin(np.deg2rad(y_angle)) * z / 1000
            cartesian_z = np.cos(np.deg2rad(z_angle)) * z / 1000

            self.publish_transform(cartesian_x, cartesian_y, cartesian_z, markerIds)


        # display the image with overlayed markers
        #cv2.imshow("Image", cv_image)
        #cv2.waitKey(1)


    def publish_transform(self, x, y, z, id):
        tf = TransformStamped()

        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'camera_link'
        tf.child_frame_id = 'aruco'
        tf.transform.translation.x = float(z) #z
        tf.transform.translation.y = float(x) +0.02
        tf.transform.translation.z = float(y) #x
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0
        #self.tf_broadcaster.sendTransform(tf)
        self.found_aruco_counter += 1
        
        try: # gør så den også publisher nummer to hvis der er det
            if self.found_aruco_counter >= 10 and int(id[0]) != 53:
                msg = Int8()
                msg.data = int(id[0])
                self.tf_broadcaster.sendTransform(tf)
                self.status_publisher.publish(msg)
                self.found_aruco_counter = 0
        except:
            self.get_logger().info("exception with ids")








def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



