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


class ImageSquareDetectNode(Node):
    def __init__(self):
        super().__init__('image_square_detect_node')
        self.bridge = CvBridge()
        self.image_sub = message_filters.Subscriber(self, Image,'/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image,'/camera/aligned_depth_to_color/image_raw')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Bool, '/search_golfhole', self.start_callback, 10)
        self.subscription
        self.status_publisher = self.create_publisher(Int8, '/status_golfhole',10)
        self.found_hole_counter = 0
        
        self.search = True
   
        # Syncronize topics
        ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 1)
        ts.registerCallback(self.image_callback)
    
    def start_callback(self, msg):
        self.search = msg.data
        print("searching")

    def image_callback(self, rgb_image, depth_image):
        print("got images")
        if self.search:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_image, desired_encoding='passthrough')
            cv_depth = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
            self.process_image(cv_image, cv_depth)
    
    def process_image(self, cv_image, cv_depth):
        gau = cv2.GaussianBlur(cv_image, (11, 11), 0)
        cv2.imshow("after blur",cv_image)
        frame_HSV = cv2.cvtColor(gau, cv2.COLOR_BGR2HSV)

        frame_threshold_blue = cv2.inRange(frame_HSV, (17, 156, 54), (255, 255, 255))

        blur = self.post_process(frame_threshold_blue)

        contours, _ = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #cnts = imutils.grab_contours(cnts)
        for cnt in contours:
            x1,y1 = cnt[0][0]
            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            #approx = cv2.approxPolyDP(cnt)
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(cnt)
                ratio= float(w)/h
                if ratio>=0.9 and ratio<=1.1:
                    #cv2.putText('Square')
                    cv2.putText(cv_image, 'Square', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    #cv2.drawContours(cv_image, [cnt], -1, (0,255,255), 3)
                else:
                    #cv2.putText('Rectangle')
                    cv2.putText(cv_image, 'Rectangle', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    #cv2.drawContours(cv_image, [cnt], -1, (0,255,0), 3)
        
        #cv2.imshow("image or",cv_image)
        #cv2.imshow("blur",blur)
        #cv2.imshow("Shapes", cv_image)



    def post_process(self, image):
        kernel = np.ones((9, 9), np.uint8)
        image = cv2.erode(image, kernel)
        image = cv2.dilate(image, kernel)
        blur = cv2.medianBlur(image, 9)
        #text = "post process: " + str(color)
        #cv2.imshow(text, blur)
        return blur






def main(args=None):
    rclpy.init(args=args)
    node = ImageSquareDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    