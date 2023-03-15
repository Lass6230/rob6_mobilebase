import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
import numpy as np
import message_filters 

class ImageSubscriberNode(Node):

  
    

    def __init__(self):
        super().__init__('image_subscriber_node')
        self.bridge = CvBridge()
        self.image_sub = message_filters.Subscriber(self, Image,'/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image,'/camera/aligned_depth_to_color/image_raw')
   
        # Syncronize topics
        #ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 1, 0.1)
        ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 1)
        ts.registerCallback(self.image_callback)
  
        

    def image_callback(self, rgb_image, depth_image):
        cv_image = self.bridge.imgmsg_to_cv2(rgb_image, desired_encoding='bgr8')
        cv_depth = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
        self.process_image(cv_image, cv_depth)


    def process_image(self, cv_image, cv_depth):
        print("4")
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        parameters =  cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(cv_image)
        
        cv2.aruco.drawDetectedMarkers(cv_image, markerCorners, markerIds, (0, 255, 0))
        
        if len(markerCorners) > 0:
            # print("numa huan x:" + str(markerCorners[0][0][0][0]) + "y: " + str(markerCorners[0][0][0][1]))
            # print("2 x:" + str(markerCorners[0][0][1][0]) + "y: " + str(markerCorners[0][0][1][1]))
            # print("3x:" + str(markerCorners[0][0][2][0]) + "y: " + str(markerCorners[0][0][2][1]))
            # print("4x:" + str(markerCorners[0][0][3][0]) + "y: " + str(markerCorners[0][0][3][1]))
        

            x = int((markerCorners[0][0][0][0] + markerCorners[0][0][2][0]) /2)
            y = int((markerCorners[0][0][0][1] + markerCorners[0][0][2][1]) /2)

            cv2.circle(cv_image, (x,y), 5, (0,0,255))
            print(cv_depth[y,x])

        # display the image with overlayed markers
        cv2.imshow("Image", cv_image)
        cv2.waitKey(1)

    # Create image subscribers
    


    # sub_color_image = message_filters.Subscriber('/camera/color/image_raw/', Image)
    # sub_depth_image = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)

    # ts = message_filters.TimeSynchronizer([sub_color_image, sub_depth_image], 1000)
    # ts.registerCallback(synchronized_callback)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



