import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
import numpy as np
#import time

class ImageSubscriberNode(Node):

    
    cv_depth = 0

    def __init__(self):
        super().__init__('image_subscriber_node')
        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.subscription = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # pass the image message to the arbitrary method
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_image(cv_image)

    def depth_callback(self, msg):
        # pass the image message to the arbitrary method
        self.cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        #print(cv_depth[100,100])
        

    def process_image(self, cv_image):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        parameters =  cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(cv_image)
        
        #corners, ids, _ = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)
        cv2.aruco.drawDetectedMarkers(cv_image, markerCorners, markerIds, (0, 255, 0))
        
        if len(markerCorners) > 0:
            # print("numa huan x:" + str(markerCorners[0][0][0][0]) + "y: " + str(markerCorners[0][0][0][1]))
            # print("2 x:" + str(markerCorners[0][0][1][0]) + "y: " + str(markerCorners[0][0][1][1]))
            # print("3x:" + str(markerCorners[0][0][2][0]) + "y: " + str(markerCorners[0][0][2][1]))
            # print("4x:" + str(markerCorners[0][0][3][0]) + "y: " + str(markerCorners[0][0][3][1]))
        

            x = int((markerCorners[0][0][0][0] + markerCorners[0][0][2][0]) /2)
            y = int((markerCorners[0][0][0][1] + markerCorners[0][0][2][1]) /2)

            cv2.circle(cv_image, (x,y), 5, (0,0,255))
            print(self.cv_depth[y,x])
        # display the image with overlayed markers
        cv2.imshow("Image", cv_image)
        cv2.waitKey(1)




        # ids = ids.flatten()

        # for (markerCorner, markerID) in zip(corners, ids):
        #     corners = markerCorner.reshape((4, 2))
        #     (topLeft, topRight, bottomRight, bottomLeft) = corners

        #     topRight = (int(topRight[0]), int(topRight[1]))
        #     bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        #     bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        #     topLeft = (int(topLeft[0]), int(topLeft[1]))

        #     cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
        #     cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
        #     cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
        #     cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

        #     cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        #     cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        #     cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

        #     cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
        #                 0.5, (0, 255, 0), 2)
        #     print("[Inference] ArUco marker ID: {}".format(markerID))



def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



