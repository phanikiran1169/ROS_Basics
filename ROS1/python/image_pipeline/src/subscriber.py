#!/usr/bin/env python3

import sys
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


class convertROStoCV2:
    def __init__(self):
        self.image_sub = rospy.Subscriber("image_topic", Image, self.callback)
        self.bridge = CvBridge()
        
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Display", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

def main(args):
    rospy.init_node('ROS_to_CV2')
    _ = convertROStoCV2()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)