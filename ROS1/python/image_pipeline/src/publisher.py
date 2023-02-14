#!/usr/bin/env python3

import sys
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class convertCV2toROS:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=100)
        self.bridge = CvBridge()
        self.image_publisher()
    
    def image_publisher(self):
        frequency = rospy.get_param("/image_acquisition/frequency")
        rate = rospy.Rate(frequency)
        input_type = rospy.get_param("/image_acquisition/input_type")
        
        video_capture = None
        if input_type == "camera_usb":
            video_capture = cv2.VideoCapture(0)
        else:
            video_path = rospy.get_param("/image_acquisition/video/video_path_0")
            video_capture = cv2.VideoCapture(video_path)
        
        while not rospy.is_shutdown():
            ret, frame = video_capture.read()
            if frame is not None:
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(msg)
                rate.sleep()

def main(args):
    rospy.init_node('CV2_to_ROS')
    try:
        _ = convertCV2toROS()
    except rospy.ROSInterruptException:
        print("Publisher node shutting down")

if __name__ == '__main__':
    main(sys.argv)