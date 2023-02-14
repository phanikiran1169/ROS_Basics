#!/usr/bin/env python3

import sys
import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist

class turtle:
    def __init__(self):
        self.server = rospy.Service('toggle_forward', Empty, self.toggleForward)
        self.pub_vel = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=100)
        
        self.forward = False
        # self.msgString = "Rotate"
        self.twist = Twist()

        self.commandVel()

    def toggleForward(self, req):
        self.forward = not self.forward
        rospy.loginfo("Now sending %s commands", "Forward" if self.forward else "Rotate")
        return EmptyResponse()

    def commandVel(self):
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():

            msg = self.twist
            msg.linear.x = 1.0 if self.forward else 0.0
            msg.angular.z = 0.0 if self.forward else 1.0
            
            self.pub_vel.publish(msg)
            rate.sleep()

def main(args):
    rospy.init_node("toggle_forward_server")
    _ = turtle()
    
if __name__ == "__main__":
    main(sys.argv)