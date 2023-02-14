#! /usr/bin/env python3 
# Indicate that python3 interpretor is used

import rospy # Python client lib for ROS

rospy.init_node("hello")           # Initiate a node called hello
rate = rospy.Rate(1)               # We create a Rate object to control the execution speed of while loop to 1Hz
while not rospy.is_shutdown():     # Continous loop
   print("My first ROS package Hello world ")
   rate.sleep()                    # We sleep the needed time to maintain the above Rate
