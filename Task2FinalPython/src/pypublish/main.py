#!/usr/bin/env python

"""This is the main file that is used to run the ROS Publisher and Server service.

The ROS publisher will only publish a value when the server service receives a value of True from the Arduino client service. Once a value of True is received, the ROS publisher will publish True to the topic 'toggle_led'
"""


from pypublishernode import pypublisher
from pypublishernode import pyserver
import rospy


# Initialise the ROS node 
rospy.init_node('pypublisher')
rate = rospy.Rate(10) # set rate to be 10 Hz ie 10 cycles per second
# Instantiate the Publisher Instance and the Server Instance
PublisherInstance1 = pypublisher()
ServerInstance1 = pyserver()

# While the node is still running, listen for requests from the Arduino client
while not rospy.is_shutdown():
    if ServerInstance1.toggle:
	PublisherInstance1.toggle = True
	PublisherInstance1.publish()
	ServerInstance1.toggle = False
    rate.sleep()
