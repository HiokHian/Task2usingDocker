#!/usr/bin/env python

"""ROS Publisher and Server service using Python.

This is an OOP implementation of a ROS publisher and server service with efforts made to align with the S.O.L.I.D principles such as Single Responsibility Principle and Object Encapsulation. These classes are meant to be imported into the main file to be used.

  Typical usage example:

  PublisherInstance1 = pypublisher()
  ServerInstance1 = pyserver()
"""

# license removed for brevity
import rospy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse


class pypublisher:
    """This is the Publisher that will publish a Bool to the topic 'toggle_led'. The Arduino
	subscriber will subscribe to this topic and listen for the Boolean values. Upon receiving
	True, the Arduino subscriber will toggle the BUILT-IN-LED on the Arduino board """
    def __init__(self):
	"""Init instance attribute toggle and the ROS publisher"""
        self.toggle = False
	# Object encapsulation: the publisher node is set to be a private member to prevent 
	# user from directly accessing it
        self.__pub = rospy.Publisher('toggle_led', Bool, queue_size=10)
	rospy.loginfo("Publisher Setup Completed")
	
    def publish(self):
	"""This publishes the value stored in self.toggle to the topic 'toggle LED' """
	pubmessage = self.toggle 
        self.__pub.publish(pubmessage)
	rospy.loginfo("Publishing {}".format(pubmessage))


class pyserver:
    """This is the server service that will receive requests from the Arduino client which will 
	broadcast Bool values to this server. Upon receiving a value from the client, that value 	 will be stored in the instance attribute 'toggle' """
    def __init__(self):
	"""Init the instance attritbute toggle and the ROS server service"""
	self.toggle = False
	# Object encapsulation: the server service is set to be a private member to prevent
	# user from directly accessing it
	self.__receive_request = rospy.Service("receive_request", SetBool, self.callback_receive_request)

    def callback_receive_request(self, req):
	"""This is the callback funciton used by the server service"""
	rospy.loginfo("Received request from client to toggle LED")
	rospy.loginfo("Request was {}".format(req.data))
        if req.data == True: # if the client sent True to the server
            self.toggle = req.data #set the instance attribute 'toggle' to True
	    rospy.loginfo("instance's toggle attribute has been set to {}".format(self.toggle))
	response = SetBoolResponse() 
        return response



