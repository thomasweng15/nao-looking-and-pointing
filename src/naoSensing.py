#!/usr/bin/env python

import rospy
from naoqi import ALProxy, ALBroker, ALModule
from nao_looking_and_pointing.msg import Touch
from optparse import OptionParser
import sys

# Global variable to store the module instance
SensingModule = None
memory = None

class NaoSensing(ALModule):
	"""
	Sense head tactile touches and bumper touches.
	"""
	def __init__(self, name):
		ALModule.__init__(self, name)

		self.moduleName = name

		# Initialize ROS node
		rospy.init_node("nao_sensing")

		# Create publisher for touch messages
		self.touchpub = rospy.Publisher("touch", Touch, queue_size = 10)

		# Possible touch events to listen for
		self.headtouches = ["FrontTactilTouched",
					   "MiddleTactilTouched",
					   "RearTactilTouched"]
		self.bumpertouches = ["RightBumperPressed","LeftBumperPressed"]
		
		# Subscribe to touch events
		global memory
		memory = ALProxy("ALMemory")
		for event in self.headtouches:
			memory.subscribeToEvent(event, self.moduleName, "onTouch")
		for event in self.bumpertouches:
			memory.subscribeToEvent(event, self.moduleName, "onTouch")

	def onTouch(self, eventName, eventValue, subscriberIdentifier):
		""" Called each time a touch is detected."""

		# Create a Touch message
		touchmessage = Touch()
		if eventName in self.headtouches:
			touchmessage.touch_type = "HeadTactile"
		elif eventName in self.bumpertouches:
			touchmessage.touch_type = "Bumper"

		self.touchpub.publish(touchmessage)

def main():
	""" Connect to broker and start sensing module. """

	# Get the Nao's IP and port from file if possible
	pip = None
	pport = None
	try:
		ipFile = open("ip.txt")
		lines = ipFile.read().replace("\r", "").split("\n")
		pip = lines[0] # The IP address of the robot
		pport = int(lines[1]) # The port NAOqi is listening to
	except Exception as e:
		print "Could not open file ip.txt"
		pip = raw_input("Please write Nao's IP address... ")
		pport = raw_input("Please write Nao's port... ")

	# We need this broker to be able to construct
	# NAOqi modules and subscribe to other modules.
	# The broker must stay alive until the program exits.
	# (per doc.aldebaran.com/1-14/dev/python/reacting_to_events.html)
	broker = ALBroker("sensing_broker",
		"0.0.0.0", 	# listen to anyone
		0,			# find a free port and use it
		pip,		# parent broker IP
		pport)		# parent broker port

	global SensingModule
	SensingModule = NaoSensing("SensingModule")

	rospy.spin()

	broker.shutdown()
	sys.exit(0)

if __name__ == '__main__':
	main()