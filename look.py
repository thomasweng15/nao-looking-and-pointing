import sys
import time

import motion
import almath
from naoqi import ALProxy

# Get the Nao's IP and port
ipAdd = None
port = None
try:
    ipFile = open("ip.txt")
    lines = ipFile.read().replace("\r", "").split("\n")
    ipAdd = lines[0]
    port = int(lines[1])
except Exception as e:
    print "Could not open file ip.txt"
    ipAdd = raw_input("Please write Nao's IP address... ") 

# Set motionProxy
try:
    motionProxy = ALProxy("ALMotion", ipAdd, port)
except Exception as e:
    print "Could not create proxy to ALMotion"
    print "Error was: ", e
    sys.exit()

# Set postureProxy
try:
    postureProxy = ALProxy("ALRobotPosture", ipAdd, port)
except Exception, e:
    print "Could not create proxy to ALRobotPosture"
    print "Error was: ", e

head = ["HeadYaw", "HeadPitch"]
fractionMaxSpeed = 0.05
useSensorValues = False
frame = motion.FRAME_TORSO
axisMask = 7 # just control position

motionProxy.wakeUp()
postureProxy.goToPosture("StandInit", 0.5)

origAngles = [0.0, 0.0]
targetAngles = [0.5, 0.3] # radians

motionProxy.setAngles(head, targetAngles, fractionMaxSpeed)
time.sleep(2)

motionProxy.setAngles(head, origAngles, fractionMaxSpeed)
time.sleep(2)

motionProxy.rest()