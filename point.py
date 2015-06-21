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

names = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand"]
fractionMaxSpeed = 0.2
useSensorValues = False
frame = motion.FRAME_TORSO
axisMask = 7 # just control position

postureProxy.goToPosture("StandInit", 0.5)

straightArmAngles = [1.3, 0.3, -2, 0, 0.5, 0] # point straight outward

# Set arm straight
motionProxy.setAngles(names, straightArmAngles, fractionMaxSpeed)
time.sleep(2)

print motionProxy.getPosition("Head", frame, useSensorValues)

# Point to object
# 

postureProxy.goToPosture("StandInit", 0.5)

