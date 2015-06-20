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

print lines

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

effector = "LArm"
head = ["HeadYaw", "HeadPitch"]

fractionMaxSpeedBody = 0.5
fractionMaxSpeedHead = 0.05
useSensorValues = False
frame = motion.FRAME_TORSO
axisMask = 7 # just control position

motionProxy.wakeUp()
postureProxy.goToPosture("StandInit", 0.5)

bodyCurrentPos = motionProxy.getPosition(effector, frame, useSensorValues)

# target positions should be within acceptable range of motion ...
bodyTargetPos = [
    0.1, 
    1, 
    0.1, 
    currentPos[3], 
    currentPos[4], 
    currentPos[5]
]

bodyTargetPos2 = [
    0.6, 
    0.133, 
    0.07, 
    currentPos[3], 
    currentPos[4], 
    currentPos[5]
]

headTargetAngles = [0.4, 0.2]

motionProxy.setPositions(effector, frame, targetPos, fractionMaxSpeedBody, axisMask)
motionProxy.setAngles(head, headTargetAngles, fractionMaxSpeedHead)
time.sleep(5)
# motionProxy.setPositions(effector, frame, targetPos2, fractionOfMaxSpeed, axisMask)
# time.sleep(2)

# motionProxy.getAngles()

postureProxy.goToPosture("StandInit", 0.5)
motionProxy.rest()