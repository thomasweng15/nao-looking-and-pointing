import sys
import time
import math
import numpy

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
fractionMaxSpeed = 0.7
useSensorValues = False
frame = motion.FRAME_TORSO
axisMask = 7 # just control position

postureProxy.goToPosture("StandInit", 0.5)

# vector from torso to object
torsoObjectVector = [1.0, 1.0, 1.0]

# vector from shoulder to object
torsoShoulderOffset = numpy.array([0.0, 0.09000000357627869, 0.10599999874830246])
shoulderObjectVector = torsoObjectVector - torsoShoulderOffset

# scale vector by arm length
armLength = 0.225 # in meters, rounded down
shoulderObjectVectorMagn = math.sqrt(shoulderObjectVector[0]**2 + shoulderObjectVector[1]**2 + shoulderObjectVector[2]**2)
ratio = armLength / shoulderObjectVectorMagn
armIKTarget = [shoulderObjectVector[0]*ratio, shoulderObjectVector[1]*ratio, shoulderObjectVector[2]*ratio]

# get scaled vector in torso coordinate frame
armIKTarget += torsoShoulderOffset
IKTarget = [armIKTarget[0], armIKTarget[1], armIKTarget[2], 0, 0, 0]

# move arm to point
motionProxy.setPosition("LArm", frame, IKTarget, fractionMaxSpeed, axisMask)
time.sleep(5)

# Set arm to original posture
straightArm = [0.11841137707233429, 0.13498550653457642, -0.04563630372285843, -1.2062638998031616, 0.4280231297016144, 0.03072221577167511]

motionProxy.setPosition("LArm", frame, straightArm, fractionMaxSpeed, axisMask)
time.sleep(2)

postureProxy.goToPosture("StandInit", 0.5)

