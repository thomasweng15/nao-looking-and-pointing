import sys
import time
import numpy
import math

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

TORSO_HEAD_OFFSET = numpy.array([0.0, 0.0, 0.1264999955892563])

# compute magnitude of 3D vector
def magn(v):
    return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

def look(torsoObjectVector):
    head = ["HeadPitch", "HeadYaw"]
    fractionMaxSpeed = 0.05
    useSensorValues = False
    frame = motion.FRAME_TORSO
    axisMask = 7 # just control position

    postureProxy.goToPosture("StandInit", 0.5)

    # get unit vector from head to object
    headObjectVector = torsoObjectVector - TORSO_HEAD_OFFSET
    headObjectUnitVector = [x / magn(headObjectVector) for x in headObjectVector]

    # compute pitch and yaw of unit vector
    pitch = -math.asin(headObjectUnitVector[2])
    yaw = math.acos(abs(headObjectUnitVector[1]))
    if headObjectUnitVector[1] < 0:
        yaw *= -1

    # move head to look. setAngles() is a non-blocking call
    motionProxy.setAngles(head, [pitch, yaw], fractionMaxSpeed)
    time.sleep(2)

    # move head back. angleInterpolation() is a blocking call
    originalAngles = [0.0, 0.0]
    secs = [2, 2]
    isAbsolute = True
    motionProxy.angleInterpolation(head, originalAngles, secs, isAbsolute)

    postureProxy.goToPosture("StandInit", 0.5)

torsoObjectVector = [1.0, -1.0, -1.0]
look(torsoObjectVector)