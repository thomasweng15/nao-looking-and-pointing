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

TORSO_LSHOULDER_OFFSET = numpy.array([0.0, 0.09000000357627869, 0.10599999874830246])
TORSO_RSHOULDER_OFFSET = numpy.array([0.0, -0.09000000357627869, 0.10599999874830246])
LARM_INIT_POSITION = [0.11841137707233429, 0.13498550653457642, -0.04563630372285843, -1.2062638998031616, 0.4280231297016144, 0.03072221577167511]
RARM_INIT_POSITION = [0.11877211928367615, -0.13329118490219116, -0.04420270770788193, 1.2169694900512695, 0.4153063893318176, -0.012792877852916718]
ARM_LENGTH = 0.22 # in meters, rounded down

# compute magnitude of 3D vector
def magn(v):
    return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

def point(pointingArm, torsoObjectVector):
    names = None
    shoulderOffset = None
    initArmPosition = None
    if pointingArm == "LArm":
        names = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand"]
        shoulderOffset = TORSO_LSHOULDER_OFFSET
        initArmPosition = LARM_INIT_POSITION
    elif pointingArm == "RArm":
        names = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"]
        shoulderOffset = TORSO_RSHOULDER_OFFSET
        initArmPosition = RARM_INIT_POSITION
    else:
        print "ERROR: Must provide point() with LArm or RArm" 
        return

    fractionMaxSpeed = 0.7
    useSensorValues = False
    frame = motion.FRAME_TORSO
    axisMask = 7 # just control position

    motionProxy.wakeUp()
    postureProxy.goToPosture("StandInit", 0.5)

    # vector from shoulder to object
    shoulderObjectVector = torsoObjectVector - shoulderOffset

    # scale vector by arm length
    shoulderObjectVectorMagn = magn(shoulderObjectVector)
    ratio = ARM_LENGTH / shoulderObjectVectorMagn
    IKTarget = [x*ratio for x in shoulderObjectVector]

    # get scaled vector in torso coordinate frame
    IKTarget += shoulderOffset
    IKTarget = list(numpy.append(IKTarget, [0.0, 0.0, 0.0]))

    # move arm to point. setPosition() is a non-blocking call
    print "Moving arm to point..."
    motionProxy.setPosition(pointingArm, frame, IKTarget, fractionMaxSpeed, axisMask)
    time.sleep(4)

    # Set arm to original posture. positionInterpolations() is a blocking call
    print "Moving arm back..."
    motionProxy.positionInterpolations(pointingArm, frame, [initArmPosition], [axisMask], [2])
    postureProxy.goToPosture("StandInit", 0.5)

torsoObjectVector = [1.0, 1.0, 1.0]
point("LArm", torsoObjectVector)