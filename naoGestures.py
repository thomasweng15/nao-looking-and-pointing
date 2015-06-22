import sys
import time
import math
import numpy
import motion
from naoqi import ALProxy

class NaoGestures():
    def __init__(self):
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
            port = raw_input("Please write Nao's port... ")

        # Set motionProxy
        try:
            self.motionProxy = ALProxy("ALMotion", ipAdd, port)
        except Exception as e:
            print "Could not create proxy to ALMotion"
            print "Error was: ", e
            sys.exit()

        # Set postureProxy
        try:
            self.postureProxy = ALProxy("ALRobotPosture", ipAdd, port)
        except Exception, e:
            print "Could not create proxy to ALRobotPosture"
            print "Error was: ", e
            sys.exit()

        # Set constants
        self.torsoHeadOffset = numpy.array([0.0, 0.0, 0.1264999955892563])
        self.torsoLShoulderOffset = numpy.array([0.0, 0.09000000357627869, 0.10599999874830246])
        self.torsoRShoulderOffset = numpy.array([0.0, -0.09000000357627869, 0.10599999874830246])
        self.lArmInitPos = [0.11841137707233429, 0.13498550653457642, -0.04563630372285843, -1.2062638998031616, 0.4280231297016144, 0.03072221577167511]
        self.rArmInitPos = [0.11877211928367615, -0.13329118490219116, -0.04420270770788193, 1.2169694900512695, 0.4153063893318176, -0.012792877852916718]
        self.armLength = 0.22 # in meters, rounded down
        self.frame = motion.FRAME_TORSO
        self.axisMask = 7 # just control position
        self.useSensorValues = False

    def doGesture(self, gestureType, torsoObjectVector):
        self.postureProxy.goToPosture("StandInit", 0.5)
        if gestureType == "look":
            self.look(torsoObjectVector)
        elif gestureType == "point":
            arm = "LArm" if torsoObjectVector[1] >= 0 else "RArm"
            self.point(arm, torsoObjectVector)
        elif gestureType == "lookandpoint":
            arm = "LArm" if torsoObjectVector[1] >= 0 else "RArm"
            self.lookAndPoint(arm, torsoObjectVector)
        else: 
            print "Error: gestureType must be 'look', 'point', or 'lookandpoint'"
            return
        self.postureProxy.goToPosture("StandInit", 0.5)

    def look(self, torsoObjectVector):
        pitch, yaw = self.getPitchAndYaw(torsoObjectVector)
        sleepTime = 2 # seconds
        self.moveHead(pitch, yaw, sleepTime) # Move head to look
        self.moveHead(0, 0, sleepTime) # Move head back

    def point(self, pointingArm, torsoObjectVector):
        shoulderOffset, initArmPosition = self.setArmVars(pointingArm)
        IKTarget = self.getIKTarget(torsoObjectvector, shoulderOffset)
        sleepTime = 3 # seconds
        self.moveArm(pointingArm, IKTarget, sleepTime) # Move arm to point
        self.moveArm(pointingArm, initArmPosition, sleepTime) # Move arm back

    def lookAndPoint(self, pointingArm, torsoObjectVector):
        pitch, yaw = self.getPitchAndYaw(torsoObjectVector)
        shoulderOffset, initArmPosition = self.setArmVars(pointingArm)
        IKTarget = self.getIKTarget(torsoObjectVector, shoulderOffset)
        sleepTime = 0 # set individual sleep times to 0
        
        # Move arm and head to gesture
        self.moveArm(pointingArm, IKTarget, sleepTime)
        self.moveHead(pitch, yaw, sleepTime)
        time.sleep(3)
        
        # Move arm and head back
        self.moveArm(pointingArm, initArmPosition, sleepTime) 
        self.moveHead(0, 0, sleepTime) 
        time.sleep(3)

    def getPitchAndYaw(self, torsoObjectVector):
        # Get unit vector from head to object
        headObjectVector = torsoObjectVector - self.torsoHeadOffset
        headObjectUnitVector = [x / self.magn(headObjectVector) for x in headObjectVector]

        # Compute pitch and yaw of unit vector
        pitch = -math.asin(headObjectUnitVector[2])
        yaw = math.acos(abs(headObjectUnitVector[1]))
        if headObjectUnitVector[1] < 0:
            yaw *= -1
        return pitch, yaw

    def magn(self, v):
        return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

    def moveHead(self, pitch, yaw, sleepTime):
        head = ["HeadPitch", "HeadYaw"]
        fractionMaxSpeed = 0.1
        self.motionProxy.setAngles(head, [pitch, yaw], fractionMaxSpeed)
        time.sleep(sleepTime)

    def setArmVars(self, pointingArm):
        shoulderOffset = None
        initArmPosition = None
        if pointingArm == "LArm":
            shoulderOffset = self.torsoLShoulderOffset
            initArmPosition = self.lArmInitPos
        elif pointingArm == "RArm":
            shoulderOffset = self.torsoRShoulderOffset
            initArmPosition = self.rArmInitPos
        else:
            print "ERROR: Must provide point() with LArm or RArm" 
            sys.exit(1)
        return shoulderOffset, initArmPosition

    def getIKTarget(self, torsoObjectVector, shoulderOffset):
        # vector from shoulder to object
        shoulderObjectVector = torsoObjectVector - shoulderOffset

        # scale vector by arm length
        shoulderObjectVectorMagn = self.magn(shoulderObjectVector)
        ratio = self.armLength / shoulderObjectVectorMagn
        IKTarget = [x*ratio for x in shoulderObjectVector]

        # get scaled vector in torso coordinate frame
        IKTarget += shoulderOffset
        IKTarget = list(numpy.append(IKTarget, [0.0, 0.0, 0.0]))
        return IKTarget

    def moveArm(self, pointingArm, IKTarget, sleepTime):
        fractionMaxSpeed = 0.9
        self.motionProxy.setPosition(pointingArm, self.frame, IKTarget, fractionMaxSpeed, self.axisMask)
        time.sleep(sleepTime)


if __name__ == '__main__':
    torsoObjectVector = [1.0, -1.0, 1.0]
    naoGestures = NaoGestures()
    naoGestures.doGesture("look", torsoObjectVector)
    naoGestures.doGesture("point", torsoObjectVector)
    naoGestures.doGesture("lookandpoint", torsoObjectVector)