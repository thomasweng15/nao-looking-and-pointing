import sys
import os
import time
import math
import random
import numpy
import motion
import almath
from naoqi import ALProxy

class NaoGestures():
    def __init__(self, robotIp=None, robotPort=None):
        """
        Initialize a Nao connection.

        Attempts to connect to the Nao at the IP address and port provided.
        If none are provided, asks user to input IP and port.

        Arguments:
        robotIp -- The IP of the robot, defaults to None
        robotPort -- The port for the robot, defaults to None
        """
        # Get the Nao's IP and port
        ipAdd = robotIp
        port = robotPort
        if not ipAdd:
            ipAdd = raw_input("Please enter Nao's IP address: ")
        if not port:
            port = int(raw_input("please enter Nao's port: "))
        
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

        # Set ttsProxy
        try:
            self.ttsProxy = ALProxy("ALTextToSpeech", ipAdd, port)
        except Exception, e:
            print "Could not create proxy to ALTextToSpeech"
            print "Error was: ", e
            sys.exit()

        # Set constants
        self.torsoHeadOffset = numpy.array([0.0, 
                                            0.0, 
                                            #0.1264999955892563])
                                            0.16])
        self.torsoLShoulderOffset = numpy.array([0.0, 
                                                 0.09000000357627869, 
                                                 0.10599999874830246])
        self.torsoRShoulderOffset = numpy.array([0.0, 
                                                -0.09000000357627869, 
                                                 0.10599999874830246])
        self.lArmInitPos = [0.11841137707233429, 
                            0.13498550653457642, 
                           -0.04563630372285843, 
                           -1.2062638998031616, 
                            0.4280231297016144, 
                            0.03072221577167511]
        self.rArmInitPos = [0.11877211928367615, 
                           -0.13329118490219116, 
                           -0.04420270770788193, 
                            1.2169694900512695, 
                            0.4153063893318176, 
                           -0.012792877852916718]
        self.armLength = 0.22 # in meters, rounded down
        self.frame = motion.FRAME_TORSO
        self.axisMask = 7 # just control position
        self.useSensorValues = False
        self.gesturing = False # is the robot executing the doGesture function?

    def speak(self, text):
        """
        Command the robot to read the provided text out loud.

        Arguments:
        text -- the text for the robot to read

        Returns: none (but causes robot action)
        """

        self.ttsProxy.say(text)

    def doIdleBehaviors(self):
        """
        Perform small life-like idle behaviors by shifting body and head.
        """
        doIdle = True

        self.motionProxy.setStiffnesses("Body", 1.0)

        # Send robot to Stand posture
        self.motionProxy.wakeUp()

        # Send robot to Stand Init posture
        self.postureProxy.goToPosture("Stand", 0.5)

        # Enable whole body balancer
        self.motionProxy.wbEnable(True)

        # Legs are constrained fixed
        self.motionProxy.wbFootState("Fixed", "Legs")

        # Constraint balance motion
        self.motionProxy.wbEnableBalanceConstraint(True, "Legs")

        useSensorValues = False
        frame = motion.FRAME_ROBOT
        effectorList = ["Torso","LArm","RArm"]

        dy_max = 0.04
        dz_max = 0.04

        startTf = self.motionProxy.getTransform("Torso", frame, useSensorValues)
        startLArmTf = self.motionProxy.getTransform("LArm", frame, useSensorValues)
        startRArmTf = self.motionProxy.getTransform("RArm", frame, useSensorValues)


        while doIdle:
            # Pick a random distance for hip sway
            dy = random.uniform(0,dy_max)
            dz = random.uniform(0,dy) # looks weird if robot squats more than it shifts

            # Alternate sides of hip sway
            target1Tf = almath.Transform(startTf)
            target1Tf.r2_c4 += dy
            target1Tf.r3_c4 -= dz

            target2Tf = almath.Transform(startTf)
            target2Tf.r2_c4 -= dy
            target2Tf.r3_c4 -= dz

            # Hip sways from side 1 to middle to side 2 to middle
            pathTorso = [list(target1Tf.toVector()),
                         startTf,
                         list(target2Tf.toVector()),
                         startTf]

            # Arm sway
            lArmTarget1Tf = almath.Transform(startLArmTf)
            lArmTarget1Tf.r3_c4 -= dz
            lArmTarget2Tf = almath.Transform(startLArmTf)
            lArmTarget2Tf.r3_c4 += dz

            rArmTarget1Tf = almath.Transform(startRArmTf)
            rArmTarget1Tf.r3_c4 -= dz
            rArmTarget2Tf = almath.Transform(startRArmTf)
            rArmTarget2Tf.r3_c4 += dz

            pathLArm = [list(lArmTarget1Tf.toVector()),
                        startLArmTf,
                        list(lArmTarget1Tf.toVector()),
                        startLArmTf]
            pathRArm = [list(rArmTarget1Tf.toVector()),
                        startRArmTf,
                        list(rArmTarget1Tf.toVector()),
                        startRArmTf]

            axisMaskList = [almath.AXIS_MASK_ALL, # for "Torso"
                            almath.AXIS_MASK_VEL, # for "LArm"
                            almath.AXIS_MASK_VEL] # for "RArm"

            timescoef = 1.5 # time between each shift
            nTimeSteps = 4 # number of discrete points in the idle motion
            movementTimes = [timescoef*(i+1) for i in range(nTimeSteps)]
            timesList = [movementTimes] * 3

            pathList = [pathTorso, pathLArm, pathRArm]

            if not self.gesturing:
                # Use all idle behaviors, including arms
                self.motionProxy.post.transformInterpolations(
                    effectorList, frame, pathList, axisMaskList, timesList)
            else:
                # Avoid using arms in idle behaviors
                self.motionProxy.post.transformInterpolations(
                    effectorList[0], frame, pathList[0], axisMaskList[0], timesList[0])

        # Deactivate body and send robot to sitting pose
        self.motionProxy.wbEnable(False)
        self.postureProxy.goToPosture("Stand", 0.3)
        self.motionProxy.rest()

    def doGesture(self, gestureType, torsoObjectVector):
        self.gesturing = True
        self.postureProxy.goToPosture("Stand", 0.5)
        if gestureType == "none":
            pass
        elif gestureType == "look":
            self.look(torsoObjectVector)
        elif gestureType == "point":
            arm = "LArm" if torsoObjectVector[1] >= 0 else "RArm"
            self.point(arm, torsoObjectVector)
        elif gestureType == "lookandpoint":
            arm = "LArm" if torsoObjectVector[1] >= 0 else "RArm"
            self.lookAndPoint(arm, torsoObjectVector)
        else:
            print "Error: gestureType must be 'none', 'look', 'point', or 'lookandpoint'"
            return
        self.postureProxy.goToPosture("Stand", 0.5)
        self.gesturing = False

    def look(self, torsoObjectVector):
        print "----calcluating look----"
        pitch, yaw = self.getPitchAndYaw(torsoObjectVector)
        sleepTime = 2 # seconds
        self.moveHead(pitch, yaw, sleepTime) # Move head to look
        self.moveHead(0, 0, sleepTime) # Move head back

    def point(self, pointingArm, torsoObjectVector):
        shoulderOffset, initArmPosition = self.setArmVars(pointingArm)
        IKTarget = self.getIKTarget(torsoObjectVector, shoulderOffset)
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

        print 'headObjectVector = ' + str(headObjectVector)
        print 'headObjectUnitVector = ' + str(headObjectUnitVector)

        # Compute pitch and yaw of unit vector
        pitch = -math.asin(headObjectUnitVector[2])
        yaw = math.acos(headObjectUnitVector[0])
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

        # Get current arm position from sensor
        initArmPosition = self.motionProxy.getPosition(
            pointingArm, self.frame, True)
 
        if pointingArm == "LArm":
            shoulderOffset = self.torsoLShoulderOffset
        elif pointingArm == "RArm":
            shoulderOffset = self.torsoRShoulderOffset
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


    def testMovements(self):
        """ A test function that looks, points, then looks and points, to a hardcoded target. """

        torsoObjectVector = [1.0, -1.0, 1.0]
        self.doGesture("look", torsoObjectVector)
        self.doGesture("point", torsoObjectVector)
        self.doGesture("lookandpoint", torsoObjectVector)

if __name__ == '__main__':
    naoGestures = NaoGestures()
    
