import sys
import os
import atexit
import threading
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
        # Register shutdown function in case of exit
        atexit.register(self.robotShutdown)

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

        # Start a thread for idling
        self.idleThread = threading.Thread(name='idle_thread',
                                           target=self.doIdleBehaviors)
        self.idle = threading.Event()
        self.idleThread.start()

        # Set an exit flag for the thread
        self.exit = threading.Event()


    def speak(self, text):
        """
        Command the robot to read the provided text out loud.

        Arguments:
        text -- the text for the robot to read

        Returns: none (but causes robot action)
        """

        self.ttsProxy.say(text)

    def startIdle(self):
        self.idle.set()

    def stopIdle(self):
        self.idle.clear()


    def doIdleBehaviors(self):
        """
        Perform small life-like idle behaviors by shifting body and head.
        """
        self.idle.wait() # Wait for idle flag to be set


        # Send robot to Stand posture
        self.motionProxy.setStiffnesses("Body", 1.0)
        self.postureProxy.goToPosture("Stand",0.5)

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

        hipside = 0

        while True:
            if self.exit.isSet():
                print threading.currentThread().getName(), " exiting"
                return

            # Wait for idle flag to be set
            if not self.idle.isSet():
                if self.exit.isSet():
                    return
                time.sleep(1.0)
                continue

            print str(self.exit.isSet())
            # Pick a random distance for hip sway
            dy = random.uniform(0,dy_max)
            dz = random.uniform(0,dy) # looks weird if robot squats more than it shifts

            # Alternate sides of hip sway
            if hipside == 0:
                targetTf = almath.Transform(startTf)
                targetTf.r2_c4 += dy
                targetTf.r3_c4 -= dz
                hipside = 1
            else:
                targetTf = almath.Transform(startTf)
                targetTf.r2_c4 -= dy
                targetTf.r3_c4 -= dz
                hipside = 0

            # Hip sways from side to middle
            pathTorso = [list(targetTf.toVector()), startTf]

            # Arm sway
            if hipside == 1:
                lArmTargetTf = almath.Transform(startLArmTf)
                lArmTargetTf.r3_c4 -= dz
                rArmTargetTf = almath.Transform(startRArmTf)
                rArmTargetTf.r3_c4 -= dz
            else:
                lArmTargetTf = almath.Transform(startLArmTf)
                lArmTargetTf.r3_c4 += dz
                rArmTargetTf = almath.Transform(startRArmTf)
                rArmTargetTf.r3_c4 += dz

            pathLArm = [list(lArmTargetTf.toVector()),
                        startLArmTf]
            pathRArm = [list(rArmTargetTf.toVector()),
                        startRArmTf]

            axisMaskList = [almath.AXIS_MASK_ALL, # for "Torso"
                            almath.AXIS_MASK_VEL, # for "LArm"
                            almath.AXIS_MASK_VEL] # for "RArm"

            timescoef = 1.5 # time between each shift
            nTimeSteps = 2 # number of discrete points in the idle motion
            movementTimes = [timescoef*(i+1) for i in range(nTimeSteps)]
            timesList = [movementTimes] * 3

            pathList = [pathTorso, pathLArm, pathRArm]

            self.motionProxy.post.transformInterpolations(
                effectorList, frame, pathList, axisMaskList, timesList)

            time.sleep(timescoef+0.5) # add a half-second pause

            # if not self.gesturing:
            #     # Use all idle behaviors, including arms
            #     self.motionProxy.post.transformInterpolations(
            #         effectorList, frame, pathList, axisMaskList, timesList)
            # else:
            #     # Avoid using arms in idle behaviors
            #     self.motionProxy.post.transformInterpolations(
            #         effectorList[0], frame, pathList[0], axisMaskList[0], timesList[0])

        # Deactivate body
        self.motionProxy.wbEnable(False)
        self.postureProxy.goToPosture("Stand", 0.3)
        return

    def stand(self):
        """ Stand up. """
        self.postureProxy.goToPosture('Stand',0.5)

    def sitAndRelax(self):
        """ Go to sitting position and shut off motor stiffness. """
        self.postureProxy.goToPosture('Crouch',0.5)
        self.motionProxy.setStiffnesses('Body',0)

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

    def robotShutdown(self):
        """ Sends robot to sit position and turns off stiffness. """

        print("Sending robot to safe position for shutdown.")
        self.exit.set() # turn off idle thread
        self.motionProxy.rest()

    def testMovements(self):
        """ A test function that looks, points, then looks and points, to a hardcoded target. """

        torsoObjectVector = [1.0, -1.0, 1.0]
        self.doGesture("look", torsoObjectVector)
        self.doGesture("point", torsoObjectVector)
        self.doGesture("lookandpoint", torsoObjectVector)

class PausableThread(threading.Thread):
    """ Thread class that can be paused. """

if __name__ == '__main__':
    naoGestures = NaoGestures()
    
