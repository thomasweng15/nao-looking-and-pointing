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

        # Set an exit flag for the thread
        self.exit = threading.Event()

        # Start a thread for idling
        self.idleThread = threading.Thread(name='idle_thread',
                                           target=self.doIdleBehaviors)
        self.idle = threading.Event()
        self.idleThread.start()

        # Start a thread for head scanning
        self.headscanThread = threading.Thread(name='headscan_thread',
                                                target=self.doHeadScan)
        self.headscan = threading.Event()
        self.headscanThread.start()


    def speak(self, text, blocking=False):
        """
        Command the robot to read the provided text out loud.

        Arguments:
        text -- the text for the robot to read
        blocking -- should this function block (defaults to False)

        Returns: none (but causes robot action)
        """
        if blocking:
            self.ttsProxy.say(text)
        else:
            self.ttsProxy.post.say(text)

    def startIdle(self):
        self.idle.set()

    def stopIdle(self):
        self.idle.clear()

    def startHeadScan(self):
        self.headscan.set()

    def stopHeadScan(self):
        self.headscan.clear()

    def doIdleBehaviors(self):
        """
        Perform small life-like idle behaviors by shifting body and head.
        """
        #self.idle.wait() # Wait for idle flag to be set

        # Send robot to Stand posture
        self.postureProxy.goToPosture("Stand",0.5)

        while True:
            if self.exit.isSet():
                print threading.currentThread().getName(), "exiting"
                return

            # Wait for idle flag to be set
            if not self.idle.isSet():
                if self.exit.isSet():
                    print threading.currentThread().getName(), "exiting"
                    return
                if self.motionProxy.getBreathEnabled('Body'):
                    self.motionProxy.setBreathEnabled('Body',False)
                time.sleep(1.0)
                continue

            # Turn on breathing
            self.motionProxy.setBreathEnabled('Body',True)

    def doHeadScan(self):
        """ Look down and slowly scan with the head left and right. """

        joints = ["HeadYaw", "HeadPitch"]
        yaw = 0.3
        pitch = 0.25
        speed = 0.05

        side = 0
        while True:
            if self.exit.isSet():
                print threading.currentThread().getName(), "exiting"
                return

            if not self.headscan.isSet():
                if self.exit.isSet():
                    print threading.currentThread().getName(), "exiting"
                    return
                time.sleep(1.0)
                continue

            # Turn on head scan
            angles = [yaw, pitch]
            if side == 0:
                side = 1
                angles[0] = yaw * -1
            else:
                side = 0
            self.motionProxy.setAngles(joints, angles, speed)
            time.sleep(3.0)


        # Add a random blink in here

    def stand(self):
        """ Stand up. """
        self.stopIdle()
        self.postureProxy.goToPosture('Stand',0.5)

    def sitAndRelax(self):
        """ Go to sitting position and shut off motor stiffness. """
        self.stopIdle()
        self.postureProxy.goToPosture('Crouch',0.5)
        self.motionProxy.setStiffnesses('Body',0)

    def doGesture(self, gestureType, torsoObjectVector, blocking=True):
        self.gesturing = True
        if gestureType == "none":
            pass
        elif gestureType == "look":
            self.look(torsoObjectVector, blocking)
        elif gestureType == "point":
            arm = "LArm" if torsoObjectVector[1] >= 0 else "RArm"
            self.point(arm, torsoObjectVector, blocking)
        elif gestureType == "lookandpoint":
            arm = "LArm" if torsoObjectVector[1] >= 0 else "RArm"
            self.lookAndPoint(arm, torsoObjectVector, blocking)
        else:
            print "Error: gestureType must be 'none', 'look', 'point', or 'lookandpoint'"
            return
        self.gesturing = False

    def look(self, torsoObjectVector, blocking=True):
        pitch, yaw = self.getPitchAndYaw(torsoObjectVector)
        sleepTime = 1 # seconds
        self.moveHead(pitch, yaw, blocking) # Move head to look
        time.sleep(sleepTime)
        self.moveHead(0, 0, blocking) # Move head back

    def point(self, pointingArm, torsoObjectVector, blocking=True):
        shoulderOffset, initArmPosition = self.setArmVars(pointingArm)
        IKTarget = self.getIKTarget(torsoObjectVector, shoulderOffset)
        sleepTime = 2 # seconds
        self.moveArm(pointingArm, IKTarget, blocking) # Move arm to point
        time.sleep(sleepTime)
        self.moveArm(pointingArm, initArmPosition, blocking) # Move arm back

    def lookAndPoint(self, pointingArm, torsoObjectVector, blocking=True):
        pitch, yaw = self.getPitchAndYaw(torsoObjectVector)
        shoulderOffset, initArmPosition = self.setArmVars(pointingArm)
        IKTarget = self.getIKTarget(torsoObjectVector, shoulderOffset)
        sleepTime = 0 # set individual sleep times to 0

        # Move arm and head to gesture
        self.moveArm(pointingArm, IKTarget, blocking)
        self.moveHead(pitch, yaw, blocking)
        time.sleep(3)

        # Move arm and head back
        self.moveArm(pointingArm, initArmPosition, blocking)
        self.moveHead(0, 0, blocking)
        #time.sleep(3)

    def getPitchAndYaw(self, torsoObjectVector):
        # Get vector from head to object
        headObjectVector = torsoObjectVector - self.torsoHeadOffset
        normalVector = [1.0, 0.0, 0.0]

        # Get two dimensional vectors, x-z for pitch, x-y for yaw
        xzHeadObjectVector = [headObjectVector[0], headObjectVector[2]]
        xzNormalVector = [normalVector[0], normalVector[2]]
        xyHeadObjectvector = [headObjectVector[0], headObjectVector[1]]
        xyNormalVector = [normalVector[0], normalVector[1]]

        pitch = self.getAngleBetweenVectors(xzNormalVector, xzHeadObjectVector)
        yaw = self.getAngleBetweenVectors(xyNormalVector, xyHeadObjectvector)

        # specify rotation direction
        if headObjectVector[2] > normalVector[2]:
            pitch = pitch * -1 # negative is pitching towards the back
        if headObjectVector[1] < normalVector[1]:
            yaw = yaw * -1 # negative is pitching to the Nao's right

        return pitch, yaw

    def getAngleBetweenVectors(self, a, b):
        """
        Computes the angle between two vectors.
        """
        dotProduct = numpy.dot(a, b)
        magnitudeA = numpy.linalg.norm(a) # magnitude of vector
        magnitudeB = numpy.linalg.norm(b) # magnitude of normal
        return math.acos(dotProduct / (magnitudeA * magnitudeB))

    def moveHead(self, pitch, yaw, blocking=True):
        self.stopIdle()
        head = ["HeadPitch", "HeadYaw"]
        fractionMaxSpeed = 0.1
        if blocking:
            self.motionProxy.setAngles(head, [pitch, yaw], fractionMaxSpeed)
        else:
            self.motionProxy.post.setAngles(head, [pitch, yaw], fractionMaxSpeed)

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
        shoulderObjectVectorMagn = numpy.linalg.norm(shoulderObjectVector)
        ratio = self.armLength / shoulderObjectVectorMagn
        IKTarget = [x*ratio for x in shoulderObjectVector]

        # get scaled vector in torso coordinate frame
        IKTarget += shoulderOffset
        IKTarget = list(numpy.append(IKTarget, [0.0, 0.0, 0.0]))
        return IKTarget

    def moveArm(self, pointingArm, IKTarget, blocking=True):
        self.stopIdle()
        fractionMaxSpeed = 0.9
        if blocking:
            self.motionProxy.setPosition(pointingArm, self.frame, IKTarget, fractionMaxSpeed, self.axisMask)
        else:
            self.motionProxy.post.setPosition(pointingArm, self.frame, IKTarget, fractionMaxSpeed, self.axisMask)

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


if __name__ == '__main__':
    naoGestures = NaoGestures()
    
