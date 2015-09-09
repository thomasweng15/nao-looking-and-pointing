#!/usr/bin/env python

"""
Author: Henny Admoni 
Date Created: 8/1/2015

This module runs the HRI construction interaction. 
"""
# ROS imports
import rospy
from kinect2_pointing_recognition.msg import ObjectsInfo, GazePointInfo, MarkerInfo
from nao_looking_and_pointing.msg import ScriptObjectRef, RobotBehavior
from std_msgs.msg import String
# --------------------------------
import sys
import os
import signal
import atexit
import cv2
import re
import argparse
import subprocess
import threading
import random
import ast
import numpy as np
from time import sleep, time
from kinectObjects import KinectObjects
from lego import Bin
from naoGestures import NaoGestures
from scriptReader import ScriptReader
from nvbModel import NVBModel

dirpath = '/home/kinect/catkin/src/nao_looking_and_pointing/src/'

class InteractionController():
    """ High-level class that runs the HRI interaction. """

    def __init__(self, 
                 usernum, 
                 instructions_script_filename,
                 task1_script_filename,
                 task2_script_filename, 
                 validation_script_filename, 
                 robotIP, 
                 robotPort, 
                 cameraID=0,
                 nonverbalBehavior = True,
                 interruption = True,
                 nObjects=8):
        """
        Initialize the controller for the HRI interaction.

        Arguments:
        usernum -- ID number for participant
        instructions_script_filename -- string name of a script for the HRI instructions
        task1_script_filename -- string name of a script for the first HRI task
        task2_script_filename -- string name of a script for the second HRI task
        validation_script_filename -- string name of a script for the validation interaction
        robotIP -- Nao's IP address
        robotPort -- Nao's port
        cameraID -- int ID for user view camera, defaults to 0
        nonverbalBehavior -- bool for whether Nao should perform NVB, defaults to True
        interruption -- bool for whether an interruption will happen, defaults to True
        """

        # Handle shutdown gracefully
        #atexit.register(self.shutdown)

        rospy.init_node('interaction_controller', log_level=rospy.INFO)
        rospy.loginfo('Initializing interaction controller')

        # Subscribe to relevant topics
        self.nvb_command_listener = rospy.Subscriber('/script_object_reference', ScriptObjectRef, self.objectRefCallback)
        self.gazepoint_info_listener = rospy.Subscriber('/gazepoint_info', GazePointInfo, self.gazepointInfoCallback)
        self.script_status_listener = rospy.Subscriber('/script_status', String, self.scriptStatusCallback)
        self.marker_info_listener = rospy.Subscriber('/marker_info', MarkerInfo, self.markerInfoCallback)

        # Create relevant publishers
        self.robot_behavior_pub = rospy.Publisher('/robot_behavior', RobotBehavior)
        self.script_status_pub = rospy.Publisher('/script_status', String)

        # Start rosbags in a separate thread
        rosbagthread = threading.Thread(target=self.recordRosbags, 
                                        args=(usernum,))
        rosbagthread.start()

        # User ID
        self.userID = usernum

        # dictionary of objects, key = object ID, value = Lego object
        self.objects = KinectObjects()

        # arguments specifying the study conditions
        assert isinstance(nonverbalBehavior,bool)
        self.nvb = nonverbalBehavior
        assert isinstance(interruption,bool)
        self.interrupt = interruption

        # fiducial markers visible - updated in self.markerInfoCallback
        self.markersVisible = []

        self.numObj = nObjects

        rospy.loginfo('Creating a NaoGestures object')
        # Nao robot
        self.nao = NaoGestures(robotIP, robotPort)

        rospy.loginfo('Creating an NVBModel object')
        # NVB model
        self.model = NVBModel()

        rospy.loginfo('Creating a ScriptReader object (interruption: %s)' 
            % str(self.interrupt))
        # Initialize script reader and set scripts
        self.scriptreader = ScriptReader(self.nao, self.interrupt)
        self.instructions = dirpath + instructions_script_filename
        self.task1 = dirpath + task1_script_filename
        self.task2 = dirpath + task2_script_filename
        self.validationScript = dirpath + validation_script_filename

        # rospy.loginfo('Initializing hardcoded objects')
        # # FOR TESTING!
        # self.objects.createFakeObjects()

        # Precompute the saliency scores
        self.nao.startHeadScan() # for expressiveness
        self.saliency_scores = self.precomputeSaliencyScores(int(cameraID))

        # Precompute the gaze and point scores
        self.gazescores = dict()
        self.pointscores = dict()
        self.waitForGazePointScores()
        self.nao.stopHeadScan() # for expressiveness


    def recordRosbags(self, usernum):
        """ Start recording rosbags of selected topics. """
        rosbagdir = '/home/kinect/catkin/src/nao_looking_and_pointing/src/rosbags/'
        self.rosbags = subprocess.Popen(['rosbag','record',
            'objects_info',                     # from Kinect
            'script_object_reference',          # from script
            'script_status',                    # from script
            'gazepoint_info',                   # from Kinect
            'face_info',                        # from Kinect
            'robot_behavior',                   # from program
            'timer_info',                       # from timer
            'sysval_selections',                # from sysval calibration menu
            '-o',rosbagdir+'p'+str(usernum),    # file name of bag
            '-q'])                              # suppress output

    def waitForGazePointScores(self):
        """ 
        Wait for the GazePointInfo messages to arrive. 

        Technically this function only waits for the first 'gaze' and
        first 'point' GazePointInfo messages, but we expect them all 
        to come together as a pack.
        """
        # Send message to initiate gazepoint score computation
        #self.script_status_pub.publish("ResetBlocks")
        # This message may not work (it may be too early), in which 
        # case the user will have to manually press the "Compute Scores"
        # button on the kinect interface. Hopefully this ROS warning
        # message will remind them.
        rospy.logwarn("Waiting for gaze and point scores.")
        while not (self.gazescores and self.pointscores):
            sleep(1.0)
        sleep(1.0) # give time for all messages to be processed by callback

    def gazepointInfoCallback(self, data):
        """
        Receive and record gaze and point scores for each object.

        Parse the GazePointInfo message and populate the dictionary of
        scores for gaze and pointing for each object. Each dictionary
        (gazescores and pointscores) has target object ID as key and
        a list of scores as value, where the index of the list is the 
        object ID associated with that score.
        """
        if data.type == 'gaze':
            self.gazescores[data.target_id] = data.scores
        elif data.type == 'point':
            self.pointscores[data.target_id] = data.scores
        else:
            rospy.logerr("Unrecognized type in GazePointInfo message: " 
                + str(data.type))

    def precomputeSaliencyScores(self, cameraID):
        """ 
        Precompute the saliency scores of the scene. 

        Connects to the user view camera signified by cameraID, and 
        computes the saliency score for each object in self.objects.objdict

        Arguments: 
        cameraID -- int signifying ID of user view camera

        Returns: a dict of saliency scores
        """

        rospy.logwarn('Precomputing saliency scores. This might take a moment.')

        # Initialize camera
        rospy.loginfo('Connecting to participant view camera.')
        cam = cv2.VideoCapture(cameraID)

        # Grab user view snapshot from camera for saliency detection
        cam.read() # a hack, the second image seems to be better
        s,self.user_view_img = cam.read()
        user_view_fname = dirpath + 'results/p' + str(self.userID) \
            + '_userview.jpg'
        if not s:
            raise IOError("Could not take camera image!")
        else:
            # Save camera image
            cv2.imwrite(user_view_fname,self.user_view_img)

        # Close camera
        cam.release()

        # Call saliency score computing function from NVB model
        saliency_scores = self.model.calculateSaliencyScores(
            user_view_fname, self.objects.objdict)

        rospy.loginfo('Saved participant view to ' + user_view_fname)

        return saliency_scores

    def objectRefCallback(self, objectRefMsg):
        """
        Send appropriate NVB command to the robot based on received object reference.

        Parse the ROS message (ScriptObjectRef), find the object to be referenced,
        calculate the appropriate nonverbal behavior using self.findNVBForRef(),
        then send that behavior command to NaoGestures.

        Note that this function sends an action command to the robot!

        Arguments: objectRefMsg -- a ROS message of type ScriptObjectRef

        Returns: none (but moves the robot)
        """
        rospy.loginfo('Script object reference received: %d, %s' %
            (objectRefMsg.object_id, objectRefMsg.words))
        
        # Parse object reference message
        target_id = objectRefMsg.object_id
        try:
            self.objects.objdict[target_id]
        except KeyError:
            rospy.logerr('No object with ID %d in objects dictionary, \
            object reference fails' % target_id)
            return

        words_spoken = objectRefMsg.words

        # Find location of target
        target_loc = self.objects.objdict[target_id].loc

        # Calculate the correct nonverbal behavior to indicate the target
        assert self.numObj < 9
        if (self.nvb and                       # NVB is expected, and
            target_id == 8 or target_id == 9): # target obj is a bin
            action_type = 'lookandpoint'
        else:
            action_type = self.findNVBForRef(target_id, words_spoken)
            if not self.nvb:
                action_type = 'none'
            rospy.loginfo("Proposed action: %s" % action_type)

        # Publish message about robot's behavior
        self.robot_behavior_pub.publish(action=action_type, 
                                        object_id=target_id)

        # Send action command to the robot
        self.nao.doGesture(action_type, target_loc, True)

    def scriptStatusCallback(self, msg):
        """ Listen for script status. """
        status = msg.data

        # Turn on idle behaviors when instructions or interruption
        # are over, in case NVB during instructions turned them off
        if status == "InstructionsStop":
            self.nao.startIdle()
        elif status == "InterruptionStop":
            self.nao.startIdle()
            

    def markerInfoCallback(self, msg):
        """ Update the list of visible fiducial markers. """
        self.markersVisible = []
        for m in msg.markerlist:
            self.markersVisible.append(m)

    def waitForAllMarkers(self):
        rospy.loginfo("Markers visible: " + str(self.markersVisible))
        
        markerTimer = time()
        if len(self.markersVisible) is not self.numObj:
            rospy.logwarn("Waiting for all markers to be visible.")
            self.nao.startHeadScan()
        while len(self.markersVisible) is not self.numObj:
            sleep(0.5)
            # If more than 15 seconds have passed, verbally prompt
            # the user to adjust the markers
            if (time() - markerTimer) > 15:
                print "Reading marker prompt"
                self.nao.speak("I am waiting for all of the blocks to be "
                                "visible to me. Please spread the blocks "
                                "apart and make sure all of the tags are "
                                "facing me.")
                markerTimer = time()

        rospy.loginfo("Markers visible: " + str(self.markersVisible))
        self.nao.stopHeadScan()


    def systemValidation(self):
        """
        Perform the system validation portion of the experiment.
        """
        # Figure out the "correct" NVB for each object and store this to 
        # a file that will be used for data analysis. In the meantime, 
        # generate a list of actions that include every permutation of
        # look/point/lookandpoint/none, objects, and object words
        action_list = []
        outfile = dirpath \
                  + 'results/p'+str(self.userID) \
                  + '_correctactionlist.txt'
        f = open(outfile,'w')
        nonBinObjects = {k:self.objects.objdict[k] for k in range(0,self.numObj)}
        for obj in nonBinObjects.values():
            idnum = obj.idnum
            assert len(obj.words) > 0
            words = obj.words[0]
            correct_act = self.findNVBForRef(idnum, words)
            f.write(str(idnum) + ":" + correct_act + "\n")
            for act in ['none','look','point','lookandpoint']:
                action_list.append((act, idnum, words))
        f.close()
        return # TODO remove

        # Randomize the list
        random.shuffle(action_list)

        self.nao.stopHeadScan()

        # Add descriptive speech
        self.scriptreader.readScript(self.validationScript)

        # Have the robot act out the list
        for action in action_list:
            prompt = "Select the " + str(action[2] + " block.")
            target = action[1]
            nvb_action = action[0]
            
            self.actOutReference(prompt, nvb_action, target)

        self.script_status_pub.publish("SystemValidationComplete")


    def actOutReference(self, prompt, action, target):
        """
        Helper function for systemValidation()

        Do spatial reference with provided action and prompt to location.
        """
        starttime = time()
        totaltime = starttime + 5 # total action should take 5 seconds

        location = self.objects.objdict[target].loc

        # Publish message about robot's behavior
        self.robot_behavior_pub.publish(action=action, 
                                        object_id=target)

        self.nao.speak(prompt, False)
        self.nao.doGesture(action, location, False)

        waittime = totaltime - time()
        sleep(waittime) # wait for action to finish

    def findNVBForRef(self, target_id, words_spoken):
        """
        Use NVB model to select appropriate NVB commands for the target object.


        Call the calcluateNVBForRef function from NVBModel to calcluate the appropriate
        nonverbal behavior to reference the target object. First, takes an image from 
        the user view camera (for saliency detection). This image is also saved to disk
        for future reference.

        Arguments:
        target_id -- the taret object's ID (as in objdict)
        words_spoken -- the speech that accompanies the reference

        Returns: a text string indicating the NVB to perform (see NaoGestures for options)
        """
        rospy.loginfo('Finding NVB for reference to object %d' % target_id)

        # Turn words spoken into a list without characters
        words_list = re.findall(r"[\w']+",words_spoken)
        
        # Call NVBModel function that calculates saliency
        nonBinObjects = {k: self.objects.objdict[k] for k in range(self.numObj)}
        nvb = self.model.calculateNVBForRef(
            self.saliency_scores, target_id, nonBinObjects, words_list,
            self.gazescores[target_id], self.pointscores[target_id])
        return nvb

    def startup(self):
        """ Perform some functions to start the experiment. """
        self.nao.stand()
        sleep(5) # wait for standing behavior to finish
        self.nao.startIdle()

        # Assign words to the objects, which should be in objdict by now
        assert len(self.objects.objdict) == self.numObj
        self.objects.objdict[0].words = ['small red']
        self.objects.objdict[1].words = ['large orange']
        self.objects.objdict[2].words = ['small yellow']
        self.objects.objdict[3].words = ['large lime']
        self.objects.objdict[4].words = ['small green']
        self.objects.objdict[5].words = ['small blue']
        self.objects.objdict[6].words = ['small blue']
        self.objects.objdict[7].words = ['small red']

        # Assign left and right bins to object dict as well
        self.objects.objdict[8] = Bin()
        self.objects.objdict[9] = Bin()
        rightBinKinectCoord = [0.5157492756843567, \
                             -0.7387366890907288, \
                              1.5461519956588745]
        leftBinKinectCoord = [-0.17079749703407288, \
                              -0.7140763998031616, \
                               1.5642540454864502]
        self.objects.objdict[8].loc = \
            self.objects.convertCoords(leftBinKinectCoord)
        self.objects.objdict[9].loc = \
            self.objects.convertCoords(rightBinKinectCoord)


        # Publish information about this participant
        self.script_status_pub.publish('userID:' + str(self.userID))
        self.script_status_pub.publish('nvb:' + str(self.nvb))
        self.script_status_pub.publish('interruption:' + str(self.interrupt))


    def shutdown(self):
        """ Shut down cleanly. """
        rospy.logwarn("Shutting down...")

        # Shut down robot cleanly
        self.nao.stopHeadScan()
        self.nao.robotShutdown()

        # End rosbag recording (from answers.)
        rospy.loginfo('Ending rosbag recording')
        ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" %
            self.rosbags.pid, shell=True, stdout=subprocess.PIPE)
        ps_output = ps_command.stdout.read()
        retcode = ps_command.wait()
        assert retcode is not None, "ps command expected integer, returned %d" % retcode
        for pid_str in ps_output.split("\n")[:-1]:
            result = os.kill(int(pid_str), signal.SIGINT)
            rospy.loginfo('Kill rosbag child node (%s) returned: %s' 
                % (pid_str, str(result)))
        term_return = self.rosbags.terminate()
        rospy.loginfo('Terminating rosbag parent process returned: %s' 
            % str(term_return))

    def main(self):
        self.startup()

        # Play system validation portion of experiment.
        self.systemValidation() 

        # Play HRI script
        #self.nao.startIdle()
        #self.scriptreader.readScript(self.instructions)
        # self.waitForAllMarkers()
        # self.scriptreader.readScript(self.task1)
        # self.waitForAllMarkers()
        # self.scriptreader.readScript(self.task2)

        self.shutdown()


if __name__ == "__main__":
    # Remove extraneous ROS arguments
    sys.argv = rospy.myargv(argv=sys.argv)

    parser = argparse.ArgumentParser(description="HRI interaction controller")
    parser.add_argument('usernum',
        help='ID number of the participant (should be unique to participant)',
        type=int)
    parser.add_argument('validation_scriptname',
        help='the file name of the script for the validation portion')
    parser.add_argument('robotIP',
        help="the robot's IP address",
        default="192.168.1.2")
    parser.add_argument('robotPort',
        help="the robot's port number",
        default=9559,
        type=int)
    parser.add_argument('cameraID',
        help='ID number of the user view camera',
        default=0,
        type=int)
    parser.add_argument('nvb_on',
        help='boolean indicating whether robot should perform nonverbal behavior',
        type=ast.literal_eval)
    parser.add_argument('interruption_on',
        help='boolean indicating whether an interruption will occur',
        type=ast.literal_eval)
    parser.add_argument('--script-instructions',
        dest="instructions",
        help='the file name of the script for the instructions',
        default='testscript.txt')
    parser.add_argument('--script-task1',
        dest='task1',
        help='the file name of the script for the 1st HRI portion',
        default='testscript.txt')
    parser.add_argument('--script-task2',
        dest='task2',
        help='the file name of the script for the 2nd HRI portion',
        default='testscript.txt')

    args = parser.parse_args()
    usernum = args.usernum
    val_script = args.validation_scriptname
    ip = args.robotIP
    port = args.robotPort
    camID = args.cameraID
    nvb = args.nvb_on
    interrupt = args.interruption_on
    instructions = args.instructions
    task1 = args.task1
    task2 = args.task2

    ic = InteractionController(usernum, instructions, task1, task2,
        val_script,  ip, port, camID, nvb, interrupt)
    ic.main()

    sys.exit(0)