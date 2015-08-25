#!/usr/bin/env python

"""
Author: Henny Admoni 
Date Created: 8/1/2015

This module runs the HRI construction interaction. 
"""
# ROS imports
import rospy
from kinect2_pointing_recognition.msg import ObjectsInfo, GazePointInfo
from nao_looking_and_pointing.msg import ScriptObjectRef
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
import numpy as np
from time import sleep
from lego import Lego
from naoGestures import NaoGestures
from scriptReader import ScriptReader
from nvbModel import NVBModel

class InteractionController():
    """ High-level class that runs the HRI interaction. """

    def __init__(self, usernum, script_filename, cameraID=0):
        """
        Initialize the controller for the HRI interaction.

        Arguments:
        usernum -- ID number for participant
        script_filename -- string name of a script for the interaction
        cameraID -- int ID for user view camera, defaults to 0
        """

        # Handle shutdown gracefully
        atexit.register(self.shutdown)

        rospy.init_node('interaction_controller', log_level=rospy.INFO)
        rospy.loginfo('Initializing interaction controller')

        # Subscribe to relevant topics
        self.objects_info_listener = rospy.Subscriber('/objects_info', ObjectsInfo, self.objectsInfoCallback)
        self.nvb_command_listener = rospy.Subscriber('/script_object_reference', ScriptObjectRef, self.objectRefCallback)
        self.gazepoint_info_listener = rospy.Subscriber('/gazepoint_info', GazePointInfo, self.gazepointInfoCallback)
        
        # Start rosbags in a separate thread
        rosbagthread = threading.Thread(target=self.recordRosbags, 
                                        args=(usernum,))
        rosbagthread.start()

        # dictionary of objects, key = object ID, value = Lego object
        self.objdict = dict()

        rospy.loginfo('Creating a NaoGestures object')
        # Nao robot
        self.nao = NaoGestures()

        rospy.loginfo('Creating an NVBModel object')
        # NVB model
        self.model = NVBModel()

        rospy.loginfo('Creating a ScriptReader object')
        # Initialize script reader
        self.scriptreader = ScriptReader(script_filename)

        # rospy.loginfo('Initializing hardcoded objects')
        # # FOR TESTING!
        # self.initializeObjects()

        # Precompute the gaze and point scores
        self.gazescores = dict()
        self.pointscores = dict()
        self.waitForGazePointScores()

        # Precompute the saliency scores
        self.saliency_scores = self.precomputeSaliencyScores(0)

    def recordRosbags(self, usernum):
        """ Start recording rosbags of selected topics. """
        rosbagdir = '/home/kinect/catkin/src/nao_looking_and_pointing/src/rosbags/'
        self.rosbags = subprocess.Popen(['rosbag','record',
            'objects_info',                     # record topic
            'script_object_reference',          # record topic
            'gazepoint_info',                   # record topic
            'face_info',                        # record topic
            '-o',rosbagdir+'p'+str(usernum),   # file name of bag
            '-q'])                              # suppress output

    def initializeObjects(self):
        """
        For testing: initialize hardcoded objects.

        Each object consists of colors, ID, and descriptor words.

        Arguments: none

        Returns: none
        """
        o1 = Lego(1,[0,0,0],[255,40,40],[200,0,0],['small','red','cube']) #dummy object
        o2 = Lego(2,[0.5,0.5,0],[40,255,40],[0,100, 0],['small','green','cube'])
        o3 = Lego(3,[1,0,0],[40,40,255],[0,0,100],['large','blue','block'])

        self.objdict[o1.idnum] = o1
        self.objdict[o2.idnum] = o2
        self.objdict[o3.idnum] = o3

        # TODO: Hardcode some objects

    def waitForGazePointScores(self):
        """ 
        Wait for the GazePointInfo messages to arrive. 

        Technically this function only waits for the first 'gaze' and
        first 'point' GazePointInfo messages, but we expect them all 
        to come together as a pack.
        """
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
        computes the saliency score for each object in self.objdict

        Arguments: 
        cameraID -- int signifying ID of user view camera

        Returns: a dict of saliency scores
        """

        rospy.logwarn('Precomputing saliency scores. This might take a moment.')

        # Initialize camera
        rospy.loginfo('Connecting to participant view camera.')
        cam = cv2.VideoCapture(cameraID)

        # Grab user view snapshot from camera for saliency detection
        s,self.user_view_img = cam.read()
        user_view_fname = 'userview.jpg'
        if not s:
            raise IOError("Could not take camera image!")
        else:
            # Save camera image
            cv2.imwrite(user_view_fname,self.user_view_img)

        # Close camera
        cam.release()

        # Call saliency score computing function from NVB model
        saliency_scores = self.model.calculateSaliencyScores(
            user_view_fname, self.objdict)

        rospy.loginfo('Saved participant view to ./' + user_view_fname)

        return saliency_scores

    def objectsInfoCallback(self, objectMsg):
        """
        Update object positions.

        Parse the ROS message (ObjectsInfo) and update the objects dictionary with
        new information about existing objects. If the object is not in the dictionary,
        create an entry for it.

        Arguments:
        objectMsg -- a ROS message of type ObjectsInfo

        Returns: none
        """
        obj_id = int(objectMsg.object_id)

        if obj_id in self.objdict:
            # if the object location has changed, update it
            if not objectMsg.pos == self.objdict[obj_id].loc:
                self.objdict[obj_id].loc = objectMsg.pos
        else:
            # if this is a new object, add it to objdict
            o = Lego(obj_id,   # ID number
                     objectMsg.pos,         # 3D position
                     objectMsg.color_upper, # upper RGB color threshold
                     objectMsg.color_lower, # lower RGB color threshold
                     '')                    # descriptor words
            self.objdict[obj_id] = o
            rospy.logdebug("Adding new object (id %d) to object list", obj_id)

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
            self.objdict[target_id]
        except KeyError:
            rospy.logerr('No object with ID %d in objects dictionary, \
            object reference fails' % target_id)
            return

        words_spoken = objectRefMsg.words

        # Find location of target
        target_loc = self.objdict[target_id].loc

        # Calculate the correct nonverbal behavior to indicate the target
        action_type = self.findNVBForRef(target_id, words_spoken)
        print "proposed action: " + action_type

        # Send action command to the robot
        self.nao.doGesture(action_type, target_loc)

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
        nvb = self.model.calculateNVBForRef(
            self.saliency_scores, target_id, self.objdict, words_list)
        return nvb

    def shutdown(self):
        """ Shut down cleanly. """
        rospy.logwarn("Shutting down...")

        # End rosbag recording (from answers.)
        ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" %
            self.rosbags.pid, shell=True, stdout=subprocess.PIPE)
        ps_output = ps_command.stdout.read()
        retcode = ps_command.wait()
        assert retcode == 0, "ps command expected 0, returned %d" % retcode
        for pid_str in ps_output.split("\n")[:-1]:
            os.kill(int(pid_str), signal.SIGINT)
        self.rosbags.terminate()

    def main(self):
        # TODO: Wait for saliency maps and pointing and gaze scores arrays
        # to initialize, will receive via ROS message
        self.scriptreader.readScript()


if __name__ == "__main__":
    sys.argv = rospy.myargv(argv=sys.argv)

    parser = argparse.ArgumentParser(description="HRI interaction controller")
    parser.add_argument('scriptname',
        help='the file name of the script for the interaction',
        default='/home/kinect/catkin/src/nao_looking_and_pointing/src/testscript.txt')
    parser.add_argument('usernum',
        help='ID number of the participant (should be unique to participant)')

    args = parser.parse_args()
    script = args.scriptname
    usernum = args.usernum

    ic = InteractionController(usernum, script)
    ic.main()

    sys.exit(0)