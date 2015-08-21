#!/usr/bin/env python

"""
Author: Henny Admoni 
Date Created: 8/1/2015

This module runs the HRI construction interaction. 
"""
# ROS imports
import rospy
from kinect2_pointing_recognition.msg import ObjectsInfo
from nao_looking_and_pointing.msg import ScriptObjectRef
from std_msgs.msg import String
# --------------------------------
import sys
import cv2
import re
import numpy as np
from naoGestures import NaoGestures
from scriptReader import ScriptReader
from nvbModel import NVBModel

class InteractionController():
    """ High-level class that runs the HRI interaction. """

    def __init__(self, script_filename, cameraID = 0):
        """
        Initialize the controller for the HRI interaction.

        Arguments:
        script_filename -- string name of a script for the interaction
        cameraID -- int ID for user view camera, defaults to 0
        """
        rospy.init_node('interaction_controller')
        rospy.logdebug('initializing interaction controller')

        # Subscribe to relevant topics
        self.objects_info_listener = rospy.Subscriber('/objects_info', ObjectsInfo, self.objectsInfoCallback)
        self.nvb_command_listener = rospy.Subscriber('/script_object_reference', ScriptObjectRef, self.objectRefCallback)
        

        # dictionary of objects, key = object ID, value = Lego object
        self.objdict = dict()

        rospy.logwarn('Creating a NaoGestures object')
        # Nao robot
        self.nao = NaoGestures()

        rospy.logwarn('Creating an NVBModel object')
        # NVB model
        self.model = NVBModel()

        rospy.logwarn('Connecting to participant view camera')
        # Initialize camera
        self.cam = cv2.VideoCapture(cameraID)

        rospy.logwarn('Creating a ScriptReader object')
        # Initialize script reader
        self.scriptreader = ScriptReader(script_filename)

        rospy.logwarn('Initializing hardcoded objects')
        # FOR TESTING!
        self.initializeObjects()

    def initializeObjects(self):
        """
        For testing: initialize hardcoded objects.

        Each object consists of colors, ID, and descriptor words.

        Arguments: none

        Returns: none
        """
        o1 = Lego(1,[0,0,0],[255,0,0],[200,0,0],['small','red','cube']) #dummy object
        o2 = Lego(2,[0.5,0.5,0],[0,100, 0],[0,255,0],['small','green','cube'])
        o3 = Lego(3,[1,0,0],[0,0,100], [0,0,255],['large','blue','block'])

        self.objdict[o1.idnum] = o1
        self.objdict[o2.idnum] = o2
        self.objdict[o3.idnum] = o3

        # TODO: Hardcode some objects

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
        if objectMsg.object_id in self.objdict:
            # if the object location has changed, update it
            if not objectMsg.pos == self.objdict[objectMsg.object_id].loc:
                self.objdict[objectMsg.object_id].loc = objectMsg.pos
        else:
            # if this is a new object, add it to objdict
            o = Lego(objectMsg.object_id,   # ID number
                     objectMsg.pos,         # 3D position
                     objectMsg.color_upper, # upper RGB color threshold
                     objectMsg.color_lower, # lower RGB color threshold
                     '')                    # descriptor words
            self.objdict[objectMsg.object_id] = o
            rospy.logdebug("Adding new object (id %d) to object list", 
                objectMsg.object_id)

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
        rospy.logwarn('Object reference received: %d, %s',
            objectRefMsg.object_id, objectRefMsg.words)
        
        # Parse object reference message
        target_id = objectRefMsg.object_id
        try:
            self.objdict[target_id]
        except KeyError:
            rospy.logerr('No object with ID %d in objects dictionary, \
            object reference fails', target_id)
            return

        words_spoken = objectRefMsg.words

        # Find location of target
        target_loc = self.objdict[target_id].loc

        # Calculate the correct nonverbal behavior to indicate the target
        action_type = self.findNVBForRef(target_id, words_spoken)

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
        rospy.logwarn('Finding NVB for reference to object %d', target_id)

        # Grab user view snapshot from webcam for saliency detection
        s,self.user_view_img = self.cam.read()
        user_view_fname = 'userview.jpg'
        if not s:
            raise IOError("Could not take camera image!")
        else:
            cv2.imwrite(user_view_fname,self.user_view_img) # save camera image

        # Turn words spoken into a list without characters
        words_list = re.findall(r"[\w']+",words_spoken)
        
        # Call NVBModel function that calculates saliency
        nvb = self.model.calculateNVBForRef(user_view_fname, target_id, self.objdict, words_list)
        return nvb

    def main(self):
        self.scriptreader.start()

       

class Lego():
    """ A Lego object, which is the object to be manipulated in the experiment. """

    def __init__(self, idnum, location, color_upper, color_lower, descriptor_words):
        """
        Location is a length 3 array of floats. Color is a length 3 array of
        ints represeting an RGB array for upper and lower threshold values. 
        Descriptor words is an array of words.
        """
        self.idnum = idnum
        self.loc = location
        self.color_upper = color_upper
        self.color_lower = color_lower
        self.words = descriptor_words


if __name__ == "__main__":
    if len(sys.argv) > 1:
        scriptname = sys.argv[1]
    else:
        scriptname = "testscript.txt"
    ic = InteractionController(scriptname)
    ic.main()
