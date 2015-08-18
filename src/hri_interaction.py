#!/usr/bin/env python

"""
Author: Henny Admoni 
Date Created: 8/1/2015

This module runs the HRI construction interaction. 
"""
# UNCOMMENT THE FOLLOWING FOR ROS
#import rospy
#from kinect2_pointing_recognition.msg import ObjectsInfo
#from nao-looking-and-pointing.msg import ScriptObjectRef
#from std_msgs.msg import String
# --------------------------------
import cv2
import numpy as np
import saliency_detector
from naoGestures import NaoGestures
from scriptReader import ScriptReader
from nvbModel import NVBModel

class InteractionController():
    """ High-level class that runs the HRI interaction. """

    def __init__(self, script_filename):
        # UNCOMMENT THE FOLLOWING FOR ROS
        # rospy.init_node('interaction_controller')

        # Subscribe to relevant topics
        #self.objects_info_listener = rospy.Subscriber('/objects_info', ObjectsInfo, self.objectsInfoCallback)
        #self.nvb_command_listener = rospy.Subscriber('/script_object_reference', ScriptObjectRef, self.objectRefCallback)
        # ---------------------------------


        # dictionary of objects, key = object ID, value = Lego object
        self.objdict = dict()

        # Nao robot
        self.nao = NaoGestures()

        # NVB model
        self.model = NVBModel()

        # Initialize camera
        self.cam = cv2.VideoCapture(0)

        # Initialize script reader
        self.scriptreader = ScriptReader(script_filename)

    def initializeObjects(self):
        """
        Initialize the hardcoded object colors, IDs, and descriptor words.

        Arguments: none

        Returns: none
        """
        o1 = Lego(1,[0,0,0],[255,0,0],'[small, red, cube]') #dummy object

        self.objdict[1] = o1

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
            o = Lego(objectMsg.object_id, objectMsg.pos, [], '')
            self.objdict[objectMsg.object_id] = o
            #rospy.logdebug("Adding new object (id %d) to object list", objectMsg.object_id) UNCOMMENT

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
        
        # Parse object reference message
        target_id = objectRefMsg.target_object_id
        if not self.objdict[target_id]:
            raise ValueError('No object with ID %d in objects dictionary, object reference fails',
                    target_id)
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
        
        # Grab user view snapshot from webcam for saliency detection
        s,user_view_img = self.cam.read()
        if not s:
            raise IOException("Could not take camera image!")
        else:
            cv2.imwrite("userview.jpg",img) # save camera image

        
        # Call NVBModel function that calculates saliency
        nvb = self.model.calculateNVBForRef(user_view_img, target_id, self.objdict, words_spoken)
        return nvb

    def main(self):
        self.scriptreader.start()

       

class Lego():
    """ A Lego object, which is the object to be manipulated in the experiment. """

    def __init__(self, idnum, location, color, descriptor_words):
        """
        Location is a length 3 array of floats. Color is an RGB array. 
        Descriptor words is an array of words.
        """
        self.idnum = idnum
        self.loc = location
        self.color = color
        self.words = descriptor_words


if __name__ == "__main__":
    ic = InteractionController('testscript.txt')
    ic.main()
