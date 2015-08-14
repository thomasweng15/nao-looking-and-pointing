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


class NVBModel():
    """ The nonverbal behavior model. """

    # Global variables - for NaoGestures.doGesture()
    VERBAL = "none"
    GAZE = "look"
    POINT = "point"
    GAZEANDPOINT = "lookandpoint"

    def __init__(self):
        # Parameters - TODO: Define these
        self.w_saliency = 1
        self.w_verbal = 1
        self.w_gaze = 1
        self.w_gesture = 1

        self.certainty_threshold = 0.75

    def calculateNVBForRef(self, user_view, target_id, object_list, spoken_words):
        """
        Calculate the nonverbal behavior for the target object.
        
        Calculate the gaze and pointing behaviors that are most effective
        when referring to the target object, given the parameters.

        Arguments:
        user_view -- an image from the user's view camera
        target_id -- the ID number of the target object from object_list
        object_list -- a dictionary of objects with their IDs as the key
        spoken_words -- a list of words being spoken

        Returns: a string representing the best nonverbal behavior (see NaoGestures for options)
        """
        # Calculate saliency score
        saliency_scores = self.calculateSaliencyScores(user_view, object_list)
        saliency_scores = saliency_scores * w_saliency

        # Calculate verbal reference score
        verbal_scores = self.calculateVerbalScores(spoken_words, object_list)
        verbal_scores = verbal_scores * w_verbal

        # Calculate gaze score with the robot looking at target object
        gaze_scores = self.calculateGazeScores(target_id, object_list)

        # Calculate pointing score with the robot pointing to target object
        point_scores = self.calculatePointScores(target_id, object_list)

        # Calculate likelihood scores for all objects given current scene and different combinations
        # of available nonverbal behaviors (verbal only, verbal+gaze, verbal+gesture, verbal+gaze+gesture)
        score_verbal = dict()
        score_verbal_gaze = dict()
        score_verbal_point = dict()
        score_verbal_gaze_point = dict()
        
        for o in object_list:
            score_verbal[o.idnum] = saliency_scores[o.idnum] + verbal_scores[o.idnum]
            score_verbal_gaze[o.idnum] = score_verbal[o.idnum] + gaze_scores[o.idnum]
            score_verbal_point[o.idnum] = score_verbal[o.idnum] + point_scores[o.idnum]
            score_verbal_gaze_point[o.idnum] = score_verbal_gaze[o.idnum] + point_scores[o.idnum]

        # Calculate the certainty score of the target object
        certainty_verbal = self.calculateCertaintyScore(target_id, score_verbal)
        certainty_verbal_gaze = self.calcluateCertaintyScore(target_id, score_verbal_gaze)
        certainty_verbal_point = self.calcluateCertaintyScore(target_id, score_verbal_point)
        certainty_verbal_gaze_point = self.calculateCertaintyScore(target_id, score_verbal_gaze_point)

        # Select NVB based on certainty score
        if certainty_verbal > self.certainty_threshold:
            return VERBAL
        elif certainty_verbal_gaze > self.certainty_threshold:
            return GAZE
        elif certainty_verbal_point > self.certainty_threshold:
            return POINT
        else:
            return GAZEANDPOINT


    def calculateCertaintyScore(self, target_id, scores):
        """
        Calculate the certainty score for the target object.

        Certainty score is the number of standard deviations away from the mean
        the target object's score is. The mean only includes non-zero values.

        Arguments:
        target_id -- an integer representing the target object's ID
        scores -- a dictionary of likelihood scores, with object IDs as keys and scores as values

        Returns: a certainty score as a float
        """

        # Remove zero values from the scores list
        scores_list = [value for value in scores.itervalues() if value != 0]

        target_diff = score_verbal[target_id] - np.mean(scores_list)
        stdev = np.std(scores_list,ddof=1) # DOF = 1 for more accuracy

        certainty = target_diff / stdev

        return certainty

    def calculateSaliencyScores(self, user_view_img, object_list):
        """
        Calculate the saliency of each object from the user's point of view.

        Arguments:
        user_view_img -- an image
        object_list -- a dictionary containing objects (e.g., Legos)

        Returns: a dictionary with object ID as key and saliency as value
        """
        salmap = saliency_detector.generateSaliencyMap(self.user_view_img)

        salscores = dict()
        for obj in object_list:
            # Find object position in rgb image by color
            obj_pos_array = getObjectPixelPosition(user_view_img, obj.color)

            salience = saliency_detector.identifySaliencyAtLocation(salmap, obj_pos_array)
            salscores[obj.idnum] = salience

        return salscores

    def calculateVerbalScores(self, spoken_words, object_list):
        """
        Calculate the verbal reference score for each object.
        
        Calculate the proportion of the spoken words that describe that object.

        Arguments:
        spoken_words -- a list of words that have been spoken
        object_list -- a dictionary containing objects (e.g., Legos)

        Returns: a dictionary with object ID as key and verbal scores as value
        """
        verbalscores = dict()
        
        for o in object_list:
            score = 0
            for w in o.words:
                if w in spoken_words:
                    score = score + 1
            score = float(score) / len(spoken_words)
            verbalscores[o.idnum] = score

        return verbalscores

    def calculateGazeScores(self, target_id, object_list):
        """
        Calculate the gaze score for each object in object_list.

        The gaze score represents how likely it is that the robot is looking
        at the object in question. It is calculated for each object with the 
        robot's gaze directed at the target object (indicated by target_id).

        Arguments:
        target_id -- the ID of the object being gazed at
        object_list -- a dictionary of all objects in the scene

        Returns: a dictionary with object ID as key and gaze score as value

        """
        pass # TODO: Thomas? Raytracing or some other metric like distance from center of gaze cone.

    def calculatePointScores(self, target_id, object_list):
        """
        Calculate the pointing score for each object in object_list.

        The pointing score represents how likely it is that the robot is pointing
        at the object in question. It is calculated for each object with the
        robot's point directed at the target object (indicated by target_id).

        Arguments:
        target_id -- the ID of the object being pointed to
        object_list -- a dictionary of all objects in the scene

        Returns: a dictionary with object ID as key and pointing score as value.
        """
        pass # TODO: Thomas? Raytracing or some other metric like distance for center of pointing cone.

    def getObjectPixelPosition(self, img, color):
        """
        Helper function for calculateSaliencyScores.

        Find the (row,col) indices of the pixels in img that have the identified color.

        Arguments:
        img -- an image
        color -- an RGB tuple

        Returns: array of (row,col) tuples representing pixels in img
        """
        # Define color boundaries in RGB 
        boundaries = ([min(val*0.9,0) for val in color],[max(val*1.1,255) for val in color])

        lower_array = np.array(boundaries[0], dtype = "uint8")
        upper_array = np.array(boundaries[1], dtyle = "uint8")

        # find where in the image the specified color occurs, and create
        # a binary image based on that
        binary_img = cv2.inRange(img, lower, upper)

        # get image size
        nrows, ncols = binary_img.shape

        # create the array to return
        white_pixels = []

        # iterate through the binary image and find white pixels
        # TODO: optimize this loop traversal!
        for row in range(1,nrows):
            for col in range(1,ncols):
                if binary_img[row,col] == 255:
                    white_pixels.append((row,col))

        return white_pixels

        

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


