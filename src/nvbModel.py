#!/usr/bin/env python

"""
Author: Henny Admoni 
Date Created: 8/14/2015

This module defines the nonverbal behavior model. 
"""
import cv2
import numpy as np
import saliency_detector

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

 
