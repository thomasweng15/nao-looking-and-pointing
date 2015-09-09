#!/usr/bin/env python

"""
Author: Henny Admoni 
Date Created: 8/14/2015

This module defines the nonverbal behavior model. 
"""
import cv2
import numpy as np
import saliency_detector
from matplotlib import pyplot as plt

DEBUG = True
DISPLAYIMAGES = True

class NVBModel():
    """ The nonverbal behavior model. """

    def __init__(self):
        # Parameters - TODO: Define these
        self.w_saliency = 1
        self.w_verbal = 1
        self.w_gaze = 1
        self.w_point = 1

        self.certainty_threshold = 0.75

        # Global variables - for NaoGestures.doGesture()
        self.VERBAL = "none"
        self.GAZE = "look"
        self.POINT = "point"
        self.GAZEANDPOINT = "lookandpoint"

    def calculateNVBForRef(self, saliency, target_id, object_list, 
        spoken_words, gazescores, pointscores):
        """
        Calculate the nonverbal behavior for the target object.
        
        Calculate the gaze and pointing behaviors that are most effective
        when referring to the target object, given the parameters.

        Arguments:
        saliency -- EITHER: the file name of an image from the user's view camera,
                    OR: a dictionary of pre-computed saliency scores
        target_id -- the ID number of the target object from object_list
        object_list -- a dictionary of objects with their IDs as the key
        spoken_words -- a list of words being spoken
        gazescores -- a list of gaze scores (pre-calculated for each object)
        pointscores -- a list of point scores (pre-calculated for each object)

        Returns: a string representing the best nonverbal behavior (see NaoGestures for options)
        """

        if DEBUG: print "NVB model: calculate NVB for ref"

        # Check if saliency scores have been precomputed. If not,
        # calculate the saliency scores
        if isinstance(saliency, dict):
            saliency_scores = saliency
        else:
            saliency_scores = self.calculateSaliencyScores(saliency, object_list)
        for key, value in saliency_scores.iteritems():
            saliency_scores[key] = value * self.w_saliency
        if DEBUG: print "saliency scores: " + str(saliency_scores)

        # Calculate verbal reference score
        verbal_scores = self.calculateVerbalScores(spoken_words, object_list)
        for key, value in verbal_scores.iteritems():
            verbal_scores[key] = value * self.w_verbal
        if DEBUG: print "verbal scores: " + str(verbal_scores)

        # Calculate gaze score with the robot looking at target object
        #gaze_scores = self.calculateGazeScores(target_id, object_list)
        gaze_scores = gazescores
        gaze_scores = [val * self.w_gaze for val in gaze_scores]

        # Calculate pointing score with the robot pointing to target object
        #point_scores = self.calculatePointScores(target_id, object_list)
        point_scores = pointscores
        point_scores = [val * self.w_point for val in point_scores]

        # Calculate likelihood scores for all objects given current scene and different combinations
        # of available nonverbal behaviors (verbal only, verbal+gaze, verbal+gesture, verbal+gaze+gesture)
        score_verbal = dict()
        score_verbal_gaze = dict()
        score_verbal_point = dict()
        score_verbal_gaze_point = dict()

        if DEBUG:
            print "gaze scores: " + str(gaze_scores)
        
        for idnum in object_list:
            score_verbal[idnum] = saliency_scores[idnum] + verbal_scores[idnum]
            score_verbal_gaze[idnum] = score_verbal[idnum] + gaze_scores[idnum]
            score_verbal_point[idnum] = score_verbal[idnum] + point_scores[idnum]
            score_verbal_gaze_point[idnum] = score_verbal_gaze[idnum] + point_scores[idnum]

        if DEBUG: print score_verbal

        # Calculate the certainty score of the target object
        certainty_verbal = self.calculateCertaintyScore(target_id, score_verbal)
        certainty_verbal_gaze = self.calculateCertaintyScore(target_id, score_verbal_gaze)
        certainty_verbal_point = self.calculateCertaintyScore(target_id, score_verbal_point)
        certainty_verbal_gaze_point = self.calculateCertaintyScore(target_id, score_verbal_gaze_point)

        # Select NVB based on certainty score
        if certainty_verbal > self.certainty_threshold:
            return self.VERBAL
        elif certainty_verbal_gaze > self.certainty_threshold:
            return self.GAZE
        elif certainty_verbal_point > self.certainty_threshold:
            return self.POINT
        else:
            return self.GAZEANDPOINT


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
        if DEBUG: print "calculating certainty score..."

        # Remove zero values from the scores list
        scores_list = []
        for value in scores.itervalues():
            if value != 0:
                scores_list.append(value)
        target_score = scores[target_id]
        if DEBUG: 
            print "...scores list: " + str(scores_list)
            print "...target score: " + str(target_score)

        target_diff = target_score - np.mean(scores_list)
        stdev = np.std(scores_list,ddof=1) # DOF = 1 for more accuracy

        certainty = target_diff / stdev

        if DEBUG: print "...certainty score: " + str(certainty)

        return certainty

    def calculateSaliencyScores(self, img_fname, object_list):
        """
        Calculate the saliency of each object from the user's point of view.

        Arguments:
        img_fname -- an image file name
        object_list -- a dictionary containing objects (e.g., Legos)

        Returns: a dictionary with object ID as key and saliency as value
        """
        if DEBUG: print("calculating saliency...")

        # Create saliency map
        salmap = saliency_detector.generateSaliencyMap(img_fname)
        nrows, ncols = salmap.shape
        if DEBUG: 
            print("...saliency map image size: " + str(nrows) + " x " + str(ncols))

        user_view_img = cv2.imread(img_fname)

        salscores = dict()
        for idnum, obj in object_list.iteritems():
            # Find object position in rgb image by color
            obj_pos_array = self.getObjectPixelPosition(
                user_view_img, obj.color_upper, obj.color_lower)

            salience = saliency_detector.identifySaliencyAtLocation(
                salmap, obj_pos_array)
            salscores[idnum] = salience

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
        
        # Sanity check - spoken_words should be a list of strings, not a string
        assert isinstance(spoken_words, list)

        for idnum, obj in object_list.iteritems():
            score = 0
            for w in obj.words:
                if w in spoken_words:
                    score = score + 1
            total = float(score) / len(spoken_words)
            verbalscores[idnum] = total

        return verbalscores

    def calculateGazeScores(self, target_id, object_list):
        """
        NOTE: This is currently done on the Windows side by the Kinect code.
        Do not use this function unless it is completed, in which case, remove
        this note.

        Calculate the gaze score for each object in object_list.

        The gaze score represents how likely it is that the robot is looking
        at the object in question. It is calculated for each object with the 
        robot's gaze directed at the target object (indicated by target_id).

        Arguments:
        target_id -- the ID of the object being gazed at
        object_list -- a dictionary of all objects in the scene

        Returns: a dictionary with object ID as key and gaze score as value

        """
        # TODO: Thomas? Raytracing or some other metric like distance from center of gaze cone.
        gazescores = dict()
        for obj in object_list:
            gazescores[obj] = 0

        return gazescores


    def calculatePointScores(self, target_id, object_list):
        """
        NOTE: This is currently done on the Windows side by the Kinect code.
        Do not use this function unless it is completed, in which case, remove
        this note.

        Calculate the pointing score for each object in object_list.

        The pointing score represents how likely it is that the robot is pointing
        at the object in question. It is calculated for each object with the
        robot's point directed at the target object (indicated by target_id).

        Arguments:
        target_id -- the ID of the object being pointed to
        object_list -- a dictionary of all objects in the scene

        Returns: a dictionary with object ID as key and pointing score as value.
        """
        # TODO: Thomas? Raytracing or some other metric like distance for center of pointing cone.
        pointscores = dict()
        for obj in object_list:
            pointscores[obj] = 0

        return pointscores


    def getObjectPixelPosition(self, img, color_upper, color_lower):
        """
        Helper function for calculateSaliencyScores.

        Find the (row,col) indices of the pixels in img that have the identified color.

        Arguments:
        img -- an image
        color_upper -- an RGB tuple for the upper bounds of the object's color
        color_lower -- an RGB tuple for the lower bounds of the object's color

        Returns: array of (row,col) tuples representing pixels in img
        """
        # Sanity check that upper and lower color values were passed properly
        for l,u in zip(color_lower, color_upper):
            if not l <= u:
                raise AssertionError(
                    "Lower color %r greater than upper color %r", l, u)


        # Because OpenCV actually returns BGR instead of RGB, swap 
        # the B and R values in the provided color arrays
        color_lower_bgr = [color_lower[2], color_lower[1], color_lower[0]]
        color_upper_bgr = [color_upper[2], color_upper[1], color_upper[0]]

        if DEBUG:
            print("...color bounds: " + 
                str(color_upper) + "->" + str(color_upper_bgr) + ', ' +
                str(color_lower) + "->" + str(color_lower_bgr))

        lower_array = np.array(color_lower_bgr, dtype = "uint8")
        upper_array = np.array(color_upper_bgr, dtype = "uint8")

        # find where in the image the specified color occurs, and create
        # a binary image based on that
        binary_img = cv2.inRange(img, lower_array, upper_array)

        # get image size
        nrows, ncols = binary_img.shape
        if DEBUG and DISPLAYIMAGES: 
            print("...image size: " + str(nrows) + " x " + str(ncols))
            cv2.imshow('binary image from saliency map',binary_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()#('binary image from saliency map')

        # create the array to return
        white_pixels = []

        # iterate through the binary image and find white pixels
        # TODO: optimize this loop traversal!
        for row in range(1,nrows):
            for col in range(1,ncols):
                if binary_img[row,col] > 240: # approxximately white
                    white_pixels.append((row,col))

        return white_pixels

 
