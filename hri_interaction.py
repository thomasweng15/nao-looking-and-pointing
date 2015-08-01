#!/usr/bin/env python

"""
Author: Henny Admoni (8/1/2015)

This module runs the HRI construction interaction. 
"""
import rospy
from naoGestures import NaoGestures
from kinect2_pointing_recognition.msg import ObjectsInfo
from std_msgs.msg import String
import cv2

class InteractionController():
    def __init__(self):
        rospy.init_node('interaction_controller')
        
        # Subscribe to relevant topics
        self.objects_info_listener = rospy.Subscriber('/objects_info', ObjectsInfo, self.objectsInfoCallback)
        self.nvb_command_listener = rospy.Subscriber('/script_object_reference', ScriptObjectRef, self.objectRefCallback)


        # dictionary of objects, key = object ID, value = Lego object
        self.objdict = dict()

        # Nao robot
        self.nao = NaoGestures()

        # NVB model
        self.model = NVBModel()

        # initialize camera
        self.cam = cv2.VideoCapture(0)

    def initializeObjects(self):
        """
        Initialize the hardcoded object colors, IDs, and descriptor words here.
        """
        o1 = Lego(1,[0,0,0],[255,0,0],'[small, red, cube]') #dummy object

        self.objdict[1] = o1


    def objectsInfoCallback(self, objectMsg):
        """
        Update object positions.
        """
        if objectMsg.object_id in self.objdict:
            # if the object location has changed, update it
            if not objectMsg.pos == self.objdict[objectMsg.object_id].loc:
                self.objdict[objectMsg.object_id].loc = objectMsg.pos
        else:
            #o = Lego(objectMsg.object_id, objectMsg.pos, '', '')
            #self.objdict[objectMsg.object_id] = o
            rospy.logwarn("New object detected (id %d) that is not in object list, ignoring object",
                    objectMsg.object_id)

    def objectRefCallback(self, objectRefMsg):
        """
        Send appropriate NVB command to the robot based on received object reference.
        """
        target = objectRefMsg.target_object_id
        target_loc = self.objdict[target]

        action_type = self.calculateNVBForRef(target)

        self.nao.doGesture(action_type, target_loc)

    def findNVBForRef(self, target):
        """
        Use NVB model to select appropriate NVB command based on the NVB model.
        """
        
        # ---- SALIENCY ----
        # Grab user view snapshot from webcam for saliency detection
        s,user_view_img = self.cam.read()
        if not s:
            raise IOException("Could not take camera image!")
        else:
            cv2.imwrite("userview.jpg",img) # save camera image

        
        # ---- VERBAL ----



        nvb = self.model.calculateNVBForRef(user_view_img, self.objdict, target)
        return nvb


class NVBModel():
    def __init__(self):
        pass

    def calculateNVBForRef(self, user_view, target, object_list):
        # Calculate saliency score
        saliency_scores = self.calculateSaliencyScores(user_view, object_list)

        # Calculate verbal reference score
        verbal_scores = self.calculateVerbalScores()

        # Calculate likelihood scores for all objects given current scene and different combinations
        # of available nonverbal behaviors (verbal only, verbal+gaze, verbal+gaze+gesture

        # Calculate the certainty score of the target object

        # Select NVB based on certainty score

    def calculateSaliencyScores(self, user_view_img, object_list):
        """
        Calculate the saliency of each object from the user's point of view.
        Takes an RGB image and a list of objects.
        Returns a list of saliencies by object.
        """
        salmap = saliency_detector.generateSaliencyMap(self.user_view_img)

        salscores = dict()
        for obj in object_list:
            # Find object position in rgb image by color
            # TODO
            salience = saliency_detector.identifySaliencyAtLocation(salmap, obj)
            salscores[obj.idnum] = salience

        return salscores

    def calculateVerbalScores(self, spoken_words, object_list)
        """
        Calculate the verbal reference score for each object, i.e.,
        the proportion of the spoken words that describe that object.
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


class Lego():
    def __init__(self, idnum, location, color, descriptor_words):
        """
        Location is a length 3 array of floats. Color is an RGB array. 
        Descriptor words is an array of words.
        """
        self.idnum = idnum
        self.loc = location
        self.color = color
        self.words = descriptor words


