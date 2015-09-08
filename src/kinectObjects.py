#!/usr/bin/env python
import rospy
from lego import Lego
from kinect2_pointing_recognition.msg import ObjectsInfo


class KinectObjects():
    def __init__(self, convertToNaoFrame=True):
        self.objdict = dict()

        self.convert = convertToNaoFrame

        rospy.Subscriber('/objects_info', ObjectsInfo, self.objects_callback)

    def objects_callback(self, objectMsg):
        """
        Update object positions.

        Parse the ROS message (ObjectsInfo) and update the objects dictionary with
        new information about existing objects. If the object is not in the dictionary,
        create an entry for it.

        Arguments:
        objectMsg -- a ROS message of type ObjectsInfo
        convertToNaoFrame -- bool for convert coordinates from Kinect to Nao frame,
                            defaults to true

        Returns: none
        """

        obj_id = int(objectMsg.object_id)
        if self.convert:
            # convert from Kinect frame to Nao frame
            obj_pos = self.convertCoords(objectMsg.pos)
        else:
            obj_pos = objectMsg.pos

        if obj_id in self.objdict:
            # if the object location has changed, update it
            if not obj_pos == self.objdict[obj_id].loc:
                self.objdict[obj_id].loc = obj_pos
        else:
            # if this is a new object, add it to objdict
            o = Lego(obj_id,                # ID number
                     obj_pos,               # 3D position
                     objectMsg.color_upper, # upper RGB color threshold
                     objectMsg.color_lower, # lower RGB color threshold
                     '')                    # descriptor words
            self.objdict[obj_id] = o
            rospy.logdebug("Adding new object (id %d) to object list", obj_id)
 
    def convertCoords(self, kinectCoords):
        '''
        The Kinect and the Nao use define coordinate systems differently. 
        This function converts Kinect coordinates to Nao coordinates. 
        '''
        # originTranslationVector stores the displacement of the origin
        # in the Kinect coordinate system to the Nao coordinate system.
        originTranslationVector = [0, -0.4, 0] # hard-coded
        x = kinectCoords[2] - originTranslationVector[2]
        y = kinectCoords[0] - originTranslationVector[0]
        z = kinectCoords[1] - originTranslationVector[1]
        return [x,y,z]

    def createFakeObjects(self):
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