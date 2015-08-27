#!/usr/bin/env python

"""
Author: Henny Admoni 
Date Created: 8/14/2015

This module is for reading a predefined script. It requires a connection to a Nao robot.
It sends Nao speech commands at timings specified in the script, and sends ScriptObjectRef
ROS messages when objects are referenced as defined in the script. 
"""
import re
import rospy
from time import sleep
from nao_looking_and_pointing.msg import ScriptObjectRef, Touch
from naoGestures import NaoGestures

class ScriptReader():

    def __init__(self, script_filename, 
                       robotIP = None, 
                       robotPort = None, 
                       refs_on = True):
        """
        Create the script reader.

        Arguments:
        script_filename -- string that indicates the filename of the script to be read
        robotIP -- Nao's IP address, defaults to None
        robotPort -- Nao's port, defaults to None
        refs_on -- bool that determines whether references get published, defaults to True
        """

        # Create publisher for object references
        self.object_reference_pub = rospy.Publisher(
            'script_object_reference', ScriptObjectRef)
        #rospy.init_node('script_reader')

        # Create listeners for bumper press and tactile feedback
        self.touch_listener = rospy.Subscriber(
            'touch',Touch,self.touchCallback)

        # Script of the interaction
        self.script = open(script_filename, 'r')

        # Initialize robot controller
        self.nao = NaoGestures(robotIP, robotPort)

        # Boolean that determines whether object reference messages get published
        self.refs_on = refs_on

        # Touch sensing
        self.bumper_touched = False
        self.head_touched = False


    def readScript(self):
        """
        Parse script and send speech commands to robot.

        Parses the interaction script. Sends speech commands to Nao with appropriate
        timings as determined by script. Broadcasts object references as ScriptObjectRef
        ROS messages at appropriate times as determined by script.

        Arguments: none

        Returns: none (but causes robot to speak)
        """
        rospy.loginfo("Beginning to read script")

        alphanumeric = re.compile('[\W_]+')

        for line in self.script:
            utterance = ''
            reference = ''
            timing = '' # a timing command
            inreference = False # is this char part of the reference?
            intiming = False # is this char part of a timing cue?
            # Parse any action commands
            for char in line:
                # Allow for comments
                if char == '#':
                    break # go to next line
                # Find object references (bracketed with "<" and ">")
                if char == "<" and not intiming:
                    inreference = True
                    continue
                if char == "[" and not inreference:
                    intiming = True
                    continue
                if inreference:
                    # Read until the next '>'
                    if not char == '>': 
                        reference = reference + char
                    else:
                        # Speak the utterance to this point
                        self.nao.speak(utterance)
                        utterance = ''

                        # Send the reference message
                        objectref = self.createObjRefMsg(reference)
                        if self.refs_on and objectref:
                            self.object_reference_pub.publish(objectref)
                        
                        # Reset to be out of reference state
                        reference = ''
                        inreference = False
                elif intiming:
                    # A timing command affects the timing of the next utterance
                    # Read until the next ']'
                    if not char == ']':
                        timing = timing + char
                    else:
                        self.performTimingCommand(timing)
                        
                        # Reset to be out of timing state
                        timing = ''
                        intiming = False
                # Extract utterances
                else:
                    utterance = utterance + char

            # Speak what's left of the utterance
            self.nao.speak(utterance)

    def createObjRefMsg(self, ref):
        """
        Creates a ScriptObjectRef.msg message from the information provided.

        The reference provided should be formatted as such:
            <n> <set of words>
        where <n> is the object ID number and <set of words> is the set of
        descriptor words used to refer to that object.

        Arguments:
        ref -- a string containing an object reference

        Returns: a ScriptObjectRef message
        """
        words = ref.split()

        # Construct the ScriptObjectRef message
        objref = ScriptObjectRef()

        # Make sure the first value is a number
        try:
            int(words[0])
        except ValueError:
            rospy.logerr('Reference in script improperly formatted. \
                    Expected int as first word, found: %s', words[0])
            return False

        objref.object_id = int(words[0])

        if not words[1:]:
            rospy.logerr('Reference in script improperly formatted. \
                Each reference must contain some words (use "test" as throwaway.)')
            return False
        objref.words = ' '.join(words[1:])

        rospy.loginfo("Sending script reference message: " + str(objref))
        return objref

    def performTimingCommand(self, commandstring):
        """
        Parse the timing command and perform the specified action.

        Arguments:
        commandstring -- a string containing the timing command

        Returns: none (but affects robot behavior)
        """
        self.bumper_touched = False
        self.head_touched = False
        if commandstring == 'foot bumper press':
            # Listen for the next bumper press
            while not self.bumper_touched:
                pass
        elif commandstring == 'head press':
            # Listen for tactile head press
            while not self.head_touched:
                pass
        else:
            # Sleep for the specified amount of time
            try:
                sleep(float(commandstring))
            except ValueError:
                rospy.logerr('Timing command is not a float: %s',timing)
  
    def touchCallback(self, data):
        if data.touch_type == "Bumper":
            self.bumper_touched = True
            rospy.loginfo("Bumper press sensed")
        elif data.touch_type == "HeadTactile":
            self.head_touched = True
            rospy.loginfo("Head tap sensed")
        else:
            raise ValueError("Touch type not recognized: %s", data.touch_type)