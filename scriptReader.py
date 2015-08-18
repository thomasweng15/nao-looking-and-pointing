#!/usr/bin/env python

"""
Author: Henny Admoni 
Date Created: 8/14/2015

This module is for reading a predefined script. It requires a connection to a Nao robot.
It sends Nao speech commands at timings specified in the script, and sends ScriptObjectRef
ROS messages when objects are referenced as defined in the script. 
"""
import re
from nao-looking-and-pointing.msg import ScriptObjectRef
from naoGestures import NaoGestures

class ScriptReader():

    def __init__(self, script_filename, refs_on = True):
        """
        Create the script reader.

        Arguments:
        script_filename -- a string that indicates the filename of the script to be read
        refs_on -- boolean that determines whether references get published, defaults to True
        """

        # Create publisher for object references
        self.object_reference_pub = rospy.Publisher('script_object_reference', ScriptObjectRef)

        # Script of the interaction
        self.script = open(script_filename, 'r')

        # Initialize robot controller
        self.nao = NaoGestures()

        # Boolean that determines whether object reference messages get published
        self.refs_on = refs_on


    def readScript(self):
        """
        Parse script and send speech commands to robot.

        Parses the interaction script. Sends speech commands to Nao with appropriate
        timings as determined by script. Broadcasts object references as ScriptObjectRef
        ROS messages at appropriate times as determined by script.

        Arguments: none

        Returns: none (but causes robot to speak)
        """

        alphanumeric = re.compile('[\W_]+')

        for line in self.script:
            utterance = ''
            reference = ''
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
                        if self.refs_on:
                            self.object_reference_pub.publish(objectref)
                        reference = ''

                        inreference = False
                elif intiming:
                    # A timing command will cause the robot to sleep
                    timing = '' # a timing command
                    # Read until the next ']'
                    if not char == ']':
                        timing = timing + char
                    else:
                        try:
                            sleep(float(timing))
                        except ValueError:
                            rospy.logerror('Timing command is not a float: %s',timing)
                        intiming = False
                # Extract utterances
                else:
                    utterance = utterance + char

            # Speak what's left of the utterance
            self.nao.speak(utterance)
            print "utterance: " + utterance

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
        
        # Make sure the first value is a number
        try:
            int(words[0])
        except ValueError:
            rospy.logerror('Reference in script improperly formatted. \
                    Expected int as first word, found %s', words[0])

        # Construct the ScriptObjectRef message
        objref = ScriptObjectRef()
        objref.object_id = str(words[0])
        objref.words = ' '.join(words[1:])

        return objref

#TODO include functionality to "pause" the script until the user taps the robot's head? Could be used for going between segments of one session.
