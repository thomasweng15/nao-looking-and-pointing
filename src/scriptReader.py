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
from nao_looking_and_pointing.msg import ScriptObjectRef, Touch, Timer
from std_msgs.msg import String
from naoGestures import NaoGestures

class ScriptReader():

    def __init__(self, robot = None,
                       interruption = True):
        """
        Create the script reader.

        Arguments:
        robot -- a NaoGestures object
        interruption -- bool, should reading stop for an interruption?, defaults True
        """

        # Create publisher for object references
        self.object_reference_pub = rospy.Publisher(
            'script_object_reference', ScriptObjectRef)
        self.script_status = rospy.Publisher(
            'script_status', String)

        #rospy.init_node('script_reader', log_level=rospy.INFO)

        # Create listeners for the cues to wait for
        self.touch_listener = rospy.Subscriber(
            'touch',Touch,self.touchCallback)
        self.signal_listener = rospy.Subscriber(
            'timer_info',Timer,self.timerCallback)
        self.sysval_listener = rospy.Subscriber(
            'sysval_selections',String,self.sysvalCallback)

        # Initialize robot controller
        self.nao = robot

        # Interruption signal
        self.interrupt = interruption

        # Touch sensing
        self.bumper_touched = False
        self.head_touched = False

        # Signals to wait for
        self.timer_start = False
        self.timer_stop = False
        self.timer_reset = False
        self.sysval_done = False


    def readScript(self, script_filename):
        """
        Parse script and send speech commands to robot.

        Parses the interaction script. Sends speech commands to Nao with appropriate
        timings as determined by script. Broadcasts object references as ScriptObjectRef
        ROS messages at appropriate times as determined by script.

        Arguments: none

        Returns: none (but causes robot to speak)
        """
        rospy.loginfo("Beginning to read script " + script_filename)

        self.script = open(script_filename, 'r')

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
                        self.nao.speak(utterance, True) # blocking
                        utterance = ''

                        # Send the reference message
                        objectref = self.createObjRefMsg(reference)
                        if objectref:
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
            self.nao.speak(utterance, True)

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

        rospy.loginfo("Script command: " + commandstring)

        if commandstring == 'foot bumper press':
            # Listen for the next bumper press
            while not self.bumper_touched:
                sleep(0.5)
        elif commandstring == 'head press':
            # Listen for tactile head press
            while not self.head_touched:
                sleep(0.5)
        elif commandstring == 'sysval done':
            while not self.sysval_done:
                sleep(0.5)
        elif commandstring == 'timer start':
            while not self.timer_start:
                sleep(0.5)
        elif commandstring == 'timer stop':
            while not self.timer_stop:
                sleep(0.5)
        elif commandstring == 'timer reset':
            while not self.timer_reset:
                sleep(0.5)
        elif commandstring == 'timer start request':
            self.timer_start = False
        elif commandstring == 'timer stop request':
            self.timer_stop = False
        elif commandstring == 'timer reset request':
            self.script_status.publish("TimerReset")
            self.timer_reset = False
        elif commandstring == 'interruption start':
            if self.interrupt:
                # wait for head press to signal interruption start
                self.script_status.publish("InterruptionStart")
                while not self.head_touched:
                    sleep(0.5)
        elif commandstring == 'interruption stop':
            if self.interrupt:
                self.script_status.publish("InterruptionStop")
                # wait for head press to signal interruption stop
                while not self.head_touched:
                    sleep(0.5)
        elif commandstring == 'task 1':
            self.script_status.publish('Task1')
        elif commandstring == 'task 2':
            self.script_status.publish('Task2')
        elif (commandstring == 'instructions 1 start' 
                or commandstring == 'instructions 2 start'):
            # publish status
            self.script_status.publish("InstructionsStart")
        elif (commandstring == 'instructions 1 stop'
                or commandstring == 'instructions 2 stop'):
            # publish status
            self.script_status.publish("InstructionsStop")
        elif commandstring == 'reset blocks':
            self.script_status.publish("ResetBlocks")
        elif commandstring == 'hri construction':
            self.script_status.publish("HRIConstruction")
        elif commandstring == 'system validation':
            self.script_status.publish("SystemValidation")
        elif commandstring == 'timer practice start':
            self.script_status.publish("TimerPracticeStart")
        elif commandstring == 'timer practice stop':
            self.script_status.publish("TimerPracticeStop")
        else:
            # Sleep for the specified amount of time
            try:
                sleep(float(commandstring))
            except ValueError:
                rospy.logerr('Timing command is not a float: %s' % commandstring)
  
    def touchCallback(self, data):
        if data.touch_type == "Bumper":
            self.bumper_touched = True
            rospy.loginfo("Bumper press sensed")
        elif data.touch_type == "HeadTactile":
            self.head_touched = True
            rospy.loginfo("Head tap sensed")
        else:
            raise ValueError("Touch type not recognized: %s", data.touch_type)

    def timerCallback(self, data):
        if data.event == 'start':
            self.timer_start = True
            rospy.loginfo("Timer start signal")
        elif data.event == 'stop':
            self.timer_stop = True
            rospy.loginfo("Timer stop signal")
        elif data.event == 'reset':
            self.timer_reset = True
            rospy.loginfo("Timer reset signal")

    def sysvalCallback(self, data):
        if data.data == 'done':
            self.sysval_done = True