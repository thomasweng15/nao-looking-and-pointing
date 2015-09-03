#!/usr/bin/env python

"""
Author: Henny Admoni 
Date Created: 8/27/2015

Simple timer that sends a ROS message for each timer stop and start.
The ROS message is *not* for timer time, just start and stop events.
"""
# ROS imports
import rospy
from nao_looking_and_pointing.msg import Timer
from std_msgs.msg import String
# Gui imports
from Tkinter import *
import tkFont

class MyTimer():
	def __init__(self, label, starttime=0.0):
		self.t = starttime
		self.label = label
		self.running = True

	def tick(self):
		if self.running:
			self.t += 0.1
			output = "{:.1f}".format(self.t)
			self.label['text'] = output
			self.label.after(100, self.tick)

def startTimer():
	global timer, timepub
	if timer: 
		print "Only one timer instance permitted."
		return
	timer = MyTimer(time, curTime)
	# Send ROS message
	timepub.publish("start", curTime)
	timer.tick()

def stopTimer():
	global timer, curTime, timepub
	if timer:
		timer.running = False
		curTime = timer.t # for continuing time where it left off
		timer = None
		# Send ROS message
		timepub.publish("stop", curTime)

def clearTimer():
	global timer, curTime
	timepub.publish("reset", curTime)
	curTime = 0.0
	# Display 0.0 in Label time
	time['text'] = curTime

def statusCallback(msg):
	global startButton, stopButton, resetButton
	print "data received: " + str(msg.data)
	if msg.data == 'InstructionsStart':
		startButton['state'] = DISABLED
		stopButton['state'] = DISABLED
		resetButton['state'] = DISABLED
	elif msg.data == 'InstructionsStop':
		startButton['state'] = NORMAL
		stopButton['state'] = NORMAL
		resetButton['state'] = DISABLED
	elif msg.data == 'ResetBlocks':
		startButton['state'] = DISABLED
		stopButton['state'] = DISABLED
		resetButton['state'] = NORMAL
	elif msg.data == 'TimerPracticeStart':
		startButton['state'] = NORMAL
		stopButton['state'] = NORMAL
		resetButton['state'] = NORMAL
	elif msg.data == 'TimerPracticeStop':
		startButton['state'] = DISABLED
		stopButton['state'] = DISABLED
		resetButton['state'] = DISABLED




timer = None
enableButtons = False
curTime = 0.0

root = Tk()
root.geometry("800x800")
root.title('Timer')

# ROS publisher
timepub = rospy.Publisher('timer_info', Timer)
rospy.init_node("timer")

# ROS subscriber
naolistener = rospy.Subscriber('/script_status', String, statusCallback)

spacerFrame = Frame(root, width=200, height=200)

helvetica114 = tkFont.Font(family='Helvetica', size=114, weight='bold')
time = Label(root, font=helvetica114, text=curTime)
spacerFrame.pack()
time.pack()
spacerFrame.pack()


buttonFrame = Frame(root, width=200, height=400)
buttonFrame.pack()

helvetica36 = tkFont.Font(family='Helvetica', size=36, weight='bold')
startButton = Button(buttonFrame, bg='green', text='Start', 
	font=helvetica36, command=startTimer, state=DISABLED)
stopButton = Button(buttonFrame, bg='red', text='Stop', 
	font=helvetica36, command=stopTimer, state=DISABLED)
resetButton = Button(buttonFrame, bg='grey', font=helvetica36, 
	command=clearTimer, text="Reset", state=DISABLED)
startButton.pack(side=LEFT)
stopButton.pack(side=LEFT)
resetButton.pack(side=LEFT)

root.mainloop()