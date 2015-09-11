#!/usr/bin/env python

"""
Author: Henny Admoni 
Date Created: 9/7/2015

Menu for selecting objects for the system validation task.

"""
# ROS imports
import rospy
from std_msgs.msg import String
# Gui imports
from Tkinter import *
import tkFont

def buttonPressCallback(button):
	pub.publish(str(button))

def statusCallback(msg):
	if msg.data == "SystemValidation":
		for button in buttonlist:
			button['state'] = NORMAL
	elif msg.data == "SystemValidationComplete":
		doneButton['state'] = NORMAL
		for button in buttonlist:
			button['state'] = DISABLED

def close():
	pub.publish('done')
	root.destroy()

root = Tk()
root.geometry("1400x800")
root.title('Calibration Task')
#root.config(cursor='none') # no cursor


# ROS publisher
pub = rospy.Publisher('/sysval_selections', String)
rospy.init_node("sysval_menu")

# ROS subscriber
naolistener = rospy.Subscriber('/script_status', String, statusCallback)

# Spacer
spacerFrame = Frame(root, width=500, height=40)

# Selection buttons
buttonfont = tkFont.Font(family='Helvetica', size=24, weight='bold')
red1Button = Button(root, bg='red', text='red', 
	font=buttonfont, command= lambda: buttonPressCallback(0), state=DISABLED,
	height=2, width=8, wraplength=50)
red2Button = Button(root, bg='red', text='red', 
	font=buttonfont, command= lambda: buttonPressCallback(7), state=DISABLED,
	height=2, width=8, wraplength=50)
orangeButton = Button(root, bg='orange', text='orange', 
	font=buttonfont, command= lambda: buttonPressCallback(1), state=DISABLED,
	height=6, width=12)
yellowButton = Button(root, bg='yellow', text='yellow', 
	font=buttonfont, command= lambda: buttonPressCallback(2), state=DISABLED,
	height=2, width=8)
limeButton = Button(root, bg='#99FF00', text='lime', 
	font=buttonfont, command= lambda: buttonPressCallback(3), state=DISABLED,
	height=6, width=12)
greenButton = Button(root, bg='#009900', text='green', 
	font=buttonfont, command= lambda: buttonPressCallback(4), state=DISABLED,
	height=2, width=4)
blue1Button = Button(root, bg='#3B9C9C', text='blue', 
	font=buttonfont, command= lambda: buttonPressCallback(5), state=DISABLED,
	height=2, width=8)
blue2Button = Button(root, bg='#3B9C9C', text='blue', 
	font=buttonfont, command= lambda: buttonPressCallback(6), state=DISABLED,
	height=2, width=8)
doneButton = Button(root, bg='grey', text='done', 
	font=buttonfont, command=close, state=DISABLED, height=1, width=8)

buttonlist = [red1Button, red2Button, orangeButton,yellowButton, limeButton, \
				greenButton, blue1Button, blue2Button]

# for button in buttonlist:
# 	button.pack(fill=X)
# Lay out buttons according to real world spatial arrangement
red1Button.grid(row=3,column=7)
red2Button.grid(row=3,column=5)
orangeButton.grid(row=1,column=4)
yellowButton.grid(row=5,column=3)
limeButton.grid(row=1,column=8)
greenButton.grid(row=1,column=1)
blue1Button.grid(row=5,column=8)
blue2Button.grid(row=5,column=1)

rowspacer0 = Label(root,text='',height=2, width=10)
rowspacer2 = Label(root,text='',height=2, width=10)
rowspacer4 = Label(root,text='',height=2, width=10)
rowspacer6 = Label(root,text='',height=7, width=10)
rowspacer0.grid(row=0,column=0,columnspan=5)
rowspacer2.grid(row=2,column=0,columnspan=5)
rowspacer4.grid(row=4,column=0,columnspan=5)
rowspacer6.grid(row=6,column=0,columnspan=5)

colspacer0 = Label(root,text='',height=10, width=8)
colspacer2 = Label(root,text='',height=10, width=2)
colspacer6 = Label(root,text='',height=10, width=2)
colspacer0.grid(column=0,row=0,rowspan=6)
colspacer2.grid(column=2,row=0,rowspan=6)
colspacer6.grid(column=6,row=0,rowspan=6)

# spacerFrame.pack()
doneButton.grid(row=7,column=8)

root.mainloop()