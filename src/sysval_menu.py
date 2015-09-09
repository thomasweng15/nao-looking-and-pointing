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
root.geometry("500x800")
root.title('Calibration Task')
root.config(cursor='none') # no cursor


# ROS publisher
pub = rospy.Publisher('/sysval_selections', String)
rospy.init_node("sysval_menu")

# ROS subscriber
naolistener = rospy.Subscriber('/script_status', String, statusCallback)

# Spacer
spacerFrame = Frame(root, width=500, height=40)

# Selection buttons
helvetica36 = tkFont.Font(family='Helvetica', size=36, weight='bold')
red1Button = Button(root, bg='red', text='red block (my right)', 
	font=helvetica36, command= lambda: buttonPressCallback(0), state=DISABLED)
red2Button = Button(root, bg='red', text='red block (my left)', 
	font=helvetica36, command= lambda: buttonPressCallback(7), state=DISABLED)
orangeButton = Button(root, bg='orange', text='orange block', 
	font=helvetica36, command= lambda: buttonPressCallback(1), state=DISABLED)
yellowButton = Button(root, bg='yellow', text='yellow block', 
	font=helvetica36, command= lambda: buttonPressCallback(2), state=DISABLED)
limeButton = Button(root, bg='#99FF00', text='lime block', 
	font=helvetica36, command= lambda: buttonPressCallback(3), state=DISABLED)
greenButton = Button(root, bg='#009900', text='green block', 
	font=helvetica36, command= lambda: buttonPressCallback(4), state=DISABLED)
blue1Button = Button(root, bg='#3B9C9C', text='blue block (my right)', 
	font=helvetica36, command= lambda: buttonPressCallback(5), state=DISABLED)
blue2Button = Button(root, bg='#3B9C9C', text='blue block (my left)', 
	font=helvetica36, command= lambda: buttonPressCallback(6), state=DISABLED)
doneButton = Button(root, bg='grey', text='done', 
	font=helvetica36, command=close, state=DISABLED)

buttonlist = [red1Button, red2Button, orangeButton,yellowButton, limeButton, \
				greenButton, blue1Button, blue2Button]

for button in buttonlist:
	button.pack(fill=X)

spacerFrame.pack()
doneButton.pack(fill=X)

root.mainloop()