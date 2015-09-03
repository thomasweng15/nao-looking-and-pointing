#!/usr/bin/env python

"""
Author: Henny Admoni 
Date Created: 8/31/2015

Parse rosbags from NVB experiment. 
"""
import sys
import argparse
import glob
import re
import rosbag
import atexit


class RosbagParser():
	def __init__(self, userid=None, bagfname=None):

		atexit.register(self.shutdown)

		rosbagdir = '../rosbags/'

		if bagfname:
			userid = int(re.findall(r'\d+', bagfname)[0]) # first integer
		elif userid:
			potential_files = glob.glob(rosbagdir+'p'+str(userid)+'_*.bag')
			if len(potential_files) == 0:
				print("No file found for participant %d, exiting" % userid)
				sys.exit()
			elif len(potential_files) == 1:
				bagfname = potential_files[0]
			else:
				print("More than one bag file for this participant, "
					"try running with bag filename directly. Exiting.")
				sys.exit()
		else:
			raise NameError('Must provide one of bagfname or userid')

		self.bag = rosbag.Bag(bagfname)
		self.user = userid


	def getCondition(self):
		"""
		Report the NVB and Interruption condition.

		Returns a string "X,Y" where X={True,False} for 
		NVB and Y={True,False} for Interruption
		"""
		nvb = None
		interrupt = None

		for topic, msg, t in self.bag.read_messages(
			topics='script_status'):
			# Listen for and record NVB condition, which will
			# come in as "nvb:X" where X = {True, False}
			msgtokens = msg.data.split(':')
			if len(msgtokens) == 2:
				if msgtokens[0] == 'nvb':
					nvb = msgtokens[1]
				elif msgtokens[0] == 'interruption':
					interrupt = msgtokens[1]


		# Make sure we actually captured the data
		#assert nvb is not None
		#assert interrupt is not None

		datastring = str(nvb) + ',' + str(interrupt)
		return datastring

	def getCompletionTimes(self):
		""" 
		Report completion time for task 1 and 2.

		Returns a CSV with "time1,time2".
		"""

		times = []

		# flags and counters
		practiceDone = False
		tmpTime = 0.0


		for topic, msg, t in self.bag.read_messages(
			topics=['timer_info','script_status']):
			# Listen for and calculate total task time
			# for tasks 1 and 2
			if topic == 'script_status':
				if msg.data == 'TimerPracticeStop':
					practiceDone = True
			elif topic == 'timer_info' and practiceDone:
				if msg.event == 'start':
					tmpTime = msg.time
				elif msg.event == 'stop':
					totalTime = msg.time - tmpTime
					times.append(totalTime)
					totalTime = 0.0


		# Make sure all the data came through
		#assert len(times) == 2, "Expected 2 times, found %d" % len(times)

		strtimes = [str(t) for t in times]
		datastring = ','.join(strtimes)
		return datastring


	def getBlockTouchesInSysval(self):
		""" 
		Do people touch the correct block? And how quickly do they do so?

		Measures accuracy of block touches for different NVB actions, as 
		well as response times for *correct* block touches.

		Returns a list of accuracies and a list of RTs by block.
		"""
		print("----- System validation analysis -----")

		# Possible actions of the robot
		action_list = ['none','look','point','lookandpoint']

		# Time window for correct response in seconds
		t_window = 4.0

		# trackers
		start_time = None
		target_obj = None
		nvb_action = None

		# totals
		total_actions = dict((act,0.0) for act in action_list)
		accuracy = dict((act,0.0) for act in action_list)
		responsetime = dict((act,0.0) for act in action_list)

		# For each object reference, find whether people touch the
		# target object within t_window seconds of that reference.
		for topic, msg, t in self.bag.read_messages(
			topics=['human_behavior','robot_behavior','script_status']):
			if topic == 'script_status':
				if msg.data == 'HRIConstruction':
					break
			if topic == 'robot_behavior':
				# A new robot behavior was produced, so reset all
				# trackers (start_time, target_obj, nvb_action)
				start_time = t
				target_obj = msg.object_id
				nvb_action = msg.action
				total_actions[nvb_action] += 1
			elif (topic == 'human_behavior' 
				and target_obj is not None
				and t - start_time < t_window):
				# This human behavior followed a robot behavior
				# within the specified time window
				objlist = msg.target
				effectorlist = msg.effector
				assert len(objlist) > 0
				assert len(objlist) == len(effectorlist)
				for i in range(len(objlist)):
					if effectorlist[i] == 'leftarm' or \
						effectorlist[i] == 'rightarm':
						touch_obj = objlist[i]
						# If this is a successful touch, record the accuracy
						# and RTs and then reset all trackers (so we only save
						# the first such correct touch)
						if touch_obj == target_obj:
							assert nvb_action in action_list
							accuracy[nvb_action] += 1
							touch_time = t - start_time
							responsetime[nvb_action] += touch_time
							start_time = None
							target_obj = None
							nvb_action = None
							break # leave the for loop

		print "Raw data:"
		print "  total actions: " + str(total_actions)
		print "  accuracy: " + str(accuracy)
		print "  response times: " + str(responsetime)

		# Construct a data string that can be plugged into SPSS
		# format: none, look, point, lookandpoint
		accuracy_datastring = ''
		rt_datastring = ''

		# Calculate average accuracy and response time per NVB action
		for action in action_list:
			aveAcc = accuracy[action] / total_actions[action]
			aveRt = responsetime[action] / total_actions[action]
			accuracy_datastring += ', ' + str(aveAcc)
			rt_datastring += ', ' + str(aveRt)

		print "Calculated data:"
		print "  average accuracy: " + accuracy_datastring
		print "  average RTs: " + rt_datastring

		return [accuracy_datastring, rt_datastring]


	def printMessages(self, topicslist):

		for topic, msg, t in self.bag.read_messages(
			topics=topicslist):
			print '---' + str(t) + '---'
			print topic
			print msg

	def shutdown(self):
		if self.bag:
			self.bag.close()

if __name__ == '__main__':

	parser = argparse.ArgumentParser(description="Analyze data from NVB experiment.")
	parser.add_argument('--bag',
		help='the bag file to analyze')
	parser.add_argument('--user',
		help='participant number',
		type=int)

	args = parser.parse_args()
	userid = args.user
	bagfname = args.bag

	parser = RosbagParser(userid, bagfname)
	print parser.getCondition()
	print parser.getCompletionTimes()
	print parser.getBlockTouchesInSysval()