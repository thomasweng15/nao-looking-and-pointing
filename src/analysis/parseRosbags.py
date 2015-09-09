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
import rospy


class RosbagParser():
	def __init__(self, userid=None, bagfname=None):

		atexit.register(self.shutdown)

		rospy.init_node('rosbag_parser')

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
		assert nvb is not None
		assert interrupt is not None

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


	def getSysvalSelections(self, cond1objs, cond2objs, cond3objs):
		""" 
		Do people select the correct block? And how quickly do they do so?

		Measures accuracy of block selections for different NVB actions, as 
		well as response times for *correct* block touches.

		Splits the analysis over three conditions:
			condition 1: unambiguous reference
			condition 2: ambiguous, far apart
			condition 3: ambiguous, close together

		Note: only stores the first selection in response to a robot behavior.

		Arguments:
		cond1objs -- list of IDs for blocks that fall into condition 1
		cond2objs -- list of IDs for blocks that fall into condition 2
		cond3objs -- list of IDs for blocks that fall into condition 3

		Returns a list of lists of accuracies and a list of lists of RTs 
		for each block by condition. I.e., 
			[[cond1acc, cond2acc, cond3acc], [cond1rt, cond2rt, cond3rt]]
		"""

		# Possible actions of the robot
		action_list = ['none','look','point','lookandpoint']

		# Time window for correct response in seconds
		t_window = rospy.Duration(5)

		# trackers
		start_time = None
		target_obj = None
		nvb_action = None

		# totals
		total_actions = dict((act,0.0) for act in action_list)
		accuracy = dict((act,0.0) for act in action_list)
		responsetime = dict((act,rospy.Duration(0)) for act in action_list)

		# separate by condition
		cond1 = {'total':dict(total_actions), 'accuracy':dict(accuracy), 'rt':dict(responsetime)}
		cond2 = {'total':dict(total_actions), 'accuracy':dict(accuracy), 'rt':dict(responsetime)}
		cond3 = {'total':dict(total_actions), 'accuracy':dict(accuracy), 'rt':dict(responsetime)}
		
		# For each object reference, find whether people selected the
		# target object within t_window seconds of that reference.
		for topic, msg, t in self.bag.read_messages(
			topics=['sysval_selections','robot_behavior','script_status']):
			if topic == 'script_status':
				if msg.data == 'HRIConstruction':
					break
			if topic == 'robot_behavior':
				# A new robot behavior was produced, so reset all
				# trackers (start_time, target_obj, nvb_action)
				start_time = t
				target_obj = msg.object_id
				nvb_action = msg.action
				if target_obj in cond1objs:
					cond1['total'][nvb_action] += 1
				elif target_obj in cond2objs:
					cond2['total'][nvb_action] += 1
				elif target_obj in cond3objs:
					cond3['total'][nvb_action] += 1
				else:
					rospy.logwarn("System validation: ID not in condition list: " 
						+ str(target_obj))
			elif (topic == 'sysval_selections' 
				and target_obj is not None
				and (t - start_time) < t_window):
				# This selection followed a robot behavior
				# within the specified time window
				selection = int(msg.data)
				if selection == target_obj:
					assert nvb_action in action_list
					if selection in cond1objs:
						condlist = cond1
					elif selection in cond2objs:
						condlist = cond2
					elif selection in cond3objs:
						condlist = cond3
					else:
						rospy.logwarn("System validation: ID not in condition list: " 
						+ str(selection))
					condlist['accuracy'][nvb_action] += 1
					touch_time = t - start_time
					condlist['rt'][nvb_action] += touch_time
					# A response was sensed, so reset all trackers
					start_time = None
					target_obj = None
					nvb_action = None

		# print "Raw data:"
		# print "  cond1 total actions: " + str(cond1['total'])
		# print "  cond1 accuracy: " + str(cond1['accuracy'])
		# print "  cond1 RTs: " + str(cond1['rt'])
		# print "  cond2 total actions: " + str(cond2['total'])
		# print "  cond2 accuracy: " + str(cond2['accuracy'])
		# print "  cond2 RTs: " + str(cond2['rt'])
		# print "  cond3 total actions: " + str(cond3['total'])
		# print "  cond3 accuracy: " + str(cond3['accuracy'])
		# print "  cond3 RTs: " + str(cond3['rt'])

		# Construct a data string that can be plugged into SPSS
		# format: none, look, point, lookandpoint
		accuracy_datastrings = [''] * 3
		rt_datastrings = [''] * 3

		# Calculate average accuracy and response time per NVB action
		accuracyStringIdx = 0
		rtStringIdx = 0
		for c in [cond1, cond2, cond3]:
			for action in action_list:
				aveAcc = c['accuracy'][action] / c['total'][action]
				aveRt = c['rt'][action] / c['total'][action]
				accuracy_datastrings[accuracyStringIdx] += str(aveAcc) + ','
				rt_datastrings[rtStringIdx] += str(aveRt.to_sec()) + ','

			# Delete the trailing comma
			accuracy_datastrings[accuracyStringIdx] = \
				accuracy_datastrings[accuracyStringIdx][:-1]
			rt_datastrings[rtStringIdx] = rt_datastrings[rtStringIdx][:-1]

			accuracyStringIdx += 1
			rtStringIdx += 1

		# print "Calculated data:"
		# print "  cond1 ave acc: " + accuracy_datastrings[0]
		# print "  cond1 ave RTs: " + rt_datastrings[0]
		# print "  cond2 ave acc: " + accuracy_datastrings[1]
		# print "  cond2 ave RTs: " + rt_datastrings[1]
		# print "  cond3 ave acc: " + accuracy_datastrings[2]
		# print "  cond3 ave RTs: " + rt_datastrings[2]

		return [accuracy_datastrings, rt_datastrings]


	def printMessages(self, topicslist):

		for topic, msg, t in self.bag.read_messages(
			topics=topicslist):
			print '---' + str(t) + '---'
			print topic
			print msg

	def shutdown(self):
		if hasattr(self,'bag'):
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
	print parser.getSysvalSelections([1,2,3,4],[5,6],[0,7])