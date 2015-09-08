#!/usr/bin/env python

"""
Author: Henny Admoni 
Date Created: 9/2/2015

Analyze all data for NVB experiment within the directory. 
"""
import glob
import csv
import parseRosbags

qualtricsfile = ''

class QualtricsParser():
	def __init__(self, fname):
		self.f = open(fname,'r')
		self.csvreader = csv.reader(self.f)

		self.id_idx = 11 # index of participant id

	def getDataForUser(self, user):
		rowsToSkip = 2
		counter = 0
		""" Return the line of data tied to the user number indicated. """
		for row in self.csvreader:
			try:
				if int(row[self.id_idx]) == user:
					# reset the read head for the next go-round
					self.f.seek(0)
					# return the whole row minus last three entries (junk)
					return row[self.id_idx:-3]
			except ValueError:
				continue

		return None

# Create output files
f_sysAcc = open('result_sysval_acc.csv','w')
f_sysRt = open('result_sysval_rt.csv','w')
f_task = open('result_taskcomp.csv','w')
f_survey = open('result_survey.csv','w')

# Write headers to files
f_sysAcc.write('id,c1_a1,c1_a2,c1_a3,c1_a4,'
			   'c2_a1,c2_a2,c2_a3,c2_a4,'
			   'c3_a1,c3_a2,c3_a3,c3_a4\n')
f_sysRt.write('id,c1_a1,c1_a2,c1_a3,c1_a4,'
			   'c2_a1,c2_a2,c2_a3,c2_a4,'
			   'c3_a1,c3_a2,c3_a3,c3_a4\n')

# Keep count of how many bags are opened (for fun)
count = 0

# Qualtrics parser only needs to be opened once because
# there's only one Qualtrics file
qualtricsfile = '../results/Postinteraction_questionnaire.csv'
qParser = QualtricsParser(qualtricsfile)

# Iterate through all files in a given directory
filesToMatch = '../rosbags/p*.bag'
for fname in glob.glob(filesToMatch):
	print("Analyzing " + fname)

	# Create parsers for rosbag and Qualtrics files
	parser = parseRosbags.RosbagParser(bagfname=fname)

	# Get user ID and condition (nvb and interruption) info 
	user = parser.user
	cond = parser.getCondition()

	# Get task completion times
	taskTimes = parser.getCompletionTimes()
	f_task.write(str(user) + ',' + cond + ',' + taskTimes + '\n')

	# Get system validation information
	sysval_cond1 = [1,2,3,4]
	sysval_cond2 = [5,6]
	sysval_cond3 = [0,7]
	[acclist, rtlist] = parser.getSysvalSelections(
		sysval_cond1, sysval_cond2, sysval_cond3)
	assert len(acclist) == len(rtlist)

	# Write system validation information in useable format
	sysvalAccResult = str(user)
	for acc in acclist:
		sysvalAccResult += ',' + acc
	f_sysAcc.write(sysvalAccResult + '\n')
	sysvalRtResult = str(user)
	for rt in rtlist:
		sysvalRtResult += ',' + rt
	f_sysRt.write(sysvalRtResult + '\n')

	# Get subjective survey response data
	subjlist = qParser.getDataForUser(user)
	if subjlist:
		subjective = ','.join(subjlist)
		f_survey.write(str(user) + ',' + cond + ',' + subjective + '\n')
	else:
		print("No survey response for user %d" % user)


	count += 1

f_sysAcc.close()
f_sysRt.close()
f_task.close()
f_survey.close()

print(str(count) + " rosbags analyzed.")