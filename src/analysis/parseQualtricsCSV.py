#!/usr/bin/env python

"""
Author: Henny Admoni 
Date Created: 9/2/2015

Parse Qualtrics survey data for HRI experiment.
"""
import csv
import argparse


def parsefile(infile, outfile):
	""" Parse the given csv file into SPSS format. """
	
	# CSV template
	dataStart = 10 # index of the first piece of data we care about

	# Create a csv reader
	with open(infile, 'rb') as f:
		reader = csv.reader(f)
		for row in reader:
			print row



if __name__ == '__main__':
	parser = argparse.ArgumentParser(description="Parse a Qualtrics CSV file.")
	parser.add_argument('input',
		help='the csv file to parse')
	parser.add_argument('output',
		help='the output file name')

	args = parser.parse_args()
	fname = args.input
	outfile = args.output

	parsefile(fname, outfile)