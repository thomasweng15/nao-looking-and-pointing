#!/usr/bin/env python

"""
Author: Henny Admoni 
Date Created: 9/9/2015

Automatically extract color threshold ranges from saliency image.
With thanks to opencv-python-tutroals.readthedocs.org [sic]
"""

import glob
import cv2
import numpy as np
from matplotlib import pyplot as plt

# Read in saliency camera image
img = cv2.imread('userview.jpg')

# Pixel locations of each color in the image
mask_dict = {
	'red':[164,186,537,580],\
	'orange':[234,319,86,171],\
	'yellow':[126,151,234,278],\
	'lime':[114,194,366,449],\
	'green':[271,292,262,283],\
	'blue':[116,140,82,127]
}

thresh = 10 # min num of pixels in the image of this color

# Output
minRGBVals = {}
maxRGBVals = {}

for color, pixels in mask_dict.items():
	mask = np.zeros(img.shape[:2], np.uint8)
	mask[pixels[0]:pixels[1], pixels[2]:pixels[3]] = 255 # paint the mask white
	masked_img = cv2.bitwise_and(img, img, mask=mask)

	print(color + " block")

	# Find color upper and lower thresholds based on histogram
	clrs = ('b','g','r')
	hist = [0] * len(clrs)
	minRGBVals[color] = [0] * len(clrs)
	maxRGBVals[color] = [0] * len(clrs)

	for i in range(len(clrs)): # for each channel
		# Calculate histogram
		curhist = cv2.calcHist([img],[i],mask,[256],[0,256])
		hist[i] = curhist

		# Find threshold values
		minThresh = float("inf")
		maxThresh = float(0)
		for p in range(len(curhist)):
			if curhist[p] > thresh:
				if p < minThresh:
					minThresh = p
				if p > maxThresh:
					maxThresh = p

		# Record threshold values as BGR for now!
		minRGBVals[color][i] = minThresh
		maxRGBVals[color][i] = maxThresh

		# print("channel: " + clrs[i] 
		# 	+ ", min: " + str(minThresh)
		# 	+ ", max: " + str(maxThresh))

	# Swap B and R so we return an RGB, not a BGR
	minRGBVals[color][0], minRGBVals[color][2] = \
		minRGBVals[color][2], minRGBVals[color][0]
	maxRGBVals[color][0], maxRGBVals[color][2] = \
		maxRGBVals[color][2], maxRGBVals[color][0]

	# Print threshold values in useable format
	print(str(minRGBVals[color]) + ',' + str(maxRGBVals[color]))
	print("---------")

	# Show the histograms
	plt.subplot(221), plt.imshow(masked_img)
	plt.subplot(222), plt.plot(hist[0],color=clrs[0])
	plt.subplot(223), plt.plot(hist[1],color=clrs[1])
	plt.subplot(224), plt.plot(hist[2],color=clrs[2])
	#plt.show()


	