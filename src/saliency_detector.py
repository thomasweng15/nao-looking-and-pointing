#!/usr/bin/env python

"""
Saliency detector. 

Uses established CVPR algorithm to detect saliency of an image.
Extracts saliency of specific objects.

Author: Henny Admoni

Date: July 30, 2015


Dont forget to cite: Eleonora Vig, Michael Dorr, David Cox. "Large-Scale Optimization of Hierarchical Features for Saliency Prediction in Natural Images," IEEE Computer Vision and Pattern Recognition (CVPR), 2014.
"""

import numpy as np
import eDNsaliency as saliency


def generateSaliencyMap(rawimgfile):
    """
    Create a saliency map using the CVPR algorithm. 
    Returns a greyscale image where pixel value indicates saliency, and
    saves this greyscale image in the local directory.
    """
    optparser = saliency.get_optparser()
    opts, args = optparser.parse_args([''])
    outimgfile = 'salmap_' + rawimgfile

    # This returns a numpy array of arrays, where each row
    # represents a row of pixels in the image.
    salimg = saliency.eDNsaliency(rawimgfile, outimgfile, opts)

    return salimg

def identifySaliencyAtLocation(salimg, pixelidxarray):
    """
    Returns the average saliency of the indicated pixels in the raw image.
    Takes a saliency map and an array of pixel indices as (row,col) tuples.
    """       
    npixels = len(pixelidxarray)

    if npixels > 0:
        salpixels = [salimg[r][c] for r,c in pixelidxarray]
        meansalience = sum(salpixels) / npixels

        return meansalience
    else:
        return 0
      
     
if __name__ == '__main__':
    print identifySaliencyAtLocation('test.jpg',[(0,0),(0,25)])
