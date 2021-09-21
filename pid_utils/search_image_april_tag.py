'''

    Synopsis: Script to analyze frames for presence of target.
    Author: Nikhil Venkatesh
    Contact: mailto:nikv96@gmail.com

'''

#Python Imports
import urllib
import os
import math
import time
from copy import copy

import apriltag
import numpy as np


from aruco_utils import rgb2gray

#--------------- APRIL TAG  INIT SECTION -------------#
# define the AprilTags detector options and then detect the AprilTags
# in the input image
print("[INFO] detecting AprilTags...")
options = apriltag.DetectorOptions(families="tag25h9")
detector = apriltag.Detector(options)


#Global Variables
hres = 640
vres = 480
vfov = 48.7
hfov = 49.7

current_milli_time = lambda: int(round(time.time() * 1000))

def analyze_frame(child_conn, img, location, attitude, priorized_tag, priorized_tag_counter):

	start = current_milli_time()
	im_gray = rgb2gray(img).astype(np.uint8)
	h,  w = im_gray.shape[:2]
	results = detector.detect(im_gray)

	if len(results)==0:
		stop = current_milli_time()
		child_conn.send((stop-start, None, None,priorized_tag_counter, priorized_tag))
	else:
	
		corners_dict={}
		for r in results:
		
		
			corners_dict[str(r.tag_id)]={
				"corner":r.corners,
			}
					
		smaller_tag_id = min(corners_dict.keys())
		if smaller_tag_id not in priorized_tag_counter.keys():
			priorized_tag_counter[smaller_tag_id]=0
			
		if priorized_tag_counter[smaller_tag_id] <10:
			priorized_tag_counter[smaller_tag_id]=priorized_tag_counter[smaller_tag_id]+1


		min_tag_detected=min(priorized_tag_counter.keys())
		if (priorized_tag_counter[min_tag_detected] >=10 and priorized_tag >min_tag_detected) or priorized_tag==0:
			priorized_tag=min_tag_detected
		
		

		if smaller_tag_id != priorized_tag:
			
			stop = current_milli_time()
			child_conn.send((stop-start, None, None, priorized_tag_counter, priorized_tag))
			return
		
		
		target = corners_dict[smaller_tag_id]["corner"]
	
	

		max_term = np.amax(target,axis=0)
		min_term = np.amin(target,axis=0)

		x = min_term[0]
		y = min_term[1]
		w = max_term[0]-x
		h = max_term[1]-y

		x_true = x + w/2.0 - hres/2.0
		y_true = -(y + h/2.0) + vres/2.0
		center = (x_true, y_true)
		stop = current_milli_time()
		child_conn.send((stop-start, center, target, priorized_tag_counter, priorized_tag))

	print("Detected :",len(results), "targets")



if __name__ == "__main__":
	print("In search_image")