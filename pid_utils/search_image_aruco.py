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
import yaml

#Opencv Imports
from cv2 import aruco
import cv2
import numpy as np


from aruco_utils import rgb2gray

#--------------- ARUCO TAG  INIT SECTION -------------#
aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_6X6_1000 )

arucoParams = aruco.DetectorParameters_create()

with open('calibration.yaml') as f:
    print("LOADED")
    loadeddict = yaml.load(f)

mtx = loadeddict.get('camera_matrix')
dist = loadeddict.get('dist_coeff')
mtx = np.array(mtx)
dist = np.array(dist)

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
	corners, ids, rejectedImgPoints = aruco.detectMarkers(im_gray, aruco_dict, parameters=arucoParams)

	img_aruco = img.copy()
	if len(corners)==0:
		stop = current_milli_time()
		child_conn.send((stop-start, None, None))
	else:
		img_aruco = aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))
		try:
			if ids == None:
				img_aruco = image.copy()
				return
		except:
			pass
		corners_dict={}
		for corner, id1 in zip(corners,ids):
			markerLength = 0
			if id1[0]==20: 
				markerLength=14.6
			if id1[0]==21: 
				markerLength=29.2    
			if id1[0]==22: 
				markerLength=58.5
			if id1[0]==23:                         
				markerLength=117.8 

			corners_dict[str(id1[0])]={
				"corner":corner[0],
				"markerLength":markerLength
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

	print("Detected :",len(corners), "targets")



if __name__ == "__main__":
	print("In search_image")