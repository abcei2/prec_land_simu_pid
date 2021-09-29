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


DETECTIONS_TO_ENGAGE = 5
#--------------- ARUCO TAG  INIT SECTION -------------#
aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_6X6_1000 )
#aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_APRILTAG_16h5  )

arucoParams = aruco.DetectorParameters_create()

with open(os.environ.get('PL_CALIBRATION_FILE', 'aruco_utils/calibration.yaml')) as f:
    print("LOADED")
    loadeddict = yaml.load(f)

mtx = loadeddict.get('camera_matrix')
dist = loadeddict.get('dist_coeff')
mtx = np.array(mtx)
dist = np.array(dist)

#Global Variables
hres = 640
vres = 480
vfov = 60
hfov = 80

current_milli_time = lambda: int(round(time.time() * 1000))

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])
#180 deg rotation matrix around the x axis
R_flip = np.zeros((3,3),dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] = -1.0
R_flip[2,2] = -1.0


def analyze_frame(img, location, attitude, priorized_tag, priorized_tag_counter,forensic_message):

	center = None
	target = None
	yaw_correction = None

	start = current_milli_time()
	im_gray = rgb2gray(img).astype(np.uint8)
	h,  w = im_gray.shape[:2]
	corners, ids, rejectedImgPoints = aruco.detectMarkers(im_gray, aruco_dict, parameters=arucoParams)

	img_aruco = img.copy()
	if len(corners)==0:
		stop = current_milli_time()
		forensic_message["marker_stats"]["some_tag_detected"]=False
		return stop-start, center, target, priorized_tag_counter, priorized_tag, yaw_correction
		
	else:
		img_aruco = aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))
		forensic_message["marker_stats"]["corners"]=corners
		forensic_message["marker_stats"]["ids"]=ids
		forensic_message["marker_stats"]["some_tag_detected"]=True
	
		corners_dict={}
		for corner, id1 in zip(corners,ids):
			markerLength = 14.6
	
			if id1[0]==11: 
				markerLength=14.2
			elif id1[0]==16: 
				markerLength=6.6
			elif id1[0]==19: 
				markerLength=3.3
			elif id1[0]==23: 
				markerLength=1.5
			corners_dict[str(id1[0])]={
				"corner":corner[0],
				"markerLength":markerLength
			}
					
		smaller_tag_id = min(corners_dict.keys())
		rvec, tvec, _obj_points = aruco.estimatePoseSingleMarkers([corners_dict[smaller_tag_id]["corner"]],corners_dict[smaller_tag_id]["markerLength"], mtx, dist)
		# Obtain the rotation matrix tag -> camera
		attitude_vec=np.asarray([attitude.roll,attitude.pitch,attitude.yaw])

		Attitude_ct = np.matrix(cv2.Rodrigues(attitude_vec)[0])
		Attitude_tc = Attitude_ct.T

		R_ct = np.matrix(cv2.Rodrigues(rvec[0])[0])
		R_tc = R_ct.T

		#-- Get the attitude in terms of euler 321 (Needs to be flipped first)
		roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)
		att_roll, att_pitch, att_yaw = rotationMatrixToEulerAngles(R_flip*Attitude_tc)
	
		
		
		if smaller_tag_id not in priorized_tag_counter.keys():
			priorized_tag_counter[smaller_tag_id]=0
		if priorized_tag_counter[smaller_tag_id] <DETECTIONS_TO_ENGAGE:
			priorized_tag_counter[smaller_tag_id]=priorized_tag_counter[smaller_tag_id]+1

		min_tag_detected=min([int(a) for a in priorized_tag_counter.keys()])
		if (priorized_tag_counter[str(min_tag_detected)] >=DETECTIONS_TO_ENGAGE and priorized_tag >min_tag_detected) or priorized_tag==0:
			priorized_tag=min_tag_detected
		
	
		

		if smaller_tag_id != str(priorized_tag):
			
			stop = current_milli_time()
			return stop-start, center, target, priorized_tag_counter, priorized_tag, yaw_correction
			
		
		
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
		yaw_correction=math.degrees(att_yaw-yaw_marker)-math.degrees(att_yaw)

	return stop-start, center, target, priorized_tag_counter, priorized_tag, yaw_correction




if __name__ == "__main__":
	print("In search_image")