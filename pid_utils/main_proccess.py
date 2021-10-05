import math
from dronekit import VehicleMode
import time 

import pid_utils.control as control
import pid_utils.search_image_aruco as search_image_aruco

from pid_utils.flight_assist import send_velocity
from pid_utils.flight_assist import condition_yaw
from pid_utils.system_info import draw_info

from pid_utils.utils import measure_distance



def procces_frame(frame,forensic_video, forensic_message, vehicle):

    if time.time()>forensic_message["proccesing_timer"]+1/forensic_message["proccesing_hz"]:
        
        forensic_message["vehicle_stats"]["vehicle_mode"] = vehicle.mode.name
        forensic_message["vehicle_stats"]["vehicle_arm"] = vehicle.armed
        forensic_message["proccesing_timer"]=time.time()
        if vehicle.armed or vehicle.mode == VehicleMode('LAND') :
            
            before_time= time.time()
            location = vehicle.location.global_relative_frame
            attitude = vehicle.attitude
            forensic_message["vehicle_stats"]["altitude"]=location.alt
            forensic_message["vehicle_stats"]["yaw"]=math.degrees(attitude.yaw)
            try:
                forensic_message["vehicle_stats"]["distance_to_home"]=measure_distance(vehicle.home_location.lat, vehicle.home_location.lon,location.lat,location.lon)

                if not forensic_message["start_land"] and forensic_message["vehicle_stats"]["distance_to_home"] < 1 and location.alt<10 and vehicle.mode ==  VehicleMode('RTL'):
                    forensic_message["start_land"] = True
                    
                if forensic_message["start_land"]:              
                    time_spend, center, target, priorized_tag_counter, priorized_tag, yaw_correction =search_image_aruco.analyze_frame(frame, location, attitude,forensic_message["priorized_tag"],forensic_message["priorized_tag_counter"],forensic_message)
                    forensic_message["marker_stats"]["priorized_tag"]=priorized_tag
                    forensic_message["marker_stats"]["priorized_tag_counter"]=priorized_tag_counter
                    if yaw_correction != None  :

                        orientation = 1
                        aligned = False
                        if yaw_correction<0:
                            orientation = -1 
                        if "aligned" in forensic_message["yaw_align"].keys():
                     
                            aligned=forensic_message["yaw_align"]["aligned"]
                        if abs(yaw_correction)>1 and not aligned:                    
                            forensic_message["yaw_align"]["aligned"]=False                            
                        else:
                            forensic_message["yaw_align"]["aligned"]=True
            
                        forensic_message["yaw_align"]["orientation"]=orientation
                        forensic_message["yaw_align"]["value"]=yaw_correction           
                    
                    control.land(vehicle, center, attitude, location,forensic_message)
                    time.sleep(0.1)
                    
                    draw_info(frame,forensic_video,forensic_message)
 
                  
                forensic_message["vehicle_stats"]["home_set"]=True
            except AttributeError:
                forensic_message["vehicle_stats"]["home_set"]=False
            forensic_message["proccesing_hz"]=1/((time.time()-before_time)*1.5)

        else:
            forensic_message["marker_stats"] = {}
            forensic_message["control_stats"] = {}  
            forensic_message["yaw_align"]={}
            forensic_message["start_land"]= False
        
    else:
        pass