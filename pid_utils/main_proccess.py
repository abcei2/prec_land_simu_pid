import math
import pid_utils.control as control
from pid_utils.flight_assist import send_velocity
from dronekit import VehicleMode
from pid_utils.flight_assist import condition_yaw
from pid_utils.system_info import draw_info
import pid_utils.search_image_aruco as search_image_aruco
import time 

def measure_distance(lat1, lon1, lat2, lon2):
    R = 6378.137
    dLat = lat2 * math.pi / 180 - lat1 * math.pi / 180
    dLon = lon2 * math.pi / 180 - lon1 * math.pi / 180
    a = math.sin(dLat/2) * math.sin(dLat/2) +  math.cos(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) *    math.sin(dLon/2) * math.sin(dLon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c
    return d * 1000; 


def procces_frame(frame,forensic_video, forensic_message, vehicle):

    if time.time()>forensic_message["proccesing_timer"]+1/forensic_message["proccesing_hz"]:
        
        forensic_message["vehicle_stats"]["vehicle_mode"] = vehicle.mode.name
        forensic_message["vehicle_stats"]["vehicle_arm"] = vehicle.armed
        forensic_message["proccesing_timer"]=time.time()
        if vehicle.armed :
            
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
                    if yaw_correction != None :
                        orientation = 1
                        if yaw_correction<0:
                            orientation = -1 
                        if abs(yaw_correction)>1:
                    
                            forensic_message["yaw_align"]["aligned"]=False
                            condition_yaw(vehicle,abs(yaw_correction), relative=True, orientation=orientation)
                            
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
            forensic_message["start_land"]= False
        
    else:
        pass