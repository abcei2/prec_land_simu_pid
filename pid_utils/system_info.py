
import cv2
from cv2 import aruco
forensic_message_tags = {
    "vehicle_stats":{},
    "yaw_align":{},
    "marker_stats":{},
    "control_stats":{}
}
def draw_info(frame,forensic_video,forensic_message, show=True):
    vehicle_stats_text = "mode: "+forensic_message["vehicle_stats"]["vehicle_mode"] 
    if "distance_to_home" in forensic_message["vehicle_stats"].keys():
        vehicle_stats_text += ", distance_to_home: "+str(forensic_message["vehicle_stats"]["distance_to_home"])[0:5]+" cm"


    control_stats = "vx: "+str(forensic_message["control_stats"]["vx"])[0:5]+" vy: "+str(forensic_message["control_stats"]["vy"])[0:5]+" vz: "+str(forensic_message["control_stats"]["vz"])[0:5]
    if forensic_message["yaw_align"]=={}:

        yaw_stats = "No yaw stats"
    else:
        yaw_stats = "Aligned: "+str(forensic_message["yaw_align"]["aligned"])
        yaw_stats += ", Yaw error: "+str(forensic_message["yaw_align"]["value"])[0:5]
        yaw_stats +=  ", Orientation: "+ "clockwise" if forensic_message["yaw_align"]["orientation"]>0 else ", Orientation: "+ "counter clockwise"

    main_tag_text = "no tag"
    if forensic_message["marker_stats"]["some_tag_detected"]:
        main_tag_text = "following_tag #"+str(forensic_message["marker_stats"]["priorized_tag"])
        main_tag_text += ", distance_to_tag: "+str(forensic_message["marker_stats"]["dist_error"])[0:5]+" m"
        frame = aruco.drawDetectedMarkers(frame, forensic_message["marker_stats"]["corners"], forensic_message["marker_stats"]["ids"], (0,255,0))
        
    font = cv2.FONT_HERSHEY_SIMPLEX    
    thickness = 1    
    color = (255, 0, 255)
    
    fontScale = 0.4
    cv2.putText(frame, main_tag_text, (10, 10), font, 
                   fontScale, color, thickness, cv2.LINE_AA)
                   
    cv2.putText(frame, vehicle_stats_text, (10, 30), font, 
                   fontScale, color, thickness, cv2.LINE_AA)
                   
    cv2.putText(frame, control_stats, (10, 50), font, 
                   fontScale, color, thickness, cv2.LINE_AA)

    cv2.putText(frame, yaw_stats, (10, 70), font, 
                   fontScale, color, thickness, cv2.LINE_AA)
    
    forensic_video.write(frame)

    if show:
        cv2.imshow("frame", frame)
        cv2.waitKey(10)

if __name__ == "__main__":
	print("wwwwwwwaaaaaaaaaa")