

import asyncio
import pygazebo
import cv2
from PIL import Image
import numpy as np

from dronekit import connect

# Helper Libraries Imports
from pid_utils.main_proccess import  procces_frame


# Python Imports
import time

global forensic_video, forensic_message, start_land, priorized_tag, priorized_tag_counter
global vehicle, proccesing_hz, proccesing_timer

fourcc = cv2.VideoWriter_fourcc(*'XVID')
forensic_video = cv2.VideoWriter('output.avi', fourcc, 10.0, (640,  480))
forensic_message = {
    "vehicle_stats":{},
    "yaw_align":{},
    "marker_stats":{},
    "control_stats":{}
}
forensic_message["start_land"] = False
forensic_message["proccesing_hz"] = 10.0
forensic_message["proccesing_timer"] = time.time()


connection_string = 'udp:127.0.0.1:14551'

print("Connecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=57600)
# Download the vehicle waypoints (commands). Wait until download is complete.
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

forensic_message["priorized_tag"] = 0
forensic_message["priorized_tag_counter"] = {}


    
def retrieve_image(image):
    
    global forensic_video, forensic_message,vehicle    
    frame=Image.frombytes('RGB', (640,480), image, 'raw')
    frame=np.array(frame) 

    procces_frame(frame,forensic_video, forensic_message, vehicle)
    
    print("- ")
   
        

        
async def publish_loop():
    
    manager = await pygazebo.connect()
    
    

    publish_loop.is_waiting = True  

    subscriber=manager.subscribe('/gazebo/default/iris_demo/iris_demo/gimbal_small_2d/tilt_link/camera/image',
                    'gazebo.msgs.Image',
                    retrieve_image)
    returned = await subscriber.wait_for_connection()
        
   
    while (publish_loop.is_waiting):
        await asyncio.sleep(1)

   
loop = asyncio.get_event_loop()
loop.run_until_complete(publish_loop())




