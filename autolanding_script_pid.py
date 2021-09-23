

import asyncio
import pygazebo
import cv2
from PIL import Image
import numpy as np

from dronekit import connect, VehicleMode

# Helper Libraries Imports
import pid_utils.search_image_aruco as search_image_aruco
import pid_utils.search_image as search_image
import pid_utils.control as control
import multiprocessing
import math
from pid_utils.flight_assist import send_velocity


# Python Imports
import time
import queue as Queue

global start_land, image_retrieve, image_readed, parent_conn_im, child_conn_im, imageQueue, vehicleQueue, frame_count, vehicle, proccesing_hz, proccesing_timer

    
start_land = False
image_retrieve = []
image_readed = True
proccesing_hz = 10.0
proccesing_timer = time.time()


parent_conn_im, child_conn_im = multiprocessing.Pipe()
imageQueue = Queue.Queue()
vehicleQueue = Queue.Queue()
frame_count =0

connection_string = 'udp:127.0.0.1:14551'

print("Connecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=57600)
# Download the vehicle waypoints (commands). Wait until download is complete.
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

with_aruco=True #False uses haarcascade
if with_aruco:
    priorized_tag = 0
    priorized_tag_counter = { }

def measure_distance(lat1, lon1, lat2, lon2):
    R = 6378.137
    dLat = lat2 * math.pi / 180 - lat1 * math.pi / 180
    dLon = lon2 * math.pi / 180 - lon1 * math.pi / 180
    a = math.sin(dLat/2) * math.sin(dLat/2) +  math.cos(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) *    math.sin(dLon/2) * math.sin(dLon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c
    return d * 1000; 

def retrieve_image(image):
    global start_land, priorized_tag,priorized_tag_counter, image_retrieve, image_readed, parent_conn_im, child_conn_im, imageQueue, vehicleQueue, frame_count, vehicle, proccesing_hz, proccesing_timer
    if time.time()>proccesing_timer+1/proccesing_hz:
        
        proccesing_timer=time.time()
        if vehicle.armed :
            
            before_time= time.time()
            location = vehicle.location.global_relative_frame
            attitude = vehicle.attitude
            try:
                distance_to_home=measure_distance(vehicle.home_location.lat, vehicle.home_location.lon,location.lat,location.lon)

                print(vehicle.mode,distance_to_home,vehicle.home_location.alt,start_land)
                if not start_land and distance_to_home < 1 and location.alt<10 and vehicle.mode ==  VehicleMode('RTL'):
                    start_land = True
                if start_land:
                    frame=Image.frombytes('RGB', (640,480), image, 'raw')
                    frame=np.array(frame) 
                    imageQueue.put(frame)
                    vehicleQueue.put((location,attitude))

                    if with_aruco:
                        img = multiprocessing.Process(name="img",target=search_image_aruco.analyze_frame, args = (child_conn_im, frame, location, attitude,priorized_tag,priorized_tag_counter))
                    else:
                        img = multiprocessing.Process(name="img",target=search_image.analyze_frame, args = (child_conn_im, frame, location, attitude))
                    img.daemon = True
                    img.start()

                    results = parent_conn_im.recv()
                    if with_aruco:
                        try:
                            priorized_tag_counter=results[3]
                            priorized_tag=results[4]
                        except IndexError:
                            print("some index error")
                        
                    
                    frame_count += 1
                    
                    img = imageQueue.get()
                    location, attitude = vehicleQueue.get()
                    

                    control.land(vehicle, results[1], attitude, location)
                    time.sleep(0.1)
                    proccesing_hz=1/((time.time()-before_time)*1.2)

                    #print("Takes", time.time()-before_time,"Seconds")
            except AttributeError:
                print("no home location set")
            proccesing_hz=1/((time.time()-before_time)*1.5)
            #print("Takes", time.time()-before_time,"Seconds")
        else:
            print("disarmed")
            start_land= False
    else:
        pass
        
async def publish_loop():
    
    global image_retrieve, image_readed
    manager = await pygazebo.connect()
    
    

    publish_loop.is_waiting = True  

    subscriber=manager.subscribe('/gazebo/default/iris_demo/iris_demo/gimbal_small_2d/tilt_link/camera/image',
                    'gazebo.msgs.Image',
                    retrieve_image)
    returned = await subscriber.wait_for_connection()
        
   
    while (publish_loop.is_waiting):
   
        await asyncio.sleep(0.1)

   
loop = asyncio.get_event_loop()
loop.run_until_complete(publish_loop())




