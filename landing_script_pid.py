

import asyncio
import pygazebo
import cv2
from PIL import Image
import numpy as np

from dronekit import connect

# Helper Libraries Imports
import pid_utils.search_image_aruco as search_image_aruco
import pid_utils.search_image as search_image
import pid_utils.control as control
import multiprocessing


# Python Imports
import time
import queue as Queue

global image_retrieve, image_readed, parent_conn_im, child_conn_im, imageQueue, vehicleQueue, frame_count, vehicle, proccesing_hz, proccesing_timer
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

with_aruco=True
if with_aruco:
    priorized_tag = 0
    priorized_tag_counter = { }




def retrieve_image(image):
    global priorized_tag,priorized_tag_counter, image_retrieve, image_readed, parent_conn_im, child_conn_im, imageQueue, vehicleQueue, frame_count, vehicle, proccesing_hz, proccesing_timer
            
    if time.time()>proccesing_timer+1/proccesing_hz:
        
        proccesing_timer=time.time()
        if vehicle.armed :

            frame=Image.frombytes('RGB', (640,480), image, 'raw')
            frame=np.array(frame) 
            before_time= time.time()
            location = vehicle.location.global_relative_frame
            attitude = vehicle.attitude
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
            proccesing_hz=1/((time.time()-before_time)*1.5)
            print("Takes", time.time()-before_time,"Seconds")
    else:
        print("not proc")
        
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




