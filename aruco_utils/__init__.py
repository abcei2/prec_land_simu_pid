import cv2
from cv2 import aruco
import numpy as np
from pathlib import Path
import shutil
from tqdm import tqdm
import yaml
from pymavlink import mavutil
import math 
import time


def permutation(arr, n):
    # Set to check the count
    # of non-repeating elements
    s = set()
    maxEle = 0
    for i in range(n):
        # Insert all elements in the set
        s.add(arr[i])
        # Calculating the max element
        maxEle = max(maxEle, arr[i])
    if (maxEle != n):
        return False
    # Check if set size is equal to n
    if (len(s) == n):
        return True
    return False


def rgb2gray(rgb):
    return np.dot(rgb[..., :3], [0.2989, 0.5870, 0.1140])


def recreate_folder_data(path):

    main_path = Path(path)

    if main_path.exists() and main_path.is_dir():
        shutil.rmtree(main_path)
    main_path.mkdir(parents=True, exist_ok=True)


def detect_good_board_data(image, arucoParams, aruco_dict, horizontal_tag_ammount, vertical_tag_ammount, data_ammount, path, show_detections=False):

    name = path + str(data_ammount)+".jpg"
    img_gray = rgb2gray(image).astype(np.uint8)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        img_gray, aruco_dict, parameters=arucoParams)

    if len(corners) != horizontal_tag_ammount*vertical_tag_ammount:
        #print ("board is not detected correctly")
        pass
    elif corners == None or corners == []:
        #print ("pass")
        pass
    else:
        ids_list = ids.T[0]+1
        n = len(ids_list)
        print(n)
        if (permutation(ids_list, n)):
            print(name)
            cv2.imwrite(name, image)
            data_ammount = data_ammount+1
        img_aruco = aruco.drawDetectedMarkers(
            image.copy(), corners, ids, (0, 255, 0))
        if show_detections:
            cv2.imshow("image with board", img_aruco)
            cv2.waitKey(10)
        else:
            print("No", ids_list)
    return data_ammount


def calib_camera(arucoParams, aruco_dict, horizontal_tag_ammount, vertical_tag_ammount, markerLength, markerSeparation, path, file_name="calibration"):
    # create arUco board
    board = aruco.GridBoard_create(
        horizontal_tag_ammount, vertical_tag_ammount, markerLength, markerSeparation, aruco_dict)

    main_path = Path(path)

    img_list = []
    calib_fnms = main_path.glob('*.jpg')
    print('Using ...', calib_fnms, end='')
    for idx, fn in enumerate(calib_fnms):
        print(idx, '', end='')
        img = cv2.imread(str(fn))
        img_list.append(img)
        h, w, c = img.shape

    print('Calibration images')

    counter, corners_list, id_list = [], [], []
    first = True
    for im in tqdm(img_list):
        img_gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            img_gray, aruco_dict, parameters=arucoParams)
        if first == True:
            corners_list = corners
            id_list = ids
            first = False
        else:
            corners_list = np.vstack((corners_list, corners))
            id_list = np.vstack((id_list, ids))
        img_aruco = aruco.drawDetectedMarkers(im, corners, ids, (0, 255, 0))
        counter.append(len(ids))
    print('Found {} unique markers'.format(np.unique(ids)))

    counter = np.array(counter)
    print("Calibrating camera .... Please wait...")
    #mat = np.zeros((3,3), float)
    ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraAruco(
        corners_list, id_list, counter, board, img_gray.shape, None, None)

    print("Camera matrix is \n", mtx,
          "\n And is stored in calibration.yaml file along with distortion coefficients : \n", dist)

    data = {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(
        dist).tolist(), 'rvecs': np.asarray(rvecs).tolist(), 'tvecs': np.asarray(tvecs).tolist()}
    with open(file_name+".yaml", "w") as f:
        yaml.dump(data, f)

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).


def rotationMatrixToEulerAngles(R):

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])
