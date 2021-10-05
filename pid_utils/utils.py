import math
import numpy as np
def measure_distance(lat1, lon1, lat2, lon2):
    R = 6378.137
    dLat = lat2 * math.pi / 180 - lat1 * math.pi / 180
    dLon = lon2 * math.pi / 180 - lon1 * math.pi / 180
    a = math.sin(dLat/2) * math.sin(dLat/2) +  math.cos(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) *    math.sin(dLon/2) * math.sin(dLon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c
    return d * 1000; 

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