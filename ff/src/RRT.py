import numpy as np
from matplotlib import pyplot as plt
from PIL import Image
import random
from math import sqrt, atan2
from scipy.ndimage import binary_dilation
import math

# ==================== MAIN FUNCTIONS ====================
def rand_conf(C, p, qgoal):
    '''

    C --> grid_map
    p --> probability
    qgoal --> goal point

    '''

    # Set bounds of the map
    x_limit = C.shape[0]
    y_limit = C.shape[1]

    # Roll a value between 0 and 1
    roll = random.random()

    if roll > p:
        # Generate a random vertex
        valid = False

        while valid is not True:
            x_rand = random.randrange(0, x_limit, 1)
            y_rand = random.randrange(0, y_limit, 1)

            # Checks if it is an empty space
            if C[x_rand][y_rand] == 0:
                valid = True

        return np.array([x_rand, y_rand])

    else:
        return qgoal


def nearest_vertex(qrand, G):
    '''

    qrand --> random vertex
    G --> graph list

    '''
    min_distance = np.inf

    for v in G:
        temp_distance = distance(v, qrand)
        if temp_distance < min_distance:
            min_distance = temp_distance
            v_min = v

    return v_min


def is_segment_free(qnear, qnew, C, resolution):
    '''

    qnear --> nearest vertex to qnew
    qnew --> new vertex generated
    C --> grid_map

    '''
    # Compute slope of the line created by the two points
    if (qnew[0] - qnear[0]) == 0:
        return False

    m = (qnew[1] - qnear[1]) / (qnew[0] - qnear[0])
    b = qnew[1] - m * qnew[0]

    # Set the incremental value
    delta_x = 0.05

    # Check all points within the line if they are free
    if qnear[0] < qnew[0]:
        x1, x2 = qnear[0], qnew[0]
    else:
        x2, x1 = qnear[0], qnew[0]
    for x in np.arange(x1, x2, delta_x):
        y = int(m * x + b)
        x = int(x)

        #################################################################3 3ks
        if C[x][y] > 50 :
            return False

        dist = int(0.3 / resolution)

        # robot cells margins
        min_x, max_x = x-dist, x+dist
        min_y, max_y = y-dist, y+dist

        # setting map size in x and y
        bound_x = C.shape[0]
        bound_y = C.shape[1]

        # Return True if free, False if occupied and self.is_unknown_valid if unknown.
        for i in range(min_x, max_x):
            for j in range(min_y, max_y):
                if i < 0 or j < 0 or i >= bound_x or j >= bound_y:
                    return False # index out of bounds, invalid state
                if C[i][j] > 50:
                    return False # surroundings occupied, terminate once detected

    return True


def new_conf(qnear, qrand, delta_q):
    '''

    qnear --> nearest vertex to qrand
    qrand --> random vertex generated
    delta_q --> distance threshold

    '''

    if distance(qnear, qrand) > delta_q:
        m = slope(qrand, qnear)

        delta_x = int(np.cos(m) * delta_q)
        delta_y = int(np.sin(m) * delta_q)

        qnew = np.array([qnear[0] + delta_y, qnear[1] + delta_x])
        return qnew
    else:
        return qrand


def fill_path(index_dict, cameFrom):
    current = max(index_dict.keys())
    total_path = [current]

    while 0 not in total_path:
        current = cameFrom[current]
        total_path.append(current)

    return list(reversed(total_path))


def smooth(path, index_dict, C, resolution):
    smooth_path = [path[0]]
    inv_path = list(reversed(path))

    current = path[0]
    last = path[-1]

    while last not in smooth_path:
        if is_segment_free(index_dict[current], index_dict[last], C, resolution):
            smooth_path.append(last)
            return smooth_path
        else:
            for index in inv_path:
                if is_segment_free(index_dict[current], index_dict[index], C, resolution):
                    smooth_path.append(index)
                    current = index
                    break


# ==================== AUXILIARY FUNCTIONS ====================
def distance(v1, v2):
    return sqrt((v1[0]-v2[0])**2 + (v1[1]-v2[1])**2)


def slope(v2, v1):
    m = atan2((v2[0] - v1[0]), (v2[1] - v1[1]))
    if m < 0:
        m = m + 2 * 3.14
    return m


# ==================== RRT FUNCTION ====================
# This function has to plan a path from qstart to qgoal. 
# The planner returns a path that is a list of poses.
def RRTT(C, K, delta_q, p, qstart, qgoal, resolution):


    radius = 4
    structure = np.ones((2 * radius + 1, 2 * radius + 1))
    inflated_map = binary_dilation(C, structure)
    #C = inflated_map

    index_dict = {0: [qstart[0], qstart[1]]}
    cameFrom = {0: 0}
    #print("start & goal are:", qstart, qgoal)

    j = 1

    G = [qstart]
    for i in range(1, K):
        qrand = rand_conf(C, p, qgoal)
        qnear = nearest_vertex(qrand, G)
        qnew = new_conf(qnear, qrand, delta_q)

        if is_segment_free(qnear, qnew, C, resolution) and C[qnew[0]][qnew[1]] == 0:
            index_dict[j] = [qnew[0], qnew[1]]

            for k in index_dict:
                if index_dict[k] == [qnear[0], qnear[1]]:
                    cameFrom[j] = k
                    break

            j += 1
            G.append(qnew)

            if qnew[0] == qgoal[0] and qnew[1] == qgoal[1]:
                path = fill_path(index_dict, cameFrom)
                path2 = smooth(path, index_dict, C, resolution)
                #print("smooth path:", path2)

                path_x = []
                path_y = []
                path_x2 = []
                path_y2 = []
                for index in path:
                    path_x.append(G[index][0])
                    path_y.append(G[index][1])
                for index in path2:
                    path_x2.append(G[index][0])
                    path_y2.append(G[index][1])
                
                paath1=list(zip(path_x, path_y))
                #print("path:", paath1)
                #for index in paath1:
                    #print("values:", C[index[0]][index[1]])

                paath=list(zip(path_x2, path_y2))
                print("final path:", paath)
                return paath

    print("Path not found!")