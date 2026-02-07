import numpy as np
import pandas as pd
import copy

def rotate2D(theta,p_point):
    rotation_array = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    q_point = np.matmul(rotation_array,p_point)
    return q_point

def rotate3D(axis_of_rotation,theta,p_point):
    rotation_array = np.array()

    if(axis_of_rotation == 'x'):
        rotation_array = np.array([[1,0,0],[0,np.cos(theta),-np.sin(theta),0],[0,np.sin(theta),np.cos(theta)]])
    elif(axis_of_rotation == 'y'):
        rotation_array = np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
    elif(axis_of_rotation == 'y'):
        rotation_array = np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
    else:
        raise Exception("Invalid coordinate frame")
    q_point = np.matmul(rotation_array,p_point)
    return q_point

def rotate_3D_many_times(rotations,p_point):
    q_point = copy.deepcopy(p_point)
    for i in range(0,len(rotations)):
        q_point = rotate3D(rotations[i][0],rotations[i][1],q_point)
    return q_point