# This is a sample Python script for exercise 2 and 3.
import numpy as np
import math
import time

import transformations as t
from random import  uniform
from pynput import keyboard
from coppeliasim_zmqremoteapi_client import *


keypress=''  # global variable


############################################################################
def on_press(key):
    global keypress
    try:
        keypress = key.char
    except AttributeError:
        keypress = key
############################################################################
def on_release(key):
    global keypress
    keypress = ''


############################################################################
# this needs to be implemented
def convert_quaternion_to_matrix(quat):
    x = quat[0]
    y = quat[1]
    z = quat[2]
    w = quat[3]
    # First row of the rotation matrix
    r00 = 1 - 2 * (y*y) - 2 * (z*z)
    r01 = 2*x*y - 2*w*z
    r02 = 2 * x * z + 2 * w * y

    # Second row of the rotation matrix
    r10 = 2 * x * y + 2 * z * w
    r11 = 1 - 2 * (x*x) - 2*(z*z)
    r12 = 2 * y * z - 2 * w * x
    # Third row of the rotation matrix
    r20 = 2 * x * z - 2 * w * y
    r21 = 2 * y * z + 2 * w * x
    r22 = 1 - 2 * (x*x) - 2 * (y*y)
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
    return rot_matrix
############################################################################
# this needs to be implemented
def euler_from_matrix(R):
    eps = 1e-7
    beta = math.asin(min(1.0, max(-1.0, -R[2][0])))
    if math.cos(beta) > eps:
        ax = math.atan2(R[2][1], R[2][2])
        ay = beta
        az = math.atan2(R[1][0], R[0][0])
    else:
        ax = math.atan2(R[1][0], R[1][1])
        ay = beta
        az = 0

    return [ax, ay, az]
############################################################################
# this needs to be implemented
def matrix_from_euler(angles):
    
    alpha = angles[0]
    beta = angles[1]
    gamma = angles[2]
    # First row of the rotation matrix
    r00 = math.cos(beta) * math.cos(gamma)
    r01 = -(math.cos(beta)) * math.sin(gamma)
    r02 = math.sin(beta)

    # Second row of the rotation matrix
    r10 = math.cos(alpha) * math.sin(gamma) + math.sin(alpha) * math.sin(beta) * math.cos(gamma)
    r11 = math.cos(alpha) * math.cos(gamma) - math.sin(alpha) * math.sin(beta) * math.sin(gamma)
    r12 = -(math.sin(alpha)) * math.cos(beta)
    # Third row of the rotation matrix
    r20 = math.sin(alpha) * math.sin(gamma) - math.cos(alpha) * math.sin(beta) * math.cos(gamma)
    r21 = math.sin(alpha) * math.cos(gamma) + math.cos(alpha) * math.sin(beta) * math.sin(gamma)
    r22 = math.cos(alpha) * math.cos(beta)
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
    return rot_matrix


################################################################################
def forward_kinematics(rob, joints):
    # TCP = t.identity_matrix()
    # Rz1  = t.rotation_matrix(0, [0,0,1])
    # T1   = t.translation_matrix([1, 0, 0])
    # Rz2  = t.rotation_matrix(90 * math.pi/180.0, [0,0,1])
    # T2   = t.translation_matrix([1, 0, 0])
    # TCP = t.concatenate_matrices(TCP, Rz1, T1, Rz2, T2)

    TCP = t.identity_matrix()

    base_translation = t.translation_matrix(rob[0][3:6])
    TCP = t.concatenate_matrices(TCP, base_translation)


    for i in range(len(joints)):
        curr_row = rob[i + 1] 

        rotation_axis = curr_row[0:3]
        transition_vector = curr_row[3:6]

        Rotatejoint = t.rotation_matrix(joints[i], rotation_axis)
        TransformLink = t.translation_matrix(transition_vector)

        TCP = t.concatenate_matrices(TCP,Rotatejoint,TransformLink)

    return TCP


################################################################################
if __name__ == '__main__':

    print('Program started')

    # degrees of freedom of snake robot
    dof = 6
    # kinematic structure of snake robot  - to be filled out
    snake_robot = np.array([
        [0, 0, 0, 0, 0, 0], #base
        [0, 0, 1, 0.25, 0, 0], #1
        [0, 0, 1, 0.25, 0, 0], #2
        [0, 0, 1, 0.25, 0, 0], #3
        [0, 0, 1, 0.25, 0, 0], #4
        [0, 0, 1, 0.25, 0, 0], #5
        [0, 0, 1, 0.25, 0, 0]])#6

    # connect to Remote API
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    # Getting Handles of the Tool Centre Point (TCP)
    handle_tcp = sim.getObjectHandle('TCP')

    jh = np.empty(dof, dtype=object)  # create an empty array for joint handles
    # getting the handles for the joints
    jh[0]=sim.getObjectHandle('Revolute_joint1')
    jh[1]=sim.getObjectHandle('Revolute_joint2')
    jh[2]=sim.getObjectHandle('Revolute_joint3')
    jh[3]=sim.getObjectHandle('Revolute_joint4')
    jh[4]=sim.getObjectHandle('Revolute_joint5')
    jh[5]=sim.getObjectHandle('Revolute_joint6')
    time.sleep(1)
    joints = np.empty(dof, dtype=object)  # create an empty array for the joints
    # set all joints to 0
    for i in range(dof):
        sim.setJointPosition(jh[i], 0.0)

    listener = keyboard.Listener(on_press=on_press,on_release=on_release)
    listener.start()

    while True:
        print("Wait for keyboard input")
        while keypress=='':  # wait for keyboard input
            pass
        if keypress == 'q':
            print("You pressed q to quit")
            break
        # set joints to random position between [0, pi/4]
        for i in range(dof):
            sim.setJointPosition(jh[i], uniform(0.0, math.pi/4))

        angles_rad = sim.getObjectOrientation(handle_tcp, -1)  # Try to retrieve orientation
        angles_degree = np.degrees(angles_rad)

        pos    = sim.getObjectPosition(handle_tcp, -1)  # Try to retrieve position
        # retrieve the joint positions
        for i in range(dof):
            joints[i] = sim.getJointPosition(jh[i])

        print('angles: ', angles_degree)
        R1 = matrix_from_euler(angles_rad)
        print('TCP Rotation: ')
        print(R1)
        print('TCP Position: ', pos)
        TCP_frame = forward_kinematics(snake_robot, joints)
        print(TCP_frame)
        rot = TCP_frame[:3,:3]
        print(euler_from_matrix(rot))
        print("----------------------------------------------------   ")
        #time.sleep(0.005)

        keypress = ''

    listener.stop()
    print('Program ended')