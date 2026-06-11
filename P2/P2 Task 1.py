# This is a sample Python script for exercise 2 and 3.
import numpy as np
import math
import time
from coppeliasim_zmqremoteapi_client import *

from pynput import keyboard

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
    beta = math.asin(min(1.0, R[0][2]))
    if math.cos(beta) > eps:
        ax = math.atan2(-R[1][2],R[2][2])
        ay = math.atan2(R[0][2],math.sqrt(((R[1][2])**2) + (R[2][2])**2))
        az = math.atan2(-R[0][1],R[0][0])
    else:
        ax = math.atan2(R[1][0], R[1][1])
        ay = math.atan2(R[0][2], math.sqrt(R[0][0]**2 + R[0][1]**2))
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
if __name__ == '__main__':

    print('Program started')
    # connect to Remote API
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    # Getting Handle of the Reference Frame
    handle_ref_frame = sim.getObject('./ReferenceFrame')
    print("Reference frame handle: ", handle_ref_frame)
    # Now retrieve streaming data (i.e. in a non-blocking fashion):
    startTime = time.time()
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    while True:
        while keypress == '':  # wait for keyboard input
            pass
        if keypress == 'q':
            print("You pressed q to quit")
            break

        quat=sim.getObjectQuaternion(handle_ref_frame, -1)
        print('quat: ', quat)
        # you should implement the two functions
        # convert quaternion to matrix
        R = convert_quaternion_to_matrix(quat)
        print(R)
        # calculate euler angles from matrix
        euler_angles = euler_from_matrix(R)
        # print euler angles
        print("alpha:", euler_angles[0]*180.0/math.pi)  # print alpha in degree
        print("beta :", euler_angles[1]*180.0/math.pi)  # print beta  in degree
        print("gamma:", euler_angles[2]*180.0/math.pi)  # print gamma in degree
        print("---------------------------------------------")
        R1 = matrix_from_euler(euler_angles)
        #print(R1)
        time.sleep(0.005)
        keypress = ''

    listener.stop()

    print('Program ended')
