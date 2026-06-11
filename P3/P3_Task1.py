# This is a sample Python script for the exercise 2 and 3.

import numpy as np
import math
import transformations as t
import time
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

def inverse_kinematics_2dof(x,y,elbow_up):
    #            q1             q2
    # Elbow up : alpha + beta, theta - 180
    # Elbow down: alpha - beta, 180 - theta

    r = math.sqrt(math.pow(x,2) + math.pow(y,2))
    l1 = 0.25
    l2 = 0.25

    alpha = math.atan2(y,x)
    beta = math.acos((math.pow(r,2) + math.pow(l1,2) - math.pow(l2,2)) / (2 * r * l2))
    theta = math.acos((math.pow(r,2) - math.pow(l1,2) - math.pow(l2,2)) / (2 * l1 * l2))

    if elbow_up == True:
        q1 = alpha + beta
        q2 = theta - math.pi
    else:
        q1 = alpha - beta
        q2 = math.pi - theta
    return q1,q2

def forward_kinematics_2d(t1,t2):
    l1,l2 = 0.25,0.25
    x = l1 * math.cos(t1) + l2 * math.cos(t1 + t2)
    y = l1 * math.sin(t1) + l2 * math.sin(t1 + t2)
    return x,y

################################################################################
if __name__ == '__main__':
    print('Program started')
    # connect to Remote API
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    # degrees of freedom of 2-DOF robot
    dof = 2
    # kinematic structure of 2-DOF robot  - to be filled out
    dx = np.array([0.0, 0.0, 0.0])
    DOF2_robot = np.array([
        [0, 0, 0, 0, 0, 0],     #transformation from world coordinate system (WCS) to robot base
        [0.25, 0, 0, 0, 0, 0],   # transformation from robot base to link2
        [0.25, 0, 0, 0, 0, 0]])  # transformation from link2 to TCP

    defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
    sim.setInt32Param(sim.intparam_idle_fps, 0)
    client.setStepping(True)
    sim.startSimulation()

    # Getting Handles of the TCP
    handle_tcp = sim.getObjectHandle('TCP')
    jh = np.empty(dof, dtype=object)  # create an empty array for joint handles
    # getting the handles for the joints
    jh[0]=sim.getObjectHandle('./Revolute_joint1')
    jh[1]=sim.getObjectHandle('./Revolute_joint2')
    elbow_up = 0
    #time.sleep(1)
    joints = np.empty(dof, dtype=object)  # create an empty array for the joints
    # set all joints to 20 degrees
    joints[0] = 20.0 * math.pi/180.0
    joints[1] = 20.0 * math.pi/180.0

    x_target, y_target = forward_kinematics_2d(joints[0], joints[1])
    x_target += dx[0]
    y_target += dx[1]
    q1,q2 = inverse_kinematics_2dof(x_target,y_target,elbow_up)
    print(q1,q2)

    print(x_target,y_target)
    for i in range(dof):
            sim.setJointPosition(jh[i], joints[i])


    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    while True:
        while keypress == '':  # wait for keyboard input
            pass
        if keypress == 'q':
            print("You pressed q to quit")
            break

        delta_pos = 0.02  # in m for each simulation step
        if keypress == "u":   # move up
            dx = np.array([0.0, delta_pos])
        if keypress == "n":   # move down
            dx = np.array([0.0, -delta_pos])
        if keypress == "h":    # move left
            dx = np.array([-delta_pos, 0.0])
        if keypress == "j":    # move right
            dx = np.array([delta_pos, 0.0])
        if keypress == "s":    # stop movement
            dx = np.array([0.0, 0.0])
        if keypress == "e":    #
            elbow_up = elbow_up ^ 1
        print("dx: ", dx)
        print("Q1")
        
		
		
		# set all joints
        for i in range(dof):
            sim.setJointPosition(jh[i], joints[i])
        client.step()
        if (sim.getSimulationState() == sim.simulation_stopped):
            break
        keypress = ''

    listener.stop()
    # Now stop the simulation:
    sim.stopSimulation()
    sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps)
