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
    try: keypress = key.char
    except AttributeError: keypress = key

def on_release(key):
    global keypress
    keypress = ''

################################################################################
# Task 1: Implement the inverse kinematics function for the 2-DOF planar robot
################################################################################
def inverse_kinematics(x, y, elbow_up_flag):
    # Analytical solution based on basic trigonometry
    # Calculates two solutions based on the elbow layout flag

    # link lengths
    link1 = 0.25
    link2 = 0.25

    # law of cosines
    cos_t2 = (x**2 + y**2 - link1**2 - link2**2) / (2 * link1 * link2)
    cos_t2 = np.clip(cos_t2, -1.0, 1.0)

    # sin(t2) = ±sqrt(1 - cos^2(t2)) -> choosing sign based on elbow flag
    if elbow_up_flag == 1:
        sin_t2 = -np.sqrt(1 - cos_t2**2)
    else:
        sin_t2 = np.sqrt(1 - cos_t2**2)
        
    t2 = np.arctan2(sin_t2, cos_t2)
    
    # --- From Friend's Note 2: Algebraic reduction for Theta 1 ---
    k1 = link1 + link2 * cos_t2
    k2 = l2 * sin_t2
    
    t1 = np.arctan2(y, x) - np.arctan2(k2, k1)
    
    return t1, t2

################################################################################
# Task 2: Jacobian F(x)
################################################################################
def inverse_jacobian(current_joints):

    # Calculates the Moore-Penrose pseudo-inverse of a 3-DOF Jacobian matrix 
    # derived from analytical partial derivatives

    link1, link2, link3 = 0.25, 0.25, 0.25
    q1, q2, q3 = current_joints[0], current_joints[1], current_joints[2]
    
    J = np.zeros((3, 3))
    
    # Row 1: Partial derivatives of X position with respect to q1, q2, q3
    J[0, 0] = -link1*math.sin(q1) - link2*math.sin(q1 + q2) - link3*math.sin(q1 + q2 + q3)
    J[0, 1] = -link2*math.sin(q1 + q2) - link3*math.sin(q1 + q2 + q3)
    J[0, 2] = -link3*math.sin(q1 + q2 + q3)
    
    # Row 2: Partial derivatives of Y position with respect to q1, q2, q3
    J[1, 0] = link1*math.cos(q1) + link2*math.cos(q1 + q2) + link3*math.cos(q1 + q2 + q3)
    J[1, 1] = link2*math.cos(q1 + q2) + link3*math.cos(q1 + q2 + q3)
    J[1, 2] = link3*math.cos(q1 + q2 + q3)
    
    # Row 3: Partial derivatives of Orientation Phi (phi = q1 + q2 + q3)
    J[2, 0] = 1.0
    J[2, 1] = 1.0
    J[2, 2] = 1.0
    
    # Compute the inverse of the Jacobian matrix
    return np.linalg.pinv(J)

################################################################################
# 2DOF Forard Kinematics helper, used to seed the initial guess for the inverse kinematics (x_target/y_target)
################################################################################
def fk_2dof(t1, t2):
    """Return (x, y) TCP position for the 2-DOF robot given joint angles."""
    L1, L2 = 0.25, 0.25
    x = L1 * math.cos(t1) + L2 * math.cos(t1 + t2)
    y = L1 * math.sin(t1) + L2 * math.sin(t1 + t2)
    return x, y

################################################################################
# Main sim loop
################################################################################
if __name__ == '__main__':
    print('Program started')
    # connect to Remote API
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    # kinematic structure of 2-DOF robot  - to be filled out
    dx = np.array([0.0, 0.0, 0.0])
    DOF2_robot = np.array([
        [0, 0, 0, 0, 0, 0],     #transformation from world coordinate system (WCS) to robot base
        [0.25, 0, 0, 0, 0, 0],   # transformation from robot base to link2 (link 1 length)
        [0.25, 0, 0, 0, 0, 0]])  # transformation from link2 to TCP (link 2 length)

    # degrees of freedom for 
    dof2 = 2
    dof3 = 3
    
    defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
    sim.setInt32Param(sim.intparam_idle_fps, 0)
    client.setStepping(True)
    sim.startSimulation()


    # Getting Handles of the TCP
    handle_tcp = sim.getObjectHandle('TCP')

    #task 1 - 2DOF robot joints handles
    jh2 = np.empty(dof2, dtype=object)  # create an empty array for joint handles
    # getting the handles for the joints
    jh2[0]=sim.getObjectHandle('Revolute_joint1')
    jh2[1]=sim.getObjectHandle('Revolute_joint2')
    # elbow_up = 0
    #time.sleep(1)

    #task 2 - 3DOF robot joints handles
    jh3 = np.empty(dof3, dtype=object)  # create an empty array for the joints
    # getting the handles for the joints
    jh3[0]=sim.getObjectHandle('Revolute_joint1')
    jh3[1]=sim.getObjectHandle('Revolute_joint2')
    jh3[2]=sim.getObjectHandle('Revolute_joint3')   

    # Initial joint angles — 20° to avoid singularities 
    init_deg = 20.0
    init_rad = init_deg * math.pi / 180.0

    # Task 1 joint state
    joints2 = np.array([init_rad, init_rad])
    for i in range(dof2):
        sim.setJointPosition(jh2[i], joints2[i])

    # Task 1 Cartesian target — seeded from FK so the robot doesn't jump
    x_target, y_target = fk_2dof(joints2[0], joints2[1])
    print(f'[Task1] Initial TCP target: x={x_target:.4f}  y={y_target:.4f}')

    # Task 1 elbow flag
    elbow_up = 0

    # Task 2 joint state
    joints3 = np.array([init_rad, init_rad, init_rad])
    for i in range(dof3):
        sim.setJointPosition(jh3[i], joints3[i])

    for i in range(dof3):
            sim.setJointPosition(jh[i], joints[i])

################################################################################
# Keyboard listener
################################################################################
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    while True:
        while keypress == '':  # wait for keyboard input
            pass
        if keypress == 'q':
            print("You pressed q to quit")
            break

        delta_pos = 0.02  # in m for each simulation step
        if keypress == "w":   # move up
            dx = np.array([0.0, delta_pos])
        if keypress == "s":   # move down
            dx = np.array([0.0, -delta_pos])
        if keypress == "a":    # move left
            dx = np.array([-delta_pos, 0.0])
        if keypress == "d":    # move right
            dx = np.array([delta_pos, 0.0])
        if keypress == "s":    # stop movement
            dx = np.array([0.0, 0.0])
        if keypress == "e":    #
            elbow_up = elbow_up ^ 1
        print("dx: ", dx)
        
		
		
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
