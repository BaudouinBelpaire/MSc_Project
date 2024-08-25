#!/usr/bin/env python3

import rospy
from waste_collection_pipeline.srv import ForwardKinematics, ForwardKinematicsResponse
import numpy as np

# Define the parameters
L0, L1, L2, L3, L4, L5, L6, L7, L8 = 0.125, 0, 0.02, 0, 0.14, 0, 0, 0, 0.24
Alpha0, Alpha1, Alpha2, Alpha3, Alpha4, Alpha5, Alpha6, Alpha7, Alpha8 = (
    0, 0, np.pi/2, -np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0
)
d1, d2, d3, d4, d5, d6, d7, d8, d9 = 0, -0.05, 0, -0.02, 0, -0.02, 0, 0.02, 0
Theta2, Theta4, Theta5, Theta6, Theta8 = 0, 0, 0, 0, 0

def compute_forward_kinematics(Theta1, Theta3, Theta7, Theta9):
    DH_params = np.array([
        [Alpha0, L0, d1, Theta1],
        [Alpha1, L1, d2, Theta2],
        [Alpha2, L2, d3, Theta3],
        [Alpha3, L3, d4, Theta4],
        [Alpha4, L4, d5, Theta5],
        [Alpha5, L5, d6, Theta6],
        [Alpha6, L6, d7, Theta7],
        [Alpha7, L7, d8, Theta8],
        [Alpha8, L8, d9, Theta9]
    ])

    # Initialize transformation matrix
    T = np.eye(4)

    # Initialize arrays to store joint positions
    x = np.zeros(10)
    y = np.zeros(10)
    z = np.zeros(10)

    # Compute the position of each joint
    for i in range(DH_params.shape[0]):
        Alpha, L, d, Theta = DH_params[i]

        # Transformation matrix for current joint
        Tx = np.array([[1, 0, 0, L],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
        Rx = np.array([[1, 0, 0, 0],
                    [0, np.cos(Alpha), -np.sin(Alpha), 0],
                    [0, np.sin(Alpha), np.cos(Alpha), 0],
                    [0, 0, 0, 1]])
        Rz = np.array([[np.cos(Theta), -np.sin(Theta), 0, 0],
                    [np.sin(Theta), np.cos(Theta), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
        Tz = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, d],
                    [0, 0, 0, 1]])
        T = T @ Tx @ Rx @ Rz @ Tz

        # Store the position of the current joint
        x[i+1] = T[0, 3]
        y[i+1] = T[1, 3]
        z[i+1] = T[2, 3]

    return x, y, z

def handle_forward_kinematics(req):
    Theta1 = req.axis_e
    Theta3 = req.axis_d
    Theta7 = req.axis_c
    Theta9 = req.axis_b
    x, y, z = compute_forward_kinematics(Theta1, Theta3, Theta7, Theta9)
    return ForwardKinematicsResponse(x, y, z)

def forward_kinematics_server():
    rospy.init_node('forward_kinematics_server')
    s = rospy.Service('compute_forward_kinematics', ForwardKinematics, handle_forward_kinematics)
    rospy.loginfo("Ready to compute forward kinematics.")
    rospy.spin()

if __name__ == "__main__":
    forward_kinematics_server()
