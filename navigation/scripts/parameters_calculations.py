import os
import csv
import numpy as np


path = 'data_linear2.csv'

deltaTime = np.array([])
Thrust_X = []
Thrust_Y = []
Torque = []
Surge = []
Sway = []
Yaw = []
Surge_dot = []

with open(path, 'r') as csv_file:
    csv_reader = csv.reader(csv_file)
    flag = 2

    for row in csv_reader:
        if flag == 2 :
            flag = 1
            continue
        if flag == 1:
            lastTime = float(row[0])
            flag = 0
            continue
        deltaTime = np.append(deltaTime, [float(row[0]) - lastTime])
        Thrust_X = np.append(Thrust_X, [float(row[1])])
        Thrust_Y = np.append(Thrust_Y, [float(row[2])])
        Torque = np.append(Torque, [float(row[3])])
        Surge = np.append(Surge, [float(row[4])])
        Sway = np.append(Sway, [float(row[5])])
        Yaw = np.append(Yaw, [float(row[6])])
        Surge_dot = np.append(Surge_dot, [float(row[7])])

    Sway_dot = Sway/deltaTime
    Yaw_dot = Yaw/deltaTime
    As_x = np.column_stack((Surge_dot, Surge, abs(Surge)*Surge))
    As_y = np.column_stack((Sway_dot, Sway, Surge*Yaw))
    As_th = np.column_stack((Yaw_dot, Yaw))
    print(As_x.shape)

    theta_s_x = np.matmul(np.linalg.inv(np.matmul(np.transpose(As_x),As_x)),np.transpose(As_x))
    theta_s_x = np.matmul(theta_s_x, Thrust_X)
    theta_s_y = np.matmul(np.linalg.inv(np.matmul(np.transpose(As_y),As_y)),np.transpose(As_y))
    theta_s_y = np.matmul(theta_s_y, Thrust_Y-Sway_dot*theta_s_x[2])
    theta_s_th = np.matmul(np.linalg.inv(np.matmul(np.transpose(As_th),As_th)),np.transpose(As_th))
    theta_s_th = np.matmul(theta_s_th, Torque)
    print(theta_s_x)
    print(theta_s_y)
    print(theta_s_th)