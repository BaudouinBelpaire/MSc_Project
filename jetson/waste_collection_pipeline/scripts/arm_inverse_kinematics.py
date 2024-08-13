#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from waste_collection_pipeline.srv import InverseKinematics, InverseKinematicsResponse, ForwardKinematics
import numpy as np
import math
from sensor_msgs.msg import JointState

alpha = 10
tolerance = 0.0001
max_num_iterations = 1000

class InverseKinematicsService:
    def __init__(self):
        rospy.init_node('inverse_kinematics_service')
        self.service = rospy.Service('compute_inverse_kinematics', InverseKinematics, self.handle_inverse_kinematics)
        self.forward_kinematics_srv = rospy.ServiceProxy('compute_forward_kinematics', ForwardKinematics)

    def call_forward_kinematics(self, theta1, theta3, theta7, theta9):
        try:
            resp = self.forward_kinematics_srv(theta1, theta3, theta7, theta9)
            return resp.x, resp.y, resp.z
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None, None, None

    def handle_inverse_kinematics(self, req):
        z_target = req.z

        resp = InverseKinematicsResponse()
        resp.x = float('nan')
        resp.axis_d = float('nan')
        resp.axis_c = float('nan')

        if z_target < -0.3 or z_target > -0.05:
            rospy.logwarn("Z value out of bounds")
            return resp
        
        Theta1 = 0
        Theta3 = 0
        Theta7 = -np.pi / 2 - Theta3
        Theta9 = 0

        for n in range(max_num_iterations):
            Theta7 = -math.pi / 2 - Theta3
            x_arr, y_arr, z_arr = self.call_forward_kinematics(Theta1, Theta3, Theta7, Theta9)
            if x_arr is None:
                rospy.logwarn("Failed to compute forward kinematics.")
                return resp
            
            x = x_arr[-1]
            y = y_arr[-1]
            z = z_arr[-1]
            error_z = z_target - z

            if abs(error_z) < tolerance:
                rospy.loginfo(f"Inverse kinematics solved in {n} iterations.")
                resp.x = x
                resp.axis_d = Theta3
                resp.axis_c = Theta7
                print(resp)

                return resp

            Theta3 += error_z * alpha
            if n == max_num_iterations - 1:
                rospy.logwarn("Inverse kinematics did not converge.")
                return resp

if __name__ == '__main__':
    try:
        service = InverseKinematicsService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
