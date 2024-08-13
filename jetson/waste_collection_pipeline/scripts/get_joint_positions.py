#!/usr/bin/env python3

import rospy
import numpy as np
from waste_collection_pipeline.srv import InverseKinematics, PathPlanner, PathPlannerRequest, PathPlannerResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PointStamped

class Position:
    def __init__(self):
        rospy.init_node('positioning')
        self.object_detected = False
        self.threshold_positioning = 0.02
        self.service_ik = rospy.ServiceProxy('compute_inverse_kinematics', InverseKinematics)
        self.service_path_planner = rospy.ServiceProxy('joint_path_planner', PathPlanner)
        rospy.Subscriber('/alpha/waste_coordinates', PointStamped, self.callback_waste_position)
        self.target_pub = rospy.Publisher('/boat_target_position', PointStamped, queue_size=10)
        self.run()

    def callback_waste_position(self, data):
        self.waste_x = data.point.x
        self.waste_y = data.point.y
        self.waste_z = data.point.z

    def run(self):
        if self.object_detected is False:
            self.object_detected == True
            self.z_end = self.waste_z
            self.z_start = self.z_end + 0.05

            start = self.service_ik(self.z_start)
            end = self.service_ik(self.z_end)

            if (start.x != float('nan') and end.x != float('nan')):
                
                self.position_target = PointStamped()
                while(abs(start.x - self.waste_x) < self.threshold_positioning):
                    self.position_target.header.stamp = rospy.Time.now()
                    self.position_target.point.x = self.waste_x - self.start.x
                    self.position_target.point.y = 0
                    self.target_pub.publish(self.position_target)

                req = PathPlannerRequest()
                req.axis_d_start = start.axis_d
                req.axis_c_start = start.axis_c
                req.axis_d_end = end.axis_d
                req.axis_c_end = end.axis_c
                resp = self.service_path_planner(req)
                print(resp)

                #convert data to vehicle frame


if __name__ == "__main__":
    Position()