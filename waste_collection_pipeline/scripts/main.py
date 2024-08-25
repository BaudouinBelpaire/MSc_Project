#!/usr/bin/env python3

import rospy
import numpy as np
from waste_collection_pipeline.srv import InverseKinematics, PathPlanner, PathPlannerRequest, PathPlannerResponse, ForwardKinematics, ForwardKinematicsRequest, ForwardKinematicsResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class Position:
    def __init__(self):
        rospy.init_node('main_node')
        self.forward_kinematics_srv = rospy.ServiceProxy('compute_forward_kinematics', ForwardKinematics)
        self.service_ik = rospy.ServiceProxy('compute_inverse_kinematics', InverseKinematics)
        self.service_path_planner = rospy.ServiceProxy('joint_path_planner', PathPlanner)
        rospy.Subscriber('/alpha/waste_coordinates', PointStamped, self.callback_waste_position)
        rospy.Subscriber('/blueboat/alpha/joint_states', JointState, self.callback_joint_state)
        rospy.Subscriber('/blueboat/navigator/odom', Odometry, self.callback_odometry)
        self.target_pub = rospy.Publisher('/alpha/target_position', PointStamped, queue_size=10)
        self.arm_joint_state_pub = rospy.Publisher('/blueboat/alpha/desired_joint_states', JointState, queue_size=10)

        self.object_detected = False
        self.gripper_open = 0.8
        self.gripper_closed = 0.35
        self.gripper_names = ['blueboat/alpha_axis_a1', 'blueboat/alpha_axis_a2']
        self.threshold_positioning = 0.02 # 2cm accuracy for positioning

        self.waste_x = 1.5
        self.waste_y = 0
        self.x_current = 0
        self.y_current = 0

        self.run()

    def callback_waste_position(self, data):
        self.waste_x = data.point.x
        self.waste_y = data.point.y
        self.waste_z = data.point.z
    
    def callback_joint_state(self, data):
        self.axis_d_current = data.position[1]
        self.axis_c_current = data.position[2]

    def callback_odometry(self, data):
        self.x_current = data.pose.pose.position.x
        self.y_current = data.pose.pose.position.y

    def set_path_planner_request(self, axis_d_start, axis_c_start, axis_d_end, axis_c_end):
        self.req = PathPlannerRequest()
        self.req.axis_d_start = axis_d_start
        self.req.axis_c_start = axis_c_start
        self.req.axis_d_end = axis_d_end
        self.req.axis_c_end = axis_c_end
        return self.req

    def actuate_joints(self, resp):
        rospy.loginfo(f"Length of points in path : {len(resp.arm_path.points)}")
        rospy.loginfo(f"Time step between each points: {self.duration} seconds")
        for i in range(len(resp.arm_path.points)):
            arm_joint_state = JointState()
            arm_joint_state.header.stamp = rospy.Time.now()
            arm_joint_state.name = resp.arm_path.joint_names
            arm_joint_state.position = resp.arm_path.points[i].positions
            arm_joint_state.velocity = resp.arm_path.points[i].velocities
            # do the same for the boat speed or publish boat target position
            self.arm_joint_state_pub.publish(arm_joint_state)
            rospy.sleep(self.duration)
        
    def actuate_gripper(self, velocity):
        gripper = JointState()
        gripper.header.stamp = rospy.Time.now()
        gripper.name = self.gripper_names
        gripper.velocity = [-velocity, velocity]
        gripper.effort = [-10, 10]
        self.arm_joint_state_pub.publish(gripper)

    def compute_error_position(self, axis_d, axis_c):
        ref = self.forward_kinematics_srv(0, axis_d, axis_c, 0)
        out = self.forward_kinematics_srv(0, self.axis_d_current, self.axis_c_current, 0)
        x_err = abs(out.x[-1]-ref.x[-1])
        z_err = abs(out.z[-1]-ref.z[-1])
        err = (x_err ** 2 + z_err ** 2)**(0.5)
        rospy.loginfo(f"X error is {x_err} and Z error is {z_err} m")
        rospy.loginfo(f"Total error is {err} m")

    def run(self):
        if self.object_detected is False:
            self.object_detected == True
            self.z_end = -0.23 #Read data from waste coordinate topic
            self.z_start = -0.15
            self.start = self.service_ik(self.z_start)
            self.end = self.service_ik(self.z_end)
            rospy.loginfo([self.start, self.end])

            if (self.start.x != float('nan') and self.end.x != float('nan')):

                self.position_target = PointStamped()
                time_init = rospy.Time.now()
                rospy.loginfo("Boat positioning on the waste")
                rospy.loginfo(f"Located at {abs(self.x_current + self.start.x - self.waste_x)} meters away from the waste")

                while(abs(self.x_current + self.start.x - self.waste_x) > self.threshold_positioning):
                    rospy.loginfo(f"Located at {self.x_current + self.start.x - self.waste_x} meters away from the waste")
                    self.position_target.header.stamp = rospy.Time.now()
                    self.position_target.point.x = self.x_current + self.waste_x - self.start.x
                    self.position_target.point.y = self.y_current - self.waste_y #convert from cartesian to NED
                    self.target_pub.publish(self.position_target)
                    if (rospy.Time.now()-time_init > rospy.Duration(0.1)):
                        rospy.loginfo("Aborted: Boat positioning took too long")
                        break

                rospy.loginfo("Plan path to position arm above waste")
                req = self.set_path_planner_request(self.axis_d_current, self.axis_c_current, self.start.axis_d, self.start.axis_c)
                resp = self.service_path_planner(req)
                self.duration = resp.duration
                self.actuate_joints(resp)
                #Open gripper
                self.actuate_gripper(self.gripper_open)
                self.compute_error_position(self.start.axis_d, self.start.axis_c)
                rospy.loginfo("Reach Alpha 5 armed above waste!")

                rospy.sleep(2.)
                
                rospy.loginfo("Plan path to position arm on the waste")
                req = self.set_path_planner_request(self.axis_d_current, self.axis_c_current, self.end.axis_d, self.end.axis_c)
                resp = self.service_path_planner(req)
                self.duration = resp.duration
                self.actuate_joints(resp)
                #Close gripper
                self.actuate_gripper(-self.gripper_closed)
                self.compute_error_position(self.end.axis_d, self.end.axis_c)
                rospy.loginfo("Arm closed on waste!")
                
                rospy.sleep(2.)

                rospy.loginfo("Move arm back to original position")
                req = self.set_path_planner_request(self.axis_d_current, self.axis_c_current, 1.5, 1.5)
                resp = self.service_path_planner(req)
                self.duration = resp.duration
                self.actuate_joints(resp)
                #Close gripper
                self.actuate_gripper(self.gripper_open)
                self.compute_error_position(self.start.axis_d, self.start.axis_c)
                rospy.loginfo("Enf of mission!")


if __name__ == "__main__":
    Position()