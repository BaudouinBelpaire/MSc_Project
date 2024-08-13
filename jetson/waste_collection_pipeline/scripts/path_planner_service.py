#!/usr/bin/env python3

import rospy
import numpy as np
from waste_collection_pipeline.srv import PathPlanner, PathPlannerResponse, ForwardKinematics
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def linear_interpolation(start, end, steps):
    return [start + (end - start) * i / (steps - 1) for i in range(steps)]

def compute_velocity(path, time_step):
    velocities = np.diff(path, axis=0) / time_step
    return velocities

def handle_joint_path_planner(req):
    steps = 100
    duration = 2
    time_step = duration / steps

    path = np.zeros((steps, 3))
    path[:, 0] = linear_interpolation(req.axis_d_start, req.axis_d_end, steps)
    path[:, 1] = linear_interpolation(req.axis_c_start, req.axis_c_end, steps)

    # Call forward kinematics service
    rospy.wait_for_service('compute_forward_kinematics')
    forward_kinematics_srv = rospy.ServiceProxy('compute_forward_kinematics', ForwardKinematics)

    Theta1 = 0 
    Theta9= 0

    for i in range(len(path)):
        th3 = path[i, 0]
        th7 = path[i, 1]

        try:
            resp = forward_kinematics_srv(Theta1, th3, th7, Theta9)
            x = np.array(resp.x)
            y = np.array(resp.y)
            z = np.array(resp.z)
            path[i, 2] = x[-1]
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            path[i, 2] = float('nan') 

    velocity = np.zeros((len(path), 3))
    velocity[1:, 0] = compute_velocity(path[:, 0], time_step)
    velocity[1:, 1] = compute_velocity(path[:, 1], time_step)
    velocity[1:, 2] = compute_velocity(path[:, 2], time_step)

    trajectory = JointTrajectory()
    trajectory.header.stamp = rospy.Time.now()
    trajectory.joint_names = ['blueboat/alpha_axis_d', 'blueboat/alpha_axis_c', 'x_axis']
    for i in range(steps):
        point = JointTrajectoryPoint()
        point.positions = [path[i, 0], path[i, 1], path[i, 2]]
        point.velocities = [velocity[i, 0], velocity[i, 1], velocity[i, 2]]
        point.time_from_start = rospy.Duration(i * time_step)
        trajectory.points.append(point)

    return PathPlannerResponse(path=trajectory)

def joint_path_planner_server():
    rospy.init_node('joint_path_planner_server')
    s = rospy.Service('joint_path_planner', PathPlanner, handle_joint_path_planner)
    rospy.loginfo("Ready to plan joint paths.")
    rospy.spin()

if __name__ == "__main__":
    joint_path_planner_server()