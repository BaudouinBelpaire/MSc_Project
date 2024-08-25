#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PointStamped
from stonefish_ros.msg import INS
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Float64
from math import atan2, sqrt, pi, degrees
import math
import utm
import csv
from scipy.spatial.transform import Rotation as R

class PIDController:
    def __init__(self):
        rospy.init_node('pid_controller_node')

        # CSV file setup
        file_path1 = '/home/baudouin/catkin_ws/src/odometry_activePID.csv'
        self.file_path2 = '/home/baudouin/catkin_ws/src/odometry_inactivePID.csv'
        self.csv_file = open(file_path1, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['X','Y','Theta'])

        self.target_point_sub = rospy.Subscriber('/alpha/target_point', PointStamped, self.target_point_callback)
        self.odom_sub = rospy.Subscriber('/blueboat/navigator/odom', Odometry, self.odom_callback)
        self.thruster_pub = rospy.Publisher('/blueboat/controller/thruster_setpoints_sim', Float64MultiArray, queue_size=10)

        self.target_point = [2.0,0.0]
        self.current_pose = [0.0, 0.0, 0.0]
        self.last_time = rospy.Time.now()

        self.achieved = False
        self.flag = 1

       # PID parameters for linear movement
        self.linear_kP = 0.5 #0.5
        self.linear_kI = 0.01 #0.01
        self.linear_kD = 0.5 #0.5

        # PID parameters for angular movement
        self.angular_kP = 0.01  # Increased from 1.5
        self.angular_kI = 0.0  # Increased from 0.1
        self.angular_kD = 0.0  # Increased from 0.05

        # Initialize PID errors and timestamps
        self.linear_integral = 0.0
        self.previous_linear_error = 0.0
        self.angular_integral = 0.0
        self.previous_angular_error = 0.0
        self.previous_time = rospy.Time.now()

    def target_point_callback(self, msg):
        self.target_point = [msg.point.x, msg.point.y]

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        rotation = R.from_quat(quat)

        # Convert to Euler angles (in radians)
        euler_angles = rotation.as_euler('xyz', degrees=False)
        th = -euler_angles[2]

        self.current_pose = [x,y,th]
        self.csv_writer.writerow(self.current_pose)
        #rospy.loginfo(self.current_pose)

    def calculate_error(self):
        if self.target_point and self.current_pose:
            target_x, target_y = self.target_point[0], self.target_point[1]
            current_x, current_y = self.current_pose[0], self.current_pose[1]
            th = self.current_pose[2]

            distance_error = sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
            distance_error = math.copysign(distance_error, target_x-current_x)
            rospy.loginfo(f"Distance error: {distance_error} m.")
            heading_error = atan2(target_y - current_y, target_x - current_x) - th
            #rospy.loginfo(f"Heading error: {heading_error} rad")
            heading_error = 0
            
            if heading_error<-pi:
                heading_error = heading_error + 2*pi

            if heading_error>pi:
                heading_error = heading_error - 2*pi

            #rospy.loginfo([distance_error,heading_error])
            if abs(distance_error) < 0.1:
                    self.achieved = False

            return distance_error, heading_error
        
    def calculate_pid(self, kP, kI, kD, error, previous_error, integral, delta_time):
        integral += error * delta_time
        derivative = (error - previous_error) / delta_time
        output = kP * error + kI * integral + kD * derivative
        return output, integral

    def control(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if self.target_point is not None and self.current_pose is not None:
                distance_error, heading_error = self.calculate_error()
                heading_error = degrees(heading_error)
                current_time = rospy.Time.now()
                delta_time = (current_time - self.previous_time).to_sec()
                self.previous_time = current_time

                linear_velocity, self.linear_integral = self.calculate_pid(self.linear_kP, self.linear_kI, self.linear_kD, distance_error, self.previous_linear_error, self.linear_integral, delta_time)
                angular_velocity, self.angular_integral = self.calculate_pid(self.angular_kP, self.angular_kI, self.angular_kD, heading_error, self.previous_angular_error, self.angular_integral, delta_time)
                self.previous_linear_error = distance_error
                self.previous_angular_error = heading_error

                thrust = Float64MultiArray()

                thrust_port = linear_velocity + angular_velocity
                thrust_stbd = linear_velocity - angular_velocity

                max_thrust = 10
                thrust_port = max(min(thrust_port, max_thrust), -max_thrust)/max_thrust
                thrust_stbd = max(min(thrust_stbd, max_thrust), -max_thrust)/max_thrust

                thrust.data = [-thrust_port, thrust_stbd]

                if(self.achieved is True):
                    thrust.data = [0,0]
                    self.thruster_pub.publish(thrust)
                    if(self.flag==True):
                        rospy.loginfo("Point achieved")
                        self.csv_file = open(self.file_path2, 'w')
                        self.csv_writer = csv.writer(self.csv_file)
                        self.csv_writer.writerow(['X','Y','Theta'])
                        self.flag = False

                rospy.loginfo(f"Thruster commands : {thrust.data}")
                self.thruster_pub.publish(thrust)

            rate.sleep()

if __name__ == '__main__':
    try:
        controller = PIDController()
        controller.control()
    except rospy.ROSInterruptException:
        pass
