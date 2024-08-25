#!/usr/bin/env python

import rospy
import csv
import os
from stonefish_ros.msg import INS, ThrusterState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Float64

class DataCollector:
    def __init__(self):
        # Global variables to store data
        self.thruster_data = None
        self.boat_state_data = None
        self.distance_propeller = 0.570

        # CSV file setup
        file_path = '/home/baudouin/catkin_ws/src/data_linear2.csv'
        self.csv_file = open(file_path, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time', 'Thrust_X', 'Thrust_Y','Torque', 'Surge_Velocity', 'Sway_Velocity', 'Yaw_Rate', 'Surge_acceleration'])

    def thruster_callback(self, data):
        self.thruster_data = data

    def odom_callback(self, data):
        self.odom = data

    def collect_data(self):
        # Initialize the ROS node
        rospy.init_node('data_collector')
        
        # Subscribe to the topics
        rospy.Subscriber('/blueboat/controller/thruster_state', ThrusterState, self.thruster_callback)
        rospy.Subscriber('/blueboat/navigator/odom', Odometry, self.odom_callback)
        self.thruster_pub = rospy.Publisher('/blueboat/controller/thruster_setpoints_sim', Float64MultiArray, queue_size=10)

        self.thruster_data = None
        self.boat_state_data = None

        tmp_surge_vel = 0
        tmp_time = rospy.get_time()
        
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            thrust = Float64MultiArray()
            thrust.data = [0.2,0.2]
            self.thruster_pub.publish(thrust)

            if self.thruster_data is not None and self.odom is not None:
                current_time = rospy.get_time()
                L = self.distance_propeller
                thrust_x = self.thruster_data.thrust[0] + self.thruster_data.thrust[1]
                thrust_y = 0
                torque =  self.thruster_data.thrust[0]*L/2 - self.thruster_data.thrust[1]*L/2
                surge_velocity = self.odom.twist.twist.linear.x
                sway_velocity = self.odom.twist.twist.linear.y
                yaw_rate = self.odom.twist.twist.angular.z
                surge_acceleration = (surge_velocity-tmp_surge_vel)/(current_time-tmp_time)
                tmp_time = current_time
                tmp_surge_vel = surge_velocity
                
                self.csv_writer.writerow([current_time, thrust_x, thrust_y, torque, surge_velocity, sway_velocity, yaw_rate, surge_acceleration])
                
                # Reset the data
                self.thruster_data = None
                self.boat_state_data = None
                
            rate.sleep()

    def close_csv_file(self):
        self.csv_file.close()

if __name__ == '__main__':
    try:
        data_collector = DataCollector()
        data_collector.collect_data()
    except rospy.ROSInterruptException:
        pass
    finally:
        data_collector.close_csv_file()
