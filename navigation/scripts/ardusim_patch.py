#!/usr/bin/env python

import rospy
import socket
import struct
import json
import time

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np

class Patch:
    def __init__(self, node_name, namespace):
        rospy.init_node(node_name, anonymous=True)
        
        # Subscribers
        self.sub_imu = rospy.Subscriber("/blueboat/navigator/imu", Imu, self._imu_callback)
        self.sub_gps = rospy.Subscriber("/blueboat/navigator/gps", NavSatFix, self._gps_callback)
        self.sub_odom = rospy.Subscriber("/blueboat/navigator/odom", Odometry, self._odom_callback)
        
        # Publishers
        self.pub_pwm = rospy.Publisher("/blueboat/controller/thruster_setpoints_sim", Float64MultiArray, queue_size=1)
        
        # Publish everything
        self.rate = rospy.Rate(50)  # 50 Hz
        

        self.sock_sitl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_sitl.bind(('', 9002))
        self.sock_sitl.settimeout(0.1)

        self.imu = None
        self.gps = None
        self.odom = None

        self.namespace = rospy.get_namespace()
        self.namespace = namespace

    def _imu_callback(self, msg):
        self.imu = msg

    def _gps_callback(self, msg):
        self.gps = msg

    def _odom_callback(self, msg):
        self.odom = msg

    def looper(self):
        while not rospy.is_shutdown():
            if self.imu is None or self.odom is None:
                rospy.loginfo("Waiting for callbacks")
                time.sleep(0.5)
                continue
            
            rospy.loginfo_once("Callbacks received")
            
            try:
                data, address = self.sock_sitl.recvfrom(100)
            except Exception as ex:
                time.sleep(0.01)
                continue
            
            parse_format = 'HHI16H'
            magic = 18458

            if len(data) != struct.calcsize(parse_format):
                rospy.logwarn("Got packet of len %u, expected %u" % (len(data), struct.calcsize(parse_format)))
                continue
            
            decoded = struct.unpack(parse_format, data)

            if magic != decoded[0]:
                rospy.logwarn("Incorrect protocol magic %u should be %u" % (decoded[0], magic))
                continue

            frame_rate_hz = decoded[1]
            frame_count = decoded[2]
            pwm = decoded[3:]
            pwm_setpoint = [0, 0]

            if self.namespace == '/bluerov2':
                pwm_thrusters = pwm[0:8]
                pwm_setpoint = [(x - 1500) / 400 for x in pwm_thrusters]

            elif self.namespace == '/blueboat':
                TAM = np.array([[.5, .5], [1, -1]])
                pwm_setpoint_polar = np.array([(pwm[2] - 1500) / 500, (pwm[0] - 1500) / 500])
                pwm_setpoint = np.matmul(np.linalg.pinv(TAM), pwm_setpoint_polar)

            print(pwm_setpoint)
            pwm_setpoint = [pwm_setpoint[0], -pwm_setpoint[1]]
            msg_pwm = Float64MultiArray(data=pwm_setpoint)
            self.pub_pwm.publish(msg_pwm)

            # Set messages
            accel = (self.imu.linear_acceleration.x, self.imu.linear_acceleration.y, self.imu.linear_acceleration.z)
            gyro = (self.imu.angular_velocity.x, self.imu.angular_velocity.y, self.imu.angular_velocity.z)
            
            pose_position = (
                self.odom.pose.pose.position.x,
                self.odom.pose.pose.position.y,
                self.odom.pose.pose.position.z
            )

            pose_attitude = euler_from_quaternion([
                self.odom.pose.pose.orientation.x,
                self.odom.pose.pose.orientation.y,
                self.odom.pose.pose.orientation.z,
                self.odom.pose.pose.orientation.w
            ])
            
            twist_linear = (
                self.odom.twist.twist.linear.x,
                self.odom.twist.twist.linear.y,
                self.odom.twist.twist.linear.z,
                self.odom.twist.twist.angular.x,
                self.odom.twist.twist.angular.y,
                self.odom.twist.twist.angular.z,
            )
            
            c_time = rospy.get_time()

            # Build JSON format
            IMU_fmt = {
                "gyro": gyro,
                "accel_body": accel
            }
            JSON_fmt = {
                "timestamp": c_time,
                "imu": IMU_fmt,
                "position": pose_position,
                "attitude": pose_attitude,
                "velocity": twist_linear,                          
            }
            JSON_string = "\n" + json.dumps(JSON_fmt, separators=(',', ':')) + "\n"

            # Send to AP
            self.sock_sitl.sendto(bytes(JSON_string, "ascii"), address)

            self.rate.sleep()

def main():
    #patch = Patch(node_name="ardusim_patch", namespace='bluerov2')
    patch = Patch(node_name="ardusim_patch", namespace='blueboat')
    
    patch.looper()

if __name__ == "__main__":
    main()
