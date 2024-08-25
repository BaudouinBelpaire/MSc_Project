#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PointStamped
from stonefish_ros.msg import INS
from nav_msgs.msg import Odometry
from math import atan2, sqrt, pi, cos, sin
import utm

class Odom:
    def __init__(self):
        rospy.init_node('odometry_node')

        self.ins_sub = rospy.Subscriber('/blueboat/navigator/ins', INS, self.ins_callback)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        self.current_pose = [0.0, 0.0, 0.0]
        self.vx, self.vy, self.w = 0, 0, 0
        self.last_time = rospy.Time.now()

    def ins_callback(self, msg):

        self.vx = msg.body_velocity.y
        self.vy = msg.body_velocity.x
        self.w = -msg.rpy_rate.z

    def compute_odom(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()
            dt = (self.current_time - self.last_time).to_sec()
            self.last_time = self.current_time

            th = self.current_pose[2] + self.w*dt

            if th>pi:
                th = th - 2*pi
            if th<-pi:
                th = th + 2*pi

            x = self.current_pose[0] + self.vx*dt*cos(th)
            y = self.current_pose[1] + self.vx*dt*sin(th)

            self.compass = th

            self.current_pose = [x, y, th]

            odom = Odometry()
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation.z = th

            #rospy.loginfo(odom)
            self.odom_pub.publish(odom)

            rate.sleep()

if __name__ == '__main__':
    try:
        odometry= Odom()
        odometry.compute_odom()
    except rospy.ROSInterruptException:
        pass
