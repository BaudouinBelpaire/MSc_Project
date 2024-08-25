#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def publish_setpoint():
    rospy.init_node('setpoint_position_publisher', anonymous=True)
    pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    setpoint = PoseStamped()
    setpoint.header.frame_id = "world_ned"
    setpoint.pose.position.x = 5.0  # X position in meters
    setpoint.pose.position.y = 0.0   # Y position in meters
    setpoint.pose.position.z = 0.0   # Z position in meters

    # Optional: Set orientation (in quaternions)
    setpoint.pose.orientation.x = 0.0
    setpoint.pose.orientation.y = 0.0
    setpoint.pose.orientation.z = 0.0
    setpoint.pose.orientation.w = 1.0

    while not rospy.is_shutdown():
        setpoint.header.stamp = rospy.Time.now()  # Update timestamp
        pub.publish(setpoint)
        rospy.loginfo("Publishing setpoint: x=%.2f, y=%.2f, z=%.2f" % (
            setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z))
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_setpoint()
    except rospy.ROSInterruptException:
        pass
