#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PointStamped

class CoordinateTransformNode:
    def __init__(self):
        rospy.init_node('coordinate_transform_node')
        
        self.listener = tf.TransformListener()
        
        self.input_topic = rospy.get_param('~input_topic', '/camera/waste_coordinates')
        self.output_topic = rospy.get_param('~output_topic', '/alpha/waste_coordinates')
        self.target_frame = rospy.get_param('~target_frame', 'base_frame')
        
        self.subscriber = rospy.Subscriber(self.input_topic, PointStamped, self.callback)
        self.publisher = rospy.Publisher(self.output_topic, PointStamped, queue_size=10)
        
    def callback(self, point_msg):
        try:
            self.listener.waitForTransform(self.target_frame, point_msg.header.frame_id, rospy.Time(0), rospy.Duration(4.0))
            transformed_point = self.listener.transformPoint(self.target_frame, point_msg)
            self.publisher.publish(transformed_point)
            rospy.loginfo(transformed_point)
            rospy.loginfo(f"Transformed point: {transformed_point.point.x}, {transformed_point.point.y}, {transformed_point.point.z}")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"Transform error: {e}")
    
if __name__ == '__main__':
    try:
        node = CoordinateTransformNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
