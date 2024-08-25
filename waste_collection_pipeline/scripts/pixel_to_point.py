#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
from waste_collection_pipeline.msg import Detection
from geometry_msgs.msg import PointStamped
import time

class PixelToPoint:
    def __init__(self):
        rospy.init_node('pixel_to_point_node', anonymous=True)
        
        # Initialize camera intrinsic parameters
        self.fx = 0
        self.fy = 0
        self.cx = 0
        self.cy = 0

        # Flag to check if camera info has been received
        self.camera_info_received = False
        
        # Subscribing to the camera info, image, and depth topics
        self.camera_info_sub = rospy.Subscriber('/zed/zed_node/left/camera_info', CameraInfo, self.camera_info_callback)
        self.image_sub = rospy.Subscriber('/zed/zed_node/left/image_rect_color', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/zed/zed_node/depth/depth_registered', Image, self.depth_callback)
        self.detected_waste_sub = rospy.Subscriber("/yolov8/annotations", Detection, self.waste_callback)
        self.coordinate_waste_pub = rospy.Publisher("/camera/waste_coordinates", PointStamped, queue_size=1)
        
        self.bridge = CvBridge()
        self.depth_image = None

        self.object_detected = False

    def camera_info_callback(self, data):
        # Extract camera intrinsic parameters from camera info message
        self.fx = data.K[0]
        self.fy = data.K[4]
        self.cx = data.K[2]
        self.cy = data.K[5]
        self.camera_info_received = True
        #rospy.loginfo("Camera intrinsic parameters received")

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            #rospy.loginfo(self.depth_image)
        except CvBridgeError as e:
            rospy.logerr(e)
    
    def image_callback(self, data):
        if self.depth_image is None or not self.camera_info_received:
            return

        if self.object_detected:
            u = self.x_c
            v = self.y_c

            # Get depth value at (u, v)
            Z = self.depth_image[v, u]
            rospy.loginfo(Z)
            
            # Calculate X, Y, Z
            X = (u - self.cx) * Z / self.fx
            Y = (v - self.cy) * Z / self.fy
            
            rospy.loginfo(f"3D Coordinates: X={X}, Y={Y}, Z={Z}")
            self.object_detected = False

            current_time = rospy.Time.now()

            waste_position = PointStamped()
            waste_position.header.stamp = current_time
            waste_position.header.frame_id = 'lens_frame'
            waste_position.point.x = X
            waste_position.point.y = Y
            waste_position.point.z = Z
            self.coordinate_waste_pub.publish(waste_position)


    def waste_callback(self, data):
        bbox = data.bbox
        xmin, ymin, xmax, ymax = bbox.x_min, bbox.y_min, bbox.x_max, bbox.y_max #get median depth of the whole rectangle 
        self.x_c = int((xmax + xmin)/2)
        self.y_c = int((ymax + ymin)/2)
        self.object_detected = True
        #rospy.loginfo("Waste callback")

if __name__ == '__main__':
    try:
        PixelToPoint()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
