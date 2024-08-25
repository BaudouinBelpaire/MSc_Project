#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from waste_collection_pipeline.msg import BoundingBox, Detection
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import supervision as sv
import numpy as np
from geometry_msgs.msg import PointStamped

class YOLOv8ROSNode:
    def __init__(self):
        rospy.init_node('yolov8_ros_node')

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/zed/zed_node/left/image_rect_color", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/yolov8/image_annotated", Image, queue_size=10)
        self.annotation_pub = rospy.Publisher("/yolov8/annotations", Detection, queue_size=10)
        self.point_sub = rospy.Subscriber("/alpha/waste_coordinates", PointStamped, self.point_callback)
        
        self.x = 0 
        self.y = 0 
        self.z = 0
        self.point = False
        self.model = self.load_yolov8_model()
        self.bounding_box_annotator = sv.BoundingBoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()

    def load_yolov8_model(self, model_path="/home/nx/catkin_ws/src/waste_collection_pipeline/data/yolov8_weights.pt"):
        model = YOLO(model_path)
        return model
        
    def point_callback(self, data):
    	self.x = data.point.x
    	self.y = data.point.y
    	self.z = data.point.z
    	self.point = True

    def process_frame(self, frame):
        result = self.model(frame, agnostic_nms=True, show_labels=True)[0]
        detections = sv.Detections.from_ultralytics(result)
        bounding_box = detections.xyxy
        confidence = detections.confidence

        if len(detections.confidence) == 0:
        # No detections, return the original frame
            return frame

        # Get the index of the maximum confidence
        max_conf_index = np.argmax(detections.confidence)

        # Get the bounding box coordinates of the detection with the highest confidence
        max_conf_bbox = BoundingBox()
        detection = Detection()
        max_conf_bbox.x_min, max_conf_bbox.y_min, max_conf_bbox.x_max, max_conf_bbox.y_max = detections.xyxy[max_conf_index]
        x_min = max_conf_bbox.x_min
        y_min = max_conf_bbox.y_min
        detection.bbox = max_conf_bbox
        detection.confidence = detections.confidence[max_conf_index]
        detection.class_id = 0
        
        if self.point is not None:
        	label = f"X:{self.x:.2f}, Y:{self.y:.2f}, Z:{self.z:.2f} m"
        	#label = str([self.x, self.y, self.z])
        	rospy.loginfo(label)
        	cv2.putText(frame, label, (int(max_conf_bbox.x_min), int(max_conf_bbox.y_min)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        labels = [
            self.model.model.names[class_id]
            for class_id
            in detections.class_id
        ]
        
        self.annotation_pub.publish(detection)

        annotated_image = self.bounding_box_annotator.annotate(scene=frame, detections=detections)
        #annotated_image = self.label_annotator.annotate(scene=annotated_image, detections=detections, labels=labels)

        return annotated_image

    def image_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        annotated_frame = self.process_frame(frame)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        YOLOv8ROSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
