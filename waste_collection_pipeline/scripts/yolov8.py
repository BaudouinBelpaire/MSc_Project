#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from waste_collection_pipeline.msg import BoundingBox, Detection
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import supervision as sv
import numpy as np

class YOLOv8ROSNode:
    def __init__(self):
        rospy.init_node('yolov8_ros_node')

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/blueboat/camera_RGB/image_color", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/yolov8/image_annotated", Image, queue_size=10)
        self.annotation_pub = rospy.Publisher("/yolov8/annotations", Detection, queue_size=10)

        self.model = self.load_yolov8_model()
        self.bounding_box_annotator = sv.BoundingBoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()

    def load_yolov8_model(self, model_path="data/yolov8_weights.pt"):
        model = YOLO(model_path)
        return model

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
        max_conf_bbox.xmin, max_conf_bbox.ymin, max_conf_bbox.xmax, max_conf_bbox.ymax = detections.xyxy[max_conf_index]
        detection.bbox = max_conf_bbox
        detection.confidence = detections.confidence[max_conf_index]
        detection.class_id = 0

        labels = [
            self.model.model.names[class_id]
            for class_id
            in detections.class_id
        ]

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
