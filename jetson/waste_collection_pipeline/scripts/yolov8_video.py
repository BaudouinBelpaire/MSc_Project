#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from ultralytics import YOLO
from sensor_msgs.msg import CameraInfo, Image
import supervision as sv
from waste_collection_pipeline.msg import BoundingBox, Detection

class YOLOv8VideoProcessor:
    def __init__(self, video_path):
        self.video_path = video_path
        self.cap = cv2.VideoCapture(self.video_path)
        self.model = self.load_yolov8_model()
        self.bounding_box_annotator = sv.BoundingBoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()

        self.image_pub = rospy.Publisher("/yolov8/image_annotated", Image, queue_size=10)
        self.annotation_pub = rospy.Publisher("/yolov8/annotations", Detection, queue_size=10)

    def load_yolov8_model(self, model_path="/home/baudouin/catkin_ws/src/waste_collection_pipeline/data/yolov8_weights.pt"):
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
        max_conf_bbox.x_min, max_conf_bbox.y_min, max_conf_bbox.x_max, max_conf_bbox.y_max = detections.xyxy[max_conf_index]
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

    def run(self):
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break

            annotated_frame = self.process_frame(frame)
            cv2.imshow('YOLOv8 Video Processor', annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    video_path = '/home/baudouin/catkin_ws/src/waste_collection_pipeline/data/OSL2.mp4'  # Replace with your video file path
    processor = YOLOv8VideoProcessor(video_path)
    processor.run()
