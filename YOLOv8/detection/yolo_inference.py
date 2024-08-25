import cv2
from ultralytics import YOLO
import supervision as sv
import numpy as np

def initialize_camera(camera_index="videos/OSL2.mp4"): #"videos/vid3.mp4"
    cap = cv2.VideoCapture(camera_index)
    return cap

def initialize_video_recording(cap):
    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4)) 
   
    size = (frame_width, frame_height) 
    result = cv2.VideoWriter('videos/recording.mp4',  
                         cv2.VideoWriter_fourcc(*'MJPG'), 
                         10, size)
    return result

def load_yolov8_model(model_path="train/train2/weights/best.pt"):
    model = YOLO(model_path)
    return model

def process_frame(frame, model):
    result = model(frame, agnostic_nms=True, show_labels=True)[0]
    detections = sv.Detections.from_ultralytics(result)
    print(f"Results: {detections.xyxy}")

    bounding_box_annotator = sv.BoundingBoxAnnotator()
    label_annotator = sv.LabelAnnotator()

    labels = [
        model.model.names[class_id]
        for class_id
        in detections.class_id
    ]

    annotated_image = bounding_box_annotator.annotate(
        scene=frame, detections=detections)
    #annotated_image = label_annotator.annotate(
        #scene=annotated_image, detections=detections, labels=labels)
    
    return annotated_image

def main():
    cap = initialize_camera()
    out = initialize_video_recording(cap)
    model = load_yolov8_model()

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Failed to capture frame")
            break

        annotated_frame = process_frame(frame, model)

        cv2.namedWindow("yolov8", cv2.WINDOW_NORMAL) 
        cv2.resizeWindow("yolov8", 640, 480) 
        cv2.imshow("yolov8", annotated_frame)
        out.write(annotated_frame)

        if cv2.waitKey(20) == 27:
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
