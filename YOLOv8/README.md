# YOLOv8 Waste Detection

## Preprocessing:
Download the plastic dataset: <br /> https://datasetninja.com/plastic-bottles

Run the preprocessing of the dataset using the 'convert_format_file.py' script.

Inspect the remaining images with the 'dataset_inspection.py' script.

## Training:

Train the model using the 'train.py' script with the original weights 'yolov8n.pt'.

The 'dataset.yaml' file is used to define the location of the training images.

## Detection:

Run the 'detect.py' script for object detection on an image.

Run the 'yolo_inference.py' script for object detection on an video feed.
