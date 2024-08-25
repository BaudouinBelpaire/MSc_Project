# YOLOv8 Waste Detection

## Preprocessing:

![image](https://github.com/user-attachments/assets/5989cb99-718c-4407-94b4-6326200997f8)

Download the plastic dataset: <br /> https://datasetninja.com/plastic-bottles

Run the preprocessing of the dataset using the 'convert_format_file.py' script.

Inspect the remaining images with the 'dataset_inspection.py' script.

## Training:

![image](https://github.com/user-attachments/assets/7318b928-4cff-429c-a102-83b5647555e7)

Train the model using the 'train.py' script with the original weights 'yolov8n.pt'.

The 'dataset.yaml' file is used to define the location of the training images.

## Detection:

![image](https://github.com/user-attachments/assets/42e27044-e9ff-4527-8eff-277623256ac0)

Run the 'detect.py' script for object detection on an image.

Run the 'yolo_inference.py' script for object detection on an video feed.
