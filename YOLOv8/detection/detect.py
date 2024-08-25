from ultralytics import YOLO
import os

# Load a model
#model = YOLO("yolov8n.yaml")  # build a new model from scratch
model = YOLO("weights/best.pt")  # load a pretrained model (recommended for training)

# Use the model
#model.train(data="coco8.yaml", epochs=3)  # train the model
path = os.getcwd()
path = os.path.join(path, 'dataset/images/val')
for file in path[1:3]:
    img = os.path.join(path, file)
    metrics = model.val()  # evaluate model performance on the validation set
    results = model(img)  # predict on an image
    path = model.export(format="onnx")  # export the model to ONNX format