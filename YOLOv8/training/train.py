import torch
from ultralytics import YOLO

# Load a pre-trained YOLOv8 model
model = YOLO('yolov8n.pt')

# Set the model parameters for training
epochs = 30  # number of training epochs
batch_size = 16  # batch size
img_size = 640  # input image size
save_dir = 'train/'
lr0 = 0.001
lrf = 0.001

# Train the model using the dataset.yaml configuration
model.train(data='dataset.yaml', epochs=epochs, batch=batch_size, imgsz=img_size, project=save_dir, single_cls=True, lrf=lrf, lr0=lr0)

# Evaluate the model on the validation dataset
metrics = model.val(data='dataset.yaml')
print(metrics)

# Save the trained model
model.save('yolov8_final_v2.pt')