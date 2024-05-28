Waste detection: 
- Use of YoloV3 objection detection algorithm to detect waste 
- Successfully installed but works with 80 standard classes and need training on specific dataset to recognise and locate plastic bottles - required to install with python 3.8 to access tensorflow 2.11.1. Link of the Yolo Github used: https://github.com/zzh8829/yolov3-tf2
- Downloaded 'Plastic bottles in rivers' dataset gathering pictures of bottles in rivers. Link: https://datasetninja.com/plastic-bottles
- The dataset is in a similar format as COCO dataset: pictures in an image folder with '.jpg' format and bounding box annotations in a label folder with all information related into a '.json' format file. Contains 989 images with 5534 labeled objects. 
- Reduce dataset size by removing images with small bottles by selecting boxes with an area above a threshold. Took the required information of these into a '.txt' file and renamed the files. Should I resize the image when processing them, eventhough Yolo algorithm is supposed to do it? 
- Then, the data could be transformed into a tfrecord dataset where annotations and pictures are gathered in the same format as bytes for fastest computation. 
- During training, the loss after 10 epochs is around 10, however, the model does not detect any wastes on pictures. 

--> The dataset generated in tfrecord format might not reflect reality. Possibility the number of classes from 80 to 1 did not work for the trained model. Stopped here to move on next part and avoid losing too much time. Thinking to use a tracker on OpenCV to track the bottle in water once initialised to skip the use of Yolo for object detection. Other idea would be to use SSD algorithm and test but might be the same output as the one with Yolo.

Next steps of the week: Camera calibration + Frame transform

Camera calibration:
Calibrated camera using a checkerboard with Python algorithm
