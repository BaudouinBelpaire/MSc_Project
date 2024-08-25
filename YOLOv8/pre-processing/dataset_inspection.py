import os
import cv2

path = '/home/baudouin/Dissertation/dataset/plastic_bottles_dataset/ds'
img_dir = os.path.join(path, 'img_processed/')

N = len([files for files in os.listdir(img_dir)])

N_img_display = 30
steps = int(N/N_img_display)-1


for obj in os.listdir(img_dir)[0:N:steps]:

    img = cv2.imread(os.path.join(img_dir, obj))
    h, w, c = img.shape

    label_name = obj.replace(".jpg",".txt")
    label_path = os.path.join(path, 'ann_txt', label_name)
    with open(label_path, "r") as file:
        for line in file:
            values = line.split()
            values = [float(value) for value in values]
            cx = values[1]*w
            cy = values[2]*h
            width = values[3]*w
            height = values[4]*h
            x1 = int(cx-width/2)
            x2 = int(cx+width/2)
            y1 = int(cy-height/2)
            y2 = int(cy+height/2)
            cv2.rectangle(img, (x1,y1), (x2,y2), (255,0,0), 3)

        cv2.namedWindow("visualisation", cv2.WINDOW_NORMAL) 
        cv2.resizeWindow("visualisation", 640, 480) 
        cv2.imshow('visualisation',img)
        cv2.waitKey(0) #Press 'Esc.' to pass to next image
    
    # closing all open windows 
        cv2.destroyAllWindows()  
