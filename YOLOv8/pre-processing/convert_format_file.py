import os 
import json
import shutil
import cv2

img_train = 'images/train'
img_val = 'images/val'
img_test = 'images/test'
label_train = 'labels/train'
label_val = 'labels/val'
label_test = 'labels/test'

path = os.getcwd()
dataset = os.path.join(path, 'plastic_bottles_dataset/ds')
os.chdir(dataset)
img_path = os.path.join(dataset, 'img')
img_output_path = os.path.join(dataset, 'img_processed')
output_dir = os.path.join(dataset, 'ann_txt')
path = os.path.join(dataset, 'ann')

i = 1
val = 0.2
test = 0.1

#Threshold for image size:
thresh_x = 0.05
thresh_y = 0.05

N_items_max = 8 #Max number of items per picture

def remove_directory(path):
    if os.path.exists(path):
        shutil.rmtree(path)
    os.mkdir(path)

def convert_to_yolo_format(json_file, i):
    with open(json_file) as f:
        data = json.load(f)

    height_img = data['size']['height']
    width_img = data['size']['width']

    yolo_ann = []

    items = 0
    correct_items = 0 

    for obj in data['objects']:
        points = obj['points']['exterior']
        xmin = points[0][0]/width_img
        ymin = points[0][1]/height_img
        xmax = points[1][0]/width_img
        ymax = points[1][1]/height_img

        cx = (xmin + xmax)/2
        cy = (ymin + ymax)/2
        width = (xmax - xmin)
        height = (ymax - ymin)
        #print(cx,cy,width,height)
        items += 1

        if(width>thresh_x or height>thresh_y):
            yolo_ann.append(f"{0} {cx} {cy} {width} {height}")
            correct_items += 1

    if yolo_ann != [] and len(yolo_ann)<N_items_max and correct_items == items:
        filename = str(i) + '.jpg'
        image_name = os.path.basename(json_file).replace(".json","")
        output_file = os.path.join(output_dir, str(i) + '.txt')
        with open(output_file, 'w') as f:
            f.write('\n'.join(yolo_ann))
            f.close()
        img = cv2.imread(os.path.join(img_path, image_name))
        img = cv2.resize(img, (1280, 720))
        cv2.imwrite(os.path.join(img_output_path, filename), img)
        i = i + 1

    return i

def split_dataset(split_img_output_path, split_label_output_path, img_name):
    label_name = img_name.replace(".jpg",".txt")
    shutil.copy(os.path.join(img_output_path, img_name), os.path.join(split_img_output_path, img_name))
    shutil.copy(os.path.join(output_dir, label_name), os.path.join(split_label_output_path, label_name))

remove_directory('images/')
remove_directory('labels/')
remove_directory('ann_txt/')
remove_directory('img_processed/')

os.mkdir(img_train)
os.mkdir(img_val)
os.mkdir(img_test)
os.mkdir(label_train)
os.mkdir(label_val)
os.mkdir(label_test)

for filename in os.listdir(path):
    if filename.endswith('.json'):
        i = convert_to_yolo_format(os.path.join(path, filename), i)

files = [files for files in os.listdir(os.path.join(dataset,img_output_path))]
N = len(files)
print(N)

N_val = int(N*val)
N_test = int(N*test) 
N_train = N-N_val-N_test

for n in range(0,N_train):
    split_dataset(img_train, label_train, files[n])
for n in range(N_train,N_train+N_val):
    split_dataset(img_val, label_val, files[n])
for n in range(N_train+N_val,N):
    split_dataset(img_test, label_test, files[n])