import os 
import json
import shutil 

dataset = '/home/baudouin/Dissertation/dataset/plastic_bottles_dataset'
img_path = os.path.join(dataset, 'ds/img')
img_output_path = os.path.join(dataset, 'ds/img_new')
output_dir = os.path.join(dataset, 'ds/ann_txt')
path = os.path.join(dataset, 'ds/ann')

def convert_to_yolo_format(json_file, i):
    with open(json_file) as f:
        data = json.load(f)

    height_img = data['size']['height']
    width_img = data['size']['width']

    yolo_ann = []

    for obj in data['objects']:
        points = obj['points']['exterior']
        xmin = points[0][0]/width_img
        ymin = points[0][1]/height_img
        xmax = points[1][0]/width_img
        ymax = points[1][1]/height_img

        cx = (xmin + xmax)/2/width_img
        cy = (ymin + ymax)/2/height_img
        width = (xmax - xmin)
        height = (ymax - ymin)

        if(width>0.03 and height>0.03):
            yolo_ann.append(f"{0} {xmin} {ymin} {xmax} {ymax} {width_img} {height_img}")

    if yolo_ann != []:
        filename = str(i) + '.jpg'
        output_file = os.path.join(output_dir, filename + '.txt')
        with open(output_file, 'w') as f:
            f.write('\n'.join(yolo_ann))
            f.close()
        image_name = os.path.basename(json_file).replace(".json","")
        shutil.copy(os.path.join(img_path, image_name), os.path.join(img_output_path, filename))

i = 1

for filename in os.listdir(path):
    if filename.endswith('json'):
        convert_to_yolo_format(os.path.join(path, filename), i)
        i = i + 1