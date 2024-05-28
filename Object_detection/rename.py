import os 

img_path = '/home/baudouin/Dissertation/dataset/ds_train/img_new'
label_path = '/home/baudouin/Dissertation/dataset/ds_train/ann_txt'

i=1

buffer = []

for file in os.listdir(img_path):
    buffer.append(file)

for file in buffer:
    print(file)
    file_txt = file + '.txt'
    filename_img = str(i) + '.jpg'
    filename_txt = filename_img + '.txt'
    os.chdir(img_path)
    os.rename(file, filename_img)
    os.chdir(label_path)
    os.rename(file_txt, filename_txt)
    i = i + 1