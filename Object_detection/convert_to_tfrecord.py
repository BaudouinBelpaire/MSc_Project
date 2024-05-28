import os
import tensorflow as tf
from object_detection.utils import dataset_util

writer_val = tf.io.TFRecordWriter('output_val.tfrecord')
writer_train = tf.io.TFRecordWriter('output_train.tfrecord')
path = '/home/baudouin/Dissertation/dataset/plastic_bottles_dataset/ds/ann_txt'
img_dir = '/home/baudouin/Dissertation/dataset/plastic_bottles_dataset/ds/img_new'

N = len([entry for entry in os.listdir(path)])

N_validation = int(0.1 * N)
i = 1

def write_img_tfrecord(writer, filename, image_path):
    with tf.io.gfile.GFile(image_path, 'rb') as fid:
        encoded_image_data = fid.read()

    file = open(filename, "r")
    for txt in file:
        data = txt.split(" ")
        data[-1] = str(int(data[-1]))

        tf_label_and_data = tf.train.Example(features=tf.train.Features(feature={
        'image/height': dataset_util.int64_feature(int(data[6])),
        'image/width': dataset_util.int64_feature(int(data[5])),
        'image/filename': dataset_util.bytes_feature(filename.encode('utf-8')),
        'image/source_id': dataset_util.bytes_feature(filename.encode('utf-8')),
        'image/encoded': dataset_util.bytes_feature(encoded_image_data),
        'image/format': dataset_util.bytes_feature('jpg'.encode('utf-8')),
        'image/object/bbox/xmin': dataset_util.float_list_feature([float(data[1])]),
        'image/object/bbox/xmax': dataset_util.float_list_feature([float(data[3])]),
        'image/object/bbox/ymin': dataset_util.float_list_feature([float(data[2])]),
        'image/object/bbox/ymax': dataset_util.float_list_feature([float(data[4])]),
        'image/object/class/text': dataset_util.bytes_feature('plastic_bottle'.encode('utf-8')),
        'image/object/class/label': dataset_util.int64_list_feature([int(data[0])]),
    }))
        writer.write(tf_label_and_data.SerializeToString())
        print(f'TFRecord written to my path')

for file in os.listdir(path):
    filename = os.path.join(path, file)
    image_name = file.replace(".txt", "")
    img_path = os.path.join(img_dir, image_name)
    if i<N_validation:
        write_img_tfrecord(writer_val, filename, img_path)
    else:
        write_img_tfrecord(writer_train, filename, img_path)
    i = i + 1

writer_val.close()
writer_train.close()

dataset_val = tf.data.TFRecordDataset('output_val.tfrecord')
for raw_record in dataset_val.take(1):
    example = tf.train.Example()
    example.ParseFromString(raw_record.numpy())
    print(example)