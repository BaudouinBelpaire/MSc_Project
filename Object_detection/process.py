import glob, os

# Current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

current_dir = '/home/baudouin/Dissertation/dataset/ds_train/all'

# Percentage of images to be used for the test set
percentage_test = 20;

# Create and/or truncate train.txt and test.txt
file_train = open('train.txt', 'w')  
file_test = open('test.txt', 'w')

# Populate train.txt and test.txt
counter = 1  
index_test = round(100 / percentage_test)

for pathAndFilename in os.listdir(current_dir):
    if pathAndFilename.endswith('.jpg'):
        filename = os.path.join(current_dir, pathAndFilename)
        print(counter)
        if counter == index_test:
                counter = 1
                file_test.write(filename + "\n")
        else:
                file_train.write(filename + "\n")
                counter = counter + 1