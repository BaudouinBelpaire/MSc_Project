import cv2
import numpy as np
import os
import time

path = '/home/baudouin/Dissertation/camera calibration/images'
os.chdir(path)

# Load chessboard images
images = os.listdir(path)  # List of chessboard images

# Define chessboard size
pattern_size = (12, 5)
square_size = 11

# Arrays to store object points and image points
object_points = []  # 3D points of chessboard corners in real world
image_points = []  # 2D points of chessboard corners in image

# Generate object points
objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size

for image_file in images:
    image = cv2.imread(image_file)

    # Convert image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

    if ret:
        # Store object points and image points
        object_points.append(objp)
        image_points.append(corners)

# Calibrate camera
ret, camera_matrix, distortion_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    object_points, image_points, gray.shape[::-1], None, None)

print(camera_matrix)