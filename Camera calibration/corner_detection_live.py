import cv2

cap = cv2.VideoCapture(0)
pattern_size = (13,6)

while(True):
    ret, frame = cap.read()

    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        print(corners)

    cv2.imshow('frame',frame)