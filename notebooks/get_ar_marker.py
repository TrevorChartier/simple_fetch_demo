#!/usr/bin/env python
import cv2
import cv2.aruco as aruco

# Pick a dictionary: 4x4 markers, 50 unique IDs
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

marker_id = 0
marker_size = 256  # pixels

# Generate marker image
img = aruco.drawMarker(aruco_dict, marker_id, marker_size)

# Save to PNG
cv2.imwrite("aruco_marker_0.png", img)

print("Saved aruco_marker_0.png")
