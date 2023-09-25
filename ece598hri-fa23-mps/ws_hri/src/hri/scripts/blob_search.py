#!/usr/bin/env python3
import cv2

def blob_search(image_raw, color):
    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()
    # Filter by Color
    params.filterByColor = False
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 500
    params.maxArea = 700
    # Filter by Circularity
    params.filterByCircularity = False
    # params.minCircularity = 0.3
    # params.maxCircularity = 0.9
    # square is 0.785
    # Filter by Inerita
    params.filterByInertia = False
    # Filter by Convexity
    params.filterByConvexity = False
    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)
    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)
    # lower = (110,50,50)     # blue lower
    # upper = (130,255,255)   # blue upper
    if color == "green":
        lower = (40, 100, 50)     #(50,50,50)     # green lower
        upper = (80, 255, 255)               #(70,255,255)   # green upper
    if color == "yellow":
        lower = (20, 100, 100) #(20,50,50)     # yellow lower
        upper = (30, 255, 255) #(40,255,255)   # yellow upper
    if color == "red":
        lower = (0, 70, 50)#(20,50,50)     # red lower
        upper = (10, 255, 255) #(40,255,255)   # red upper
    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)
    keypoints = detector.detect(mask_image)
    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))
    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, 0, (0,0,255))
    return blob_image_center, im_with_keypoints