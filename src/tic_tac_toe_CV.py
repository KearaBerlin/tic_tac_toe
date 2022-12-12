# -*- coding: utf-8 -*-
"""
Created on Sun Oct 23 15:42:50 2022

@author: amara
"""

import cv2 as cv
import matplotlib.pyplot as plt

def get_image():
	
    # define a video capture object
    vid = cv.VideoCapture("/dev/video2")
    # Capture the video frame
    # by frame
    ret, frame = vid.read()
    # while frame is None or frame.empty:
    #     ret, frame = vid.read()
        
    # Display the resulting frame
    cv.imshow('frame', frame)
    # if input('Continue? y/n') == 'y':
    	# After the loop release the cap object
    #    vid.release()
        # Destroy all the windows
    #    cv.destroyAllWindows()
    return frame

def create_game_state(img):
    # img = cv.imread("test_images/Board space6.png", 1)
    #cv.imshow('img', img)
    
    img = preprocess_image(img)
    
    height, width, c = img.shape 
    h = height // 3
    w = width // 3

    board = []
    for i in range(3):
        for j in range(3):
            #filename = str(i*3 + j) + ".jpg"
            #cv.imwrite(filename, img[i*h:(i+1)*h,j*h:(j+1)*h,:])
            detect_pred = detect(img[i*h:(i+1)*h,j*w:(j+1)*h,:])
            if detect_pred == -1:
                letter = 'O'
            elif detect_pred == 1:
                letter = 'X'
            else:
                letter = ""
            print(i*3 + j, ': ', letter)
            board.append(letter)
    
    return board
    
def detect(img):
    #detect X, O or empty
    height, width, c = img.shape
    b_count, r_count = 0, 0
    for r in range(height):
        for c in range(width):
            blue = int(img[r, c, 0])
            red = int(img[r, c, 2])
            if blue - red > 55:
                b_count += 1
            elif red - blue > 55:
                r_count += 1
                
    if r_count - b_count > 60:
        return -1
    elif b_count - r_count > 50:
        return 1
    else:
        return 0
    
                
    return []

def preprocess_image(img):
    height, width, c = img.shape
    #plt.imshow(img[:,:,::-1])
    #cv.imwrite('og_img.jpg', img[:,:,:])
    center = (400, 200)
    m = cv.getRotationMatrix2D(center=center, angle=175, scale=1)
    img = cv.warpAffine(src=img, M=m, dsize=(width, height))
    cropped_img = img[120:230,365:465,:]
    #plt.imshow(cropped_img[:,:,::-1])
    #cv.imwrite('img.jpg', cropped_img[:,:,:])
    return cropped_img
    
        
print(create_game_state(get_image()))

# # convert the image to grayscale format
# img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

# ret, thresh = cv.threshold(img_gray, 150, 255, cv.THRESH_BINARY)
# # visualize the binary image
# cv.imshow('Binary image', thresh)
# #cv.waitKey(0)
# cv.imwrite('image_thres1.jpg', thresh)
# cv.destroyAllWindows()

# # detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
# contours, hierarchy = cv.findContours(image=thresh, mode=cv.RETR_TREE, method=cv.CHAIN_APPROX_NONE)
                                      
# # draw contours on the original image
# img_copy = img.copy()
# cv.drawContours(img_copy, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv.LINE_AA)
                
# # see the results
# cv.imshow('None approximation', img_copy)
# # cv.waitKey(0)
# cv.imwrite('contours_none_image1.jpg', img_copy)
# # cv2.destroyAllWindows()
# #cv.imshow('img', img)
