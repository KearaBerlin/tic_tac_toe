# -*- coding: utf-8 -*-
"""
Created on Sun Oct 23 15:42:50 2022

@author: amara
"""

import cv2 as cv


def create_game_state():
    return []
    
def detect(img):
    #detect X, O or empty
    height, width, c = img.shape
    b_count, r_count = 0, 0
    for r in range(height):
        for c in range(width):
            blue = int(img[r, c, 0])
            red = int(img[r, c, 2])
            if blue - red > 50:
                b_count += 1
            elif red - blue > 50:
                r_count += 1
                
    if r_count > b_count:
        return -1
    elif b_count > r_count:
        return 1
    else:
        return 0
    
                
    return []    
img = cv.imread("Board space2.png", 1)
#cv.imshow('img', img)

height, width, c = img.shape
h = height // 3
w = width // 3

for i in range(3):
    for j in range(3):
        filename = str(i*3 + j) + ".jpg"
        #cv.imwrite(filename, img[i*h:(i+1)*h,j*h:(j+1)*h,:])
        if detect(img[i*h:(i+1)*h,j*h:(j+1)*h,:]) == -1:
            letter = 'O'
        elif detect(img[i*h:(i+1)*h,j*h:(j+1)*h,:]) == 1:
            letter = 'X'
        else:
            letter = " "
        print(i*3 + j, ': ', letter)

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