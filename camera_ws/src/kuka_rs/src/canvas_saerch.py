#!/usr/bin/env python

import math
import sys
import numpy as np
import cv2 as cv

def findGreatesContour(contours):
    largest_area = 0
    largest_contour_index = -1
    i = 0
    total_contours = len(contours)
    while (i < total_contours ):
        area = cv.contourArea(contours[i])
        if(area > largest_area):
            largest_area = area
            largest_contour_index = i
        i+=1

    return largest_area, largest_contour_index

def find_angle(box):
    edge1 = np.int0((box[1][0] - box[0][0],box[1][1] - box[0][1]))
    edge2 = np.int0((box[2][0] - box[1][0], box[2][1] - box[1][1]))

    usedEdge = edge1
    if cv.norm(edge2) > cv.norm(edge1):
        usedEdge = edge2
    reference = (1,0)

    angle = math.acos((reference[0]*usedEdge[0] + reference[1]*usedEdge[1]) / (cv.norm(reference) *cv.norm(usedEdge)))
    return(angle)


#if __name__ == '__main__':
def canvas_find(rgb_arr):
    hsv_min = np.array((7, 0, 183), np.uint8)
    hsv_max = np.array((85, 38, 212), np.uint8)
    #fn = '42.jpg' #
    #img = cv.imread(fn)
    img = rgb_arr
    cv.imshow('Original image',img)
    print(img.dtype)
    print(img.shape)

    hsv = cv.cvtColor( img, cv.COLOR_BGR2HSV )
    cv.imshow('hsv image', hsv)

    thresh = cv.inRange( hsv, hsv_min, hsv_max ) #
    #hsv[thresh > 0] = (39, 26, 200)
    cv.imshow('thresh image',thresh)

    #gb = cv.cvtColor(hsv, cv.COLOR_HSV2BGR)
    ##gray = cv.cvtColor(rgb, cv.COLOR_BGR2GRAY) #
    #ret, threshold_image = cv.threshold(gray, 145, 255, 0)

    #cv.imshow('Gray image', gray)

    #cv.imshow('bin image', threshold_image)
    #thresh = cv.inRange( hsv, hsv_min, hsv_max ) #


    _, contours, hierarchy = cv.findContours( thresh.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    largest_area, largest_contour_index = findGreatesContour(contours)
    lar_con = contours[largest_contour_index]
    #print(lar_con)

    #cv.drawContours( img, contours, largest_contour_index, (255,0,0), 3, cv.LINE_AA, hierarchy, 1 )
    #cv.imshow('contours0', img) #

    #print(largest_area)
    #print(largest_contour_index)
    #print(len(contours))
    rect = cv.minAreaRect(lar_con) #
    box = cv.boxPoints(rect) #
    angle = find_angle(box)
    box = np.int0(box)
    cv.drawContours(img,[box],0,(255,0,0),2) #
    center = (int(rect[0][0]), int(rect[0][1]))

    cv.circle(img, center, 5, (0,255,255), 2) #
    cv.putText(img, "%d" % int(angle * 180.0/math.pi), (center[0]+20, center[1]-20),
        cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)

    cv.imshow('contours', img) #
    cv.waitKey(0)
    cv.destroyAllWindows()
    return(rect, box, angle)
    '''
if __name__ == '__main__':
    fn = '14.jpg' #
    img = cv.imread(fn)

    rect, box = canvas_find(img)
    print(fn)
    print(rect[2])
    '''
