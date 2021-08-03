#!/usr/bin/env python

import numpy as np
import cv2 as cv
import math


def find_greater_contour(contours):
    """
    search the greatest contour
    :param contours: array
    :return: list, int
    """
    largest_area = 0
    largest_contour_index = -1
    i = 0
    total_contours = len(contours)
    while (i < total_contours):
        area = cv.contourArea(contours[i])
        if (area > largest_area):
            largest_area = area
            largest_contour_index = i
        i += 1

    return largest_area, largest_contour_index


def find_angle(box):
    """
    calculate angle between OX and longer side
    :param box: array
    :return:int
    """
    edge1 = np.int0((box[1][0] - box[0][0], box[1][1] - box[0][1]))
    edge2 = np.int0((box[2][0] - box[1][0], box[2][1] - box[1][1]))

    used_edge = edge1
    if cv.norm(edge2) > cv.norm(edge1):
        used_edge = edge2
    reference = (1, 0)

    angle = math.acos(
        (reference[0] * used_edge[0] + reference[1] * used_edge[1]) / (cv.norm(reference) * cv.norm(used_edge)))
    return (angle)


def canvas_find(bgr_arr):
    hsv_min = np.array((0, 0, 180), np.uint8)
    hsv_max = np.array((180, 40, 255), np.uint8)

    img = bgr_arr
    cv.imshow('Original image', img)

    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    cv.imshow('hsv image', hsv)

    thresh = cv.inRange(hsv, hsv_min, hsv_max)
    cv.imshow('thresh image', thresh)

    _, contours, hierarchy = cv.findContours(thresh.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    largest_area, largest_contour_index = find_greater_contour(contours)
    lar_con = contours[largest_contour_index]

    epsilon = 0.1 * cv.arcLength(lar_con, True)
    approx = cv.approxPolyDP(lar_con, epsilon, True)
    cv.drawContours(img, contours, largest_contour_index, (255, 255, 0), 2, cv.LINE_AA, hierarchy, 0)
    cv.drawContours(img, approx, -1, (0, 0, 255), 5)
    rect = cv.minAreaRect(lar_con)
    box = cv.boxPoints(rect)
    angle = find_angle(box)
    center = (int(rect[0][0]), int(rect[0][1]))

    cv.circle(img, center, 5, (225, 0, 0), 2)
    cv.putText(img, "%d" % int(angle * 180.0 / math.pi), (center[0] + 20, center[1] - 20),
               cv.FONT_HERSHEY_SIMPLEX, 1, (225, 0, 0), 2)

    cv.imshow('contours', img)
    cv.waitKey(0)
    cv.destroyAllWindows()
    return (rect, angle, approx)
