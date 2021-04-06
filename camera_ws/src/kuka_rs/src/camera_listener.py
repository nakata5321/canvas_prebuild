#!/usr/bin/env python

from math import sqrt
import rospy
import numpy as np
import pypcd
from sensor_msgs.msg import PointCloud2
import ros_numpy
from transform  import binary
import cv2 as cv
from canvas_saerch import findGreatesContour, canvas_find

global sub

def RGB_pic(point_arr):
    rgb_arr = np.zeros((480,848,3), np.int32)
    for i in range(point_arr.shape[0]):
        for j in range(point_arr.shape[1]):
            int = binary(point_arr[i, j][3])
            rgb_arr[i][j][0] = int[1]
            rgb_arr[i][j][1] = int[2]
            rgb_arr[i][j][2] = int[3]
    return(rgb_arr)

# points start from left top and go clockwise
def sort_points(arr):    #TODO check parallel
    points = np.zeros((4,2))
    min_x = 10000
    min_y = 10000
    max_x = 0
    max_y = 0
    for i in range(arr.shape[0]):
        if arr[i][0] > max_x:
            max_x = arr[i][0]
            points[2] = arr[i]
        if arr[i][0] < min_x:
            min_x = arr[i][0]
            points[0] = arr[i]
        if arr[i][1] > max_y:
            max_y = arr[i][1]
            points[3] = arr[i]
        if arr[i][1] < min_y:
            min_y = arr[i][1]
            points[1] = arr[i]
    return(points)

#TODO solve problem with [0, 0, 0, 0] point
def XYZ_points(points):
    xyz = np.zeros((4,3))
    for i in range(len(points)):
        xyz_data = data[int(points[i][1]), int(points[i][0])]
        if(xyz_data[2] == 0):
            xyz_data = data[int(points[i][1]), int(points[i][0] - 5)]
        print(int(points[i][1]))
        xyz[i][0] = xyz_data[0]
        xyz[i][1] = xyz_data[1]
        xyz[i][2] = xyz_data[2]
        #print(data[int(n[1]), int(n[0])])
    return(xyz)

def width_height(xyz_data):
    h1 = sqrt((xyz_data[1][0] - xyz_data[0][0])**2 + (xyz_data[1][1] - xyz_data[0][1])**2)
    h2 = sqrt((xyz_data[2][0] - xyz_data[1][0])**2 + (xyz_data[2][1] - xyz_data[1][1])**2)
    if(h2 > h1):
        height = h2
        width = h1
    else:
        height = h1
        width = h2
    return(width, height)


#TODO kD tree for search all points in rectangle
def callback(data):

    '''
    global sub
    pc = ros_numpy.numpify(data)
    np.save("np_arr", pc)
    '''


    #x = pc[0, 0]
    #print(len(x))
    #print(x.shape)
    #print(x)
    #print(x[3])
    #print(pc.shape)
    #print(pc.shape[0])
    #print(pc[0])
    #print(pc1.dtype_list)
    #pc = pypcd.PointCloud.from_msg(data)
    #print(pc.pc_data['x'][0:10])
    #print(pc.pc_data)
    #print(pc.fields)
    #pc.save('foo.pcd')

    '''
    sub.unregister()
    print("done")
    pic = RGB_pic(pc)
    '''

    pic = RGB_pic(data)

    print("get RGB")
    print(pic.shape)

    pic = pic.astype(np.uint8)
    print(pic.shape)
    #Separated the channels in my new image
    #new_image_red, new_image_green, new_image_blue = pic
    #Stacked the channels
    #new_rgb = np.dstack([new_image_red, new_image_green, new_image_blue])
    #print(new_rgb.shape)

    #Displayed the image
    #cv.imwrite('42.jpg', pic)
    #hsv = cv.cvtColor( pic, cv.COLOR_BGR2HSV )
    #cv.imshow('hsv image', hsv)
    rect, box = canvas_find(pic)

    center = np.int0(rect[0])
    print(center
    )    
    print(rect)
    #print(box)
    #print(box[0][0])

    points = sort_points(box)
    print(points)
    xyz = XYZ_points(points)
    print(xyz)
    w, h = width_height(xyz)
    print(w, h)

    cv.imshow("WindowNameHere", pic)
    cv.waitKey(0)

#    h,w = pic.shape[0], pic.shape[1]
#    vis2 = cv.cvtColor(pic)
#    cv.imshow('rgb image', vis2)



def listener():
    global sub
    rospy.init_node('rs_camera', anonymous=True)
    sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    '''
    listener()
    '''
    data = np.load("np_arr.npy")
    print(data.shape)
    callback(data)
