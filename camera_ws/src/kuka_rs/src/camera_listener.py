#!/usr/bin/env python

import rospy
import numpy as np
import pypcd
from sensor_msgs.msg import PointCloud2
import ros_numpy
from transform  import binary
import cv2 as cv

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

def callback(data):
    global sub
    pc = ros_numpy.numpify(data)
    x = pc[0, 0]
    print(len(x))
    print(x.shape)
    print(x)
    print(x[3])
    print(pc.shape)
    print(pc.shape[0])
    #print(pc[0])
    #print(pc1.dtype_list)
    #pc = pypcd.PointCloud.from_msg(data)
    #print(pc.pc_data['x'][0:10])
    #print(pc.pc_data)
    #print(pc.fields)
    #pc.save('foo.pcd')
    sub.unregister()
    print("done")
    pic = RGB_pic(pc)
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
    cv.imwrite('41.jpg', pic) 
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
    listener()
