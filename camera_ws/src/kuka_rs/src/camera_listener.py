#!/usr/bin/env python

import math
import rospy
import numpy as np
#import pypcd
from sensor_msgs.msg import PointCloud2
#import ros_numpy
from transform  import binary
import cv2 as cv
from canvas_saerch import findGreatesContour, canvas_find

import tf
import tf2_ros as tf2
from geometry_msgs.msg import TransformStamped
#from geometry_msgs.msg import Quaternion, Vector3
import rospkg

from math0 import rpy_and_quat_from_xyz

from kuka_rs.msg import Pose
from kuka_rs.srv import RequestCanvas, RequestCanvasResponse

global sub, transform_stamp, res

transform_stamp = TransformStamped()
res = RequestCanvasResponse()

def RGB_pic(point_arr):
    #get rgb image from point cloud
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
    h1 = math.sqrt((xyz_data[1][0] - xyz_data[0][0])**2 + (xyz_data[1][1] - xyz_data[0][1])**2)
    h2 = math.sqrt((xyz_data[2][0] - xyz_data[1][0])**2 + (xyz_data[2][1] - xyz_data[1][1])**2)
    if(h2 > h1):
        height = h2
        width = h1
    else:
        height = h1
        width = h2
    return(width, height)

'''
def convert_frame(xyz):
    new_xyz = np.zeros((4,3))
    new_xyz[:,0] = xyz[:,1]*(-1)
    new_xyz[:,1] = xyz[:,0]*(-1)
    new_xyz[:,2] = xyz[:,2]*(-1)
    return(new_xyz)

def convert_frame(xyz):
    new_xyz = np.array([[0, 0, 0]]).T
    trans = tfBuffer.lookup_transform("base_link", "camera_link", rospy.Time(0), rospy.Duration(3))
    print(trans)
    camera_quat = Quaternion(trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w)
    camera_trans = Vector3(trans.transform.translation.x,
        trans.transform.translation.y,
        trans.transform.translation.z)

    camera_rot = np.asarray(tf.transformations.quaternion_matrix(camera_quat))

    new_xyz = np.dot(camera_rot, xyz) + camera_trans
    return(new_xyz)
'''
def convert_frame(xyz):
    new_xyz = np.array([[0, 0, 0]]).T
    trans = tfBuffer.lookup_transform("base_link", "camera_link", rospy.Time(0), rospy.Duration(3))
    print(trans)
    camera_quat = np.asarray([trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w])
    print(camera_quat)
    print(type(camera_quat))
    camera_trans = np.array([[trans.transform.translation.x,
        trans.transform.translation.y,
        trans.transform.translation.z]]).T
    print(camera_trans)
    print(camera_trans.shape)
    #quat = np.array([camera_quat.x, camera_quat.y, camera_quat.z, camera_quat.w])
    #print(quat)
    camera_rot = np.asarray(tf.transformations.quaternion_matrix(camera_quat))
    print(camera_rot)
    camera_rot = camera_rot[0:3, 0:3]
    print(camera_rot)
    print(camera_rot.shape)
    print(xyz.shape)
    new_xyz = np.dot(camera_rot, xyz) + camera_trans
    new_xyz[2, 0] = new_xyz[2, 0] + 0.22
    print("new_xyz")
    print(new_xyz)

    return(new_xyz)

#TODO kD tree for search all points in rectangle
def callback(data):
    global transform_stamp

    global sub
    #pc = ros_numpy.numpify(data)
    #np.save("np_arr", pc)



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

    #sub.unregister()
    #print("done")
    #pic = RGB_pic(pc)
    #print(data[0][0])
    pic = RGB_pic(data)

    print("get RGB")
    print(pic.shape)

    pic = pic.astype(np.uint8)
    #Separated the channels in my new image
    #new_image_red, new_image_green, new_image_blue = pic
    #Stacked the channels
    #new_rgb = np.dstack([new_image_red, new_image_green, new_image_blue])
    #print(new_rgb.shape)

    #Displayed the image
    #cv.imwrite('42.jpg', pic)
    #hsv = cv.cvtColor( pic, cv.COLOR_BGR2HSV )
    #cv.imshow('hsv image', hsv)
    rect, box, angle = canvas_find(pic)
    print("rect info")
    print(rect[0])
    print(rect[1])
    print(rect[2])
    #print(rect[3])


    center = np.int0(rect[0])
    print(center)
    print(center[1])
    xyzr_center = data[int(center[1]), int(center[0])]
    xyz_center = np.array([[xyzr_center[0], xyzr_center[1], xyzr_center[2]]]).T
    print(xyz_center)
    print(rect)
    #print(box)
    #print(box[0][0])

    points = sort_points(box)
    print(points)
    xyz = XYZ_points(points)
    print(xyz)

    new_xyz_center = convert_frame(xyz_center)

    #rpy, q = rpy_and_quat_from_xyz(new_xyz)
    #print(q)
    w, h = width_height(xyz)
    print(w, h)

    cv.imshow("WindowNameHere", pic)
    cv.waitKey(0)


    ####     tf.transformations.euler_from_quaternion(quaternion, axes='sxyz')


    #holder_quat = Quaternion(trans.transform.rotation)
    #holder_trans = Vector3(trans.transform.translation)
    #holder_rot = tf.transformations.euler_matrix(rpy)
    #cam_trans = Vector3(-xyz_center[1], -xyz_center[0], -xyz_center[2])
    #new_tran = holder_trans + holder_rot*cam_trans

    #TODO tramsform


    transform_stamp.header.frame_id = "base_link"
    transform_stamp.child_frame_id = "canvas_link"
    transform_stamp.transform.translation.x = new_xyz_center[0]
    transform_stamp.transform.translation.y = new_xyz_center[1]
    transform_stamp.transform.translation.z = new_xyz_center[2]
    transform_stamp.transform.rotation.x = 0  # q[0]
    transform_stamp.transform.rotation.y = 0  #q[1]
    transform_stamp.transform.rotation.z = 0  #q[2]
    transform_stamp.transform.rotation.w = 1 #q[3]

    res.p.x = new_xyz_center[0]
    res.p.y = new_xyz_center[1]
    res.p.z = new_xyz_center[2]
    res.p.phi = 0 #rpy[0]
    res.p.theta = 0 #rpy[0]
    res.p.psi = angle #rpy[0]
    res.width = w
    res.height = h
#    h,w = pic.shape[0], pic.shape[1]
#    vis2 = cv.cvtColor(pic)
#    cv.imshow('rgb image', vis2)


'''
def listener():
    global sub
    rospy.init_node('rs_camera', anonymous=True)
    sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback)
    rospy.spin()
'''

def canvasCallback(req):
    global res
    return(res)

if __name__ == '__main__':
    '''
    listener()
    '''
    rospy.init_node('rs_camera', anonymous=True)
    tfBuffer = tf2.Buffer()
    listener = tf2.TransformListener(tfBuffer)
    rospack = rospkg.RosPack()
    dirname = rospack.get_path("kuka_rs")
    data = np.load(dirname + "/src/np_arr.npy")
    print(data.shape)
    callback(data)

    global sub

    broadcaster = tf2.StaticTransformBroadcaster()
    rate = rospy.Rate(10)
    canvas_server = rospy.Service('request_canvas', RequestCanvas, canvasCallback)

    #sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback)
    while not rospy.is_shutdown():
        transform_stamp.header.stamp = rospy.get_rostime()
        broadcaster.sendTransform(transform_stamp)
        rate.sleep()
