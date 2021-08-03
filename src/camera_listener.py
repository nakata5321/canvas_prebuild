#!/usr/bin/env python

import math
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import ros_numpy
from transform import binary
import cv2 as cv
from canvas_saerch import canvas_find
import tf
import tf2_ros as tf2
from geometry_msgs.msg import TransformStamped
from kuka_rs.srv import RequestCanvas, RequestCanvasResponse

global sub, transform_stamp, res

transform_stamp = TransformStamped()
res = RequestCanvasResponse()


def rgb_pic(point_arr):
    """
    Get rgb image from point cloud
    :param point_arr: np.array
    :return: np.array
    """
    rgb_arr = np.zeros((480, 848, 3), np.int32)
    for i in range(point_arr.shape[0]):
        for j in range(point_arr.shape[1]):
            integer = binary(point_arr[i, j][3])
            rgb_arr[i][j][0] = integer[1]
            rgb_arr[i][j][1] = integer[2]
            rgb_arr[i][j][2] = integer[3]
    return (rgb_arr)


def sort_points_z(arr):  # TODO check parallel
    """
    Sort 4 edge points of canvas
    points start from left top and go clockwise
    :param arr: np.array
    :return: sorted np.array
    """
    points = np.zeros((4, 2))
    min_x = 10000
    min_y = 10000
    max_x = 0
    max_y = 0
    for i in range(arr.shape[0]):
        if arr[i][0][0] > max_x:
            max_x = arr[i][0][0]
        if arr[i][0][0] < min_x:
            min_x = arr[i][0][0]
        if arr[i][0][1] > max_y:
            max_y = arr[i][0][1]
        if arr[i][0][1] < min_y:
            min_y = arr[i][0][1]

    for i in range(arr.shape[0]):
        if arr[i][0][0] == max_x:
            points[2] = arr[i][0]
        if arr[i][0][0] == min_x:
            points[0] = arr[i][0]
        if arr[i][0][1] == max_y:
            points[3] = arr[i][0]
        if arr[i][0][1] == min_y:
            points[1] = arr[i][0]
    return (points)


# TODO solve problem with [0, 0, 0, 0] point
def xyz_points(pc, points):
    """
    find XYZ coordinates from pixels coordinates edges of canvas
    :param pc: np.array
    :param points: np.array
    :return: np.array
    """
    xyz = np.zeros((4, 3))
    for i in range(len(points)):
        xyz_data = pc[int(points[i][1]), int(points[i][0])]
        xyz[i][0] = xyz_data[0]
        xyz[i][1] = xyz_data[1]
        xyz[i][2] = xyz_data[2]
    return (xyz)


def width_height(xyz_data):
    """
    calculate width and height of canvas
    :param xyz_data: np.array
    :return: float, float
    """
    h1 = math.sqrt((xyz_data[1][0] - xyz_data[0][0]) ** 2 +
                   (xyz_data[1][1] - xyz_data[0][1]) ** 2 + (xyz_data[1][2] - xyz_data[0][2]) ** 2)
    h2 = math.sqrt((xyz_data[2][0] - xyz_data[1][0]) ** 2 +
                   (xyz_data[2][1] - xyz_data[1][1]) ** 2 + (xyz_data[2][2] - xyz_data[1][2]) ** 2)
    if (h2 > h1):
        height = h2
        width = h1
    else:
        height = h1
        width = h2
    return (width, height)


def convert_frame(xyz):
    """
    convert data from camera frame to base frame
    :param xyz: np.array
    :return: np.array
    """
    trans = tfBuffer.lookup_transform("base_link", "camera_link", rospy.Time(0), rospy.Duration(3))
    camera_quat = np.asarray([trans.transform.rotation.x,
                              trans.transform.rotation.y,
                              trans.transform.rotation.z,
                              trans.transform.rotation.w])
    camera_trans = np.array([[trans.transform.translation.x,
                              trans.transform.translation.y,
                              trans.transform.translation.z]]).T
    camera_rot = np.asarray(tf.transformations.quaternion_matrix(camera_quat))
    camera_rot = camera_rot[0:3, 0:3]
    new_xyz = np.array([[0, 0, 0]]).T
    new_xyz = np.dot(camera_rot, xyz) + camera_trans
    new_xyz[0, 0] = new_xyz[0, 0]
    new_xyz[1, 0] = new_xyz[1, 0]
    new_xyz[2, 0] = new_xyz[2, 0] + 0.24  # add brush length
    return (new_xyz)


def xyz_center_avg(xyz):
    """
    calculate central point of canvas
    :param xyz: np.array
    :return: np.array
    """
    xyz_center = np.array([[0., 0., 0.]]).T
    xyz_center[0, 0] = (xyz[0, 0] + xyz[2, 0]) / 2.0
    xyz_center[1, 0] = (xyz[0, 1] + xyz[2, 1]) / 2.0
    xyz_center[2, 0] = (xyz[0, 2] + xyz[2, 2]) / 2.0
    return (xyz_center)


# TODO kD tree for search all points in rectangle
def callback(data):
    """
    main part
    :param data: PointCloud2
    """
    global transform_stamp
    global sub
    # convert np.array PointCloud2 to numpy array
    pc = ros_numpy.numpify(data)
    pic = rgb_pic(pc)
    pic = pic.astype(np.uint8)
    rect, angle, approx = canvas_find(pic)
    points = sort_points_z(approx)

    xyz = xyz_points(pc, points)
    xyz_center = xyz_center_avg(xyz)
    new_xyz_center = convert_frame(xyz_center)

    w, h = width_height(xyz)
    cv.imshow("WindowNameHere", pic)
    cv.waitKey(0)

    transform_stamp.header.frame_id = "base_link"
    transform_stamp.child_frame_id = "canvas_link"
    transform_stamp.transform.translation.x = new_xyz_center[0]
    transform_stamp.transform.translation.y = new_xyz_center[1]
    transform_stamp.transform.translation.z = new_xyz_center[2]
    transform_stamp.transform.rotation.x = 0
    transform_stamp.transform.rotation.y = 0
    transform_stamp.transform.rotation.z = 0
    transform_stamp.transform.rotation.w = 1

    res.p.x = new_xyz_center[0]
    res.p.y = new_xyz_center[1]
    res.p.z = new_xyz_center[2]
    res.p.phi = 0
    res.p.theta = 0
    res.p.psi = angle + 1.57
    res.width = round(w, 2)
    res.height = round(h, 2)
    sub.unregister()


def canvas_callback(req):
    global res
    return (res)


if __name__ == '__main__':
    global sub
    rospy.init_node('rs_camera', anonymous=True)
    rospy.loginfo("node is up")
    tfBuffer = tf2.Buffer()
    listener = tf2.TransformListener(tfBuffer)
    sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback)

    broadcaster = tf2.StaticTransformBroadcaster()
    rate = rospy.Rate(10)
    canvas_server = rospy.Service('request_canvas', RequestCanvas, canvas_callback)
    rospy.loginfo("publish")
    while not rospy.is_shutdown():
        transform_stamp.header.stamp = rospy.get_rostime()
        broadcaster.sendTransform(transform_stamp)
        rate.sleep()
