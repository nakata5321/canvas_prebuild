import sys
import numpy as np
import cv2 as cv
#import ros_numpy
import pypcd

pc = pypcd.PointCloud.from_path('1.pcd')
print(pc.pc_data.dtype)
print(pc.fields.dtype)
#print(pc.pc_data['rgb'])
#print(pc.pc_data.shape)
#print(pc.pc_data[0])
#print(pc.pc_data[0][0])
#print(pc.pc_data[0:10])
#print(type(pc.pc_data))

x = pc.pc_data['x']
y = pc.pc_data['y']
z = pc.pc_data['z']
x =np.reshape(x,(47774, 1))
y =np.reshape(y,(47774, 1))
z =np.reshape(z,(47774, 1))
XYZ = np.concatenate((x, y, z), axis = 1)

print(XYZ.shape)
print(XYZ[0:10])
print(XYZ[0])

# ros_numpy try
#file =  open('data.txt')
#data = file.read()
#file.close()
#print(data)
#print(type(data))
#data1 = np.fromstring(data, dtype=np.float32, sep=', ')
#print(type(data1))
#print(data1.dtype)
#pc2 = ros_numpy.point_cloud2.array_to_pointcloud2(data1)
#print(type(pc2))
