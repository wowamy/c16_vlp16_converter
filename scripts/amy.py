#!/usr/bin/env python
#coding=utf-8

import tf
import rospy
import time
import math as m
import numpy as np
import struct
import sensor_msgs.msg as smsgs
import std_msgs.msg as stdmsgs
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2

rospy.init_node('amy')
pub = rospy.Publisher('velodyne_points', PointCloud2, queue_size=10)

points = []
lim = 8
for i in range(lim):
    for j in range(lim):
        for k in range(lim):   
                for l in range(lim):
                        x = float(i) / lim
                        y = float(j) / lim
                        z = float(k) / lim
                        intensity = 20
                        ring = float(l)

                        pt = [x, y, z, intensity, ring]
        
        # print pt
fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('intensity', 16, PointField.FLOAT32, 1),
          PointField('ring', 20, PointField.UINT16, 1)
          ]

# raw_unit8_data = np.array([240, 1, 7, 6, 45, 127, 0, 0], dtype='uint8')
# print np.fromstring(raw_unit8_data.tostring(), dtype='float32')

N = len(points)

# print points

msg_header = stdmsgs.Header()
msg_header.stamp = rospy.Time.now()
msg_header.frame_id = "/map"


msg_pointcloud2 = PointCloud2()

msg_pointcloud2.header = msg_header
msg_pointcloud2.fields = fields
msg_pointcloud2.data = np.asarray(points, np.float32).tostring()
msg_pointcloud2.width = len(pt)
msg_pointcloud2.height = 1
msg_pointcloud2.point_step = 32
msg_pointcloud2.row_step = msg_pointcloud2.point_step * N
msg_pointcloud2.is_dense = True
msg_pointcloud2.is_bigendian = False

# pc2 = point_cloud2.create_cloud(msg_header, fields, points)

while not rospy.is_shutdown():
    msg_header.stamp = rospy.Time.now()
    # pub.publish(pc2)
    # rospy.sleep(1.0)
    pub.publish(msg_pointcloud2)
    rospy.sleep(1.0)