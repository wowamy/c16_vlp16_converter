#!/usr/bin/env python
# PointCloud2 color cube
import rospy
import struct
import numpy as np

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


rospy.init_node("create_cloud_xyzrgb")
pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)

points = []
lim = 8
for i in range(lim):
    for j in range(lim):
        for k in range(lim):
            x = float(i) / lim
            y = float(j) / lim
            z = float(k) / lim
            intensity = 1
            ring = 1
            pt = [x, y, z, 0, intensity, ring, 0, 0]
            points.append(pt)

# N = len(points)

fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('', 12, PointField.FLOAT32, 1),
          PointField('intensity', 16, PointField.FLOAT32, 1),
          PointField('ring', 20, PointField.UINT16, 1),
          PointField('', 24, PointField.FLOAT32, 1),
          PointField('', 28, PointField.FLOAT32, 1)
          ]

header = Header()
header.stamp = rospy.Time.now()
header.frame_id = "map"

# msg_pointcloud2 = PointCloud2()
# 
# msg_pointcloud2.header = header
# msg_pointcloud2.fields = fields
# msg_pointcloud2.data = np.asarray(points, np.float32).tostring()
# msg_pointcloud2.width = len(pt)
# msg_pointcloud2.height = 1
# msg_pointcloud2.point_step = 32
# msg_pointcloud2.row_step = msg_pointcloud2.point_step * N
# msg_pointcloud2.is_dense = True
# msg_pointcloud2.is_bigendian = False

pc2 = point_cloud2.create_cloud(header, fields, points)

while not rospy.is_shutdown():
    header.stamp = rospy.Time.now()
    pub.publish(pc2)
    rospy.sleep(1.0)