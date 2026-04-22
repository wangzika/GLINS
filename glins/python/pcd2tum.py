from pypcd.pypcd import *
import numpy as np
import quaternion
pointcloud = PointCloud.from_path('/home/wangchuji/catkins_lidar/LIO-SAM-master/map/hk/20231202/transformations.pcd')

with open('pcd2tum.txt', 'w') as f:
    for point in pointcloud.pc_data:
        q_point = quaternion.from_euler_angles(point['roll'], point['pitch'], point['yaw'])
        f.write(str(point['time']) + ' ' + str(point['x']) + ' ' + str(point['y']) + ' ' + str(point['z']) + ' ' +
                str(q_point.x) + ' ' + str(q_point.y) + ' ' + str(q_point.z) + ' ' + str(q_point.w) + '\n')