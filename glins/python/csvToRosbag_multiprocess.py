# coding=utf-8
import multiprocessing
import os
import sys
import numpy as np
# import cv2
import rosbag
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import struct
import time
import concurrent.futures
ringMap = np.array([0,8,1,9,2,10,3,11,4,12,5,13,6,14,7,15],dtype="uint16")

def resolveHourAmiguity(timestamp,computer_timestamp):
    half_hour_to_sec = 1800
    tmptimestamp = timestamp
    if(computer_timestamp > timestamp):
        if (computer_timestamp - timestamp) > half_hour_to_sec:
            tmptimestamp = tmptimestamp + 2*half_hour_to_sec
    elif (timestamp - computer_timestamp) > half_hour_to_sec:
        tmptimestamp = tmptimestamp - 2*half_hour_to_sec
    return tmptimestamp


def findFiles(root_dir, filter_type, reverse=False):
    """
    在指定目录查找指定类型文件 -> paths, names, files
    :param root_dir: 查找目录
    :param filter_type: 文件类型
    :param reverse: 是否返回倒序文件列表，默认为False
    :return: 路径、名称、文件全路径
    """

    separator = os.path.sep
    paths = []
    names = []
    files = []
    for parent, dirname, filenames in os.walk(root_dir):
        for filename in filenames:
            if filename.endswith(filter_type):
                paths.append(parent + separator)
                names.append(filename)
    for i in range(paths.__len__()):
        files.append(paths[i] + names[i])
    print(names.__len__().__str__() + " files have been found.")

    paths = np.array(paths)
    names = np.array(names)
    files = np.array(files)

    index = np.argsort(files)

    paths = paths[index]
    names = names[index]
    files = files[index]

    paths = list(paths)
    names = list(names)
    files = list(files)

    if reverse:
        paths.reverse()
        names.reverse()
        files.reverse()
    return paths, names, files

def point_data(timestamp,line):
    parts = line.split(",")
    pos_x = np.float32(parts[1])
    pos_y = np.float32(parts[2])
    pos_z = np.float32(parts[3])
    intensity = np.float32(parts[7])
    ring = ringMap[int(parts[8])]
    time = np.float32((int(parts[9]) - timestamp) / 1000000)
    return [pos_x, pos_y, pos_z, intensity, ring, time]

def readLidarCSV(file_path):
    point_list = []
    fin = open(file_path, 'r')
    fin.readline()
    lines = fin.readlines() #.strip()
    parts = lines[0].split(",")
    # parts = lines[len(lines) - 1].split(",")
    # first_timestamp = int(parts[9])
    first_timestamp = int(parts[6])
    # timestamp = int(float(parts[0]) / 3600) * 3600 + int(parts[9]) / 1000000
    timestamp = int(float(parts[0]) / 3600) * 3600 + int(parts[6]) / 1000000
    timestamp = resolveHourAmiguity(timestamp, float(parts[0]))

    # func_ = partial(point_data,first_timestamp)
    # pool = multiprocessing.Pool(7)
    # point_list = pool.map(func_,lines)
    # pool.close()
    # pool.join()

    for line in lines:
        parts = line.split(",")
        # pos_x = np.float32(parts[1])
        # pos_y = np.float32(parts[2])
        # pos_z = np.float32(parts[3])
        # intensity = np.float32(parts[7])
        # ring = np.uint16(ringMap[int(parts[8])])
        # time = np.float32((int(parts[9]) - first_timestamp)/1000000)
        # azimuth = float(parts[2])
        # distance = float(parts[3])
        # vertical_angle = float(parts[6])

        pos_x = np.float32(parts[8])
        pos_y = np.float32(parts[9])
        pos_z = np.float32(parts[10])
        intensity = np.float32(parts[1])
        ring = np.uint16(ringMap[int(parts[2])])
        time = np.float32((int(parts[6]) - first_timestamp)/1000000)
        point_list.append([pos_x, pos_y, pos_z, intensity, ring, time])

        # point_list.append([pos_x, pos_y, pos_z, intensity, distance, azimuth, vertical_angle])

        # line = fin.readline().strip()

    fin.close()
    return point_list, timestamp

def process_file(tmp_path):
    point_list, ts = readLidarCSV(tmp_path)
    points = np.asarray(point_list, object)

    points_byte = []
    for point in points:
        point_byte = struct.pack('<ffffHf', point[0], point[1], point[2], point[3], point[4], point[5])
        points_byte.append(point_byte)

    lidar_msg = PointCloud2()
    lidar_msg.header.frame_id = "velodyne"
    lidar_ts_ros = rospy.rostime.Time.from_sec(ts)
    lidar_msg.header.stamp = lidar_ts_ros

    if len(points.shape) == 3:
        lidar_msg.height = points.shape[1]
        lidar_msg.width = points.shape[0]
    else:
        lidar_msg.height = 1
        lidar_msg.width = len(points)

    lidar_msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
        PointField('ring', 16, PointField.UINT16, 1),
        PointField('time', 18, PointField.FLOAT32, 1),

        # PointField('distance', 16, PointField.FLOAT32, 1),
        # PointField('azimuth', 20, PointField.FLOAT32, 1),
        # PointField('vertical_angle', 24, PointField.FLOAT32, 1),
    ]
    lidar_msg.is_bigendian = False
    lidar_msg.point_step = 22
    lidar_msg.row_step = lidar_msg.point_step * points.shape[0]
    lidar_msg.is_dense = True
    lidar_msg.data = b''.join(points_byte)
    print('%s %.3lf' % (tmp_path, lidar_ts_ros.to_sec()))
    return lidar_msg, lidar_ts_ros

if __name__ == '__main__':
    # input_dir = "/media/wangchuji/T7/20231202/pointcloud" #sys.argv[1]
    # out_dir = "/media/wangchuji/T7/20231202" #sys.argv[2]

    input_dir = "/media/wangchuji/T7/20240129/lidar/1" #sys.argv[1]
    out_dir = "/media/wangchuji/T7/20240129" #sys.argv[2]


    # lidar参数
    lidar_dir = input_dir
    lidar_type = ".csv"
    lidar_topic_name = "/velodyne_points"

    # for parent, dirname, filenames in os.walk(input_dir):
    #     for filename in filenames:
    #         if filename.endswith(".csv"):
    #             names_ = filename.split(".")[0].split("_")
    #             newfilename = names_[0] + '_' + str('%.5d' % int(names_[1])) + '.csv'
    #             os.rename(parent + os.path.sep + filename,parent + os.path.sep + newfilename)

    # 统计信息
    fout = open(out_dir + "/summary.txt", 'w')
    # Bag输出路径
    bag_path = out_dir + "/combine1.bag"

    # 新建ROS Bag输出
    bag_out = rosbag.Bag(bag_path, 'w')

    # LiDAR数据转换
    # ----------------------------------------------------------
    paths, names, files = findFiles(lidar_dir, lidar_type)

    with concurrent.futures.ProcessPoolExecutor(max_workers=10) as executor:
        future_to_file = {executor.submit(process_file,file): file for file in files}
        for future in concurrent.futures.as_completed(future_to_file):
            file = future_to_file[future]
            try:
                lidar_msg, lidar_ts_ros = future.result()
            except Exception as exc:
                print('%r generated an exception: %s' % (file, exc))
            else:
                bag_out.write(lidar_topic_name, lidar_msg, lidar_ts_ros)

    # for i in range(len(files)):
    #     # ts = float(names[i].split(".")[0]) / 1e9
    #
    #     tmp_path = files[i]
    #
    #     point_list,ts = readLidarCSV(tmp_path)
    #     points = np.asarray(point_list,object)
    #     points_byte = []
    #     for point in points:
    #         point_byte = struct.pack('<ffffHf',point[0],point[1],point[2],point[3],point[4],point[5])
    #         points_byte.append(point_byte)
    #     lidar_msg = PointCloud2()
    #     lidar_msg.header.frame_id = "velodyne"
    #     lidar_ts_ros = rospy.rostime.Time.from_sec(ts)
    #     lidar_msg.header.stamp = lidar_ts_ros
    #
    #     if len(points.shape) == 3:
    #         lidar_msg.height = points.shape[1]
    #         lidar_msg.width = points.shape[0]
    #     else:
    #         lidar_msg.height = 1
    #         lidar_msg.width = len(points)
    #
    #     lidar_msg.fields = [
    #         PointField('x', 0, PointField.FLOAT32, 1),
    #         PointField('y', 4, PointField.FLOAT32, 1),
    #         PointField('z', 8, PointField.FLOAT32, 1),
    #         PointField('intensity', 12, PointField.FLOAT32, 1),
    #         PointField('ring', 16, PointField.UINT16, 1),
    #         PointField('time', 18, PointField.FLOAT32, 1),
    #
    #         # PointField('distance', 16, PointField.FLOAT32, 1),
    #         # PointField('azimuth', 20, PointField.FLOAT32, 1),
    #         # PointField('vertical_angle', 24, PointField.FLOAT32, 1),
    #     ]
    #     lidar_msg.is_bigendian = False
    #     lidar_msg.point_step = 22
    #     lidar_msg.row_step = lidar_msg.point_step * points.shape[0]
    #     lidar_msg.is_dense = True
    #     lidar_msg.data = b''.join(points_byte)
    #
    #     bag_out.write(lidar_topic_name, lidar_msg, lidar_ts_ros)
    #
    #     print("lidar",i + 1, "/", len(files),',',ts)

    # fout.write("LiDAR start timestamp(unit:s)\t"+str(int(names[0].split(".")[0]) / 1e9)+"\n")
    # fout.write("LiDAR end timestamp(unit:s)\t"+str(int(names[-1].split(".")[0]) / 1e9)+"\n")
    # ----------------------------------------------------------

    bag_out.close()
    fout.close()
