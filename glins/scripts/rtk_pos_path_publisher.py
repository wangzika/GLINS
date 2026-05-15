#!/usr/bin/env python3
import math
import os

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def ecef_to_llh(x, y, z):
    a = 6378137.0
    e2 = 6.69437999014e-3
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - e2))
    for _ in range(8):
        sin_lat = math.sin(lat)
        n = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
        lat = math.atan2(z + e2 * n * sin_lat, p)
    sin_lat = math.sin(lat)
    n = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    h = p / math.cos(lat) - n
    return lat, lon, h


def ecef_to_enu(origin_llh, origin_ecef, ecef):
    lat, lon, _ = origin_llh
    dx = ecef[0] - origin_ecef[0]
    dy = ecef[1] - origin_ecef[1]
    dz = ecef[2] - origin_ecef[2]
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


def load_path(pos_file, frame_id, allowed_q, min_sat, max_std, max_speed):
    points = []
    with open(pos_file, "r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("%"):
                continue
            cols = line.split()
            if len(cols) < 5:
                continue
            try:
                week = int(float(cols[0]))
                sec = float(cols[1])
                x = float(cols[2])
                y = float(cols[3])
                z = float(cols[4])
                q = int(float(cols[5]))
                ns = int(float(cols[6]))
                std = max(abs(float(cols[7])), abs(float(cols[8])), abs(float(cols[9])))
            except ValueError:
                continue
            if allowed_q and q not in allowed_q:
                continue
            if ns < min_sat:
                continue
            if std > max_std:
                continue
            points.append((week, sec, x, y, z, q, ns, std))

    path = Path()
    path.header.frame_id = frame_id
    if not points:
        return path

    origin = (points[0][2], points[0][3], points[0][4])
    origin_llh = ecef_to_llh(*origin)
    last = None
    for week, sec, x, y, z, _q, _ns, _std in points:
        if last is not None:
            dt = sec - last[0]
            jump = math.dist((x, y, z), last[1])
            if dt > 0.0 and jump / dt > max_speed:
                continue
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.from_sec(sec)
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = ecef_to_enu(
            origin_llh, origin, (x, y, z)
        )
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
        last = (sec, (x, y, z))

    path.header.stamp = rospy.Time.now()
    return path


def main():
    rospy.init_node("rtk_pos_path_publisher")
    pos_file = rospy.get_param("~pos_file")
    frame_id = rospy.get_param("~frame_id", "map")
    publish_rate = rospy.get_param("~publish_rate", 1.0)
    allowed_q = set(rospy.get_param("~allowed_quality", [1, 2]))
    min_sat = rospy.get_param("~min_sat", 6)
    max_std = rospy.get_param("~max_std", 50.0)
    max_speed = rospy.get_param("~max_speed", 30.0)
    pub = rospy.Publisher("/rtk_path", Path, queue_size=1, latch=True)

    rate = rospy.Rate(publish_rate)
    last_mtime = None
    path = Path()
    path.header.frame_id = frame_id
    while not rospy.is_shutdown():
        if os.path.exists(pos_file):
            mtime = os.path.getmtime(pos_file)
            if mtime != last_mtime:
                path = load_path(pos_file, frame_id, allowed_q, min_sat, max_std, max_speed)
                last_mtime = mtime
                rospy.loginfo("Loaded %d RTK poses from %s", len(path.poses), pos_file)
        else:
            rospy.logwarn_throttle(5.0, "Waiting for RTK pos file: %s", pos_file)

        path.header.stamp = rospy.Time.now()
        pub.publish(path)
        rate.sleep()


if __name__ == "__main__":
    main()
