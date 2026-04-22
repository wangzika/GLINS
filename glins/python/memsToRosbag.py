import math

import numpy as np
import rosbag
import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import Imu

gpst0 = [1980, 1, 6, 0, 0, 0]

class gtime_t():
    """ class to define the time """

    def __init__(self, time=0, sec=0.0):
        self.time = time
        self.sec = sec

def epoch2time(ep):
    """
    计算从纪元开始的时间

    参数：
    ep (list[str]): 由字符串组成的列表，包含年、月、日、时、分、秒的字符串表示

    返回值：
    time (gtime_t): 包含时间信息的结构体

    """
    doy = [1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335]
    time = gtime_t()
    year = int(ep[0])
    mon = int(ep[1])
    day = int(ep[2])

    if year < 1970 or year > 2099 or mon < 1 or mon > 12:
        return time
    days = (year-1970)*365+(year-1969)//4+doy[mon-1]+day-2
    if year % 4 == 0 and mon >= 3:
        days += 1
    sec = int(ep[5])
    time.time = days*86400+int(ep[3])*3600+int(ep[4])*60+sec
    time.sec = ep[5]-sec
    return time


def gpst2time(week, tow):
    """ convert to time from gps-time """
    t = epoch2time(gpst0)
    if tow < -1e9 or tow > 1e9:
        tow = 0.0
    t.time += 86400*7*week+int(tow)
    t.sec = tow-int(tow)
    return t
# imu_path = '/media/wangchuji/T7/20231202/mems_.txt'
imu_path = '/media/wangchuji/T7/20240129/imu/mems.txt'
# imu_path = '/media/wangchuji/T7/20221029/imu_adis16465.txt' #"/media/wangchuji/T7/20231202/mems_.txt" #'/home/wangchuji/catkins_gnss/rtklib_GSDC_6th/data/GNSS_INS_Data/GNSS_INS_DATA/urban/imu_adis16465.txt'
bag_path = imu_path.split('.')[0]+'.bag'
bag = rosbag.Bag(bag_path,'w')
imu_topic = "/imu/data"
print("Converting IMU to Rosbag ......")
imu_datas = np.loadtxt(imu_path,comments=None,dtype=str,delimiter=',')
# for imu_data in imu_datas:
#     gpst = gpst2time(int(imu_data[1]), float(imu_data[2]))
#     imu = Imu()
#     imu.header.frame_id = 'imu_link'
#     imu.header.stamp = rospy.Time.from_sec(float(gpst.time+gpst.sec-18))
#     imu.linear_acceleration.z = float(imu_data[8]) * 100
#     imu.linear_acceleration.y = -float(imu_data[6]) * 100
#     imu.linear_acceleration.x = float(imu_data[7]) * 100
#     imu.angular_velocity.z = float(imu_data[5]) * 100
#     imu.angular_velocity.y = -float(imu_data[3]) * 100
#     imu.angular_velocity.x = float(imu_data[4]) * 100
#     bag.write(imu_topic,imu,t=imu.header.stamp)

last_gpsWeekSec = 0.0
for imu_data in imu_datas:
    gpst = gpst2time(int(imu_data[1]), float(imu_data[2]))
    if round(math.fabs(float(imu_data[2]) - last_gpsWeekSec),2) != 0.01 and last_gpsWeekSec != 0.0:
        if float(imu_data[2]) - last_gpsWeekSec < 0.0:
            print("error: timestamp %lf behind last gps weeksec" % float(imu_data[2]))
        if float(imu_data[2]) - last_gpsWeekSec > 0.0:
            print("error: timestamp %lf ahead last gps weeksec" % float(imu_data[2]))
        if float(imu_data[2]) - last_gpsWeekSec == 0.0:
            print("error: timestamp %lf equal last gps weeksec" % float(imu_data[2]))
            continue
    imu = Imu()
    imu.header.frame_id = 'imu_link'
    imu.header.stamp = rospy.Time.from_sec(float(gpst.time+gpst.sec-18))
    imu.linear_acceleration.z = float(imu_data[4]) * 100 * 100/(np.power(2,31))
    imu.linear_acceleration.y = -float(imu_data[6]) * 100 * 100/(np.power(2,31))
    imu.linear_acceleration.x = float(imu_data[5]) * 100 * 100/(np.power(2,31))
    imu.angular_velocity.z = math.radians(float(imu_data[7]) * 100 * 360/(np.power(2,31)))
    imu.angular_velocity.y = -math.radians(float(imu_data[9]) * 100 * 360/(np.power(2,31)))
    imu.angular_velocity.x = math.radians(float(imu_data[8]) * 100 * 360/(np.power(2,31)))
    last_gpsWeekSec = float(imu_data[2])
    bag.write(imu_topic,imu,t=imu.header.stamp)

# with open('adis16465.txt','r') as f:
#     for imu_data in imu_datas:
#         str.format("%d %.3lf %")

# for imu_data in imu_datas:
#     gpst = gpst2time(int(imu_data[1]), float(imu_data[2]))
#     imu = Imu()
#     imu.header.frame_id = 'imu_link'
#     imu.header.stamp = rospy.Time.from_sec(float(gpst.time+gpst.sec-18))
#     imu.linear_acceleration.z = float(imu_data[8]) * 100
#     imu.linear_acceleration.y = float(imu_data[7]) * 100
#     imu.linear_acceleration.x = float(imu_data[6]) * 100
#     imu.angular_velocity.z = float(imu_data[5]) * 100
#     imu.angular_velocity.y = float(imu_data[4]) * 100
#     imu.angular_velocity.x = float(imu_data[3]) * 100
#     bag.write(imu_topic,imu,t=imu.header.stamp)
bag.close()
print("Convert Done!")