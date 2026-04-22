import rosbag
import rospy
from sensor_msgs.msg import PointCloud2

lidarbag = rosbag.Bag('/media/wangchuji/T7/20231202/20231202.bag', 'r')

print(int(1701507511 /3600)*3600+3510480884/1000000)
# for topic,msg,t in lidarbag.read_messages(topics='/velodyne_points'):
#     header = msg.header
