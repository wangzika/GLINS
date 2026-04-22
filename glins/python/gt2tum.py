import numpy as np
import quaternion
import rtkcmn

gt_data = np.genfromtxt('/media/wangchuji/T7/20231202/truth_no_attenna.pos', delimiter=None,dtype=np.float32,skip_header=2)

origin_point = rtkcmn.ecef2pos(gt_data[0][2:5])

with open('gt.txt', 'w') as f:
    for data in gt_data:
        time = rtkcmn.gpst2time(data[0],data[1])
        timestamp = time.time + time.sec - 18
        enu = rtkcmn.ecef2enu(origin_point,data[2:5]-gt_data[0][2:5])
        quat = quaternion.from_euler_angles(np.deg2rad(data[8]),np.deg2rad(data[7]),np.deg2rad(-data[9]))
        f.write(str(timestamp)+' '+str(enu[0])+' '+str(enu[1])+' '+str(enu[2])+' '+str(quat.x)+' '+str(quat.y)+' '+str(quat.z)+' '+str(quat.w)+'\n')

# gt_data = np.genfromtxt('/media/wangchuji/T7/20231202/fgo.pos', delimiter=None,dtype=np.float32,skip_header=1)
#
# origin_point = rtkcmn.ecef2pos(gt_data[0][2:5])
#
# with open('glio.txt', 'w') as f:
#     for data in gt_data:
#         time = rtkcmn.gpst2time(data[0],data[1])
#         timestamp = time.time + time.sec - 18
#         enu = rtkcmn.ecef2enu(origin_point,data[2:5]-gt_data[0][2:5])
#         quat = quaternion.from_euler_angles(np.deg2rad(data[7]),np.deg2rad(data[6]),np.deg2rad(-data[8]))
#         f.write(str(timestamp)+' '+str(enu[0])+' '+str(enu[1])+' '+str(enu[2])+' '+str(quat.x)+' '+str(quat.y)+' '+str(quat.z)+' '+str(quat.w)+'\n')

