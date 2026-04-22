import math

import numpy as np
import sys
import os

from matplotlib.ticker import PercentFormatter

import rtkcmn
import re
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import scienceplots
def findref(time,dataref, interval = 0.1):
    n = len(dataref)
    for i in range(n):
        if abs(time - float(dataref[i,1])) < (interval/2):
            return [True,i]
    return [False,0]

def line_split(line):
    return re.split(r"[ ]+", line)


def load_pos(file_path,ts,td):
    data_temp = np.genfromtxt(file_path, dtype=float, comments='%',delimiter=None)
    # data_temp = np.loadtxt(file_path, dtype=float, comments='%',delimiter=None,converters={0:line_split})
    data = []
    for i in range(len(data_temp)):
        if float(data_temp[i][1]) > (ts[1]) and float(data_temp[i][1]) < (td[1]):
            data.append(data_temp[i][0:6])
    return np.asarray(data)

def calerr(kinfile, reffile, ts, td):
    data_kin = load_pos(kinfile,ts,td)
    data_ref = load_pos(reffile,ts,td)
    i = 0
    data_err = []
    for data in data_kin:
        while data[1] - data_ref[i][1] > 0:
            i = i + 1
        if data[1] - data_ref[i][1] == 0:
            data_err.append([data[0],data[1],data[2]-data_ref[i][2],data[3]-data_ref[i][3],data[4]-data_ref[i][4],data[5]])

    return np.asarray(data_err)

filepath = "/media/wangchuji/T7/20240129/"
kinfile = [filepath + "glins/glins.pos",
          filepath + "imu/gins.pos",
           filepath + "gnss/rtklib.pos"]
reffile = "/media/wangchuji/T7/20240129/IEproject/20240129actII.pos"
errfile = filepath + "err.pos"
pngflname = filepath + "err.png"
# ts = [2290,551812]
# td = [2290,552300]
ts = [2299,111965]
td = [2299,113000]

dataerr = calerr(kinfile[0], reffile, ts, td)
dataerr1 = calerr(kinfile[1],reffile, ts, td)
dataerr2 = calerr(kinfile[2],reffile, ts, td)
base_pos = [-2267208.9283,5009582.9268,3221142.2773]
base_pos = rtkcmn.ecef2pos(base_pos)

data_kin1 = load_pos(kinfile[0], ts, td)
data_kin2 = load_pos(kinfile[1], ts, td)
data_kin3 = load_pos(kinfile[2], ts, td)
data_kin4 = load_pos(reffile, ts, td)

for i in range(len(data_kin1)):
    data_kin1[i,2:5] = rtkcmn.ecef2enu(base_pos,data_kin1[i,2:5])
for i in range(len(data_kin2)):
    data_kin2[i,2:5] = rtkcmn.ecef2enu(base_pos,data_kin2[i,2:5])
for i in range(len(data_kin3)):
    data_kin3[i,2:5] = rtkcmn.ecef2enu(base_pos,data_kin3[i,2:5])
for i in range(len(data_kin4)):
    data_kin4[i, 2:5] = rtkcmn.ecef2enu(base_pos, data_kin4[i, 2:5])

# with plt.style.context('science'):
#
#     fig = plt.figure(figsize=(8, 6))
#     ax = fig.add_subplot(111, projection='3d')
#
#     # 绘制轨迹
#     ax.plot(data_kin4[:,2],data_kin4[:,3],data_kin4[:,4], label='Groud Truth', color='#ff851b')
#
#     ax.plot(data_kin3[:,2],data_kin3[:,3],data_kin3[:,4], label='RTK', color='b')
#
#     ax.plot(data_kin2[:,2],data_kin2[:,3],data_kin2[:,4], label='GINS', color='g')
#
#     ax.plot(data_kin1[:,2],data_kin1[:,3],data_kin1[:,4], label='GLINS', color='r')
#     # 设置坐标轴标签
#     ax.set_xlabel('E (m)')
#     ax.set_ylabel('N (m)')
#     ax.set_zlabel('U (m)')
#     ax.grid(False)
#
#     # 添加标题
#     ax.set_title('3D Trajectory Plot')
#
#     # 添加图例
#     ax.legend()
#
#     # 显示图形
#     plt.show()
#
#     fig.savefig(pngflname, pad_inches=None, bbox_inches='tight', dpi=300)

totalErr = []
totalErr1 = []
totalErr2 = []

for i in range(len(dataerr)):
    dataerr[i,2:5] = rtkcmn.ecef2enu(base_pos,dataerr[i,2:5])
    totalErr.append(math.sqrt(dataerr[i,2]*dataerr[i,2] + dataerr[i,3]*dataerr[i,3] + dataerr[i,4]*dataerr[i,4]))
for i in range(len(dataerr1)):
    dataerr1[i,2:5] = rtkcmn.ecef2enu(base_pos,dataerr1[i,2:5])
    totalErr1.append(math.sqrt(dataerr1[i,2]*dataerr1[i,2] + dataerr1[i,3]*dataerr1[i,3] + dataerr1[i,4]*dataerr1[i,4]))
for i in range(len(dataerr2)):
    dataerr2[i,2:5] = rtkcmn.ecef2enu(base_pos,dataerr2[i,2:5])
    totalErr2.append(math.sqrt(dataerr2[i,2]*dataerr2[i,2] + dataerr2[i,3]*dataerr2[i,3] + dataerr2[i,4]*dataerr2[i,4]))

totalErr = np.sort(totalErr)
totalErr1 = np.sort(totalErr1)
totalErr2 = np.sort(totalErr2)

# with plt.style.context('science'):
#     fig = plt.figure(figsize=(6, 4))
#     ax = fig.add_subplot(111)
#     cdf = np.arange(1, len(totalErr) + 1) / len(totalErr)
#     ax.plot(totalErr, cdf,label="GLINS")
#     cdf1 = np.arange(1, len(totalErr1) + 1) / len(totalErr1)
#     ax.plot(totalErr1, cdf1,label="GINS")
#     cdf2 = np.arange(1, len(totalErr2) + 1) / len(totalErr2)
#     ax.plot(totalErr2, cdf2,label="RTK")
#     ax.yaxis.set_major_formatter(PercentFormatter(xmax=1, decimals=0))
#     ax.set_xlim(0, 1.5)
#     ax.set_ylim(0, 1)
#     ax.legend()
#     plt.xlabel('Error (m)')
#     plt.ylabel('Cumulative Distribution')
#     plt.show()
#     fig.savefig(pngflname, pad_inches=None, bbox_inches='tight', dpi=300)

with plt.style.context('science'):
    fig, axs = plt.subplots(3, 1, sharex=True)
    fig.subplots_adjust(hspace=0.12)
    colors = ('r', 'b', 'g')
    strs = ('East ', 'North ', 'Up ')
    axs[2].set_xlabel("Time(s)", labelpad=3)
    for i in range(3):
        axs[i].plot(dataerr2[:, 1], dataerr2[:, i + 2], linewidth=1.0, color=colors[0])
        axs[i].plot(dataerr1[:,1],dataerr1[:,i+2], linewidth=1.0, color=colors[1])
        axs[i].plot(dataerr[:,1],dataerr[:,i+2], linewidth=1.0, color=colors[2])
        axs[i].set_ylabel(strs[i] + '(m)', labelpad=3)
        rms = np.sqrt(np.nanmean(np.square(dataerr[:,i+2])))
        rms1 = np.sqrt(np.nanmean(np.square(dataerr1[:,i+2])))
        rms2 = np.sqrt(np.nanmean(np.square(dataerr2[:,i+2])))

        str1 = strs[i] + "%6.2f %6.2f %6.2f" % (rms, rms1, rms2) + 'm'
        print(str1)

    for i in range(2):
        axs[i].set_ylim([-2, 2])
    axs[2].set_ylim([-3, 3])
    axs[0].legend(["RTK", "RTK/INS TC", "RTK/INS/LiDAR TC"], loc='center', bbox_to_anchor=(0.5, 1.2), ncol=6,
                  fontsize=5, edgecolor='k')
    plt.show()
    fig.savefig(pngflname, pad_inches=None, bbox_inches='tight',dpi=300)






