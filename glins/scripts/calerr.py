import numpy as np
import sys
import os
import rtkcmn
def findref(time,dataref, interval = 0.1):
    n = len(dataref)
    for i in range(n):
        if abs(time - float(dataref[i,1])) < (interval/2):
            return [True,i]
    return [False,0]
kinfile = sys.argv[1]
reffile = sys.argv[2]
errfile = sys.argv[3]
ts = [2158, 95630]
td = [2158, 96079]

datakin_temp = np.loadtxt(kinfile, dtype=str, comments='%')
dataref_temp = np.loadtxt(reffile, dtype=str, comments='%')
datakin = []
dataref = []
for i in range(len(datakin_temp)):
    if float(datakin_temp[i][1]) > (ts[1]) and float(datakin_temp[i][1]) < (td[1]):
        datakin.append(datakin_temp[i][:])

for i in range(len(dataref_temp)):
    if float(dataref_temp[i][1]) > (ts[1]) and float(dataref_temp[i][1]) < (td[1]):
        dataref.append(dataref_temp[i, :])
datakin = np.array(datakin)
dataref = np.array(dataref)

n = len(datakin)
dataerr = np.zeros((n, 5), dtype=object)
for i in range(n):
    index = findref(float(datakin[i, 1]), dataref)
    if index[0] == True:
        dataerr[i, 0] = datakin[i, 0]
        dataerr[i, 1] = dataref[index[1], 1]
        ref_lla = [float(dataref[index[1], 2])+float(dataref[index[1], 3])/60+float(dataref[index[1], 4])/3600,
                   float(dataref[index[1], 5])+float(dataref[index[1], 6])/60+float(dataref[index[1], 7])/3600,
                   float(dataref[index[1],8])]
        # ref_lla = [float(dataref[index[1],2]),
        #            float(dataref[index[1],3]),
        #            float(dataref[index[1],4])]

        ref_ecef = rtkcmn.pos2ecef(ref_lla,True)
        for j in range(3):
            dataerr[i, j+2] = str(float(datakin[i, j+2]) - ref_ecef[j])
    else:
        dataerr[i, :] = np.nan

mask = ~np.isnan(dataerr[:, 0].astype(np.float))
np.savetxt(errfile,dataerr[mask],delimiter=' ',fmt='%s')






