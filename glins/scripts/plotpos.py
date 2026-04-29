import sys
import os
import math
import warnings
import datetime
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.ticker import FormatStrFormatter, MultipleLocator
import rtkcmn

def load_pos_xyz(path):
    """Load the first five columns of RTKLIB/GLINS .pos files.

    The repo produces both RTKLIB-style files whose headers start with `%`
    and helper files whose comments may start with `#`.  Older numpy.loadtxt
    calls only skipped `#`, so RTKLIB headers were parsed as data.
    """
    rows = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line or line[0] in ("%", "#"):
                continue
            parts = line.split()
            if len(parts) < 5:
                continue
            try:
                [float(v) for v in parts[:5]]
            except ValueError:
                continue
            rows.append(parts[:5])
    if not rows:
        raise ValueError(f"no position rows found in {path}")
    return np.asarray(rows, dtype=str)

def mjd2time(mjd):
    t0 = datetime.datetime(1858, 11, 17, 0, 0, 0, 0)
    return t0 + datetime.timedelta(days=mjd)

def gps_week2time(week):
    t0 = datetime.datetime(1980, 1, 6, 0, 0, 0, 0)
    return t0 + datetime.timedelta(days=week*7)
# if len(sys.argv) != 3 and len(sys.argv) != 6:
#     print('#usage  : plotkin.py kin_filename png_filename [x_ref y_ref z_ref]')
#     print('#example: plotkin.py kin_2021149_rov1 rov1_2021149')
#     print('#if no x_ref y_ref z_ref, default x_avg y_avg z_avg')
#     sys.exit(0)

if len(sys.argv) == 6:
    kinflname = [sys.argv[1]]
    pngflname = sys.argv[2]
    x_ref = float(sys.argv[3])
    y_ref = float(sys.argv[4])
    z_ref = float(sys.argv[5])

if len(sys.argv) > 6:
    argc = len(sys.argv)
    kinflname = np.array(sys.argv[1:argc-4])
    pngflname = sys.argv[argc - 4]
    x_ref = float(sys.argv[argc - 3])
    y_ref = float(sys.argv[argc - 2])
    z_ref = float(sys.argv[argc - 1])


# if len(sys.argv) == 3:
#     os.system('xyz2enu ' + kinflname + ' enu_tmp')
# else:
#     os.system('xyz2enu ' + kinflname + ' enu_tmp' + ' ' + x_ref + ' ' + y_ref + ' ' + z_ref)

# kinflname = '/home/wangchuji/catkins_lidar/data/UrbanNav-HK-Medium-Urban-1/glins.pos'
# pngflname = '/home/wangchuji/catkins_lidar/data/UrbanNav-HK-Medium-Urban-1/glins.png'
# x_ref = -2414266.9197
# y_ref = 5386768.9868
# z_ref = 2407460.0314

ref = [x_ref, y_ref, z_ref]
# Plot
fig, axs = plt.subplots(3, 1, sharex=True, figsize=(12, 12))
fig.subplots_adjust(hspace=0.12)

colors = ('k', 'r', 'b', 'g')
strs = ('East ', 'North ', 'Up ')
total_maxv = [0,0]
total_minv = [0,0]
index_file = 0
for kinf in kinflname:
    # Prepare Date
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        datatmp = load_pos_xyz(kinf)
    n = len(datatmp)
    if n == 0:
        print('Error: empty input file: enu_tmp')
        sys.exit(0)
    num = 0
    data = np.zeros((n, 5))  # week sow x y z
    for i in range(n):
        decimal = math.modf(float(datatmp[i, 1]))[0]
        if decimal < 0.05 or abs(decimal - 1) < 0.05:
            data[num, 0] = int(datatmp[i, 0])
            for j in range(4):
                data[num, j + 1] = float(datatmp[i, j + 1])
            pos = rtkcmn.ecef2pos(ref)
            data[num, 2:5] = rtkcmn.ecef2enu(pos, data[num, 2:5] - ref)
            num = num + 1

    n = num
    dts = np.zeros(n - 1)
    for i in range(n - 1):
        dts[i] = (data[i + 1, 0] - data[i, 0]) * 604800.0 + (data[i + 1, 1] - data[i, 1])
    # intv = np.nanmin(dts)
    intv = max(set(dts),key=dts.tolist().count)
    dt = (data[n - 1, 0] - data[0, 0]) * 604800.0 + (data[n - 1, 1] - data[0, 1])
    nmax = int(math.ceil(dt / intv) + 1)
    enu = np.zeros((nmax, 3))
    dates = [0] * nmax
    for i in range(n):
        dt = (data[i, 0] - data[0, 0]) * 604800.0 + (data[i, 1] - data[0, 1])
        if dt % intv < 0.5 or abs(dt % intv - intv) < 0.5:
            iepo = int(round(dt / intv))
            for j in range(3):
                enu[iepo, j] = data[i, j + 2]

    beg = gps_week2time(data[0, 0]) + datetime.timedelta(seconds=data[0, 1])
    for i in range(nmax):
        dt = i * intv
        time = beg + datetime.timedelta(seconds=dt)
        dates[i] = time
        if enu[i, 0] == 0.0 and enu[i, 1] == 0.0 and enu[i, 2] == 0.0:
            for j in range(3):
                enu[i, j] = np.nan



    # plot enu displacements
    for i in range(3):
        axs[i].plot(dates[:], enu[:, i], linewidth=1.5, color=colors[index_file])
        rms = np.sqrt(np.nanmean(np.square(enu[:, i])))
        str1 = strs[i] + "%6.2f " % (rms) + 'm'
        print(str1)
        # axs[i].text(0.98, 0.92, str1, horizontalalignment='right', verticalalignment='top',
        #             fontsize=16, transform=axs[i].transAxes)

    fwidth = 1.5
    for ax in axs:
        ax.spines['bottom'].set_linewidth(fwidth)
        ax.spines['left'].set_linewidth(fwidth)
        ax.spines['top'].set_linewidth(fwidth)
        ax.spines['right'].set_linewidth(fwidth)

    beg = gps_week2time(data[0, 0]) + datetime.timedelta(seconds=data[0, 1])
    end = gps_week2time(data[n - 1, 0]) + datetime.timedelta(seconds=data[n - 1, 1])
    dt = (end - beg).total_seconds()
    mjd = np.unique(data[:, 0])

    if dt < 86400:
        mint_intv = int(5 * (math.ceil(dt / 3600 / 3)))
        intvs = mdates.MinuteLocator(interval=mint_intv)
        if mint_intv >= 60:
            hour_intv = int(math.ceil((mint_intv) / 60))
            intvs = mdates.HourLocator(interval=hour_intv)
    else:
        nintv = int(12 / len(mjd))
        hour_intv = int(math.ceil(24 / nintv))
        if hour_intv % 2 != 0:
            hour_intv = hour_intv + 1
        intvs = mdates.HourLocator(interval=hour_intv)

    axs[2].set_xlim(beg, end)
    for ax in axs:
        ax.tick_params(axis='both', which='both', labelsize=16, pad=4)
        ax.tick_params(axis='both', which='major', length=6)
        ax.tick_params(axis='both', which='minor', length=4)
        ax.tick_params(axis='both', which='both', direction='in', right=True, top=True)
        ax.xaxis.set_major_locator(intvs)
        ax.xaxis.label.set_size(16)
        ax.yaxis.label.set_size(16)
        ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M"))
        ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))

    maxv = math.ceil(np.nanmax(enu[:, 0:2])) * 1.5
    minv = math.floor(np.nanmin(enu[:, 0:2])) - 1
    total_maxv[0] = max(maxv,total_maxv[0])
    total_minv[0] = min(minv,total_minv[0])

    maxv = math.ceil(np.nanmax(enu[:, 2])) * 1.5
    minv = math.floor(np.nanmin(enu[:, 2])) - 1
    total_maxv[1] = max(maxv,total_maxv[1])
    total_minv[1] = min(minv,total_minv[1])
    axs[2].set_xlim(beg, end)
    index_file = index_file + 1

total_minv[0] = -20
total_maxv[0] = 20
total_minv[1] = -35
total_maxv[1] = 35
for i in range(2):
    axs[i].set_ylim([total_minv[0], total_maxv[0]])
axs[2].set_ylim([total_minv[1], total_maxv[1]])



xlabel = "Time"
ylabels = ('East (m)', 'North (m)', 'Up (m)')
axs[2].set_xlabel(xlabel, labelpad=3)
for i in range(3):
    axs[i].set_ylabel(ylabels[i], labelpad=3)
    box = axs[i].get_position()
    axs[i].set_position([box.x0, box.y0, box.width , box.height* 0.8])

axs[0].legend(["RTK", "RTK/INS TC", "RTK/INS TC+LIDAR LC"],loc='center',bbox_to_anchor=(0.5, 1.2),ncol=4,fontsize=20,edgecolor='k')
plt.savefig(pngflname, pad_inches=None, bbox_inches='tight')
plt.show()
sys.exit(0)
