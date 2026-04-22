import dpkt
import collections  # 有序字典需要的模块
import time
import numpy as np
import struct

# 如果安装了pcl , 可以使用它来进行可视化
# import pcl.pcl_visualization
# viewer = pcl.pcl_visualization.PCLVisualizering()
# viewer.SetBackgroundColor(0, 0, 0)
# viewer.AddCoordinateSystem()
# viewer.InitCameraParameters()





# vlp 16 的参数
DISTANCE_RESOLUTION = 0.002   # 距离数值分辨率 转换为单位米
udp_package_num = 1
line_per_udp = 12        # 每个UDP 有多少列
point_per_udp_line = 32  # 每个UDP 的每列包含有多少个点
point_num_per_udp = point_per_udp_line * line_per_udp  # 32*12=384
thetas_lines = 16
thetas_point = thetas_lines * 2 * line_per_udp * udp_package_num
thetas_point = np.radians(thetas_point)
thetas_point_cos = np.cos(thetas_point)
thetas_point_sin = np.sin(thetas_point)


base_range = np.array(range(2, point_per_udp_line*2+1, 2))  # 32， 距离值的基础索引
angle_base_range = np.array([1])
d_range = []
r_range = []
angle_range = []
k = 0
data_gap = 2 + 2*point_per_udp_line  # 每一列的长度
for i in range(udp_package_num):
    for j in range(line_per_udp):
        d_range.append(base_range + k * data_gap + i * 2)  # 66 是 HH + HB*32
        r_range.append(base_range + k * data_gap + i * 2 + 1)
        angle_range.append(angle_base_range + k * data_gap + i * 2)
        k += 1
d_range = np.hstack(d_range)  # 多个array组成的列表
r_range = np.hstack(r_range)
angle_range = np.hstack(angle_range)

# 水平角度插值，如果有角度跳变会怎么样？  针对从360跳变到20的这部分拟合的并不是很好，误差很大；已经改正
x_index = np.arange(point_num_per_udp)
xp_index = np.arange(0, point_num_per_udp, point_per_udp_line) # array([  0,  32,  64,  96, 128, 160, 192, 224, 256, 288, 320, 352])




def unpack_udp(data):
    data_tuple = struct.unpack(data_fmt, data)  # 原始格式是元祖，要转array，元祖不能索引
    data_unpack = np.array(data_tuple, dtype=np.int64)  # np.array会多耗时15毫秒  todo 这里为什么会报错？？ 可能是时间戳的数值太大了

    distances = data_unpack[d_range]  # 115200
    refs = data_unpack[r_range] / 255
    angles = data_unpack[angle_range]
    angles = np.radians(angles / 100).astype(np.float32)  # 除以100再弧度值
    # 第一种处理角度的方式
    # angles = np.tile(angles, (32, 1)).flatten('F')  # 因为angle只有1个，数据有32个，需要复制32次
    # 第二种方式
    angles_interp = np.interp(x_index, xp_index, angles).astype(np.float32)
    if angles[0] > angles[-1]:  # 出现了角度的转折点
        # replace_angle = np.linspace(0,20,32) # 针对从360跳变到20 的角度替换
        change_index = np.argmax(angles)
        replace_index = change_index * 32 + 1
        interp_num_2 = int(angles[change_index+1]*32/40)  # 每个UDP数据包之间的角度间隔为 40，每个包有32条线；
        interp_num_1 = 32 - interp_num_2
        # angles_interp[replace_index:(replace_index+interp_num_1)] = replace_angle_1
        # angles_interp[(replace_index+interp_num_1):(replace_index+32)] = replace_angle_2


    distances = distances * DISTANCE_RESOLUTION
    x = distances * thetas_point_cos * np.sin(angles_interp)
    y = distances * thetas_point_cos * np.cos(angles_interp)
    z = distances * thetas_point_sin
    raw_points = np.stack((x, y, z), axis=1).astype(np.float32)
    # raw_points = np.stack((distances, angles_interp, refs, ), axis=1)   # 也可以只要原始数据

    return raw_points





def main(file_path):
    # f = open(file_path)          # 此写法为python2之下，
    f = open(file_path, mode='rb') #python3
    try:
        pcap = dpkt.pcap.Reader(f)  # 先按.pcap格式解析，若解析不了，则按pcapng格式解析
    except:
        print("it is not pcap ... format, pcapng format...")
        pcap = dpkt.pcapng.Reader(f)
        # 接下来就可以对pcap做进一步解析了，记住在使用结束后最好使用f.close()关掉打开的文件，虽然程序运行结束后，
        # 系统会自己关掉，但是养成好习惯是必不可少的。当前变量pcap中是按照“间戳：单包”的格式存储着各个单包
    # 将时间戳和包数据分开，一层一层解析，其中ts是时间戳，buf存放对应的包
    all_pcap_data = collections.OrderedDict()  # 有序字典
    # all_pcap_data_hex = collections.OrderedDict()  # 有序字典,存十六进制形式
    cir_point = []
    i = 1
    for (ts, buf) in pcap:
        try:
            eth = dpkt.ethernet.Ethernet(buf)  # 解包，物理层
            if not isinstance(eth.data, dpkt.ip.IP):  # 解包，网络层，判断网络层是否存在，
                continue
            ip = eth.data
            # if not isinstance(ip.data, dpkt.tcp.TCP):  # 解包，判断传输层协议是否是TCP，即当你只需要TCP时，可用来过滤
            #     continue
            if not isinstance(ip.data, dpkt.udp.UDP):#解包，判断传输层协议是否是UDP
                continue
            transf_data = ip.data  # 传输层负载数据，基本上分析流量的人都是分析这部分数据，即应用层负载流量
            if not len(transf_data.data):  # 如果应用层负载长度为0，即该包为单纯的tcp包，没有负载，则丢弃
                continue

            if len( transf_data.data) != 1206:  # 长度过滤   todo 为什么会有512字节的数据
                continue

            all_pcap_data[ts] = transf_data.data  # 将时间戳与应用层负载按字典形式有序放入字典中，方便后续分析.
            points = unpack_udp(transf_data.data)
            if i % 76 != 0:  # vlp16 每75个UDP数据包形成一圈数据
                cir_point.append(points)
            else:
                cir_udp = np.vstack(cir_point)
                print(cir_udp.shape)            # 最后需要的一圈完整的点云数据 只包含了28000余个xyz坐标
                # cloud_all = pcl.PointCloud(cir_udp[:, 0:3].astype(np.float32))   # 可视化
                # viewer.AddPointCloud(cloud_all)
                # viewer.SpinOnce(100)
                # viewer.RemoveAllPointClouds(0)
                cir_point = []

            i += 1
        except Exception as err:
            print( "[error] %s" % err)
    f.close()






if __name__ == '__main__':
    file_path="D:xxxxxx.pcap"
    main(file_path)