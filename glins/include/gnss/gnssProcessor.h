//
// Created by wangchuji on 2023/7/4.
//
#pragma once
#include "utility.h"
#include "gnss_tools.h"
static char strpath[5][256] = { "","","","","" }; /* stream paths */

static opt_t rcvopts[] = {
        {"inpstr1-path",    2,  (void*)strpath[0],         ""     },
        {"inpstr2-path",    2,  (void*)strpath[1],         ""     },
        {"inpstr3-path",    2,  (void*)strpath[2],         ""     },
        {"inpstr4-path",    2,  (void*)strpath[3],         ""     },
        {"outstr1-path",    2,  (void*)strpath[4],         ""     },

        {"",0,NULL,""}
};

//extern void postposRegisterPub(ros::NodeHandle &n);
extern void rtkposRegisterPub(ros::NodeHandle& n);
extern void pntposRegisterPub(ros::NodeHandle& n);

class gnssProcessor : public ParamServer
{
public:

    prcopt_t prcopt;            // RTKLIB 解算配置：控制定位模式、误差模型、约束等“怎么解”
    solopt_t solopt;            // RTKLIB 输出配置：控制结果格式、时间系统、表头等“怎么输出”
    filopt_t filopt = { "" };   // RTKLIB 文件配置：天线、钟差、地球潮汐等外部辅助文件路径

    /* get setup parameters */
    std::string out_folder;     // 结果输出目录

    /* flag for state */
    int n = 0;                  // 当前已登记的输入文件数量

    /* processing time setting */
    double ti = 0.0;						// processing interval  (s) (0:all)
    double tu = 0.0;						// unit time (s) (0:all)
    gtime_t ts = { 0,0 }, te = { 0,0 }; // start time and end time(0:all)

    char* rov = "", * base = "";            // rover/base 站标识，留给 RTKLIB 接口使用
    char infile_[10][1024] = { "" }, * infile[10]; // 输入文件缓存区及其指针数组
    char outfile[1024];                     // 输出结果文件路径

    gnssProcessor()
    {
        int i;

        /* 注册 GNSS 结果发布接口 */
        rtkposRegisterPub(nh);
        pntposRegisterPub(nh);

        /* 先用 RTKLIB 默认值初始化，避免未初始化；后面再由配置文件覆盖 */
        prcopt = prcopt_default;	// 默认解算参数
        solopt = solopt_default;	// 默认输出参数
        //        filopt = {""};	            // file option
        //        rtklibConfigPath = "/home/wangchuji/catkins_lidar/GLINS/config/conf/urban.conf";

        /* 从配置文件读取系统参数和接收机/文件路径参数 */
        if (!loadopts(rtklibConfigPath.c_str(), sysopts) || !loadopts(rtklibConfigPath.c_str(), rcvopts))
        {
            exit(1);
        }

        /* 将 loadopts() 读入的配置拷贝到 prcopt/solopt/filopt，覆盖前面的默认值 */
        getsysopts(&prcopt, &solopt, &filopt);



        /* 先让每个 infile[i] 指向类内自带的固定缓存区 */
        for (i = 0;i < 10;i++) infile[i] = infile_[i];

        /* 读取配置文件中的输入路径，填入前 3 个输入文件槽位 */
        for (i = 0;i < 3;i++)
        {
            //            if(prcopt.mode==PMODE_SINGLE&&i==1) continue;
            //先给 infile[i] 分配一块 1024 字节的内存
            // 再把配置文件里读到的路径 strpath[i] 复制进去
            // 后面把 infile 作为输入文件列表传给 postpos()
            infile[i] = (char*)malloc(1024); // 为路径单独分配空间，便于传给 RTKLIB 后处理接口
            strcpy(infile[i], strpath[i]);   // strpath[] 来自 rcvopts，对应配置文件中的 inpstr*-path
            n++;                             // 记录当前有效输入文件数
        }

        /* 输出结果文件路径来自配置项 outstr1-path */
        strcpy(outfile, strpath[4]);

        /* if you use the RTK mode, specify the position of the station (only used by RTKLIB)
         * following is an example position of the base HKSC in Hong Kong */
         //        prcopt.rb[0] = -2414266.9197;			// base position for relative mode {x,y,z} (ecef) (m)
         //        prcopt.rb[1] = 5386768.9868;			// base position for relative mode {x,y,z} (ecef) (m)
         //        prcopt.rb[2] = 2407460.0314;			// base position for relative mode {x,y,z} (ecef) (m)

        /* 在配置文件基础上，进一步加入本项目自定义的处理参数 */
        prcopt.DopplerConstraint = (prcopt.dynamics == 1 ? 1 : 0); // 开启动力学模型时，同时启用多普勒速度约束
        prcopt.StateModel = 1;                                      // 选择项目内定义的状态模型
        prcopt.Dop2PrRatio = 12.5;                                  // 设置多普勒与伪距残差的权重比例
        prcopt.thresdop = 10;                                       // 多普勒残差阈值
        matcpy(prcopt.default_rb, prcopt.rb, 3, 1);                 // 备份基站坐标初值，供后续流程参考
        /* 上面完成了解算器配置、文件路径和项目级参数修正，后续即可进入 RINEX 解算流程 */

    }

    void decode(gtime_t start_time = { 0 }, gtime_t end_time = { 0 })
    {
        /* 设置本次后处理的起止时刻；传入 0 值时表示按 RTKLIB 默认规则处理全部历元 */
        ts = start_time;//gpst2time(2192,463000); //462163
        //        ts = gpst2time(2192,462360);
        //        te = gpst2time(2192,464583);
        te = end_time;//gpst2time(2192,463583);//464583

        //        ts = gpst2time(2233,8*3600+35*60+86400*6);
        //        te = gpst2time(2233,8*3600+55*60+86400*6);

        /* 调用 RTKLIB 后处理主入口 postpos():
         * - 读取输入观测/星历文件
         * - 按 prcopt 指定的策略执行定位解算
         * - 按 solopt 指定的格式输出结果到 outfile */
        int stat = postpos(ts, te, ti, tu, &prcopt, &solopt, &filopt, infile, n, outfile, rov, base);

        printf("\n");

        /* postpos() 返回状态：
         *   0  : 正常完成
         *   >0 : 执行出错
         *  -1  : 处理被中止 */
        if (stat == 0)
        {
            ROS_INFO("\033[1;32m----> gnss Processor Finished.\033[0m");
        }
        else if (stat > 0)
        {
            ROS_INFO("\033[1;32m----> gnss Processor Error!!!.\033[0m");
            exit(2);
        }
        else if (stat == -1)
        {
            ROS_INFO("\033[1;32m----> gnss Processor Aborted!!!.\033[0m");
            exit(-1);
        }
        // system("pause");
    }

};


//int main(int argc, char **argv)
//{
//
//    ros::init(argc, argv, "gnss_preprocessor_node");
//
//    ROS_INFO("\033[1;32m----> gnss Processor Started.\033[0m");
//
//
//
//    ros::spin();
//
//    return 0;
//}
