//
// Created by wangchuji on 2023/7/6.
//
#pragma once
#include "ros/ros.h"
#include "utility.h"
#include "gnss_tools.h"
#include <gtsam/slam/PriorFactor.h>
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "glins/GNSS_Info.h"
#include "glins/GNSS_Info_ZD.h"
#include "glins/GNSS_Info_SD.h"
#include "factor/PseudorangeFactor.h"
#include "factor/DopplerFactor.h"
#include "factor/CarrierPhaseFactor.h"
#include "rtklib.h"
#include "gtsam/nonlinear/Marginals.h"
#include "marginalize_isam2.h"
#include <gtsam/nonlinear/ExtendedKalmanFilter.h>

//#include "MotionFactor.h"
//#include "CarrierPhaseFactor.h"
using namespace gtsam;
using namespace symbol_shorthand;

static int test_sys(int sys, int m)
{
    switch (sys)
    {
    case SYS_GPS: return m == 0;
    case SYS_SBS: return m == 0;
    case SYS_GLO: return m == 1;
    case SYS_GAL: return m == 2;
    case SYS_CMP: return m == 3;
    case SYS_QZS: return m == 4;
    case SYS_IRN: return m == 5;
    }
    return 0;
}

class gnssEstimator : public ParamServer
{
public:
    NonlinearFactorGraph graph;
    Values initialEstimate;// 优化初值，包含状态变量的初始猜测。
    Values initialVel;// 初始速度估计，主要用于构造 IMU 因子时的初值。
    ISAM2 optimizer;// ISAM2 优化器，负责增量式优化。
    Values currentEstimate;// 当前优化结果，包含最新的状态估计。

    ros::Publisher pub_FGOENU;// 发布 FGO 优化结果的路径消息。
    ros::Publisher pub_WLSENU;// 发布 WLS 结果的路径消息。
    ros::Publisher pub_predict_ENU;// 发布 FGO 预测结果的路径消息。

    std::deque<rtklib::GNSS_Info> gnss_queue;// GNSS 观测队列，存储接收到的 GNSS 观测信息。
    rtklib::GNSS_Info gnss_info;// 当前 GNSS 观测信息，包含卫星观测数据和相关状态。
    GNSS_Tools gnssTools;// GNSS 工具类，提供坐标转换、预处理等功能。

    nav_msgs::Path fgo_path;// FGO 优化结果路径消息，用于可视化优化结果。
    nav_msgs::Path wls_path;// WLS 结果路径消息，用于可视化 WLS 结果。
    nav_msgs::Path fgo_reltime_path;// FGO 预测结果路径消息，用于可视化 FGO 预测结果。

    std::map<int, double> time_map;// 时间映射表，记录每个优化轮次的时间戳，用于分析优化性能和时间分布。
    ros::Subscriber sub_gnss_raw;// 订阅 GNSS 原始观测信息的 ROS 话题，接收 GNSS 观测数据。
    std::mutex gnss_mutex;// 互斥锁，保护 GNSS 观测数据的访问，确保线程安全。

    //    std::thread optimizationThread;
    Eigen::Matrix<double, 3, 1> ENU_ref;
    Point3 latestValue;
    Vector prevAmb;
    Vector AmbFullArray;

    //    std::string logPath = "/home/wangchuji/catkins_lidar/data/UrbanNav-HK-Medium-Urban-1/rtk_fgo.pos";
    std::string logPath = "/home/wangchuji/catkins_gnss/rtklib_GSDC_6th/data/GNSS_INS_Data/urban/rtklib_fgo.pos";
    int epoch = 0;
    int lastkey = 0;
    int relinearizeSkip = 1;
    double relinearizeThreshold = 0.001;
    double average_time = 0;

    prcopt_t prcopt;
    solopt_t solopt;
    filopt_t filopt = { "" };
    bool first_init = true;
    bool slide_window = false;
    double pre_gpstime[2];
    MatrixXd posCovariance = Matrix3d::Identity();
    MatrixXd ambCovariance;

    fgo_t fgo;
    ssat_t ssat[MAXSAT];

    int windows_size = 1000;
    //#define PHASEBIAS_as_WHOLE
    //#define PHASEBIAS_as_SINGLE
#define PHASEBIAS_as_COMPRESSED
#ifdef PHASEBIAS_as_SINGLE
    VectorXi phase_count;
    vector<int> prn;
#endif

#ifdef PHASEBIAS_as_COMPRESSED
    map<int, int> last_ar_index;
#endif
    gnssEstimator()
    {

        prcopt = prcopt_default;
        solopt = solopt_default;
        if (!loadopts(rtklibConfigPath.c_str(), sysopts))
        {
            exit(1);
        }
        getsysopts(&prcopt, &solopt, &filopt);
        //        prcopt.rb[0] = -2414266.9197;			// base position for relative mode {x,y,z} (ecef) (m)
        //        prcopt.rb[1] = 5386768.9868;			// base position for relative mode {x,y,z} (ecef) (m)
        //        prcopt.rb[2] = 2407460.0314;			// base position for relative mode {x,y,z} (ecef) (m)

                /* thread for optimization */
        //        optimizationThread = std::thread(&gnssEstimator::solveOptimization,this);
                /*publisher*/
        pub_WLSENU = nh.advertise<nav_msgs::Path>("/WLS_path", 5000);

        pub_FGOENU = nh.advertise<nav_msgs::Path>("/FGOGlobalPath", 5000);

        pub_predict_ENU = nh.advertise<nav_msgs::Path>("/fgo_predict_path", 5000);
        /*subscriber*/
        sub_gnss_raw = nh.subscribe<rtklib::GNSS_Info>("gnss_raw", 100, &gnssEstimator::gnssInfoHandler, this, ros::TransportHints().tcpNoDelay());

        Vector3 ecef_ref(prcopt.rb[0], prcopt.rb[1], prcopt.rb[2]);
        ENU_ref << GNSS_Tools::ecef2llh(ecef_ref);
        //        ISAM2Params params;
        //        params.optimizationParams = ISAM2DoglegParams(1.0,1e-5,DoglegOptimizerImpl::TrustRegionAdaptationMode::ONE_STEP_PER_ITERATION);
        //        params.relinearizeThreshold = relinearizeThreshold;
        //        params.relinearizeSkip = relinearizeSkip;
        ////        params.evaluateNonlinearError = true;
        //        params.factorization = ISAM2Params::QR;
        //
        //        optimizer = ISAM2(params);
        resetOptimization();

        //        rtkinit(&rtk,&prcopt);
        GNSS_Tools::fgoinit(&fgo, &prcopt);
        prevAmb = VectorXd(NB(&prcopt)).setZero();
        ambCovariance = MatrixXd(NB(&prcopt), NB(&prcopt)).setZero();
        AmbFullArray = VectorXd(NB(&prcopt)).setZero();
#ifdef PHASEBIAS_as_SINGLE
        phase_count = VectorXi(NB(&prcopt)).setZero();
        for (int i = 0;i < NB(&prcopt) - 1;i++)
        {
            phase_count(i + 1) = phase_count(i) + windows_size;
        }
        prn.clear();
#endif

        gnss_queue.clear();
        std::ofstream foutC(fgoPath, std::ios::ate);
        foutC << "%%  GPST              x-ecef(m)      y-ecef(m)      z-ecef(m)   Q  ns   sdx(m)   sdy(m)   sdz(m)  sdxy(m)  sdyz(m)  sdzx(m) age(s)  ratio\n";
        foutC.close();
    }
    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        //        optParameters.optimizationParams = ISAM2DoglegParams(1.0,1e-5,DoglegOptimizerImpl::TrustRegionAdaptationMode::SEARCH_EACH_ITERATION);
        optParameters.relinearizeThreshold = relinearizeThreshold;
        optParameters.relinearizeSkip = relinearizeSkip;
        optParameters.factorization = gtsam::ISAM2Params::CHOLESKY;


        optimizer = ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graph = newGraphFactors;

        gtsam::Values NewGraphValues;
        initialEstimate = NewGraphValues;
    }

    /***
     *
     *
     * @param opt I
     * @param ix I
     * @param nb I
     * @param amb I
     * @param amb_cov I
     * @param joint_pos_amb I
     * @param dx O
     */
    int manage_Amb_LAMBDA(prcopt_t* opt, const MatrixXi& ix, VectorXd& bias, int nb, const VectorXd& amb, const MatrixXd& amb_cov, Vector3& dx)
    {
        double* y, * DP, * b, * db, * Qb, * Qab, * QQ, s[2], xa[3] = { 0 };
        y = mat(nb, 1); DP = mat(nb, NB(opt)); b = mat(nb, 2); db = mat(nb, 1); Qb = mat(nb, nb);
        Qab = mat(3, nb); QQ = mat(3, nb);
        int i, j, info, ratio;
        TicToc t_margin;
        t_margin.tic();
        //        optimizer.getFactorsUnsafe().printErrors(currentEstimate);
        gtsam::ISAM2::sharedFactorGraph amb_graph = optimizer.joint(X(epoch), N(epoch));
        KeyVector variables{ X(epoch),N(epoch) };
        Marginals marginals(*amb_graph, currentEstimate, Marginals::QR);

        JointMarginal pos_amb_cov = marginals.jointMarginalCovariance(variables);
        MatrixXd joint_pos_amb = pos_amb_cov(X(epoch), N(epoch));
        ROS_INFO("marginal time %.3lf", t_margin.toc());

        for (i = 0;i < nb;i++)
        {
            y[i] = amb[ix(0, i)] - amb[ix(1, i)];
        }
        for (j = 0;j < NB(opt);j++) for (i = 0;i < nb;i++)
        {
            DP[i + j * nb] = amb_cov(ix(0, i), j) - amb_cov(ix(1, i), j);
        }
        for (j = 0;j < nb;j++) for (i = 0;i < nb;i++)
        {
            Qb[i + j * nb] = DP[i + (ix(0, j)) * nb] - DP[i + (ix(1, j)) * nb];
        }
        for (j = 0;j < nb;j++) for (i = 0;i < 3;i++)
        {
            Qab[i + j * 3] = joint_pos_amb(i, ix(0, j)) - joint_pos_amb(i, ix(1, j));
        }
        if (!(info = lambda(nb, 2, y, Qb, b, s)))
        {
            ratio = s[0] > 0 ? (float)(s[1] / s[0]) : 0.0f;
            if (ratio > 999.9) ratio = 999.9f;
            for (i = 0;i < nb;i++)
            {
                bias[i] = b[i];
                y[i] -= b[i];
            }
            if (s[0] <= 0.0 || s[1] / s[0] >= prcopt.thresar[0])
            {
                if (!matinv(Qb, nb))
                {
                    ROS_INFO("ambiguity validation successed! (nb=%d ratio=%.2f s=%.2f/%.2f)\n",
                        nb, s[1] / s[0], s[0], s[1]);
                    matmul("NN", nb, 1, nb, 1.0, Qb, y, 0.0, db); /* db = Qb^-1*(b0-b) */
                    matmul("NN", 3, 1, nb, -1.0, Qab, db, 1.0, xa); /* rtk->xa = rtk->x-Qab*db */
                    memcpy(dx.data(), xa, sizeof(double) * 3);
                    free(y); free(DP); free(b); free(db); free(Qb); free(Qab); free(QQ);
                    return 1;
                }
            }
            else
            {
                ROS_ERROR("ambiguity validation failed (nb=%d ratio=%.2f s=%.2f/%.2f)\n",
                    nb, s[1] / s[0], s[0], s[1]);
            }
        }
        else
        { /* validation failed */
            ROS_ERROR("lambda error (info=%d)", info);
            nb = 0;
        }
        free(y); free(DP); free(b); free(db); free(Qb); free(Qab); free(QQ);
        return 0;


    }

    void solveOptimization()
    {
        static MatrixXi ix(2, NB(&prcopt));
        static VectorXd bias(NB(&prcopt));
        static int nb, npr, ncp, ndop, nv;
        //        gnss_queue.clear();
        while (ros::ok())
        {
            gnss_mutex.lock();
            if (!gnss_queue.empty())
            {
                nb = 0;npr = 0;ndop = 0;nv = 0;
                gnss_info = gnss_queue.front();
                Point3 pos(gnss_info.pos[0], gnss_info.pos[1], gnss_info.pos[2]);
                //                Point3 pos(0,0,0);

                fgo.rtk.sol.time = gpst2time(gnss_info.gpsWeek, gnss_info.weekSec);
                if (gnss_info.SD_Infos.size() < 1)
                {
                    gnss_queue.pop_front();
                    gnss_mutex.unlock();
                    continue;
                }
                TicToc t_epoch;
                t_epoch.tic();
                /*** initial pos ***/
                if (first_init)
                {
                    graph.add(PriorFactor<Point3>(X(epoch), pos,
                        noiseModel::Diagonal::Variances(
                            Vector3(900, 900, 900))));
                    //                                                          Vector3(gnss_info.var[0], gnss_info.var[1],
                    //                                                                  gnss_info.var[2]))));
                }
                //                if (epoch==windows_size){
                //                    gtsam::noiseModel::Gaussian::shared_ptr
                //                            updatedPosNoise = gtsam::noiseModel::Gaussian::Covariance(posCovariance);
                //#ifdef PHASEBIAS_as_WHOLE
                //                    gtsam::noiseModel::Gaussian::shared_ptr
                //                            updatedAmbNoise = gtsam::noiseModel::Gaussian::Covariance(ambCovariance);
                //#endif
                //
                ////                    cout << ambCovariance.diagonal() << endl;
                //                    resetOptimization();
                //                    graph.add(PriorFactor<Point3>(X(0), latestValue,
                //                                                  updatedPosNoise));
                //#ifdef PHASEBIAS_as_SINGLE
                //                    vector<PriorFactor<double>> priorphasefactors;
                //                    for (int i = 0; i < prn.size();i++){
                //                        gtsam::noiseModel::Gaussian::shared_ptr
                //                                updatedAmbNoise = gtsam::noiseModel::Gaussian::Covariance(ambCovariance.block(prn[i],prn[i],1,1));
                //                        priorphasefactors.push_back(PriorFactor<double>(N(phase_count(prn[i])+0),prevAmb(prn[i]),updatedAmbNoise));
                //                        initialEstimate.insert(N(phase_count(prn[i])+0),prevAmb(prn[i]));
                //                    }
                //                    graph.add(priorphasefactors);
                //#endif
                //
                //
                ////                    if (fgo.num_factor[1]!=0) {
                //#ifdef PHASEBIAS_as_WHOLE
                //                        initialEstimate.insert(N(0), prevAmb);
                //                        graph.add(PriorFactor<Vector>(N(0), prevAmb, updatedAmbNoise));
                //#endif
                //
                ////                    }
                //                    initialEstimate.insert(X(0), latestValue);
                //                    epoch = 1;
                //                    slide_window = true;
                //                }
                initialEstimate.insert(X(epoch), pos);
                nb = addDDCpFactor();
                npr = addDDPsrFactor();
                if (!first_init)
                {
                    //                    ndop = addSDDopFactor();
                    //                    if(ndop < 4) {
                    ////                        MatrixXd velcov(3, 3);
                    ////                        velcov << gnss_info.velvar[0], gnss_info.velvar[3], gnss_info.velvar[5],
                    ////                                gnss_info.velvar[3], gnss_info.velvar[1], gnss_info.velvar[4],
                    ////                                gnss_info.velvar[5], gnss_info.velvar[4], gnss_info.velvar[2];
                    ////                        gtsam::noiseModel::Gaussian::shared_ptr
                    ////                                velNoise = gtsam::noiseModel::Gaussian::Covariance(velcov);
                    ////                        noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(
                    ////                                noiseModel::mEstimator::Huber::Create(1.0), velNoise);
                    //                        graph.add(BetweenFactor<Point3>(X(epoch - 1), X(epoch),
                    //                                                        Point3(gnss_info.vel[0], gnss_info.vel[1], gnss_info.vel[2]),
                    //                                                        noiseModel::Diagonal::Variances((Vector(3) << gnss_info.velvar[0],gnss_info.velvar[1],gnss_info.velvar[2]).finished())));
                    //                    }
                }
                else
                {
                    first_init = false;
                }
                nv = npr + nb + ndop;
                fgo.num_factor[0] = npr;
                fgo.num_factor[2] = ndop;
                //
//                if ((npr < 4 || ndop < 6)&&epoch!=0){
//                    Point3 pos_prior(gnss_info.pos[0],gnss_info.pos[1],gnss_info.pos[2]);
//                    gtsam::noiseModel::Gaussian::shared_ptr
//                            constraintPosNoise = gtsam::noiseModel::Isotropic::Sigma(3,30);
//                    graph.add(PriorFactor<Point3>(X(epoch), pos_prior,
//                                                  constraintPosNoise));
//                }
                if (slide_window)
                {
                    //                    graph.printErrors(initialEstimate);
                    slide_window = false;
                }
                //                if(npr < 4){
                //                    ROS_INFO("nv is less than one");
                //                    graph.resize(0);
                //                    initialEstimate.clear();
                //                    gnss_queue.pop_front();
                //                    gnss_mutex.unlock();
                //                    continue;
                //                }
                //                {
                //                    currentEstimate = LevenbergMarquardtOptimizer(graph, initialEstimate).optimize();
                //                }
                ROS_INFO("epoch %d add factor nv:%d npr:%d nb:%d ndop:%d ncp:%d", epoch, nv, npr, nb, ndop, fgo.num_factor[1]);
                TicToc t_opt;
                t_opt.tic();
                //                if(epoch == 597) {
                //                    Values tempInitial;
                ////                    tempInitial.insert(N(epoch), initialEstimate.at(N(epoch)));
                //                    tempInitial.insert(X(epoch), initialEstimate.at(X(epoch)));
                ////                    tempInitial.insert(N(epoch-1), optimizer.calculateEstimate(N(epoch-1)));
                //////                    tempInitial.insert(X(epoch-1), optimizer.calculateEstimate(X(epoch-1)));
                ////////        if (!first_init) {
                ////////            tempInitial.insert(N(epoch - 1), prevAmb);
                ////////        }
                ////////        tempInitial.insert(X(epoch),pos);
                //                    ofstream tracefile("/home/wangchuji/catkins_lidar/GLINS/data/test3.txt",ios::ate);
                ////////                    tracefile << "compress_ar\n" << (VectorXd)initialEstimate.at(N(epoch)) << endl;
                ////////
                ////////        Eigen::Map<VectorXd> amb(gnss_info.amb.data(),gnss_info.amb.size(), 1);
                ////////        tracefile << "gnss_info_ar\n" << amb << endl;
                //                    MatrixXd jacobian = graph.linearizeToHessianFactor(tempInitial)->jacobian().first;
                //                    tracefile << "jacobian\n" << jacobian << endl;
                //                    MatrixXd information = graph.linearizeToHessianFactor(tempInitial)->information();
                //                    tracefile << "information\n" << information << endl;
                //                    tracefile.close();
                //                }
                //                PrintKeyVector(graph.keyVector());
                updateAndMarginalize(graph, initialEstimate, {}, optimizer);

                //                optimizer.update(graph, initialEstimate);

                //                optimizer.update();
                //                optimizer.update();
                //                optimizer.update();
                currentEstimate = optimizer.calculateEstimate();
                graph.resize(0);
                initialEstimate.clear();
                //    ROS_INFO("opt time %.3lf",t_opt.toc());
                static VectorXd dx(3);
                static int state;
                static double var;
                static Point3 sol_pos;
                //                ROS_INFO("calculateEstimate size:%d nv:%d npr:%d nb:%d ndop:%d",currentEstimate.size(),nv,npr,nb,ndop);
                latestValue = currentEstimate.at<Point3>(X(epoch));
                //                boost::shared_ptr<JacobianFactor> marginFactor = optimizer.marginalFactor(X(epoch),EliminatePreferCholesky);
                //                boost::shared_ptr<JacobianFactor> posFactor(new JacobianFactor(X(epoch),marginFactor->getA(),
                //                                                                               marginFactor->getb() - marginFactor->getA().transpose() *latestValue,
                //                                                                               marginFactor->get_model()));
                //                posCovariance = posFactor->information().inverse();
                //                var = (posCovariance(0,0)+posCovariance(1,1)+posCovariance(2,2))/3;
                //                ROS_INFO("SIZE B:%d" ,marginFactor->getb().size());
                //                ROS_INFO("position variance posFactor:  %.4f pos_ocv %.4f %.4f %.4f",var,posCovariance(0,0),posCovariance(1,1),posCovariance(2,2));

                posCovariance = optimizer.marginalCovariance(X(epoch));//optimizer.marginalCovariance(X(epoch));



                //                if (fgo.num_factor[1]>0)
#ifdef PHASEBIAS_as_WHOLE
                prevAmb = currentEstimate.at<Vector>(N(epoch));
                ambCovariance = optimizer.marginalCovariance(N(epoch));
#endif
#ifdef PHASEBIAS_as_COMPRESSED
                if (nb > 0)
                {
                    prevAmb = currentEstimate.at<Vector>(N(epoch));
                    ambCovariance = optimizer.marginalCovariance(N(epoch));
                }
#endif
#ifdef PHASEBIAS_as_SINGLE
                prevAmb.setZero();
                int ns = prn.size();
                for (int i = 0; i < ns;i++)
                {
                    prevAmb(prn[i]) = currentEstimate.at<double>(N(phase_count(prn[i]) + epoch));
                    //                    ambCovariance(prn[i],prn[i]) = optimizer.marginalCovariance(N(phase_count(prn[i])+epoch))(0,0);
                }
#endif
                var = (posCovariance(0, 0) + posCovariance(1, 1) + posCovariance(2, 2)) / 3;
                sol_pos = latestValue;
                state = 6;
                TicToc t_fix;
                t_fix.tic();
#define NEW_AMB_FIX
#ifdef OLD_AMB_FIX
                if (var < prcopt.thresar[1] && nb >= (prcopt.minfixsats - 1))
                {
                    if (manage_Amb_LAMBDA(&prcopt, ix, bias, nb, prevAmb, ambCovariance, dx))
                    {
                        sol_pos = latestValue + dx;
                        state = 1;
                        noiseModel::Base::shared_ptr noise = noiseModel::Isotropic::Sigma(nb, 0.1);
                        noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0), noise);
                        graph.add(GNSSFixConstraint(N(epoch), nb, ix, bias, huber));
                        //                        graph.printErrors(currentEstimate);
                        optimizer.update(graph);
                        graph.resize(0);
                        currentEstimate = optimizer.calculateEstimate();
                        ROS_INFO("ambiguity fix before: position variance %.4f pos_ocv %.4f %.4f %.4f\n", var, posCovariance(0, 0), posCovariance(1, 1), posCovariance(2, 2));
                        posCovariance = optimizer.marginalCovariance(X(epoch));
                        ambCovariance = optimizer.marginalCovariance(N(epoch));
                        var = (posCovariance(0, 0) + posCovariance(1, 1) + posCovariance(2, 2)) / 3;
                        ROS_INFO("ambiguity fix after: position variance %.4f pos_ocv %.4f %.4f %.4f\n", var, posCovariance(0, 0), posCovariance(1, 1), posCovariance(2, 2));
                        latestValue = sol_pos;//currentEstimate.at<Point3>(X(epoch));
                        prevAmb = currentEstimate.at<Vector>(N(epoch));
                    }
                }
                else
                {
                    ROS_INFO("position variance too large:  %.4f pos_ocv %.4f %.4f %.4f\n", var, posCovariance(0, 0), posCovariance(1, 1), posCovariance(2, 2));
                }
#endif
#ifdef NEW_AMB_FIX
                if (var < (prcopt.thresar[1]) && nb>0)
                {
                    TicToc t_margin;
                    t_margin.tic();
                    gtsam::ISAM2::sharedFactorGraph amb_graph = optimizer.joint(X(epoch), N(epoch));
                    KeyVector variables{ X(epoch),N(epoch) };
                    Marginals marginals(*amb_graph, currentEstimate, Marginals::CHOLESKY);
                    JointMarginal pos_amb_margin = marginals.jointMarginalCovariance(variables);
                    MatrixXd pos_amb_cov = pos_amb_margin.fullMatrix();
#ifdef PHASEBIAS_as_WHOLE
                    ambCovariance = pos_amb_cov.block(0, 0, NB(&prcopt), NB(&prcopt));
#endif
#ifdef PHASEBIAS_as_COMPRESSED
                    //                    ambCovariance = pos_amb_cov.block(0,0,nb,nb);
#endif
                    MatrixXd joint_pos_amb = pos_amb_margin(X(epoch), N(epoch));
                    //                    ROS_INFO("marginal time %.3lf",t_margin.toc());
                    TicToc t_amb;
                    t_amb.tic();
#ifdef PHASEBIAS_as_WHOLE
                    if ((nb = GNSS_Tools::manage_Amb_LAMBDA(&fgo.rtk, ix, prevAmb, ambCovariance, joint_pos_amb, bias, dx)) > 1)
                    {
#endif
#ifdef PHASEBIAS_as_COMPRESSED
                        if ((nb = GNSS_Tools::manage_Amb_LAMBDA(&fgo.rtk, ix, prevAmb, ambCovariance, joint_pos_amb, bias, dx, last_ar_index)) > 1)
                        {
#endif

                            //                        ROS_INFO("manage_Amb_LAMBDA time %.3lf",t_amb.toc());
                            sol_pos = latestValue + dx;
                            state = 1;
                            if (++fgo.rtk.nfix >= fgo.rtk.opt.minfix)
                            {
                                noiseModel::Base::shared_ptr noise = noiseModel::Isotropic::Sigma(nb, sqrt(0.1));
                                noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(
                                    noiseModel::mEstimator::Huber::Create(1.0), noise);
                                graph.add(GNSSFixConstraint(N(epoch), nb, ix, bias, huber));
                                //                            graph.printErrors(currentEstimate);
                                //                            optimizer.update(graph);
                                updateAndMarginalize(graph, {}, {}, optimizer);
                                graph.resize(0);
                                currentEstimate = optimizer.calculateEstimate();
                                //                            ROS_INFO("ambiguity fix before: position variance %.4f pos_ocv %.4f %.4f %.4f\n", var,
                                //                                     posCovariance(0, 0), posCovariance(1, 1), posCovariance(2, 2));
                                posCovariance = optimizer.marginalCovariance(X(epoch));
                                ambCovariance = optimizer.marginalCovariance(N(epoch));
                                var = (posCovariance(0, 0) + posCovariance(1, 1) + posCovariance(2, 2)) / 3;
                                //                            ROS_INFO("ambiguity fix after: position variance %.4f pos_ocv %.4f %.4f %.4f\n", var,
                                //                                     posCovariance(0, 0), posCovariance(1, 1), posCovariance(2, 2));
                                latestValue = sol_pos;//currentEstimate.at<Point3>(X(epoch));
                                prevAmb = currentEstimate.at<Vector>(N(epoch));
                            }
                        }
                        else
                        {
                            if (nb == -1) state = 4;
                            if (nb == 0) state = 3;
                            fgo.rtk.nfix = 0;
                        }
                }
                    else
                    {
                        fgo.rtk.nfix = 0;
                        ROS_INFO("position variance too large:  %.4f pos_ocv %.4f %.4f %.4f", var, posCovariance(0, 0), posCovariance(1, 1), posCovariance(2, 2));
                    }
                    for (int i = 0;i < MAXSAT;i++) for (int f = 0;f < NFREQ;f++)
                    {
                        /* inc lock count if this sat used for good fix */
                        if (!fgo.rtk.ssat[i].vsat[f]) continue;
                        fgo.rtk.ssat[i].outc[f] = 0;
                        if (fgo.rtk.ssat[i].lock[f] < 0 || (fgo.rtk.nfix > 0 && fgo.rtk.ssat[i].fix[f] == 2))
                            fgo.rtk.ssat[i].lock[f]++;
                    }
#endif
                    int ns = gnss_info.SD_Infos.size();
                    for (int k = 0;k < ns;k++)
                    {
                        int sat = gnss_info.SD_Infos[k].Mea_Rover.sat;
                        memcpy(fgo.rtk.ssat[sat - 1].rejc, gnss_info.SD_Infos[k].ssat.rejc.data(), sizeof(uint32_t) * 3);
                    }
                    memcpy(AmbFullArray.data(), gnss_info.amb.data(), sizeof(double) * NB(&prcopt));
                    for (std::map<int, int>::iterator it = last_ar_index.begin(); it != last_ar_index.end();it++)
                    {
                        AmbFullArray[it->first - 1] = prevAmb[it->second];
                    }
                    if (epoch >= 10)
                    {
                        KeySet marginalKeys;
                        marginalKeys.insert(X(epoch - 10));
                        if (optimizer.valueExists(N(epoch - 10)))
                        {
                            marginalKeys.insert(N(epoch - 10));
                        }
                        updateAndMarginalize({}, {}, marginalKeys, optimizer);
                    }
                    average_time = (average_time * epoch + t_opt.toc()) / (epoch + 1);
                    ROS_INFO("average time %.3lf", average_time);
                    ROS_INFO("opt time %.3lf", t_opt.toc());
                    //                ROS_INFO("amb fix time %.3lf",t_fix.toc());
                    gtime_t gpsTime = gpst2time(gnss_info.gpsWeek, gnss_info.weekSec);
                    time_map[epoch] = (double)gpsTime.time + gpsTime.sec;

                    //                updatePath();

                    logResult(fgoPath, sol_pos, gnss_info.gpsWeek, gnss_info.weekSec, state, nv);
                    ROS_INFO("total time %.3lf\n", t_epoch.toc());
                    pre_gpstime[0] = gnss_info.gpsWeek;
                    pre_gpstime[1] = gnss_info.weekSec;
                    epoch++;
                    gnss_queue.pop_front();
            }

                gnss_mutex.unlock();
                //            std::chrono::milliseconds dura(10);
                //            std::this_thread::sleep_for(dura);
        }

    }

        int addDDPsrFactor()
        {
            int validSat[MAXSAT] = { 0 };
            int ns = gnss_info.SD_Infos.size();
            int npr = 0;
            for (int m = 0; m < 6; m++)
            {
                for (int f = 0; f < 3; f++)
                {
                    int sysi, sysj;
                    int sati, satj;
                    int i, j;
                    for (i = -1, j = 0; j < ns; j++)
                    {
                        rtklib::sat_state ssatj = gnss_info.SD_Infos[j].ssat;
                        sysi = ssatj.sys;
                        //                    if (!ssatj.vsatP[f]) continue;
                        if (!test_sys(sysi, m) || sysi == SYS_SBS) continue;
                        if (gnss_info.SD_Infos[j].Mea_Rover.P[f] == 0.0 || gnss_info.SD_Infos[j].Mea_Base.P[f] == 0.0
                            || ssatj.azel[1] == 0.0 || ssatj.azel_b[1] == 0.0)
                            continue;
                        if (satexclude(gnss_info.SD_Infos[j].Mea_Rover.sat, ssatj.ephvar, ssatj.svh, &prcopt)) continue;
                        if (i >= 0 && gnss_info.SD_Infos[j].ssat.slip[f] & LLI_SLIP) continue;
                        if (i < 0 || ssatj.azel[1] >= gnss_info.SD_Infos[i].ssat.azel[1]) i = j;
                    }
                    if (i < 0) continue;

                    rtklib::GNSS_Info_SD Info_Master = gnss_info.SD_Infos[i];
                    vector<rtklib::GNSS_Info_SD> Infos_Else;
                    for (j = 0; j < ns; j++)
                    {
                        if (i == j) continue;  /* skip ref sat */
                        rtklib::sat_state ssatj = gnss_info.SD_Infos[j].ssat;
                        sysj = ssatj.sys;
                        satj = gnss_info.SD_Infos[j].Mea_Rover.sat;
                        if (!test_sys(sysj, m)) continue;
                        if (gnss_info.SD_Infos[j].Mea_Rover.P[f] == 0.0 || gnss_info.SD_Infos[j].Mea_Base.P[f] == 0.0
                            || ssatj.azel[1] == 0.0 || ssatj.azel_b[1] == 0.0)
                            continue;
                        if (satexclude(gnss_info.SD_Infos[j].Mea_Rover.sat, ssatj.ephvar, ssatj.svh, &prcopt)) continue;
                        if (!ssatj.vsatP[f])
                        {
                            continue;
                        }
                        Infos_Else.push_back(gnss_info.SD_Infos[j]);
                        if (f == 0)
                            validSat[gnss_info.SD_Infos[j].Mea_Rover.sat - 1] = 1;
                    }

                    if (!Infos_Else.empty())
                    {
                        npr += Infos_Else.size();
#if 0
                        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(Infos_Else.size(), Infos_Else.size());
                        Eigen::Vector3d BasePos(prcopt.rb[0], prcopt.rb[1], prcopt.rb[2]);
                        Eigen::Vector3d RxPos(gnss_info.pos[0], gnss_info.pos[1], gnss_info.pos[2]);
                        double azeli[2] = { Info_Master.ssat.azel[0],Info_Master.ssat.azel[1] };
                        double dt = Info_Master.Mea_Rover.time - Info_Master.Mea_Base.time;
                        double bl = (BasePos - RxPos).norm();
                        double pr_vari = GNSS_Tools::varerr_sdobs(Info_Master.ssat.sys, azeli[1], Info_Master.Mea_Rover.SNR[f] * SNR_UNIT, Info_Master.Mea_Base.SNR[f] * SNR_UNIT,
                            bl, dt, f, &prcopt, &(Info_Master.Mea_Rover), 1);
                        R.setConstant(pr_vari);
                        for (i = 0;i < Infos_Else.size();i++)
                        {
                            rtklib::GNSS_Info_SD Info_Else = Infos_Else.at(i);

                            R(i, i) += GNSS_Tools::varerr_sdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1], Info_Else.Mea_Rover.SNR[f] * SNR_UNIT, Info_Else.Mea_Base.SNR[f] * SNR_UNIT,
                                bl, dt, f, &prcopt, &(Info_Else.Mea_Rover), 1);

                        }
                        noiseModel::Base::shared_ptr noise = noiseModel::Gaussian::Covariance(R);
                        noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0), noise);

                        GNSSDDPsrFactor::shared_ptr dd_psr_factor(new GNSSDDPsrFactor(X(epoch), Info_Master, Infos_Else, prcopt,
                            f, true, huber));
                        graph.add(dd_psr_factor);

#endif

#if 1
                        gtsam::Vector var(Infos_Else.size());
                        var.setConstant(1.0);
                        noiseModel::Base::shared_ptr noise = noiseModel::Diagonal::Sigmas(var);
                        noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.5), noise);

                        GNSSDDPsrFactor::shared_ptr dd_psr_factor(new GNSSDDPsrFactor(X(epoch), Info_Master, Infos_Else, prcopt,
                            f, false, huber));
                        graph.add(dd_psr_factor);
#endif
                    }
                }
            }
            return npr;
        }

        /***
         *
         * @param ix index used for ambiguity fix
         * @return number of Cp size for fix
         */
         //#define PHASEBIAS_as_WHOLE
#ifdef PHASEBIAS_as_WHOLE
        int addDDCpFactor(MatrixXi & ix)
        {
            int nb, ncp, ns;
            nb = 0;ncp = 0;ns = gnss_info.SD_Infos.size();
            int slip, sat, reset, nf = NF(&fgo.rtk.opt);
            vector<int> sats;
            vector<int> freqs;
            VectorXi udbias = VectorXi(NB(&prcopt)).setZero();

            for (int n = 0; n < MAXSAT; ++n)
            {
                for (int f = 0; f < 3; ++f)
                {
                    fgo.rtk.ssat[n].vsatL[f] = 0;
                    fgo.rtk.ssat[n].vsat[f] = 0;
                    fgo.rtk.ssat[n].vs = 0;
                    fgo.rtk.ssat[n].sys = 0;
                }
                fgo.rtk.ssat[n].azel[0] = fgo.rtk.ssat[n].azel[1] = 0.0;
            }
            for (int k = 0;k < ns;k++)
            {
                sat = gnss_info.SD_Infos[k].Mea_Rover.sat;
                memcpy(fgo.rtk.ssat[sat - 1].vsat, gnss_info.SD_Infos[k].ssat.vsat.data(), sizeof(uint8_t) * 3);
                memcpy(fgo.rtk.ssat[sat - 1].vsatL, gnss_info.SD_Infos[k].ssat.vsatL.data(), sizeof(uint8_t) * 3);
                memcpy(fgo.rtk.ssat[sat - 1].slip, gnss_info.SD_Infos[k].ssat.slip.data(), sizeof(uint8_t) * 3);
                memcpy(fgo.rtk.ssat[sat - 1].azel, gnss_info.SD_Infos[k].ssat.azel.data(), sizeof(double) * 2);
                fgo.rtk.ssat[sat - 1].vs = 1;
                fgo.rtk.ssat[sat - 1].sys = gnss_info.SD_Infos[k].ssat.sys;
            }
            /** update phase-bias **/
            for (int f = 0; f < nf; ++f)
            {
                for (int n = 0; n < MAXSAT; ++n)
                {

                    reset = ++fgo.rtk.ssat[n].outc[f] > (uint32_t)fgo.rtk.opt.maxout;
                    if (reset && prevAmb(MAXSAT * f + n) != 0.0)
                    {
                        fgo.rtk.ssat[n].outc[f] = 0;
                    }
                    if (reset)
                    {
                        fgo.rtk.ssat[n].lock[f] = -fgo.rtk.opt.minlock;
                    }
                    //                if(fgo.rtk.ssat[n].vsat[f]){
                    //                    //                    if (fgo.rtk.ssat[n].lock[f]<0) fgo.rtk.ssat[n].lock[f]=0;
                    //                    fgo.rtk.ssat[n].outc[f]=0;
                    //                }
                    if (fgo.rtk.ssat[n].vs)
                    {
                        if (prevAmb(MAXSAT * f + n) == 0)
                        {
                            fgo.rtk.ssat[n].lock[f] = -fgo.rtk.opt.minlock;
                        }
                        if (!(fgo.rtk.ssat[n].slip[f] & 1) && fgo.rtk.ssat[n].rejc[f] < 2) continue;
                        prevAmb(MAXSAT * f + n) = 0;
                        fgo.rtk.ssat[n].rejc[f] = 0;
                        fgo.rtk.ssat[n].lock[f] = -fgo.rtk.opt.minlock;
                    }

                }
            }




            for (int m = 0; m < 6; m++)
            {
                for (int f = 0; f < 3; f++)
                {
                    int sysi, sysj;
                    int i, j;
                    for (i = -1, j = 0; j < ns; j++)
                    {
                        rtklib::sat_state ssatj = gnss_info.SD_Infos[j].ssat;
                        sysi = ssatj.sys;
                        if (gnss_info.SD_Infos[j].Mea_Rover.L[f] == 0.0 || gnss_info.SD_Infos[j].Mea_Base.L[f] == 0.0
                            || ssatj.azel[1] == 0.0 || ssatj.azel_b[1] == 0.0)
                            continue;
                        if (!ssatj.vsatL[f]) continue;
                        if (!test_sys(sysi, m) || sysi == SYS_SBS) continue;

                        if (satexclude(gnss_info.SD_Infos[j].Mea_Rover.sat, ssatj.ephvar, ssatj.svh, &prcopt)) continue;
                        if (i >= 0 && gnss_info.SD_Infos[j].ssat.slip[f] & LLI_SLIP) continue;
                        if (i < 0 || ssatj.azel[1] >= gnss_info.SD_Infos[i].ssat.azel[1]) i = j;
                    }
                    if (i < 0) continue;

                    rtklib::GNSS_Info_SD Info_Master = gnss_info.SD_Infos[i];

                    vector<rtklib::GNSS_Info_SD> Infos_Else;
                    for (j = 0; j < ns; j++)
                    {
                        if (i == j) continue;  /* skip ref sat */
                        rtklib::sat_state ssatj = gnss_info.SD_Infos[j].ssat;
                        sysj = ssatj.sys;
                        if (gnss_info.SD_Infos[j].Mea_Rover.L[f] == 0.0 || gnss_info.SD_Infos[j].Mea_Base.L[f] == 0.0
                            || ssatj.azel[1] == 0.0 || ssatj.azel_b[1] == 0.0)
                            continue;
                        if (!ssatj.vsatL[f]) continue;
                        if (!test_sys(sysj, m) || sysj == SYS_SBS) continue;
                        if (satexclude(gnss_info.SD_Infos[j].Mea_Rover.sat, ssatj.ephvar, ssatj.svh, &prcopt)) continue;
                        Infos_Else.push_back(gnss_info.SD_Infos[j]);
                        slip = gnss_info.SD_Infos[j].ssat.slip[f];


                        if ((!(slip & LLI_SLIP)))
                        {//&&(delta_amb[MAXSAT*f+gnss_info.SD_Infos[j].Mea_Rover.sat - 1])<1
//                        cout << amb(MAXSAT*f+gnss_info.SD_Infos[j].Mea_Rover.sat-1) <<"  " <<  prevAmb(MAXSAT*f+gnss_info.SD_Infos[j].Mea_Rover.sat-1) << endl;
//                        if (prevAmb(f*MAXSAT+gnss_info.SD_Infos[j].Mea_Rover.sat-1)!=0.0) {
                            sats.push_back(gnss_info.SD_Infos[j].Mea_Rover.sat);
                            freqs.push_back(f);
                            udbias(f * MAXSAT + gnss_info.SD_Infos[j].Mea_Rover.sat - 1) = 1;
                            //                        }
                        }

                        if (!(slip & LLI_HALFC))
                        {
                            if (prcopt.glomodear == GLO_ARMODE_OFF && test_sys(SYS_GLO, m)) continue;
                            ix(0, nb) = MAXSAT * f + Info_Master.Mea_Rover.sat - 1;
                            ix(1, nb) = MAXSAT * f + gnss_info.SD_Infos[j].Mea_Rover.sat - 1;
                            nb++;
                        }
                    }

                    if (!Infos_Else.empty())
                    {
                        ncp += Infos_Else.size();
                        sats.push_back(Info_Master.Mea_Rover.sat);
                        freqs.push_back(f);
                        udbias(f * MAXSAT + Info_Master.Mea_Rover.sat - 1) = 1;
                        gtsam::Vector var(Infos_Else.size());
                        var.setConstant(1.0);
                        noiseModel::Base::shared_ptr noise = noiseModel::Diagonal::Sigmas(var);
                        noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0), noise);
                        GNSSDDCpFactor::shared_ptr dd_cp_factor(new GNSSDDCpFactor(X(epoch), N(epoch), Info_Master, Infos_Else, prcopt,
                            f, false, huber));
                        graph.add(dd_cp_factor);
                    }
                }
            }
            //        if (!sats.empty()&&!freqs.empty()&&!first_init) { //fgo.num_factor[1]!=0
            //            noiseModel::Base::shared_ptr noise = noiseModel::Diagonal::Sigmas(Vector(sats.size()).setConstant(1e-3));
            //            noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0), noise);
            //            GNSSAmbConstraint::shared_ptr amb_constraint_factor(new GNSSAmbConstraint(N(epoch), N(epoch - 1), 0.0, sats, freqs,
            //                                                                huber));
            //            graph.add(amb_constraint_factor);
            //        }
            //        if (ncp>0) {
            Eigen::Map<Vector> amb(gnss_info.amb.data(), gnss_info.amb.size(), 1);
            initialEstimate.insert(N(epoch), (Vector)amb);
            if (first_init)
            {
                graph.add(PriorFactor<Vector>(N(epoch), (Vector)amb,
                    noiseModel::Diagonal::Sigmas(Vector(amb.size()).setConstant(30))));
            }
            else
            {
                for (int f = 0; f < nf; f++)
                {
                    for (int i = 0; i < MAXSAT; i++)
                    {
                        if (!udbias(f * MAXSAT + i))
                        {
                            graph.add(GNSSPhasePriorConstraint(N(epoch), amb(f * MAXSAT + i), i + 1, f,
                                noiseModel::Diagonal::Sigmas(Vector(1).setConstant(30))));
                        }
                    }
                }
            }
            //        ofstream tracefile("/home/wangchuji/catkins_lidar/GLINS/data/test.txt",ios::ate);
            //        tracefile << "gnss_info_ar\n" << amb << endl;
            //        MatrixXd jacobian = graph.linearizeToHessianFactor(initialEstimate)->augmentedJacobian();
            //        tracefile << "jacobian\n" << jacobian << endl;
            //        MatrixXd information = graph.linearizeToHessianFactor(initialEstimate)->augmentedInformation();
            //        tracefile << "information\n" << information << endl;
            //        tracefile.close();
            //        }
            fgo.num_factor[1] = ncp;
            return nb;
        }
#endif

#ifdef PHASEBIAS_as_COMPRESSED
        int addDDCpFactor()
        {
            int ncp, ns;
            ncp = 0;ns = gnss_info.SD_Infos.size();
            int slip, sat, reset, nf = NF(&fgo.rtk.opt);
            vector<int> sats;
            vector<int> freqs;
            VectorXi udbias = VectorXi(NB(&prcopt)).setZero();
            VectorXd compress_ar(NB(&prcopt));
            compress_ar.setZero();
            map<int, int> ar_index;
            double delta_t = gnss_info.weekSec - pre_gpstime[1];

            for (int n = 0; n < MAXSAT; ++n)
            {
                for (int f = 0; f < 3; ++f)
                {
                    fgo.rtk.ssat[n].vsatL[f] = 0;
                    fgo.rtk.ssat[n].vsat[f] = 0;
                    fgo.rtk.ssat[n].vs = 0;
                    fgo.rtk.ssat[n].sys = 0;
                }
                fgo.rtk.ssat[n].azel[0] = fgo.rtk.ssat[n].azel[1] = 0.0;
            }
            for (int k = 0;k < ns;k++)
            {
                sat = gnss_info.SD_Infos[k].Mea_Rover.sat;
                memcpy(fgo.rtk.ssat[sat - 1].vsat, gnss_info.SD_Infos[k].ssat.vsat.data(), sizeof(uint8_t) * 3);
                memcpy(fgo.rtk.ssat[sat - 1].vsatL, gnss_info.SD_Infos[k].ssat.vsatL.data(), sizeof(uint8_t) * 3);
                memcpy(fgo.rtk.ssat[sat - 1].slip, gnss_info.SD_Infos[k].ssat.slip.data(), sizeof(uint8_t) * 3);
                memcpy(fgo.rtk.ssat[sat - 1].azel, gnss_info.SD_Infos[k].ssat.azel.data(), sizeof(double) * 2);
                fgo.rtk.ssat[sat - 1].vs = 1;
                fgo.rtk.ssat[sat - 1].sys = gnss_info.SD_Infos[k].ssat.sys;
            }
            /** update phase-bias **/
            for (int f = 0; f < nf; ++f)
            {
                for (int n = 0; n < MAXSAT; ++n)
                {

                    reset = ++fgo.rtk.ssat[n].outc[f] > (uint32_t)fgo.rtk.opt.maxout;
                    if (fabs(timediff(fgo.rtk.sol.time, gpst2time(2192, 462994))) <= 3)
                    {
                        int week;
                        if (n == 7 && f == 1)
                        {
                            printf("weeksec %lf udbias obs outage %d %d %d %lf\n", time2gpst(fgo.rtk.sol.time, &week), n + 1, f, fgo.rtk.ssat[n].lock[f], gnss_info.amb[MAXSAT * f + n]);
                        }
                    }
                    //                if (reset && last_ar_index.count(MAXSAT * f + n + 1) != 0) {
                    //                    prevAmb(last_ar_index.find(MAXSAT * f + n + 1)->second)=0;
                    //                    fgo.rtk.ssat[n].outc[f] = 0;
                    //                }
                    if (reset && AmbFullArray[MAXSAT * f + n] != 0.0)
                    {
                        if (last_ar_index.count(MAXSAT * f + n + 1) != 0)
                        {
                            prevAmb(last_ar_index.find(MAXSAT * f + n + 1)->second) = 0;
                        }
                        AmbFullArray[MAXSAT * f + n] = 0.0;
                        fgo.rtk.ssat[n].outc[f] = 0;
                    }
                    if (reset)
                    {
                        fgo.rtk.ssat[n].lock[f] = -fgo.rtk.opt.minlock;
                        if (fabs(timediff(fgo.rtk.sol.time, gpst2time(2192, 462990))) <= 0.01)
                        {
                            printf("reset1 %d %d %d\n", n + 1, f, fgo.rtk.ssat[n].outc[f]);
                        }
                    }
                    //                if (fgo.rtk.ssat[n].vs){
                    //                    if (!(fgo.rtk.ssat[n].slip[f]&1)&&fgo.rtk.ssat[n].rejc[f]<2) continue;
                    //                    if (last_ar_index.count(MAXSAT * f + n + 1) != 0) {
                    //                        prevAmb(last_ar_index.find(MAXSAT * f + n + 1)->second) = 0;
                    //                    }
                    //                    AmbFullArray[MAXSAT * f + n] = 0.0;
                    //                    fgo.rtk.ssat[n].rejc[f]=0;
                    //                    fgo.rtk.ssat[n].lock[f]=-fgo.rtk.opt.minlock;
                    //                    if (fabs(timediff(fgo.rtk.sol.time, gpst2time(2192,462979)))<=0.01)
                    //                    {
                    //                        printf("%d %d %d\n",n+1,f,fgo.rtk.ssat[n].rejc[f]);
                    //                    }
                    //                }

                }
                for (int n = 0; n < MAXSAT; ++n)
                {
                    if (fgo.rtk.ssat[n].vs)
                    {
                        if (fabs(timediff(fgo.rtk.sol.time, gpst2time(2192, 462990))) <= 0.01)
                        {
                            printf("%d %d rejc: %d\n", n + 1, f, fgo.rtk.ssat[n].rejc[f]);
                        }
                        if (!(fgo.rtk.ssat[n].slip[f] & 1) && fgo.rtk.ssat[n].rejc[f] < 2) continue;
                        if (last_ar_index.count(MAXSAT * f + n + 1) != 0)
                        {
                            prevAmb(last_ar_index.find(MAXSAT * f + n + 1)->second) = 0;
                        }
                        AmbFullArray[MAXSAT * f + n] = 0.0;
                        fgo.rtk.ssat[n].rejc[f] = 0;
                        fgo.rtk.ssat[n].lock[f] = -fgo.rtk.opt.minlock;
                        if (fabs(timediff(fgo.rtk.sol.time, gpst2time(2192, 462990))) <= 0.01)
                        {
                            printf("%d %d %d\n", n + 1, f, fgo.rtk.ssat[n].rejc[f]);
                        }
                    }
                }

                for (int n = 0; n < MAXSAT; ++n)
                {
                    if (fgo.rtk.ssat[n].vs)
                    {
                        if (gnss_info.amb[MAXSAT * f + n] == 0.0 || AmbFullArray[MAXSAT * f + n] != 0.0) continue;
                        //                    if(last_ar_index.count(MAXSAT * f + n + 1)!=0 && prevAmb(last_ar_index.find(MAXSAT * f + n + 1)->second) != 0.0) continue;
                        fgo.rtk.ssat[n].lock[f] = -fgo.rtk.opt.minlock;
                        if (fabs(timediff(fgo.rtk.sol.time, gpst2time(2192, 462990))) <= 0.01)
                        {
                            printf("reset2 %d %d %d\n", n + 1, f, fgo.rtk.ssat[n].rejc[f]);
                        }
                    }
                }

            }

            for (int m = 0; m < 6; m++)
            {
                for (int f = 0; f < 3; f++)
                {
                    int sysi, sysj;
                    int sati, satj;
                    int i, j;
                    for (i = -1, j = 0; j < ns; j++)
                    {
                        rtklib::sat_state ssatj = gnss_info.SD_Infos[j].ssat;
                        sysi = ssatj.sys;
                        satj = gnss_info.SD_Infos[j].Mea_Rover.sat;
                        if (gnss_info.SD_Infos[j].Mea_Rover.L[f] == 0.0 || gnss_info.SD_Infos[j].Mea_Base.L[f] == 0.0
                            || ssatj.azel[1] == 0.0 || ssatj.azel_b[1] == 0.0)
                            continue;
                        //                    if (!ssatj.vsatL[f]) continue;
                        if (!test_sys(sysi, m) || sysi == SYS_SBS) continue;
                        if (satexclude(satj, ssatj.ephvar, ssatj.svh, &prcopt)) continue;
                        if (i >= 0 && gnss_info.SD_Infos[j].ssat.slip[f] & LLI_SLIP) continue;
                        if (i < 0 || ssatj.azel[1] >= gnss_info.SD_Infos[i].ssat.azel[1]) i = j;
                    }
                    if (i < 0) continue;

                    rtklib::GNSS_Info_SD Info_Master = gnss_info.SD_Infos[i];

                    vector<rtklib::GNSS_Info_SD> Infos_Else;

                    map<int, int> part_ar_index;
                    for (j = 0; j < ns; j++)
                    {
                        if (i == j) continue;  /* skip ref sat */
                        rtklib::sat_state ssatj = gnss_info.SD_Infos[j].ssat;
                        sysj = ssatj.sys;
                        satj = gnss_info.SD_Infos[j].Mea_Rover.sat;
                        if (gnss_info.SD_Infos[j].Mea_Rover.L[f] == 0.0 || gnss_info.SD_Infos[j].Mea_Base.L[f] == 0.0
                            || ssatj.azel[1] == 0.0 || ssatj.azel_b[1] == 0.0)
                        {
                            continue;
                        }
                        if (!test_sys(sysj, m) || sysj == SYS_SBS) continue;
                        if (satexclude(satj, ssatj.ephvar, ssatj.svh, &prcopt)) continue;
                        if (!ssatj.vsatL[f])
                        {
                            continue;
                        }
                        Infos_Else.push_back(gnss_info.SD_Infos[j]);
                        slip = gnss_info.SD_Infos[j].ssat.slip[f];
                        part_ar_index[f * MAXSAT + satj] = ncp;
                        compress_ar[ncp] = gnss_info.amb[f * MAXSAT + satj - 1];
                        ncp++;

                        if (!first_init)
                        {
                            ///&&
                            if ((!(slip & LLI_SLIP)) && last_ar_index.count(f * MAXSAT + satj) != 0)
                            {
                                if (prevAmb(last_ar_index.find(MAXSAT * f + satj)->second) != 0)
                                {
                                    sats.push_back(gnss_info.SD_Infos[j].Mea_Rover.sat);
                                    freqs.push_back(f);
                                    udbias(f * MAXSAT + gnss_info.SD_Infos[j].Mea_Rover.sat - 1) = 1;
                                    if (fabs(timediff(fgo.rtk.sol.time, gpst2time(2192, 462993))) <= 0.01)
                                    {
                                        printf("continue: %d %d\n", satj, f);
                                    }
                                }
                                else
                                {
                                    graph.add(GNSSPhasePriorConstraintCompress(N(epoch), compress_ar[ncp - 1], ncp - 1,
                                        noiseModel::Diagonal::Sigmas(Vector(1).setConstant(30))));
                                    if (fabs(timediff(fgo.rtk.sol.time, gpst2time(2192, 462993))) <= 0.01)
                                    {
                                        printf("initialize: %d %d\n", satj, f);
                                    }
                                }
                            }
                            else
                            {
                                graph.add(GNSSPhasePriorConstraintCompress(N(epoch), compress_ar[ncp - 1], ncp - 1,
                                    noiseModel::Diagonal::Sigmas(Vector(1).setConstant(30))));
                                if (fabs(timediff(fgo.rtk.sol.time, gpst2time(2192, 462993))) <= 0.01)
                                {
                                    printf("initialize: %d %d\n", satj, f);
                                }
                            }
                        }
                    }

                    if (!Infos_Else.empty())
                    {
                        part_ar_index[f * MAXSAT + Info_Master.Mea_Rover.sat] = ncp;
                        compress_ar[ncp] = gnss_info.amb[f * MAXSAT + Info_Master.Mea_Rover.sat - 1];
                        ncp++;
                        if (!first_init)
                        {
                            if (last_ar_index.count(f * MAXSAT + Info_Master.Mea_Rover.sat) != 0)
                            {
                                if (prevAmb(last_ar_index.find(f * MAXSAT + Info_Master.Mea_Rover.sat)->second) != 0)
                                {
                                    sats.push_back(Info_Master.Mea_Rover.sat);
                                    freqs.push_back(f);
                                    udbias(f * MAXSAT + Info_Master.Mea_Rover.sat - 1) = 1;
                                    if (fabs(timediff(fgo.rtk.sol.time, gpst2time(2192, 462993))) <= 0.01)
                                    {
                                        printf("continue: %d %d\n", Info_Master.Mea_Rover.sat, f);
                                    }
                                }
                                else
                                {
                                    graph.add(GNSSPhasePriorConstraintCompress(N(epoch), compress_ar[ncp - 1], ncp - 1,
                                        noiseModel::Diagonal::Sigmas(Vector(1).setConstant(30))));
                                    if (fabs(timediff(fgo.rtk.sol.time, gpst2time(2192, 462993))) <= 0.01)
                                    {
                                        printf("initialize: %d %d\n", Info_Master.Mea_Rover.sat, f);
                                    }
                                }
                            }
                            else
                            {
                                graph.add(GNSSPhasePriorConstraintCompress(N(epoch), compress_ar[ncp - 1], ncp - 1,
                                    noiseModel::Diagonal::Sigmas(
                                        Vector(1).setConstant(30))));
                                if (fabs(timediff(fgo.rtk.sol.time, gpst2time(2192, 462993))) <= 0.01)
                                {
                                    printf("initialize: %d %d\n", Info_Master.Mea_Rover.sat, f);
                                }
                            }
                        }
                        gtsam::Vector var(Infos_Else.size());
                        var.setConstant(1.0);
                        noiseModel::Base::shared_ptr noise = noiseModel::Diagonal::Sigmas(var);
                        noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.5), noise);
                        GNSSDDCpFactor::shared_ptr dd_cp_factor(new GNSSDDCpFactorCompress(X(epoch), N(epoch), Info_Master, Infos_Else, part_ar_index, prcopt,
                            f, false, huber));
                        graph.add(dd_cp_factor);
                        ar_index.insert(part_ar_index.begin(), part_ar_index.end());
                    }
                }
            }
            if (!sats.empty() && !freqs.empty() && !first_init)
            { //fgo.num_factor[1]!=0
                noiseModel::Base::shared_ptr noise = noiseModel::Diagonal::Sigmas(Vector(sats.size()).setConstant(1e-3 * sqrt(delta_t)));
                noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.5), noise);
                GNSSAmbConstraint::shared_ptr amb_constraint_factor(new GNSSAmbConstraintCompress(N(epoch), N(epoch - 1), 0.0, sats, freqs, ar_index, last_ar_index,
                    noise));
                graph.add(amb_constraint_factor);

            }
            compress_ar.conservativeResize(ncp);
            if (ncp > 0)
            {
                initialEstimate.insert(N(epoch), compress_ar);
            }
            //        noiseModel::Diagonal::Sigmas(Vector(compress_ar.size()).setConstant(30)
            if (first_init)
            {
                gtsam::Matrix priorNoiseMatrix = gtsam::Matrix::Identity(compress_ar.size(), compress_ar.size()) * 900;
                graph.add(PriorFactor<Vector>(N(epoch), (Vector)compress_ar,
                    noiseModel::Gaussian::Covariance(priorNoiseMatrix)));
            }
            //        Values tempInitial;
            //        Point3 pos(gnss_info.pos[0],gnss_info.pos[1],gnss_info.pos[2]);
            //        tempInitial.insert(N(epoch),compress_ar);
            //        if (!first_init) {
            //            tempInitial.insert(N(epoch - 1), prevAmb);
            //        }
            //        tempInitial.insert(X(epoch),pos);
            //        ofstream tracefile("/home/wangchuji/catkins_lidar/GLINS/data/test3.txt",ios::ate);
            //        tracefile << "compress_ar\n" << compress_ar << endl;
            //
            //        Eigen::Map<VectorXd> amb(gnss_info.amb.data(),gnss_info.amb.size(), 1);
            //        tracefile << "gnss_info_ar\n" << amb << endl;
            //        MatrixXd jacobian = graph.linearizeToHessianFactor(tempInitial)->augmentedJacobian();
            //        tracefile << "jacobian\n" << jacobian << endl;
            //        MatrixXd information = graph.linearizeToHessianFactor(tempInitial)->augmentedInformation();
            //        tracefile << "information\n" << information << endl;
            //        tracefile.close();
            last_ar_index = ar_index;
            fgo.num_factor[1] = ncp;
            return ncp;
        }
#endif

#ifdef PHASEBIAS_as_SINGLE
        int addDDCpFactor(MatrixXi & ix)
        {
            int nb, ncp, ns;
            nb = 0;ncp = 0;ns = gnss_info.SD_Infos.size();
            int slip, sat, reset, nf = NF(&fgo.rtk.opt);
            prn.clear();

            for (int n = 0; n < MAXSAT; ++n)
            {
                for (int f = 0; f < 3; ++f)
                {
                    fgo.rtk.ssat[n].vsatL[f] = 0;
                    fgo.rtk.ssat[n].vsat[f] = 0;
                    fgo.rtk.ssat[n].vs = 0;
                    fgo.rtk.ssat[n].sys = 0;
                }
                fgo.rtk.ssat[n].azel[0] = fgo.rtk.ssat[n].azel[1] = 0.0;
            }
            for (int k = 0;k < ns;k++)
            {
                sat = gnss_info.SD_Infos[k].Mea_Rover.sat;
                memcpy(fgo.rtk.ssat[sat - 1].vsat, gnss_info.SD_Infos[k].ssat.vsat.data(), sizeof(uint8_t) * 3);
                memcpy(fgo.rtk.ssat[sat - 1].vsatL, gnss_info.SD_Infos[k].ssat.vsatL.data(), sizeof(uint8_t) * 3);
                memcpy(fgo.rtk.ssat[sat - 1].slip, gnss_info.SD_Infos[k].ssat.slip.data(), sizeof(uint8_t) * 3);
                memcpy(fgo.rtk.ssat[sat - 1].azel, gnss_info.SD_Infos[k].ssat.azel.data(), sizeof(double) * 2);
                fgo.rtk.ssat[sat - 1].vs = 1;
                fgo.rtk.ssat[sat - 1].sys = gnss_info.SD_Infos[k].ssat.sys;
            }
            /** update phase-bias **/
            for (int f = 0; f < nf; ++f)
            {
                for (int n = 0; n < MAXSAT; ++n)
                {

                    reset = ++fgo.rtk.ssat[n].outc[f] > (uint32_t)fgo.rtk.opt.maxout;
                    if (reset && prevAmb(MAXSAT * f + n) != 0.0)
                    {
                        fgo.rtk.ssat[n].outc[f] = 0;
                    }
                    if (reset)
                    {
                        fgo.rtk.ssat[n].lock[f] = -fgo.rtk.opt.minlock;
                    }
                    //                if(fgo.rtk.ssat[n].vsat[f]){
                    //                    //                    if (fgo.rtk.ssat[n].lock[f]<0) fgo.rtk.ssat[n].lock[f]=0;
                    //                    fgo.rtk.ssat[n].outc[f]=0;
                    //                }
                    if (fgo.rtk.ssat[n].vs)
                    {
                        if (prevAmb(MAXSAT * f + n) == 0)
                        {
                            fgo.rtk.ssat[n].lock[f] = -fgo.rtk.opt.minlock;
                        }
                        if (!(fgo.rtk.ssat[n].slip[f] & 1) && fgo.rtk.ssat[n].rejc[f] < 2) continue;
                        //                    prevAmb(MAXSAT*f+n)=0;
                        fgo.rtk.ssat[n].rejc[f] = 0;
                        fgo.rtk.ssat[n].lock[f] = -fgo.rtk.opt.minlock;
                    }

                }
            }




            for (int m = 0; m < 6; m++)
            {
                for (int f = 0; f < 3; f++)
                {
                    int sysi, sysj;
                    int sati, satj;
                    int i, j;
                    for (i = -1, j = 0; j < ns; j++)
                    {
                        rtklib::sat_state ssatj = gnss_info.SD_Infos[j].ssat;
                        sysj = ssatj.sys;
                        satj = gnss_info.SD_Infos[j].Mea_Rover.sat;
                        if (gnss_info.SD_Infos[j].Mea_Rover.L[f] == 0.0 || gnss_info.SD_Infos[j].Mea_Base.L[f] == 0.0
                            || ssatj.azel[1] == 0.0 || ssatj.azel_b[1] == 0.0)
                            continue;
                        if (!ssatj.vsatL[f]) continue;
                        if (!test_sys(sysj, m) || sysj == SYS_SBS) continue;

                        if (satexclude(satj, ssatj.ephvar, ssatj.svh, &prcopt)) continue;
                        if (i >= 0 && gnss_info.SD_Infos[j].ssat.slip[f] & LLI_SLIP) continue;
                        if (i < 0 || ssatj.azel[1] >= gnss_info.SD_Infos[i].ssat.azel[1]) i = j;
                    }
                    if (i < 0) continue;

                    rtklib::GNSS_Info_SD Info_Master = gnss_info.SD_Infos[i];
                    int nv = 0;

                    for (j = 0; j < ns; j++)
                    {
                        if (i == j) continue;  /* skip ref sat */

                        rtklib::sat_state ssatj = gnss_info.SD_Infos[j].ssat;
                        sysj = ssatj.sys;
                        sati = Info_Master.Mea_Rover.sat;
                        satj = gnss_info.SD_Infos[j].Mea_Rover.sat;

                        if (gnss_info.SD_Infos[j].Mea_Rover.L[f] == 0.0 || gnss_info.SD_Infos[j].Mea_Base.L[f] == 0.0
                            || ssatj.azel[1] == 0.0 || ssatj.azel_b[1] == 0.0)
                            continue;
                        if (!ssatj.vsatL[f]) continue;
                        if (!test_sys(sysj, m) || sysj == SYS_SBS) continue;
                        if (satexclude(satj, ssatj.ephvar, ssatj.svh, &prcopt)) continue;
                        slip = gnss_info.SD_Infos[j].ssat.slip[f];

                        noiseModel::Base::shared_ptr noise = noiseModel::Diagonal::Sigmas(Vector(1).setConstant(1.0));
                        noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0), noise);

                        GNSSDDCpFactorSingle::shared_ptr dd_cp_factor_single(new GNSSDDCpFactorSingle(X(epoch), N(phase_count(f * MAXSAT + sati - 1) + epoch), N(phase_count(f * MAXSAT + satj - 1) + epoch), Info_Master, gnss_info.SD_Infos[j], prcopt,
                            f, false, huber));
                        graph.add(dd_cp_factor_single);

                        initialEstimate.insert(N(phase_count(f * MAXSAT + satj - 1) + epoch), gnss_info.amb[f * MAXSAT + satj - 1]);

                        prn.push_back(f * MAXSAT + satj - 1);

                        if ((!(slip & LLI_SLIP)) && !first_init && prevAmb(f * MAXSAT + satj - 1) != 0.0)
                        {

                            noiseModel::Base::shared_ptr processnoise = noiseModel::Diagonal::Sigmas(Vector(1).setConstant(1e-3));
                            noiseModel::Base::shared_ptr processhuber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0), processnoise);
                            GNSSAmbConstraintSingle::shared_ptr amb_constraint_factor_single(new GNSSAmbConstraintSingle(N(phase_count(f * MAXSAT + satj - 1) + epoch), N(phase_count(f * MAXSAT + satj - 1) + epoch - 1), 0.0, processnoise));
                            graph.add(amb_constraint_factor_single);

                            if (prcopt.glomodear == GLO_ARMODE_OFF && test_sys(SYS_GLO, m)) continue;
                            ix(0, nb) = MAXSAT * f + Info_Master.Mea_Rover.sat - 1;
                            ix(1, nb) = MAXSAT * f + gnss_info.SD_Infos[j].Mea_Rover.sat - 1;
                            nb++;
                        }
                        else
                        {
                            graph.add(GNSSPhasePriorConstraintSingle(N(phase_count(f * MAXSAT + satj - 1) + epoch), gnss_info.amb[f * MAXSAT + satj - 1], satj, f,
                                noiseModel::Diagonal::Sigmas(Vector(1).setConstant(30))));
                        }

                        nv++;
                    }

                    if (nv != 0.0)
                    {
                        prn.push_back(f * MAXSAT + sati - 1);
                        initialEstimate.insert(N(phase_count(f * MAXSAT + sati - 1) + epoch), gnss_info.amb[f * MAXSAT + sati - 1]);
                        if (!first_init && prevAmb(f * MAXSAT + sati - 1) != 0.0)
                        {
                            noiseModel::Base::shared_ptr processnoise = noiseModel::Diagonal::Sigmas(Vector(1).setConstant(1e-3));
                            noiseModel::Base::shared_ptr processhuber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0), processnoise);
                            GNSSAmbConstraintSingle::shared_ptr amb_constraint_factor_single(new GNSSAmbConstraintSingle(N(phase_count(f * MAXSAT + sati - 1) + epoch), N(phase_count(f * MAXSAT + sati - 1) + epoch - 1), 0.0, processnoise));
                            graph.add(amb_constraint_factor_single);
                        }
                        else
                        {
                            graph.add(GNSSPhasePriorConstraintSingle(N(phase_count(f * MAXSAT + sati - 1) + epoch), gnss_info.amb[f * MAXSAT + sati - 1], sati, f,
                                noiseModel::Diagonal::Sigmas(Vector(1).setConstant(30))));
                        }
                        ncp += nv;
                    }
                }
            }

            fgo.num_factor[1] = ncp;
            return nb;
        }
#endif
        int addSDDopFactor()
        {
            int validSat[MAXSAT] = { 0 };
            int ns = gnss_info.ZD_Infos.size();
            int ndop = 0;
            double delta_t = gnss_info.weekSec - pre_gpstime[1];
            //        if (delta_t>1.0) return;
            for (int m = 0;m < 6;m++)
            {
                for (int f = 0; f < 3; f++)
                {
                    int i, j;
                    for (i = -1, j = 0; j < ns; j++)
                    {
                        rtklib::sat_state ssatj = gnss_info.ZD_Infos[j].ssat;
                        int sys = gnss_info.ZD_Infos[j].ssat.sys;
                        if (!test_sys(sys, m) || sys == SYS_SBS) continue;
                        if (gnss_info.ZD_Infos[j].Mea.D[f] == 0.0 || ssatj.azel[1] == 0.0) continue;
                        //                    if (!ssatj.vsatD[f]) continue;
                        if (ssatj.azel[1] < prcopt.elmin) continue;
                        if (ssatj.lam[f] == 0.0) continue;
                        if (satexclude(gnss_info.ZD_Infos[j].Mea.sat, ssatj.ephvar, ssatj.svh, &prcopt)) continue;
                        if (i < 0 || ssatj.azel[1] >= gnss_info.ZD_Infos[i].ssat.azel[1]) i = j;
                    }
                    if (i < 0) continue;
                    rtklib::GNSS_Info_ZD Info_Master = gnss_info.ZD_Infos[i];
                    std::vector<rtklib::GNSS_Info_ZD> Infos_Else;
                    for (j = 0; j < ns; j++)
                    {
                        if (i == j) continue;  /* skip ref sat */
                        rtklib::sat_state ssatj = gnss_info.ZD_Infos[j].ssat;
                        int sys = ssatj.sys;
                        int sat = gnss_info.SD_Infos[j].Mea_Rover.sat;
                        if (!test_sys(sys, m) || sys == SYS_SBS) continue;
                        if (gnss_info.ZD_Infos[j].Mea.D[f] == 0.0 || ssatj.azel[1] == 0.0) continue;
                        //                        if (gnss_info.stat != SOLQ_DGPS)
                        if (ssatj.lam[f] == 0.0) continue;
                        if (ssatj.azel[1] < prcopt.elmin) continue;
                        if (satexclude(gnss_info.ZD_Infos[j].Mea.sat, ssatj.ephvar, ssatj.svh, &prcopt)) continue;
                        if (!ssatj.vsatD[f])
                        {
                            continue;
                        }
                        Infos_Else.push_back(gnss_info.ZD_Infos[j]);
                        if (f == 0)
                            validSat[gnss_info.ZD_Infos[j].Mea.sat - 1] = 1;
                    }

                    if (!Infos_Else.empty())
                    {
                        ndop += Infos_Else.size();
#if 0
                        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(Infos_Else.size(), Infos_Else.size());

                        R.setConstant(GNSS_Tools::varerr_zdobs(Info_Master.ssat.sys, Info_Master.ssat.azel[1], Info_Master.Mea.SNR[f] * SNR_UNIT,
                            f, &prcopt, &(Info_Master.Mea), 2));
                        for (int i = 0;i < Infos_Else.size();i++)
                        {
                            rtklib::GNSS_Info_ZD Info_Else = Infos_Else.at(i);

                            R(i, i) += GNSS_Tools::varerr_zdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1], Info_Else.Mea.SNR[f] * SNR_UNIT,
                                f, &prcopt, &(Info_Else.Mea), 2);
                        }

                        noiseModel::Base::shared_ptr noise = noiseModel::Gaussian::Covariance(R);
                        noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0), noise);

                        GNSSSDDopFactor::shared_ptr sd_dop_factor(new GNSSSDDopFactor(X(epoch - 1), X(epoch), Info_Master, Infos_Else, prcopt,
                            f, delta_t, true, huber));

                        graph.add(sd_dop_factor);


#endif

#if 1
                        gtsam::Vector var(Infos_Else.size());
                        var.setConstant(1.0);
                        noiseModel::Base::shared_ptr noise = noiseModel::Diagonal::Sigmas(var);
                        noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(
                            noiseModel::mEstimator::Huber::Create(1.0), noise);

                        auto* sd_dop_factor = new GNSSSDDopFactor(X(epoch - 1), X(epoch), Info_Master, Infos_Else, prcopt,
                            f, delta_t, false, huber);
                        //                    MatrixXd H1,H2;
                        //                    cout << sd_dop_factor->evaluateError(latestValue,initialEstimate.at<Point3>(X(epoch)),H1,H2) << endl;
                        //                    cout  << H1 << endl;
                        //                    cout << H2 << endl;

                        graph.add(*sd_dop_factor);
#endif
                    }
                }
            }
            return ndop;
        }

        void updatePath()
        {
            Point3 FGOENU;
            Point3 FGOECEF;
            geometry_msgs::PoseStamped poseStamped;
            fgo_path.poses.clear();
            fgo_path.header.frame_id = "map";
            //        cout << "update path "<< endl;
            for (int i = 0; i <= epoch; ++i)
            {
                //            cout << "update path "<< Symbol(X(i))<< endl;
                FGOECEF = currentEstimate.at<Point3>(X(i));
                FGOENU = gnssTools.ecef2enu(ENU_ref, FGOECEF);
                //            LOG(INFO) << "FGOENU -> " << FGOENU.transpose();
                poseStamped.header.stamp = ros::Time().fromSec(time_map[i]);
                poseStamped.header.frame_id = "map";
                poseStamped.pose.position.x = FGOENU(0);
                poseStamped.pose.position.y = FGOENU(1);
                poseStamped.pose.position.z = FGOENU(2);
                fgo_path.poses.push_back(poseStamped);
            }
            fgo_reltime_path.header.frame_id = "map";
            fgo_reltime_path.poses.push_back(poseStamped);
            Point3 RTKENU;
            Point3 RTKECEF;
            wls_path.header.frame_id = "map";
            RTKECEF << gnss_info.pos[0], gnss_info.pos[1], gnss_info.pos[2];
            RTKENU = gnssTools.ecef2enu(ENU_ref, RTKECEF);
            geometry_msgs::PoseStamped rtkpose;
            rtkpose.header.stamp = ros::Time().fromSec(time_map[epoch]);
            rtkpose.header.frame_id = "map";
            rtkpose.pose.position.x = RTKENU(0);
            rtkpose.pose.position.y = RTKENU(1);
            rtkpose.pose.position.z = RTKENU(2);
            wls_path.poses.push_back(rtkpose);
            if (pub_WLSENU.getNumSubscribers() != 0) pub_WLSENU.publish(wls_path);
            if (pub_FGOENU.getNumSubscribers() != 0) pub_FGOENU.publish(fgo_path);
            if (pub_predict_ENU.getNumSubscribers() != 0) pub_predict_ENU.publish(fgo_reltime_path);
        }

        void logResult(std::string logPath, Point3 latestValue, double week, double weeksec, int state, int nv)
        {
            std::ofstream foutC(logPath, std::ios::app);
            foutC.setf(std::ios::fixed, std::ios::floatfield);

            Eigen::Matrix<double, 3, 1> llh = gnssTools.ecef2llh(latestValue);
            /* gps time */
            foutC.precision(0);
            foutC << week << "  ";
            foutC.precision(3);
            foutC << weeksec << "  ";

            /* longitude, latitude and altitude */
            foutC.precision(4);
            foutC << latestValue(0) << "  ";
            foutC << latestValue(1) << "  ";
            foutC << latestValue(2) << "  ";
            foutC << state << "  ";
            foutC << nv << "  ";
            foutC << "0.0 0.0 0.0 0.0 0.0 0.0 0.0" << "  ";
            foutC.precision(1);
            foutC << fgo.rtk.sol.ratio << "  " << std::endl;

            //        foutC<<llh(1)<<"  ";
            //        foutC<<llh(0)<<"  ";
            //        foutC<<llh(2)<<"  "<< 6 << std::endl;
            foutC.close();

            //        std::ofstream foutC(logPath, std::ios::ate);
            //        foutC.setf(std::ios::fixed, std::ios::floatfield);
            //        Point3 pos;
            //        for (int i = 0; i <= epoch; ++i) {
            //            pos = currentEstimate.at<Point3>(X(i));
            //            int Week;
            //            double Sec;
            //            gtime_t gtime;
            //            gtime.time = (int)time_map[i];
            //            gtime.sec = time_map[i]-(int)time_map[i];
            //            Sec = time2gpst(gtime,&Week);
            //            foutC.precision(0);
            //            foutC<<Week<<"  ";
            //            foutC.precision(3);
            //            foutC<<Sec <<"  ";
            //
            //            /* longitude, latitude and altitude */
            //            foutC.precision(4);
            //            foutC<<pos(0)<<"  ";
            //            foutC<<pos(1)<<"  ";
            //            foutC<<pos(2)<<"  "<< 6 << std::endl;
            //
            //        }
            //        foutC.close();
        }

        void gnssInfoHandler(const rtklib::GNSS_InfoConstPtr & gnss_msg)
        {
            std::cout << "receive gnss obs : " << gnss_msg->gpsWeek << " " << gnss_msg->weekSec << std::endl;
            gnss_queue.push_back(*gnss_msg);
        }
};
