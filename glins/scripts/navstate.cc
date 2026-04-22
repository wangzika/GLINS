#include "process/navstate.h"
#include "navconfig.hpp"
#include "navearth.hpp"

using namespace utiltool;
using namespace constant;

namespace mscnav
{

State::Ptr State::state_(new State());

State::Ptr State::GetState()
{
    return state_;
}
/**
 * @brief  初始化
 * @note   
 * @retval 
 */
bool State::InitializeState()
{
    config_ = ConfigInfo::GetInstance();

    /* 数据相关内容 */
    bool gnss_log_enable = (config_->get<int>("gnsslog_enable") != 0);

    gnss_data_ = std::make_shared<FileGnssData>(gnss_log_enable);
    if (!gnss_data_->StartReadGnssData())
    {
        navexit();
    }

    imu_data_ = std::make_shared<FileImuData>();
    if (!imu_data_->StartReadData())
    {
        navexit();
    }

    data_queue_ = std::make_shared<DataQueue>(gnss_data_, imu_data_);


    /*计算相关内容 */
    bool filter_debug_log = (config_->get<int>("filter_debug_log_enable") != 0);
    filter_ = std::make_shared<KalmanFilter>(filter_debug_log);
    initialize_nav_ = std::make_shared<InitializedNav>(data_queue_);
    gps_process_ = std::make_shared<GpsProcess>(filter_);

    Eigen::VectorXd initial_Pvariance;
    if (initialize_nav_->StartAligning(nav_info_)) //获取初始的状态
    {
        std::cout << "Initialize the navigation information successed" << std::endl;
    }
    else
    {
        std::cerr << "Initialize the navigation information failed" << std::endl;
        return false;
    }

    initialize_nav_->SetStateIndex(filter_->GetStateIndex());                                                        //设置状态量对应的索引
    initial_Pvariance = initialize_nav_->SetInitialVariance(initial_Pvariance, nav_info_, filter_->GetStateIndex()); //设置初始方差信息
    filter_->InitialStateCov(initial_Pvariance);
    std::cout << "Initialized state successed" << std::endl;
    latest_update_time_ = nav_info_.time_;

    /**赋值小q 一定要注意单位 */
    const StateIndex &index = filter_->GetStateIndex();
    Eigen::VectorXd state_q_tmp = Eigen::VectorXd::Zero(filter_->GetStateSize());

    std::vector<double> tmp_value = config_->get_array<double>("position_random_walk");
    state_q_tmp.segment<3>(index.pos_index_)
        << tmp_value.at(0),
        tmp_value.at(1),
        tmp_value.at(2);

    tmp_value = config_->get_array<double>("velocity_random_walk");
    state_q_tmp.segment<3>(index.vel_index_)
        << tmp_value.at(0) / 60.0,
        tmp_value.at(1) / 60.0,
        tmp_value.at(2) / 60.0;

    tmp_value = config_->get_array<double>("attitude_random_walk");
    state_q_tmp.segment<3>(index.att_index_)
        << tmp_value.at(0) * deg2rad / 60.0,
        tmp_value.at(1) * deg2rad / 60.0,
        tmp_value.at(2) * deg2rad / 60.0;

    tmp_value = config_->get_array<double>("gyro_bias_std");
    state_q_tmp.segment<3>(index.gyro_bias_index_)
        << tmp_value.at(0) * dh2rs,
        tmp_value.at(1) * dh2rs,
        tmp_value.at(2) * dh2rs;

    tmp_value = config_->get_array<double>("acce_bias_std");
    state_q_tmp.segment<3>(index.acce_bias_index_)
        << tmp_value.at(0) * constant_mGal,
        tmp_value.at(1) * constant_mGal,
        tmp_value.at(2) * constant_mGal;

    if (config_->get<int>("evaluate_imu_scale") != 0)
    {
        tmp_value = config_->get_array<double>("gyro_scale_std");
        state_q_tmp.segment<3>(index.gyro_scale_index_)
            << tmp_value.at(0) * constant_ppm,
            tmp_value.at(1) * constant_ppm,
            tmp_value.at(2) * constant_ppm;

        tmp_value = config_->get_array<double>("acce_scale_std");
        state_q_tmp.segment<3>(index.acce_scale_index_)
            << tmp_value.at(0) * constant_ppm,
            tmp_value.at(1) * constant_ppm,
            tmp_value.at(2) * constant_ppm;
    }
//    std::cout << " state_q_tmp: " << std::endl;
//    std::cout <<  state_q_tmp << std::endl;
    state_q_tmp = state_q_tmp.array().pow(2);
//    std::cout << " state_q_tmp: " << std::endl;
//    std::cout <<  state_q_tmp << std::endl;
    double gbtime = config_->get<double>("corr_time_of_gyro_bias") * constant_hour;
    double abtime = config_->get<double>("corr_time_of_acce_bias") * constant_hour;


    state_q_tmp.segment<3>(index.gyro_bias_index_) *= (2.0 / gbtime);
    state_q_tmp.segment<3>(index.acce_bias_index_) *= (2.0 / abtime);
//    std::cout << " state_q_tmp: " << std::endl;
//    std::cout <<  state_q_tmp << std::endl;
    if (config_->get<double>("evaluate_imu_scale") != 0)
    {
        double gstime = config_->get<double>("corr_time_of_gyro_scale") * constant_hour;
        double astime = config_->get<double>("corr_time_of_acce_scale") * constant_hour;
        state_q_tmp.segment<3>(index.gyro_scale_index_) *= (2.0 / gstime);
        state_q_tmp.segment<3>(index.acce_scale_index_) *= (2.0 / astime);
    }
    state_q_ = state_q_tmp.asDiagonal();
//    std::cout << " state_q_: " << std::endl;
//    std::cout <<  state_q_ << std::endl;
    std::string output_file_path = config_->get<std::string>("result_output_path");
//    output_file_path.append("/navinfo.txt");
    auto output_file_name = config_.get()->output_file_name;  // 从命令行中读取output_file_name
    output_file_path.append(output_file_name);
    ofs_result_output_.open(output_file_path);
    if (!ofs_result_output_.good())
    {
        std::cerr << "Open Result File Failed\t" << output_file_path << std::endl;
    }
    // ofs_result_output_ << nav_info_ << std::endl;
    nav_info_bak_ = nav_info_;
    return true;
}

void State::ReviseState(const Eigen::VectorXd &dx)
{
    filter_->ReviseState(nav_info_, dx);
}

/**
 * @brief  数据处理入口
 * @note   
 * @retval None
 */
void State::StartProcessing()
{
    if (!InitializeState())
    {
        navexit();
    }
    static int state_count = filter_->GetStateSize();
    static int output_rate = config_->get<int>("result_output_rate");
    static int gnss_measurement = config_->get<int>("gnss_enable");
    static int nhc_enable = config_->get<int>("nhc_enable");
    static int evaluate_imu_angle = config_->get<int>("evaluate_imu_angle");  // 是否估计IMU安装角

    static int nhc_gt = config_->get<int>("nhc_gt");
    static int nhc_lstm = config_->get<int>("nhc_lstm");
    int nhc_start = config_->get<int>("nhc_start");
    int nhc_end = config_->get<int>("nhc_end");

    static auto &index = filter_->GetStateIndex();
    GnssData::Ptr ptr_gnss_data = nullptr;
    ImuData::Ptr ptr_pre_imu_data = std::make_shared<ImuData>(), ptr_curr_imu_data;
    BaseData::bPtr base_data;
    Eigen::MatrixXd PHI = Eigen::MatrixXd::Identity(state_count, state_count);
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(state_count);
    int data_rate = config_->get<int>("data_rate");
    std::cout << "Starting Processing Data...." << std::endl;
    while (true)
    {
        base_data = data_queue_->GetData();
        if (base_data->get_type() == IMUDATA)
        {
            ptr_curr_imu_data = std::dynamic_pointer_cast<ImuData>(base_data);
            *ptr_pre_imu_data = *ptr_curr_imu_data;
            ptr_pre_imu_data->set_time(nav_info_.time_);
            if (!(ptr_curr_imu_data->get_time() - nav_info_.time_ <= 1.0 / data_rate))
            {
                std::cerr << "Data queue exists bug(s)" << std::endl;
                navsleep(100);
            }
            break;
        }
    }
    while (true)
    {
        if (ptr_curr_imu_data != nullptr)
        {
            Eigen::MatrixXd phi;
            double dt = ptr_curr_imu_data->get_time() - ptr_pre_imu_data->get_time();
            (*ptr_curr_imu_data) = Compensate(*ptr_curr_imu_data, nav_info_, dt);
            nav_info_ = navmech::MechanicalArrangement(*ptr_pre_imu_data, *ptr_curr_imu_data, nav_info_, phi);
            PHI *= phi;
            Eigen::Vector3d Vbb = nav_info_.rotation_.transpose()*nav_info_.vel_; // b系下速度
            nav_info_.vel_b_ = Vbb;

            auto blh = earth::WGS84XYZ2BLH(nav_info_.pos_);
            Eigen::Vector3d Vnn = earth::CalCe2n(blh(0), blh(1)) * nav_info_.vel_;  //n系下的速度
            nav_info_.vel_n_ = Vnn;
            nav_info_.pos_blh_ = blh;

            auto &Rbe = nav_info_.rotation_;
            auto &Rbv = nav_info_.imu_angle_rotation_;  // 把估计得到的安装角转换为方向余弦矩阵
            Eigen::Vector3d webb = nav_info_.wibb_ - Rbe.transpose() * earth::wiee();
            Eigen::Vector3d Vvv = Rbv * Vbb + Rbv * skew(webb) * nav_info_.imu_leverarm_; // v系下速度
            nav_info_.vel_v_ = Vvv;


            // TODO 添加NHC约束代码
            if(nav_info_.time_.second_of_week_ >= nhc_start && nav_info_.time_.second_of_week_ <= nhc_end && nhc_enable >= 1) {
//            if(nhc_enable >= 1) {
                static const StateIndex index = filter_->GetStateIndex();
                // time update
                double dt = base_data->get_time() - latest_update_time_;
//                auto &Rbe = nav_info_.rotation_;
                Eigen::MatrixXd state_q_used = state_q_;
                state_q_used.block<3, 3>(index.pos_index_, index.pos_index_) =
                        Rbe * state_q_used.block<3, 3>(index.pos_index_, index.pos_index_) * Rbe.transpose();
                state_q_used.block<3, 3>(index.vel_index_, index.vel_index_) =
                        Rbe * state_q_used.block<3, 3>(index.vel_index_, index.vel_index_) * Rbe.transpose();
                state_q_used.block<3, 3>(index.att_index_, index.att_index_) =
                        Rbe * state_q_used.block<3, 3>(index.att_index_, index.att_index_) * Rbe.transpose();
                Eigen::MatrixXd Q = (PHI * state_q_used * PHI.transpose() + state_q_used) * 0.5 * dt;
                filter_->TimeUpdate(PHI, Q, base_data->get_time());
                PHI = Eigen::MatrixXd::Identity(state_count, state_count);
                latest_update_time_ = base_data->get_time();

                // measurement update --- NHC
                size_t state_size = filter_->GetStateCov().cols();
                Eigen::MatrixXd Hmat_all = Eigen::MatrixXd::Zero(3, state_size);
                Eigen::VectorXd Zmat_all = Eigen::VectorXd::Zero(3);
                // 虚拟观测 和 设计矩阵
                // TODO 按照严密公式推导 需要测定IMU坐标系与以后轮为中心为原点的车体坐标系的相对位姿关系
//                Eigen::Vector3d Vbb = nav_info_.rotation_.transpose() * nav_info_.vel_; // b系下速度
//                Zmat_all = Vbb;
//                std::cout << " Zmat_all0: " << std::endl;
//                std::cout << Zmat_all << std::endl;
//                Hmat_all.block<3, 3>(0, index.vel_index_) = Rbe.transpose();
//                Hmat_all.block<3, 3>(0, index.att_index_) = -Rbe.transpose() * skew(nav_info_.vel_);



                //                虚拟观测值包括前向、侧向和垂向
                if (nhc_enable == 888) {
//                    Zmat_all = Vnn;
//                    Hmat_all.block<3, 3>(0, index.vel_index_) = Eigen::MatrixXd::Identity(3, 3);
                    Hmat_all.block<3, 3>(0, index.vel_index_) = earth::CalCe2n(blh(0), blh(1));
                    Hmat_all.block<3, 3>(0, index.att_index_) = -earth::CalCe2n(blh(0), blh(1)) * skew(nav_info_.vel_);
                    Zmat_all(0) = Vnn(0) - ptr_curr_imu_data->nhc_vel_lstm.x();
                    Zmat_all(1) = Vnn(1) - ptr_curr_imu_data->nhc_vel_lstm.y();
                    Zmat_all(2) = Vnn(2) - ptr_curr_imu_data->nhc_vel_lstm.z();
                    std::cout << " Zmat_all: " << std::endl;
                    std::cout <<  Zmat_all << std::endl;
                    Eigen::MatrixXd Hmat = Hmat_all;
                    Eigen::VectorXd Zmat = Zmat_all;
                    Eigen::MatrixXd Rmat = Eigen::MatrixXd::Zero(3, 3);
                    Rmat(0, 0) = 0.0001; // 前向
                    Rmat(1, 1) = 0.0001; // 侧向
                    Rmat(2, 2) = 0.0001; // 垂向

                    dx = filter_->MeasureUpdate(Hmat, Zmat, Rmat, ptr_curr_imu_data->get_time());
                    ReviseState(dx);
                }



                if (evaluate_imu_angle) {
//                    auto &Rbv = nav_info_.imu_angle_rotation_;  // 把估计得到的安装角转换为方向余弦矩阵
//                    Eigen::Vector3d webb = nav_info_.wibb_ - Rbe.transpose() * earth::wiee();
//                    Eigen::Vector3d Vvv = Rbv * Vbb + Rbv * skew(webb) * nav_info_.imu_leverarm_; // v系下速度
//                    nav_info_.vel_v_ = Vvv;
                    Zmat_all = Vvv;
                    Hmat_all.block<3, 3>(0, index.vel_index_) = Rbv * Rbe.transpose();
                    Hmat_all.block<3, 3>(0, index.att_index_) = -Rbv * Rbe.transpose() * skew(nav_info_.vel_);
                    Hmat_all.block<3, 3>(0, index.gyro_bias_index_) = Rbv * skew(nav_info_.imu_leverarm_);
                    Hmat_all.block<3, 2>(0, index.imu_angle_index_) = skew(Vvv).block<3, 2>(0, 1); // skew(Vvv)去掉第一列
                    Hmat_all.block<3, 3>(0, index.imu_leverarm_index_) = Rbv * skew(webb);


                    //                虚拟观测值只有侧向
                    if (nhc_enable == 1) {
                        if (nhc_gt) {
                            Zmat_all(1) = Vvv(1) - ptr_curr_imu_data->nhc_vel_gt.y();  // 侧向
                        }
                        if (nhc_lstm) {
                            Zmat_all(1) = Vvv(1) - ptr_curr_imu_data->nhc_vel_lstm.y();  // 侧向
                        }

                        Eigen::MatrixXd Hmat = Eigen::MatrixXd::Zero(1, state_size);
                        Eigen::VectorXd Zmat = Eigen::VectorXd::Zero(1);
                        Zmat(0) = Zmat_all(1);  // 手机数据IMU安装角接近0°，Zmat_all的index为1；MEMS数据IMU安装角接近90°，Zmat_all的index为0
                        Hmat.block(0, 0, 1, state_size) = Hmat_all.block(1, 0, 1,state_size);
                        std::cout << " Hmat: " << std::endl;
                        std::cout << Hmat << std::endl;
                        std::cout << " Zmat: " << std::endl;
                        std::cout << Zmat << std::endl;

                        Eigen::MatrixXd Rmat = Eigen::MatrixXd::Zero(1, 1);
                        //                Rmat(0,0) = ptr_curr_imu_data->cov_nhc.x() ; // 侧向
                        Rmat(0, 0) = 0.01; // 侧向
                        //                std::cout << " NHC_Rmat: " << std::endl;
                        //                std::cout <<  Rmat << std::endl;

                        dx = filter_->MeasureUpdate(Hmat, Zmat, Rmat, ptr_curr_imu_data->get_time());
                        ReviseState(dx);
                    }

                    //                虚拟观测值只有侧向和垂向
                    if (nhc_enable == 2) {
                        if (nhc_gt) {
                            Zmat_all(1) = Vvv(1) - ptr_curr_imu_data->nhc_vel_gt.y();  // 侧向
                            Zmat_all(2) = Vvv(2) - ptr_curr_imu_data->nhc_vel_gt.z();  // 垂向
                        }
                        if (nhc_lstm) {
                            Zmat_all(1) = Vvv(1) - ptr_curr_imu_data->nhc_vel_lstm.y();  // 侧向
                            Zmat_all(2) = Vvv(2) - ptr_curr_imu_data->nhc_vel_lstm.z();  // 垂向
                        }
                        Eigen::MatrixXd Hmat = Eigen::MatrixXd::Zero(2, state_size);
                        Eigen::VectorXd Zmat = Eigen::VectorXd::Zero(2);
                        Zmat(0) = Zmat_all(1);  // 手机数据IMU安装角接近0°，Zmat_all的index为1；MEMS数据IMU安装角接近90°，Zmat_all的index为0
                        Zmat(1) = Zmat_all(2);
                        Hmat.block(0, 0, 1, state_size) = Hmat_all.block(1, 0, 1,
                                                                         state_size);  // 手机数据IMU安装角接近0°，Hmat_all.block的startRow为1；MEMS数据IMU安装角接近90°，Hmat_all.block的startRow为0
                        Hmat.block(1, 0, 1, state_size) = Hmat_all.block(2, 0, 1, state_size);
                        std::cout << " Hmat: " << std::endl;
                        std::cout << Hmat << std::endl;
                        std::cout << " Zmat: " << std::endl;
                        std::cout << Zmat << std::endl;

                        Eigen::MatrixXd Rmat = Eigen::MatrixXd::Zero(2, 2);
                        //                Rmat(0,0) = ptr_curr_imu_data->cov_nhc.x() ; // 侧向
                        //                Rmat(1,1) = ptr_curr_imu_data->cov_nhc.y() ; // 垂向
                        Rmat(0, 0) = 0.01; // 侧向
                        Rmat(1, 1) = 0.01; // 垂向
                        //                std::cout << " NHC_Rmat: " << std::endl;
                        //                std::cout <<  Rmat << std::endl;

                        dx = filter_->MeasureUpdate(Hmat, Zmat, Rmat, ptr_curr_imu_data->get_time());
                        ReviseState(dx);
                    }


                    //                虚拟观测值包括前向、侧向和垂向
                    if (nhc_enable == 3) {
                        if (Vvv(2) <= 0.0)
                        {
                            std::cout << " hhhh: " << std::endl;
                        }
                        if (nhc_gt) {
                            Zmat_all(0) = Vvv(0) - ptr_curr_imu_data->nhc_vel_gt.x();  // 前向
                            Zmat_all(1) = Vvv(1) - ptr_curr_imu_data->nhc_vel_gt.y();  // 侧向
                            Zmat_all(2) = Vvv(2) - ptr_curr_imu_data->nhc_vel_gt.z();  // 垂向
                        }
                        if (nhc_lstm) {
                            Zmat_all(0) = Vvv(0) - ptr_curr_imu_data->nhc_vel_lstm.x();  // 前向
                            Zmat_all(1) = Vvv(1) - ptr_curr_imu_data->nhc_vel_lstm.y();  // 侧向
                            Zmat_all(2) = Vvv(2) - ptr_curr_imu_data->nhc_vel_lstm.z();  // 垂向
                        }

                        std::cout << " Zmat_all: " << std::endl;
                        std::cout <<  Zmat_all << std::endl;
                        Eigen::MatrixXd Hmat = Hmat_all;
                        Eigen::VectorXd Zmat = Zmat_all;
                        Eigen::MatrixXd Rmat = Eigen::MatrixXd::Zero(3, 3);
                        //                Rmat(0,0) = ptr_curr_imu_data->cov_nhc.x() ; // 前向
                        //                Rmat(1,1) = ptr_curr_imu_data->cov_nhc.y() ; // 侧向
                        //                Rmat(2,2) = ptr_curr_imu_data->cov_nhc.z() ; // 垂向
                        Rmat(0, 0) = 0.01; // 前向
                        Rmat(1, 1) = 0.01; // 侧向
                        Rmat(2, 2) = 0.01; // 垂向

                        dx = filter_->MeasureUpdate(Hmat, Zmat, Rmat, ptr_curr_imu_data->get_time());
                        ReviseState(dx);
                    }

                    //                虚拟观测值只有前向和侧向
                    if (nhc_enable == 4) {
                        if (nhc_gt) {
                            Zmat_all(0) = Vvv(0) - ptr_curr_imu_data->nhc_vel_gt.x();  // 前向
                            Zmat_all(1) = Vvv(1) - ptr_curr_imu_data->nhc_vel_gt.y();  // 侧向
                        }
                        if (nhc_lstm) {
                            Zmat_all(0) = Vvv(0) - ptr_curr_imu_data->nhc_vel_lstm.x();  // 前向
                            Zmat_all(1) = Vvv(1) - ptr_curr_imu_data->nhc_vel_lstm.y();  // 侧向
                        }
                        Eigen::MatrixXd Hmat = Eigen::MatrixXd::Zero(2, state_size);
                        Eigen::VectorXd Zmat = Eigen::VectorXd::Zero(2);
                        Zmat(0) = Zmat_all(0);
                        Zmat(1) = Zmat_all(1);
                        Hmat.block(0, 0, 1, state_size) = Hmat_all.block(0, 0, 1,
                                                                         state_size);
                        Hmat.block(1, 0, 1, state_size) = Hmat_all.block(1, 0, 1, state_size);
                        std::cout << " Hmat: " << std::endl;
                        std::cout << Hmat << std::endl;
                        std::cout << " Zmat: " << std::endl;
                        std::cout << Zmat << std::endl;

                        Eigen::MatrixXd Rmat = Eigen::MatrixXd::Zero(2, 2);
                        //                Rmat(0,0) = ptr_curr_imu_data->cov_nhc.x() ; // 侧向
                        //                Rmat(1,1) = ptr_curr_imu_data->cov_nhc.y() ; // 垂向
                        Rmat(0, 0) = 0.0001; // 前向
                        Rmat(1, 1) = 0.0001; // 侧向
                        //                std::cout << " NHC_Rmat: " << std::endl;
                        //                std::cout <<  Rmat << std::endl;

                        dx = filter_->MeasureUpdate(Hmat, Zmat, Rmat, ptr_curr_imu_data->get_time());
                        ReviseState(dx);
                    }

                    //                虚拟观测值只有垂向
                    if (nhc_enable == 5) {
                        if (nhc_gt) {
                            Zmat_all(2) = Vvv(2) - ptr_curr_imu_data->nhc_vel_gt.z();  // 垂向
                        }
                        if (nhc_lstm) {
                            Zmat_all(2) = Vvv(2) - ptr_curr_imu_data->nhc_vel_lstm.z();  // 垂向
                        }
                        Eigen::MatrixXd Hmat = Eigen::MatrixXd::Zero(1, state_size);
                        Eigen::VectorXd Zmat = Eigen::VectorXd::Zero(1);
                        Zmat(0) = Zmat_all(2);
                        Hmat.block(0, 0, 1, state_size) = Hmat_all.block(2, 0, 1,state_size);
                        std::cout << " Hmat: " << std::endl;
                        std::cout << Hmat << std::endl;
                        std::cout << " Zmat: " << std::endl;
                        std::cout << Zmat << std::endl;

                        Eigen::MatrixXd Rmat = Eigen::MatrixXd::Zero(1, 1);
                        //                Rmat(0,0) = ptr_curr_imu_data->cov_nhc.x() ; // 侧向
                        Rmat(0, 0) = 0.0001; // 垂向
                        //                std::cout << " NHC_Rmat: " << std::endl;
                        //                std::cout <<  Rmat << std::endl;

                        dx = filter_->MeasureUpdate(Hmat, Zmat, Rmat, ptr_curr_imu_data->get_time());
                        ReviseState(dx);
                    }

                    //                虚拟观测值只有前向
                    if (nhc_enable == 6) {
                        if (nhc_gt) {
                            Zmat_all(0) = Vvv(0) - ptr_curr_imu_data->nhc_vel_gt.x();  // 前向
                        }
                        if (nhc_lstm) {
                            Zmat_all(0) = Vvv(0) - ptr_curr_imu_data->nhc_vel_lstm.x();  // 前向
                        }
                        Eigen::MatrixXd Hmat = Eigen::MatrixXd::Zero(1, state_size);
                        Eigen::VectorXd Zmat = Eigen::VectorXd::Zero(1);
                        Zmat(0) = Zmat_all(0);
                        Hmat.block(0, 0, 1, state_size) = Hmat_all.block(0, 0, 1,state_size);
                        std::cout << " Hmat: " << std::endl;
                        std::cout << Hmat << std::endl;
                        std::cout << " Zmat: " << std::endl;
                        std::cout << Zmat << std::endl;

                        Eigen::MatrixXd Rmat = Eigen::MatrixXd::Zero(1, 1);
                        //                Rmat(0,0) = ptr_curr_imu_data->cov_nhc.x() ; // 侧向
                        Rmat(0, 0) = 0.0001; // 前向
                        //                std::cout << " NHC_Rmat: " << std::endl;
                        //                std::cout <<  Rmat << std::endl;

                        dx = filter_->MeasureUpdate(Hmat, Zmat, Rmat, ptr_curr_imu_data->get_time());
                        ReviseState(dx);
                    }

                    //                虚拟观测值只有前向和垂向
                    if (nhc_enable == 7) {
                        if (nhc_gt) {
                            Zmat_all(0) = Vvv(0) - ptr_curr_imu_data->nhc_vel_gt.x();  // 前向
                            Zmat_all(2) = Vvv(2) - ptr_curr_imu_data->nhc_vel_gt.z();  // 垂向
                        }
                        if (nhc_lstm) {
                            Zmat_all(0) = Vvv(0) - ptr_curr_imu_data->nhc_vel_lstm.x();  // 前向
                            Zmat_all(2) = Vvv(2) - ptr_curr_imu_data->nhc_vel_lstm.z();  // 垂向
                        }
                        Eigen::MatrixXd Hmat = Eigen::MatrixXd::Zero(2, state_size);
                        Eigen::VectorXd Zmat = Eigen::VectorXd::Zero(2);
                        Zmat(0) = Zmat_all(0);
                        Zmat(1) = Zmat_all(2);
                        Hmat.block(0, 0, 1, state_size) = Hmat_all.block(0, 0, 1,
                                                                         state_size);
                        Hmat.block(1, 0, 1, state_size) = Hmat_all.block(2, 0, 1, state_size);
                        std::cout << " Hmat: " << std::endl;
                        std::cout << Hmat << std::endl;
                        std::cout << " Zmat: " << std::endl;
                        std::cout << Zmat << std::endl;

                        Eigen::MatrixXd Rmat = Eigen::MatrixXd::Zero(2, 2);
                        //                Rmat(0,0) = ptr_curr_imu_data->cov_nhc.x() ; // 侧向
                        //                Rmat(1,1) = ptr_curr_imu_data->cov_nhc.y() ; // 垂向
                        Rmat(0, 0) = 1; // 前向
                        Rmat(1, 1) = 0.01; // 垂向
                        //                std::cout << " NHC_Rmat: " << std::endl;
                        //                std::cout <<  Rmat << std::endl;

                        dx = filter_->MeasureUpdate(Hmat, Zmat, Rmat, ptr_curr_imu_data->get_time());
                        ReviseState(dx);
                    }
                }
            }

            ptr_pre_imu_data = ptr_curr_imu_data;
            ptr_curr_imu_data = nullptr;
        }
        else
        {
            /* time update */
            double dt = base_data->get_time() - latest_update_time_;
            auto &Rbe = nav_info_.rotation_;
            Eigen::MatrixXd state_q_used = state_q_;
            state_q_used.block<3, 3>(index.pos_index_, index.pos_index_) =
                Rbe * state_q_used.block<3, 3>(index.pos_index_, index.pos_index_) * Rbe.transpose();
            state_q_used.block<3, 3>(index.vel_index_, index.vel_index_) =
                Rbe * state_q_used.block<3, 3>(index.vel_index_, index.vel_index_) * Rbe.transpose();
            state_q_used.block<3, 3>(index.att_index_, index.att_index_) =
                Rbe * state_q_used.block<3, 3>(index.att_index_, index.att_index_) * Rbe.transpose();
            Eigen::MatrixXd Q = (PHI * state_q_used * PHI.transpose() + state_q_used) * 0.5 * dt;
            filter_->TimeUpdate(PHI, Q, base_data->get_time());
            PHI = Eigen::MatrixXd::Identity(state_count, state_count);
            latest_update_time_ = base_data->get_time();

//            /* measure update */
//            if(base_data->get_time().SecondOfWeek()>=372060 && base_data->get_time().SecondOfWeek()<372061)
//            {
//                gnss_measurement = false;
//            }
//            else
//            {
//                gnss_measurement = true;
//            }

            if (gnss_measurement && ptr_gnss_data != nullptr)
            {
                gps_process_->processing(ptr_gnss_data, nav_info_, dx);
                ReviseState(dx);
                ptr_gnss_data = nullptr;
            }
        }

//        int idt = int((nav_info_.time_.Second() + 0.00005) * output_rate) - int(nav_info_bak_.time_.Second() * output_rate);  // 这样写会导致每60s断一次，每次刚到新的一分钟，就不会输出结果，改成下面一行就可以了
        int idt = int((nav_info_.time_.SecondOfWeek() + 0.00005) * output_rate) - int(nav_info_bak_.time_.SecondOfWeek() * output_rate);
        if (idt > 0)
        {
            double second_of_week = nav_info_.time_.SecondOfWeek();
            double double_part = (second_of_week * output_rate - int(second_of_week * output_rate)) / output_rate;
            NavTime inter_time = nav_info_.time_ - double_part;
            auto output_nav = InterpolateNavInfo(nav_info_bak_, nav_info_, inter_time);
            output_nav.pos_ = earth::CorrectLeverarmPos(output_nav);
            auto blh = earth::WGS84XYZ2BLH(output_nav.pos_);
            output_nav.vel_ = earth::CalCe2n(blh(0), blh(1)) * earth::CorrectLeverarmVel(output_nav);
            ofs_result_output_ << output_nav << std::endl;
            if (fabs(int(output_nav.time_.SecondOfWeek()) - output_nav.time_.SecondOfWeek() < 0.04))
                std::cerr << output_nav.time_ << std::endl;
        }
        nav_info_bak_ = nav_info_;
        /*获取新的数据 */
        base_data = data_queue_->GetData();
        // std::cout << "base_data time: " << base_data->get_type() << "  "
        //           << std::fixed << std::setprecision(8)
        //           << base_data->get_time().SecondOfWeek() << std::endl;
        if (base_data->get_type() == IMUDATA)
        {
            ptr_curr_imu_data = std::dynamic_pointer_cast<ImuData>(base_data);
        }
        else if (base_data->get_type() == GNSSDATA)
        {
            ptr_gnss_data = std::dynamic_pointer_cast<GnssData>(base_data);
        }
        else if (base_data->get_type() == DATAUNKOWN)
        {
            break;
        }
        else
        {
            std::cerr << "Bak Data processing thread " << std::endl;
            //BAK for other data type
        }
    }
    std::cout << "all data processing finish" << std::endl;
}

NavInfo State::GetNavInfo() const
{
    return nav_info_;
}

} // namespace mscnav
