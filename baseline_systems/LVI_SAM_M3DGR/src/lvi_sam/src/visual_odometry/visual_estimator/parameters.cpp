#include "parameters.h"
#include "yaml-cpp/yaml.h"
std::string PROJECT_NAME;

double INIT_DEPTH;
double MIN_PARALLAX; // 根据平均视差决定merge最老帧还是次新帧
double ACC_N, ACC_W; // 加速度白噪声和随机游走
double GYR_N, GYR_W; // 角速度白噪声和随机游走
// camera to imu 外参
std::vector<Eigen::Matrix3d> RIC; // 旋转
std::vector<Eigen::Vector3d> TIC; // 平移

Eigen::Vector3d G{0.0, 0.0, 9.8};
// std::string vio_trajectory_path;
double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME; // 非线性优化的时间限制
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC; // 在线外参标定 (camera to imu)
int ESTIMATE_TD; // 相机与imu时间戳同步
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string IMU_TOPIC;
double ROW, COL;
double TD, TR;

// double Fx,Fy,Cx,Cy;
int USE_LIDAR;
int ALIGN_CAMERA_LIDAR_COORDINATE;
/**
 * @brief modified
 * L_RX_I represents L^RX_I
 */
double L_TX_I;
double L_TY_I;
double L_TZ_I;
double L_RX_I;
double L_RY_I;
double L_RZ_I;

int imu_Hz;

double C_TX_L;
double C_TY_L;
double C_TZ_L;
double C_RX_L;
double C_RY_L;
double C_RZ_L;

double imuGravity;
double imuAccNoise;
double imuGyrNoise;
double imuAccBiasN;
double imuGyrBiasN;
int imuHz;

//从配置文件中读取参数
void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    n.getParam("vins_config_file", config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["project_name"] >> PROJECT_NAME;
    std::string pkg_path = ros::package::getPath(PROJECT_NAME);

    fsSettings["imu_topic"] >> IMU_TOPIC;

    fsSettings["use_lidar"] >> USE_LIDAR;
    fsSettings["align_camera_lidar_estimation"] >> ALIGN_CAMERA_LIDAR_COORDINATE;

    // fsSettings["vio_trajectory_path"]>>vio_trajectory_path;

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    /**
     * @brief modified
     * 
     */
    L_TX_I = fsSettings["imu_to_lidar_tx"];
    L_TY_I = fsSettings["imu_to_lidar_ty"];
    L_TZ_I = fsSettings["imu_to_lidar_tz"];
    L_RX_I = fsSettings["imu_to_lidar_rx"];
    L_RY_I = fsSettings["imu_to_lidar_ry"];
    L_RZ_I = fsSettings["imu_to_lidar_rz"];

    imu_Hz = fsSettings["imu_hz"];

    C_TX_L = fsSettings["lidar_to_cam_tx"];
    C_TY_L = fsSettings["lidar_to_cam_ty"];
    C_TZ_L = fsSettings["lidar_to_cam_tz"];
    C_RX_L = fsSettings["lidar_to_cam_rx"];
    C_RY_L = fsSettings["lidar_to_cam_ry"];
    C_RZ_L = fsSettings["lidar_to_cam_rz"];

    imuGravity = fsSettings["g_norm"];
    imuAccNoise = fsSettings["acc_n"];
    imuGyrNoise = fsSettings["gyr_n"];
    imuAccBiasN = fsSettings["acc_w"];
    imuGyrBiasN = fsSettings["gyr_w"];
    imuHz = fsSettings["imu_hz"];


    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];

    ROS_INFO("Image dimention: ROW: %f COL: %f ", ROW, COL);

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = pkg_path + "/config/extrinsic_parameter.csv";

    }
    else 
    {
        //优化外参
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_INFO(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = pkg_path + "/config/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_INFO(" Fix extrinsic param.");

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());
        
    } 

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }
    
    fsSettings.release();
    usleep(100);
}
