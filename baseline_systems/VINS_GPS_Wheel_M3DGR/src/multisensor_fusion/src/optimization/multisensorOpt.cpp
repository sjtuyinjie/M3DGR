
#include "factor/Factors.h"
#include "optimization/multisensorOpt.h"
using namespace std;


MultisensorOptimization::MultisensorOptimization()
{
    initGPS = false;
    newGPS = false;
    WGPS_T_WVIO = Eigen::Matrix4d::Identity();
    threadOpt = std::thread(&MultisensorOptimization::optimize, this);
    windowLength = 50;

    lastLocalP = Eigen::Vector3d::Zero();
}

MultisensorOptimization::~MultisensorOptimization()
{
    threadOpt.detach();
}

void MultisensorOptimization::GPS2XYZ(double latitude, double longitude, double altitude, double *xyz)
{
    if(!initGPS)
    {
        geoConverter.Reset(latitude, longitude, altitude);
        initGPS = true;
    }
    geoConverter.Forward(latitude, longitude, altitude, xyz[0], xyz[1], xyz[2]);
}

void MultisensorOptimization::inputOdom(double t, Eigen::Vector3d odomP, Eigen::Quaterniond odomQ)
{
	mPoseMap.lock();
    vector<double> localPose{odomP.x(), odomP.y(), odomP.z(), 
    					     odomQ.w(), odomQ.x(), odomQ.y(), odomQ.z()};
    
    // 位移差1厘米，才运动
    if ((odomP - lastLocalP).norm() > 0.01)
    {
        localPoseMap[t] = localPose;

        while (localPoseMap.size() > windowLength)
        {
            localPoseMap.erase(std::begin(localPoseMap));
        }

        Eigen::Quaterniond globalQ;
        globalQ = WGPS_T_WVIO.block<3, 3>(0, 0) * odomQ;
        Eigen::Vector3d globalP = WGPS_T_WVIO.block<3, 3>(0, 0) * odomP + WGPS_T_WVIO.block<3, 1>(0, 3);
        vector<double> globalPose{globalP.x(), globalP.y(), globalP.z(),
                                globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z()};
        globalPoseMap[t] = globalPose;

        while (globalPoseMap.size() > windowLength)
        {
            globalPoseMap.erase(std::begin(globalPoseMap));
        }

        lastLocalP = odomP;
    }

    mPoseMap.unlock();
}

void MultisensorOptimization::getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ)
{
    odomP = lastP;
    odomQ = lastQ;
}

void MultisensorOptimization::inputGPS(double t, double latitude, double longitude, double altitude, double latCov, double lonCov, double altCov)
{
    double xyz[3];
    GPS2XYZ(latitude, longitude, altitude, xyz);
    std::vector<double> tmp{xyz[0], xyz[1], xyz[2], latCov, lonCov, altCov};
    // printf("new gps: t: %f x: %f y: %f z:%f \n", t, tmp[0], tmp[1], tmp[2]);
    GPSPositionMap[t] = tmp;

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = ros::Time(t);
    poseStamped.header.frame_id = "world";
    poseStamped.pose.position.x = tmp[0];
    poseStamped.pose.position.y = tmp[1];
    poseStamped.pose.position.z = tmp[2];
    poseStamped.pose.orientation.w = 1;
    poseStamped.pose.orientation.x = 0;
    poseStamped.pose.orientation.y = 0;
    poseStamped.pose.orientation.z = 0;
    gpsPath.header = poseStamped.header;
    gpsPath.poses.push_back(poseStamped);
    
    newGPS = true;
}

void MultisensorOptimization::optimize()
{
    while (true)
    {
        if (newGPS)
        {
            newGPS = false;
            // printf("Multisensor optimization\n");

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 5;
            // options.max_solver_time_in_seconds = 0.15;
            ceres::Solver::Summary summary;
            ceres::LossFunction *lossFunction;
            lossFunction = new ceres::HuberLoss(1.0);
            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

            //add param
            mPoseMap.lock();
            int length = localPoseMap.size();
            // w^t_i   w^q_i
            double t_array[length][3];
            double q_array[length][4];
            map<double, vector<double>>::iterator iter;
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
                t_array[i][0] = iter->second[0];
                t_array[i][1] = iter->second[1];
                t_array[i][2] = iter->second[2];
                q_array[i][0] = iter->second[3];
                q_array[i][1] = iter->second[4];
                q_array[i][2] = iter->second[5];
                q_array[i][3] = iter->second[6];
                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);
            }

            std::map<double, std::vector<double>>::iterator iterVIO, iterVIONext, iterGPS;
            int i = 0;
            for (iterVIO = localPoseMap.begin(); iterVIO != localPoseMap.end(); iterVIO++, i++)
            {
                //vio factor
                iterVIONext = iterVIO;
                iterVIONext++;
                if (iterVIONext != localPoseMap.end())
                {
                    Eigen::Matrix4d wTi = Eigen::Matrix4d::Identity();
                    Eigen::Matrix4d wTj = Eigen::Matrix4d::Identity();
                    wTi.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIO->second[3], iterVIO->second[4], 
                                                               iterVIO->second[5], iterVIO->second[6]).toRotationMatrix();
                    wTi.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIO->second[0], iterVIO->second[1], iterVIO->second[2]);
                    wTj.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIONext->second[3], iterVIONext->second[4], 
                                                               iterVIONext->second[5], iterVIONext->second[6]).toRotationMatrix();
                    wTj.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIONext->second[0], iterVIONext->second[1], iterVIONext->second[2]);
                    Eigen::Matrix4d iTj = wTi.inverse() * wTj;
                    Eigen::Quaterniond iQj;
                    iQj = iTj.block<3, 3>(0, 0);
                    Eigen::Vector3d iPj = iTj.block<3, 1>(0, 3);

                    ceres::CostFunction* vio_function = RelativeRTError::Create(iPj.x(), iPj.y(), iPj.z(),
                                                                                iQj.w(), iQj.x(), iQj.y(), iQj.z(),
                                                                                0.1, 0.01);
                    problem.AddResidualBlock(vio_function, NULL, q_array[i], t_array[i], q_array[i+1], t_array[i+1]);
                    /*
                    double **para = new double *[4];
                    para[0] = q_array[i];
                    para[1] = t_array[i];
                    para[3] = q_array[i+1];
                    para[4] = t_array[i+1];

                    double *tmp_r = new double[6];
                    double **jaco = new double *[4];
                    jaco[0] = new double[6 * 4];
                    jaco[1] = new double[6 * 3];
                    jaco[2] = new double[6 * 4];
                    jaco[3] = new double[6 * 3];
                    vioFunction->Evaluate(para, tmp_r, jaco);

                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 1>>(tmp_r).transpose() << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>(jaco[0]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(jaco[1]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>(jaco[2]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(jaco[3]) << std::endl
                        << std::endl;
                    */
                }

                // gps factor
                double t = iterVIO->first;
                iterGPS = GPSPositionMap.find(t);
                if (iterGPS != GPSPositionMap.end())
                {
                    ceres::CostFunction* gpsFunction = TError::Create(iterGPS->second[0], iterGPS->second[1], iterGPS->second[2],
                                                                      iterGPS->second[3], iterGPS->second[4], iterGPS->second[5]);
                    problem.AddResidualBlock(gpsFunction, lossFunction, t_array[i]);
                    /*
                    double **para = new double *[1];
                    para[0] = t_array[i];

                    double *tmp_r = new double[3];
                    double **jaco = new double *[1];
                    jaco[0] = new double[3 * 3];
                    gpsFunction->Evaluate(para, tmp_r, jaco);

                    std::cout << Eigen::Map<Eigen::Matrix<double, 3, 1>>(tmp_r).transpose() << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(jaco[0]) << std::endl
                        << std::endl;
                    */
                }
            }

            // solve problem
            ceres::Solve(options, &problem, &summary);
            // std::cout << summary.BriefReport() << std::endl;

            // update global pose
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
            	vector<double> globalPose{t_array[i][0], t_array[i][1], t_array[i][2],
            							  q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]};
            	iter->second = globalPose;
            	if(i == length - 1)
            	{
            	    Eigen::Matrix4d WVIO_T_body = Eigen::Matrix4d::Identity(); 
            	    Eigen::Matrix4d WGPS_T_body = Eigen::Matrix4d::Identity();
            	    double t = iter->first;
            	    WVIO_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(localPoseMap[t][3], localPoseMap[t][4], 
            	                                                       localPoseMap[t][5], localPoseMap[t][6]).toRotationMatrix();
            	    WVIO_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(localPoseMap[t][0], localPoseMap[t][1], localPoseMap[t][2]);
            	    WGPS_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(globalPose[3], globalPose[4], 
            	                                                        globalPose[5], globalPose[6]).toRotationMatrix();
            	    WGPS_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(globalPose[0], globalPose[1], globalPose[2]);
            	    WGPS_T_WVIO = WGPS_T_body * WVIO_T_body.inverse();
            	}
            }
            updateGlobalPath();
            mPoseMap.unlock();
        }
        // std::chrono::milliseconds dura(1000);
        // std::this_thread::sleep_for(dura);
    }
    return;
}

void MultisensorOptimization::updateGlobalPath()
{
    // global_path.poses.clear();
    if (globalPath.poses.size() > windowLength - 1)
    {
        int i = windowLength - 1;
        while (i > 0)
        {
            globalPath.poses.pop_back();
            i--;
        }
    }
    else
    {
        globalPath.poses.clear();
    }
    map<double, vector<double>>::iterator iter;
    for (iter = globalPoseMap.begin(); iter != globalPoseMap.end(); iter++)
    {
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.stamp = ros::Time(iter->first);
        poseStamped.header.frame_id = "world";
        poseStamped.pose.position.x = iter->second[0];
        poseStamped.pose.position.y = iter->second[1];
        poseStamped.pose.position.z = iter->second[2];
        poseStamped.pose.orientation.w = iter->second[3];
        poseStamped.pose.orientation.x = iter->second[4];
        poseStamped.pose.orientation.y = iter->second[5];
        poseStamped.pose.orientation.z = iter->second[6];
        globalPath.header = poseStamped.header;
        globalPath.poses.push_back(poseStamped);
    }

    // 使用优化后的最新位姿作为global odometry
    iter--;
    lastP.x() = iter->second[0];
    lastP.y() = iter->second[1];
    lastP.z() = iter->second[2];
    lastQ.w() = iter->second[3];
    lastQ.x() = iter->second[4];
    lastQ.y() = iter->second[5];
    lastQ.z() = iter->second[6];
}

