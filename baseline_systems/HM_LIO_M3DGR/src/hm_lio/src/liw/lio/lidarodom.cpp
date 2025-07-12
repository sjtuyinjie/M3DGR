#include <yaml-cpp/yaml.h>
#include "lidarodom.h"

namespace zjloc
{

     lidarodom::lidarodom(/* args */)
     {
          laser_point_cov = 0.001;

          G.setZero();
          H_T_H.setZero();
          I_STATE.setIdentity();

          index_frame = 1;
          opt_cnt = 0;
          curr_time = 0;
          total_computation = 0;
          points_world.reset(new pcl::PointCloud<pcl::PointXYZI>());
     }

     lidarodom::~lidarodom()
     {
     }

     void lidarodom::loadOptions()
     {
          auto yaml = YAML::LoadFile(config_yaml_);
          options_.surf_res = yaml["odometry"]["surf_res"].as<double>();
          options_.log_print = yaml["odometry"]["log_print"].as<bool>();
          options_.extrinsic_est_en = yaml["odometry"]["extrinsic_est_en"].as<bool>();
          options_.num_max_iterations = yaml["odometry"]["num_max_iterations"].as<int>();

          options_.size_voxel_map = yaml["odometry"]["size_voxel_map"].as<double>();
          options_.min_distance_points = yaml["odometry"]["min_distance_points"].as<double>();
          options_.max_num_points_in_voxel = yaml["odometry"]["max_num_points_in_voxel"].as<int>();
          options_.max_distance = yaml["odometry"]["max_distance"].as<double>();
          options_.weight_alpha = yaml["odometry"]["weight_alpha"].as<double>();
          options_.weight_neighborhood = yaml["odometry"]["weight_neighborhood"].as<double>();
          options_.max_dist_to_plane_icp = yaml["odometry"]["max_dist_to_plane_icp"].as<double>();
          options_.init_num_frames = yaml["odometry"]["init_num_frames"].as<int>();
          options_.voxel_neighborhood = yaml["odometry"]["voxel_neighborhood"].as<int>();
          options_.max_number_neighbors = yaml["odometry"]["max_number_neighbors"].as<int>();
          options_.threshold_voxel_occupancy = yaml["odometry"]["threshold_voxel_occupancy"].as<int>();
          options_.estimate_normal_from_neighborhood = yaml["odometry"]["estimate_normal_from_neighborhood"].as<bool>();
          options_.min_number_neighbors = yaml["odometry"]["min_number_neighbors"].as<int>();
          options_.power_planarity = yaml["odometry"]["power_planarity"].as<double>();
          options_.num_closest_neighbors = yaml["odometry"]["num_closest_neighbors"].as<int>();

          options_.sampling_rate = yaml["odometry"]["sampling_rate"].as<double>();
          options_.max_num_residuals = yaml["odometry"]["max_num_residuals"].as<int>();
          options_.min_num_residuals = yaml["odometry"]["min_num_residuals"].as<int>();

          options_.thres_orientation_norm = yaml["odometry"]["thres_orientation_norm"].as<double>();
          options_.thres_translation_norm = yaml["odometry"]["thres_translation_norm"].as<double>();
     }

     bool lidarodom::init(const std::string &config_yaml)
     {
          config_yaml_ = config_yaml;

          auto yaml = YAML::LoadFile(config_yaml_);
          delay_time_ = yaml["delay_time"].as<double>();
          // lidar和IMU外参
          std::vector<double> ext_t = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
          std::vector<double> ext_r = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();
          Vec3d lidar_T_wrt_IMU = math::VecFromArray(ext_t);
          Mat3d lidar_R_wrt_IMU = math::MatFromArray(ext_r);
          std::cout << yaml["mapping"]["extrinsic_R"] << std::endl;
          Eigen::Quaterniond q_IL(lidar_R_wrt_IMU);
          q_IL.normalized();
          lidar_R_wrt_IMU = q_IL;
          // init TIL
          TIL_ = SE3(q_IL, lidar_T_wrt_IMU);
          R_imu_lidar = lidar_R_wrt_IMU;
          t_imu_lidar = lidar_T_wrt_IMU;
          std::cout << "RIL:\n"
                    << R_imu_lidar << std::endl;
          std::cout << "tIL:" << t_imu_lidar.transpose() << std::endl;

          p_imu.reset(new ImuProcess());
          p_imu->set_extrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);

          loadOptions();

          return true;
     }

     void lidarodom::pushData(std::vector<point3D> msg, std::pair<double, double> data)
     {
          if (data.first < last_timestamp_lidar_)
          {
               LOG(ERROR) << "lidar loop back, clear buffer";
               lidar_buffer_.clear();
               time_buffer_.clear();
          }

          mtx_buf.lock();
          lidar_buffer_.push_back(msg);
          time_buffer_.push_back(data);
          last_timestamp_lidar_ = data.first;
          mtx_buf.unlock();
          cond.notify_one();
     }
     void lidarodom::pushData(IMUPtr imu)
     {
          double timestamp = imu->timestamp_;
          if (timestamp < last_timestamp_imu_)
          {
               LOG(WARNING) << "imu loop back, clear buffer";
               imu_buffer_.clear();
          }

          last_timestamp_imu_ = timestamp;

          mtx_buf.lock();
          imu_buffer_.emplace_back(imu);
          mtx_buf.unlock();
          cond.notify_one();
     }

     void lidarodom::run()
     {
          while (true)
          {
               std::vector<MeasureGroup> measurements;
               std::unique_lock<std::mutex> lk(mtx_buf);
               cond.wait(lk, [&]
                         { return (measurements = getMeasureMents()).size() != 0; });
               lk.unlock();

               for (auto &m : measurements)
               {
                    curr_time = omp_get_wtime();

                    zjloc::common::Timer::Evaluate([&]()
                                                   { ProcessMeasurements(m); },
                                                   "processMeasurement");

                    // visualize_state(); //     for vis

                    {
                         auto real_time = std::chrono::high_resolution_clock::now();
                         static std::chrono::system_clock::time_point prev_real_time = real_time;

                         if (real_time - prev_real_time > std::chrono::seconds(5))
                         {
                              auto data_time = m.lidar_end_time_;
                              static double prev_data_time = data_time;
                              auto delta_real = std::chrono::duration_cast<std::chrono::milliseconds>(real_time - prev_real_time).count() * 0.001;
                              auto delta_sim = data_time - prev_data_time;
                              printf("Processing the rosbag at %.1fX speed.", delta_sim / delta_real);

                              prev_data_time = data_time;
                              prev_real_time = real_time;
                         }
                    }
               }
          }
     }

     void lidarodom::ProcessMeasurements(MeasureGroup &meas)
     {
          measures_ = meas;

          p_imu->Process(measures_, state);

          if (!p_imu->isInitSuccess())
               return;

          std::cout << ANSI_COLOR_GREEN << "============== process frame: "
                    << index_frame << ANSI_COLOR_RESET << std::endl;

          if (options_.log_print)
          {
               V3D euler_cur = RotMtoEuler(state.rot_end);
               std::cout << "pre- states, r:" << euler_cur.transpose() * 57.3 << " ,p:"
                         << state.pos_end.transpose() << " ,v:" << state.vel_end.transpose() << " ,bg:"
                         << state.bias_g.transpose() << " ,ba:" << state.bias_a.transpose() << std::endl;
          }

          static bool first_lidar = true;
          if (first_lidar)
          {
               first_lidar_time = measures_.lidar_begin_time_;
               first_lidar = false;
          }
          SE3 pred_pose(state.rot_end, state.pos_end);
          for (auto &point_temp : measures_.lidar_)
               transformPoint(point_temp, pred_pose, R_imu_lidar, t_imu_lidar);

          flg_EKF_inited = (measures_.lidar_begin_time_ - first_lidar_time) < INIT_TIME ? false : true;

          zjloc::common::Timer::Evaluate([&]()
                                         { poseEstimation(); },
                                         "poseEstimate");

          zjloc::common::Timer::Evaluate([&]()
                                         {
               std::string laser_topic = "laser";
               SE3 pose = SE3(state.rot_end, state.pos_end);
               pub_pose_to_ros(laser_topic, pose, meas.lidar_end_time_);
               laser_topic = "velocity";

               Eigen::Vector3d vel_world = state.vel_end;
               Eigen::Vector3d vel_base = pred_pose.rotationMatrix().inverse() * vel_world;
               pub_data_to_ros(laser_topic, vel_base.x(), 0);
               if(index_frame%8==0)
               {
                    laser_topic = "dist";
                    static Eigen::Vector3d last_t = Eigen::Vector3d::Zero();
                    Eigen::Vector3d t = pred_pose.translation();
                    static double dist = 0;
                    dist += (t - last_t).norm();
                    last_t = t;
                    pub_data_to_ros(laser_topic, dist, 0);
               } },
                                         "pub cloud");
          index_frame++;
          std::vector<point3D>().swap(meas.lidar_);
     }

     void lidarodom::poseEstimation()
     {
          if (index_frame > 1)
          {
               zjloc::common::Timer::Evaluate([&]()
                                              { optimize(); },
                                              "optimize");
          }

          bool add_points = true;
          if (add_points)
          { //   update map here
               zjloc::common::Timer::Evaluate([&]()
                                              { map_incremental(); },
                                              "map update");
          }

          zjloc::common::Timer::Evaluate([&]()
                                         { lasermap_fov_segment(); },
                                         "fov segment");
     }

     void lidarodom::optimize()
     {

          std::vector<point3D> surf_keypoints;
          gridSampling(measures_.lidar_, surf_keypoints,
                       options_.sampling_rate * options_.surf_res);

          if (options_.log_print)
               std::cout << __FUNCTION__ << ", before and aft gridSampling: " << measures_.lidar_.size() << ", " << surf_keypoints.size() << std::endl;

          bool flg_EKF_converged = false;

          auto transformKeypoints = [&](std::vector<point3D> &point_frame)
          {
               // #ifdef MP_EN
               // omp_set_num_threads(MP_PROC_NUM)
               // #pragma omp parallel for
               // #endif
               int numberOfCores = 4;
#pragma omp parallel for num_threads(numberOfCores)
               for (int tt = 0; tt < point_frame.size(); tt++)
                    point_frame[tt]
                        .point = state.rot_end * (TIL_ * point_frame[tt].raw_point) + state.pos_end;
          };

          auto estimatePointNeighborhood = [&](std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &vector_neighbors,
                                               Eigen::Vector3d &location, double &planarity_weight)
          {
               auto neighborhood = computeNeighborhoodDistribution(vector_neighbors);
               planarity_weight = std::pow(neighborhood.a2D, options_.power_planarity);

               if (neighborhood.normal.dot(state.pos_end - location) < 0)
               {
                    neighborhood.normal = -1.0 * neighborhood.normal;
               }
               return neighborhood;
          };

          StatesGroup state_propagat(state); //  predict pose

          for (int iterCount = 0; iterCount < options_.num_max_iterations; iterCount++)
          {
               double lambda_weight = std::abs(options_.weight_alpha);
               double lambda_neighborhood = std::abs(options_.weight_neighborhood);
               const double kMaxPointToPlane = options_.max_dist_to_plane_icp;
               const double sum = lambda_weight + lambda_neighborhood;

               lambda_weight /= sum;
               lambda_neighborhood /= sum;

               const short nb_voxels_visited = index_frame < options_.init_num_frames
                                                   ? 2
                                                   : options_.voxel_neighborhood;

               const int kThresholdCapacity = index_frame < options_.init_num_frames
                                                  ? 1
                                                  : options_.threshold_voxel_occupancy;

               transformKeypoints(surf_keypoints);

               size_t num = surf_keypoints.size();
               int num_residuals = 0;

               std::vector<PointType> coeffSel_tmpt;
               std::vector<PointType> pointSel_tmpt;
               for (int k = 0; k < num; k++)
               {
                    auto &keypoint = surf_keypoints[k];
                    auto &raw_point = keypoint.raw_point;

                    std::vector<voxel> voxels;
                    auto vector_neighbors = searchNeighbors(voxel_map, keypoint.point,
                                                            nb_voxels_visited,
                                                            options_.size_voxel_map,
                                                            options_.max_number_neighbors,
                                                            kThresholdCapacity,
                                                            options_.estimate_normal_from_neighborhood
                                                                ? nullptr
                                                                : &voxels);

                    if (vector_neighbors.size() < options_.min_number_neighbors)
                         continue;

                    double weight;

                    Eigen::Vector3d location = TIL_ * raw_point;

                    auto neighborhood = estimatePointNeighborhood(vector_neighbors, location, weight);

                    weight = lambda_weight * weight + lambda_neighborhood *
                                                          std::exp(-(vector_neighbors[0] -
                                                                     keypoint.point)
                                                                        .norm() /
                                                                   (kMaxPointToPlane *
                                                                    options_.min_number_neighbors));

                    double point_to_plane_dist;
                    std::set<voxel> neighbor_voxels;
                    for (int i(0); i < options_.num_closest_neighbors; ++i)
                    {
                         point_to_plane_dist = (keypoint.point - vector_neighbors[i]).transpose() * neighborhood.normal;

                         if (std::abs(point_to_plane_dist) < options_.max_dist_to_plane_icp)
                         {
                              num_residuals++;

                              Eigen::Vector3d norm_vector = neighborhood.normal;
                              norm_vector.normalize();

                              double norm_offset = -norm_vector.dot(vector_neighbors[i]);

                              PointType coeff;
                              coeff.x = norm_vector[0], coeff.y = norm_vector[1], coeff.z = norm_vector[2];
                              coeff.intensity = point_to_plane_dist;

                              coeffSel_tmpt.push_back(coeff);

                              // Eigen::Vector3d point_end = p_frame->p_state->rotation.inverse() * (keypoints[k].point - p_frame->p_state->translation);
                              // Eigen::Vector3d point_end = state.rot_end.inverse() * (keypoint.point - state.pos_end);
                              Eigen::Vector3d point_end = raw_point;

                              PointType pt;
                              pt.x = point_end[0], pt.y = point_end[1], pt.z = point_end[2];
                              pt.intensity = weight;
                              pointSel_tmpt.push_back(pt);
                         }
                    }
               }

               if (num_residuals < 20)
                    std::cout << "No Enough Effective points! May catch error." << std::endl;

               Eigen::MatrixXd Hsub(num_residuals, 12);
               Eigen::VectorXd meas_vec(num_residuals);
               Hsub.setZero();

               for (int i = 0; i < num_residuals; i++)
               {
                    const PointType &laser_p = pointSel_tmpt[i];
                    double weight = laser_p.intensity;
                    Eigen::Vector3d point_this_be(laser_p.x, laser_p.y, laser_p.z);
                    Eigen::Matrix3d point_be_crossmat;
                    point_be_crossmat << SKEW_SYM_MATRX(point_this_be);

                    Eigen::Vector3d point_this = state.R_L_I * point_this_be + state.T_L_I;
                    Eigen::Matrix3d point_crossmat;
                    point_crossmat << SKEW_SYM_MATRX(point_this);

                    /*** get the normal vector of closest surface/corner ***/
                    const PointType &norm_p = coeffSel_tmpt[i];
                    Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);

                    /*** calculate the Measuremnt Jacobian matrix H ***/
                    Eigen::Vector3d C(state.rot_end.transpose() * norm_vec * weight);
                    Eigen::Vector3d A(point_crossmat * C);

                    if (options_.extrinsic_est_en)
                    {
                         Eigen::Vector3d B(point_be_crossmat * state.R_L_I.transpose() * C);
                         Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x * weight, norm_p.y * weight, norm_p.z * weight, VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
                    }
                    else
                         Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x * weight, norm_p.y * weight, norm_p.z * weight, 0, 0, 0, 0, 0, 0;

                    /*** Measuremnt: distance to the closest surface/corner ***/
                    meas_vec(i) = -norm_p.intensity * weight;
               }

               Eigen::Vector3d rot_add, t_add;
               Eigen::Matrix<double, DIM_OF_STATES, 1> solution;
               Eigen::MatrixXd K(DIM_OF_STATES, num_residuals);

               /*** Iterative Kalman Filter Update ***/
               if (!flg_EKF_inited)
               {
                    /*** only run in initialization period ***/
                    Eigen::MatrixXd H_init(Eigen::Matrix<double, 15, DIM_OF_STATES>::Zero());
                    Eigen::MatrixXd z_init(Eigen::Matrix<double, 15, 1>::Zero());
                    H_init.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
                    H_init.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
                    H_init.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
                    H_init.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
                    H_init.block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();
                    z_init.block<3, 1>(0, 0) = -Log(state.rot_end);
                    z_init.block<3, 1>(3, 0) = -state.pos_end;
                    z_init.block<3, 1>(6, 0) = -Log(state.R_L_I);
                    z_init.block<3, 1>(9, 0) = -state.T_L_I;

                    auto H_init_T = H_init.transpose();
                    auto &&K_init = state.cov * H_init_T * (H_init * state.cov * H_init_T + 0.0001 * Eigen::Matrix<double, 12, 12>::Identity()).inverse(); //  公式20）
                    solution = K_init * z_init;

                    solution.block<12, 1>(0, 0).setZero();
                    state += solution;
                    state.cov = (Eigen::MatrixXd::Identity(DIM_OF_STATES, DIM_OF_STATES) - K_init * H_init) * state.cov; //  公式19）
               }
               else
               {
                    auto &&Hsub_T = Hsub.transpose();
                    H_T_H.block<12, 12>(0, 0) = Hsub_T * Hsub;
                    //  公式(20)上下同时化简R可得
                    Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> &&K_1 =
                        (H_T_H + (state.cov / LASER_POINT_COV).inverse()).inverse(); //  由于laser point互相独立，R是对角矩阵，即这里的 LASER_POINT_COV
                    K = K_1.block<DIM_OF_STATES, 12>(0, 0) * Hsub_T;                 //  公式20）  ESKF 卡尔曼增益  ---  [3]

                    auto vec = state_propagat - state;

                    solution = K * meas_vec + vec - K * Hsub * vec.block<12, 1>(0, 0);
                    state += solution; //   公式18）  ESKF 更新估计值 --- [4]

                    rot_add = solution.block<3, 1>(0, 0);
                    t_add = solution.block<3, 1>(3, 0);

                    flg_EKF_converged = false;

                    if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.1)) //   0.01度, 0.1cm->1mm
                    {
                         flg_EKF_converged = true;
                    }

                    deltaR = rot_add.norm() * 57.3;
                    deltaT = t_add.norm();
               }

               // std::cout << "dR & dT: " << deltaR << ", " << deltaT << std::endl;

               if (flg_EKF_converged || iterCount == options_.num_max_iterations - 1)
               {
                    /*** Covariance Update ***/
                    G.block<DIM_OF_STATES, 12>(0, 0) = K * Hsub;
                    state.cov = (I_STATE - G) * state.cov; //   公式19）  ESKF  估计值和真值的后验协方差   -----  [5]
                    total_distance += (state.pos_end - position_last).norm();
                    position_last = state.pos_end;
                    // if (options_.log_print)
                    std::cout << ANSI_COLOR_YELLOW << "position: " << state.pos_end.transpose() << " total distance: " << total_distance
                              << ", iterCount: " << iterCount << ", ekf_converged: " << flg_EKF_converged << ANSI_COLOR_RESET << std::endl;

                    break;
               }
          }

          std::vector<point3D>().swap(surf_keypoints);

          //   transpose point before added
          transformKeypoints(measures_.lidar_);
     }

     Neighborhood lidarodom::computeNeighborhoodDistribution(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points)
     {
          Neighborhood neighborhood;
          // Compute the normals
          Eigen::Vector3d barycenter(Eigen::Vector3d(0, 0, 0));
          for (auto &point : points)
          {
               barycenter += point;
          }

          barycenter /= (double)points.size();
          neighborhood.center = barycenter;

          Eigen::Matrix3d covariance_Matrix(Eigen::Matrix3d::Zero());
          for (auto &point : points)
          {
               for (int k = 0; k < 3; ++k)
                    for (int l = k; l < 3; ++l)
                         covariance_Matrix(k, l) += (point(k) - barycenter(k)) *
                                                    (point(l) - barycenter(l));
          }
          covariance_Matrix(1, 0) = covariance_Matrix(0, 1);
          covariance_Matrix(2, 0) = covariance_Matrix(0, 2);
          covariance_Matrix(2, 1) = covariance_Matrix(1, 2);
          neighborhood.covariance = covariance_Matrix;
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance_Matrix);
          Eigen::Vector3d normal(es.eigenvectors().col(0).normalized());
          neighborhood.normal = normal;

          double sigma_1 = sqrt(std::abs(es.eigenvalues()[2]));
          double sigma_2 = sqrt(std::abs(es.eigenvalues()[1]));
          double sigma_3 = sqrt(std::abs(es.eigenvalues()[0]));
          neighborhood.a2D = (sigma_2 - sigma_3) / sigma_1;

          if (neighborhood.a2D != neighborhood.a2D)
          {
               throw std::runtime_error("error");
          }

          return neighborhood;
     }

     ///  ===================  for search neighbor  ===================================================
     using pair_distance_t = std::tuple<double, Eigen::Vector3d, voxel>;

     struct comparator
     {
          bool operator()(const pair_distance_t &left, const pair_distance_t &right) const
          {
               return std::get<0>(left) < std::get<0>(right);
          }
     };

     using priority_queue_t = std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, comparator>;

     std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
     lidarodom::searchNeighbors(const voxelHashMap &map, const Eigen::Vector3d &point,
                                int nb_voxels_visited, double size_voxel_map,
                                int max_num_neighbors, int threshold_voxel_capacity,
                                std::vector<voxel> *voxels)
     {

          if (voxels != nullptr)
               voxels->reserve(max_num_neighbors);

          short kx = static_cast<short>(point[0] / size_voxel_map);
          short ky = static_cast<short>(point[1] / size_voxel_map);
          short kz = static_cast<short>(point[2] / size_voxel_map);

          priority_queue_t priority_queue;

          voxel voxel_temp(kx, ky, kz);
          for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx)
          {
               for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy)
               {
                    for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz)
                    {
                         voxel_temp.x = kxx;
                         voxel_temp.y = kyy;
                         voxel_temp.z = kzz;

                         auto search = map.find(voxel_temp);
                         if (search != map.end())
                         {
                              const auto &voxel_block = search.value();
                              if (voxel_block.NumPoints() < threshold_voxel_capacity)
                                   continue;
                              for (int i(0); i < voxel_block.NumPoints(); ++i)
                              {
                                   auto &neighbor = voxel_block.points[i];
                                   double distance = (neighbor - point).norm();
                                   if (priority_queue.size() == max_num_neighbors)
                                   {
                                        if (distance < std::get<0>(priority_queue.top()))
                                        {
                                             priority_queue.pop();
                                             priority_queue.emplace(distance, neighbor, voxel_temp);
                                        }
                                   }
                                   else
                                        priority_queue.emplace(distance, neighbor, voxel_temp);
                              }
                         }
                    }
               }
          }

          auto size = priority_queue.size();
          std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> closest_neighbors(size);
          if (voxels != nullptr)
          {
               voxels->resize(size);
          }
          for (auto i = 0; i < size; ++i)
          {
               closest_neighbors[size - 1 - i] = std::get<1>(priority_queue.top());
               if (voxels != nullptr)
                    (*voxels)[size - 1 - i] = std::get<2>(priority_queue.top());
               priority_queue.pop();
          }

          return closest_neighbors;
     }

     void lidarodom::addPointToMap(voxelHashMap &map, const Eigen::Vector3d &point,
                                   const double &intensity, double voxel_size,
                                   int max_num_points_in_voxel, double min_distance_points,
                                   int min_num_points)
     {
          short kx = static_cast<short>(point[0] / voxel_size);
          short ky = static_cast<short>(point[1] / voxel_size);
          short kz = static_cast<short>(point[2] / voxel_size);

          voxelHashMap::iterator search = map.find(voxel(kx, ky, kz));

          if (search != map.end())
          {
               auto &voxel_block = (search.value());

               if (!voxel_block.IsFull())
               {
                    double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
                    for (int i(0); i < voxel_block.NumPoints(); ++i)
                    {
                         auto &_point = voxel_block.points[i];
                         double sq_dist = (_point - point).squaredNorm();
                         if (sq_dist < sq_dist_min_to_points)
                         {
                              sq_dist_min_to_points = sq_dist;
                         }
                    }
                    if (sq_dist_min_to_points > (min_distance_points * min_distance_points))
                    {
                         if (min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points)
                         {
                              voxel_block.AddPoint(point);
                         }
                    }
               }
          }
          else
          {
               if (min_num_points <= 0)
               {
                    voxelBlock block(max_num_points_in_voxel);
                    block.AddPoint(point);
                    map[voxel(kx, ky, kz)] = std::move(block);
               }
          }
          addPointToPcl(points_world, point, intensity);
     }

     void lidarodom::addPointToPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points, const Eigen::Vector3d &point, const double &intensity)
     {
          pcl::PointXYZI cloudTemp;

          cloudTemp.x = point.x();
          cloudTemp.y = point.y();
          cloudTemp.z = point.z();
          cloudTemp.intensity = intensity;
          // cloudTemp.intensity = 50 * (point.z() - p_frame->p_state->translation.z());
          pcl_points->points.push_back(cloudTemp);
     }

     void lidarodom::map_incremental(int min_num_points)
     {
          //   only surf
          for (const auto &point : measures_.lidar_)
          {
               addPointToMap(voxel_map, point.point, point.intensity,
                             options_.size_voxel_map, options_.max_num_points_in_voxel,
                             options_.min_distance_points, min_num_points);
          }

          {
               std::string laser_topic = "laser";
               pub_cloud_to_ros(laser_topic, points_world, measures_.lidar_end_time_);
          }
          points_world->clear();
     }

     void lidarodom::lasermap_fov_segment()
     {
          //   use predict pose here
          Eigen::Vector3d location = state.pos_end;

          std::vector<voxel> voxels_to_erase;
          for (auto &pair : voxel_map)
          {
               Eigen::Vector3d pt = pair.second.points[0];
               if ((pt - location).squaredNorm() > (options_.max_distance * options_.max_distance))
               {
                    voxels_to_erase.push_back(pair.first);
               }
          }
          for (auto &vox : voxels_to_erase)
               voxel_map.erase(vox);
          std::vector<voxel>().swap(voxels_to_erase);
     }

     std::vector<MeasureGroup> lidarodom::getMeasureMents()
     {
          std::vector<MeasureGroup> measurements;
          while (true)
          {
               if (imu_buffer_.empty())
                    return measurements;

               if (lidar_buffer_.empty())
                    return measurements;

               if (imu_buffer_.back()->timestamp_ - time_curr < delay_time_)
                    return measurements;

               MeasureGroup meas;

               meas.lidar_ = lidar_buffer_.front();
               meas.lidar_begin_time_ = time_buffer_.front().first;
               meas.lidar_end_time_ = meas.lidar_begin_time_ + time_buffer_.front().second;

               lidar_buffer_.pop_front();
               time_buffer_.pop_front();

               time_curr = meas.lidar_end_time_;

               double imu_time = imu_buffer_.front()->timestamp_;
               meas.imu_.clear();
               while ((!imu_buffer_.empty()) && (imu_time <= meas.lidar_end_time_))
               {
                    imu_time = imu_buffer_.front()->timestamp_;
                    if (imu_time > meas.lidar_end_time_)
                    {
                         break;
                    }
                    meas.imu_.push_back(imu_buffer_.front());
                    imu_buffer_.pop_front();
               }

               // if (!imu_buffer_.empty())
               //      meas.imu_.push_back(imu_buffer_.front()); //   added for Interp

               measurements.push_back(meas);
          }
     }

     void lidarodom::visualize_state()
     {
          std::ifstream stat_stream("/proc/self/statm", std::ios_base::in);
          long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024;
          unsigned long vm = 0, rss = 0;
          stat_stream >> vm >> rss;
          stat_stream.close();
          double vm_usage = vm * page_size_kb / 1024.0;
          double resident_set = rss * page_size_kb / 1024.0;
          total_computation += (omp_get_wtime() - curr_time);
          opt_cnt += 1;
          std::system("clear");

          std::cout << "\033[1;36m"
                    << "**** Multiple Asynchronous LiDAR Inertial Odometry (MA-LIO) ****"
                    << "\033[0m" << std::endl;
          std::cout << std::endl;
          /*std::cout.precision(20);
          std::cout << "\033[1;33m"
                    << "[Timestamp of Current State]: "
                    << "\033[0m"
                    << "\033[1;32m" << Measures.lidar_end_time[2] << " secs"
                                                                     "\033[0m"
                    << std::endl;
          std::cout.precision(5);
          std::cout << "\033[1;33m"
                    << "[Position]: "
                    << "\033[0m"
                    << "\033[1;32m"
                    << "\n\t[x]: " << state_point.pos(0) << " meter"
                    << "\n\t[y]: " << state_point.pos(1) << " meter"
                    << "\n\t[z]: " << state_point.pos(2) << " meter"
                    << "\033[0m" << std::endl;

          std::cout << "\033[1;33m"
                    << "[Orientation]: "
                    << "\033[0m"
                    << "\033[1;32m"
                    << "\n\t[qx]: " << geoQuat.x
                    << "\n\t[qy]: " << geoQuat.y
                    << "\n\t[qz]: " << geoQuat.z
                    << "\n\t[qw]: " << geoQuat.w << "\033[0m" << std::endl;

          std::cout << "\033[1;33m"
                    << "[Velocity]: "
                    << "\033[0m"
                    << "\033[1;32m"
                    << "\n\t[x]: " << state_point.vel(0) << " m/s"
                    << "\n\t[y]: " << state_point.vel(1) << " m/s"
                    << "\n\t[z]: " << state_point.vel(2) << " m/s"
                    << "\033[0m" << std::endl;*/

          /*for (int num = 0; num < lid_num; num++)
          {
              V3D extrinsic_t = *(static_cast<MTK::vect<3, double> *>(state_point.vect_state_ptr[1 + num]));
              Eigen::Quaterniond extrinsic_q = *(static_cast<MTK::SO3<double> *>(state_point.SO3_state_ptr[1 + num]));
              std::cout.precision(20);
              std::cout << "\033[1;33m"
                        << "[LiDAR-IMU Extrinsic " << num << "]: "
                        << "\033[0m"
                        << "\033[1;32m"
                        << "\n\tTimestamp for the Lastest Point: "
                        << Measures.lidar_end_time[lid_num - 1 - num] << " secs";
              std::cout.precision(5);
              std::cout << "\033[1;32m"
                        << "\n\tTranslation: "
                        << "\n\t\t[x]: " << extrinsic_t(0) << " meter"
                        << "\n\t\t[y]: " << extrinsic_t(1) << " meter"
                        << "\n\t\t[z]: " << extrinsic_t(2) << " meter"
                        << "\n\tRotation: "
                        << "\n\t\t[qx]: " << extrinsic_q.x()
                        << "\n\t\t[qy]: " << extrinsic_q.y()
                        << "\n\t\t[qz]: " << extrinsic_q.z()
                        << "\n\t\t[qw]: " << extrinsic_q.w() << "\033[0m" << std::endl;
          }

          for (int num = 0; num < lid_num - 1; num++)
          {
              std::cout << "\033[1;33m"
                        << "[Temporal discrepancy between LiDAR " << num << " and " << num + 1 << "]: "
                        << "\033[0m"
                        << "\033[1;32m" << (Measures.lidar_end_time[lid_num - 1 - num] - Measures.lidar_end_time[lid_num - 1 - (num + 1)]) * 1000 << " msecs"
                        << "\033[0m" << std::endl;
          }

          total_distance += sqrt((state_point.pos(0) - prev_pos(0)) * (state_point.pos(0) - prev_pos(0)) + (state_point.pos(1) - prev_pos(1)) * (state_point.pos(1) - prev_pos(1)) + (state_point.pos(2) - prev_pos(2)) * (state_point.pos(2) - prev_pos(2)));
          std::cout << "\033[1;33m"
                    << "[Total Distance]: "
                    << "\033[0m"
                    << "\033[1;32m" << total_distance << " meter"
                    << "\033[0m" << std::endl;
          std::cout << "\033[1;33m"
                    << "[Duration]: "
                    << "\033[0m"
                    << "\033[1;32m" << Measures.lidar_end_time[lid_num - 1] - first_lidar_time << " secs"
                    << "\033[0m" << std::endl;
          std::cout << "\033[1;33m"
                    << "[Map Point Count (Downsample w/ " << filter_size_map_min << ")]: "
                    << "\033[0m"
                    << "\033[1;32m" << ikdtree.size() << "\033[0m" << std::endl;*/
          std::cout << "\033[1;33m"
                    << "[Computation Time]: "
                    << "\033[0m"
                    << "\033[1;32m" << (omp_get_wtime() - curr_time) * 1000 << " msecs"
                    << "\033[0m" << std::endl;
          std::cout << "\033[1;33m"
                    << "[Computation Time (Avg)]: "
                    << "\033[0m"
                    << "\033[1;32m" << total_computation / opt_cnt * 1000 << " msecs"
                    << "\033[0m" << std::endl;
          std::cout << "\033[1;33m"
                    << "[RAM Allocation]: "
                    << "\033[0m"
                    << "\033[1;32m" << resident_set << " MB"
                    << "\033[0m" << std::endl;
          std::cout << "\033[1;33m"
                    << "[VSZ]: "
                    << "\033[0m"
                    << "\033[1;32m" << vm_usage << " MB"
                    << "\033[0m"
                    << "\n"
                    << std::endl;
     }
}