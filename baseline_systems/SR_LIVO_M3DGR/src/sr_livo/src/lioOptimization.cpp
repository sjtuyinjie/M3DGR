#include "lioOptimization.h"

cloudFrame::cloudFrame(std::vector<point3D> &point_frame_, state *p_state_)
{
    point_frame.insert(point_frame.end(), point_frame_.begin(), point_frame_.end());

    p_state = p_state_;
}

cloudFrame::cloudFrame(cloudFrame *p_cloud_frame)
{
    time_sweep_begin = p_cloud_frame->time_sweep_begin;
    time_sweep_end = p_cloud_frame->time_sweep_end;
    time_frame_begin = p_cloud_frame->time_frame_begin;
    time_frame_end = p_cloud_frame->time_frame_end;

    id = p_cloud_frame->id;
    sub_id = p_cloud_frame->sub_id;
    frame_id = p_cloud_frame->frame_id;

    p_state = p_cloud_frame->p_state;

    point_frame.insert(point_frame.end(), p_cloud_frame->point_frame.begin(), p_cloud_frame->point_frame.end());

    offset_begin = p_cloud_frame->offset_begin;
    offset_end = p_cloud_frame->offset_end;
    dt_offset = p_cloud_frame->dt_offset;
}

void cloudFrame::release()
{
    std::vector<point3D>().swap(point_frame);

    if(p_state != nullptr)
       p_state->release();

    delete p_state;

    p_state = nullptr;

    if (!rgb_image.empty())
        rgb_image.release();

    if (!gray_image.empty())
        gray_image.release();
}

bool cloudFrame::if2dPointsAvailable(const double &u, const double &v, const double &scale, double fov_mar)
{
    double used_fov_margin = p_state->fov_margin;

    if (fov_mar > 0.0)
        used_fov_margin = fov_mar;

    if ((u / scale >= (used_fov_margin * image_cols + 1)) && (std::ceil(u / scale) < ((1 - used_fov_margin) * image_cols)) &&
        (v / scale >= (used_fov_margin * image_rows + 1)) && (std::ceil(v / scale) < ((1 - used_fov_margin) * image_rows)))
        return true;
    else
        return false;
}

bool cloudFrame::getRgb(const double &u, const double &v, int &r, int &g, int &b)
{
    r = rgb_image.at<cv::Vec3b>(v, u)[2];
    g = rgb_image.at<cv::Vec3b>(v, u)[1];
    b = rgb_image.at<cv::Vec3b>(v, u)[0];

    return true;
}

template<typename T>
inline T getSubPixel(cv::Mat & mat, const double & row, const  double & col, double pyramid_layer = 0)
{
    int floor_row = floor(row);
    int floor_col = floor(col);

    double frac_row = row - floor_row;
    double frac_col = col - floor_col;

    int ceil_row = floor_row + 1;
    int ceil_col = floor_col + 1;

    if (pyramid_layer != 0)
    {
        int pos_bias = pow(2, pyramid_layer - 1);

        floor_row -= pos_bias;
        floor_col -= pos_bias;
        ceil_row += pos_bias;
        ceil_row += pos_bias;
    }

    return ((1.0 - frac_row) * (1.0 - frac_col) * (T)mat.ptr<T>(floor_row)[floor_col]) +
               (frac_row * (1.0 - frac_col) * (T)mat.ptr<T>(ceil_row)[floor_col]) +
               ((1.0 - frac_row) * frac_col * (T)mat.ptr<T>(floor_row)[ceil_col]) +
               (frac_row * frac_col * (T)mat.ptr<T>(ceil_row)[ceil_col]);
}

Eigen::Vector3d cloudFrame::getRgb(double &u, double &v, int layer, Eigen::Vector3d *rgb_dx, Eigen::Vector3d *rgb_dy)
{
    const int ssd = 5;

    cv::Vec3b rgb = getSubPixel<cv::Vec3b>(rgb_image, v, u, layer);

    if (rgb_dx != nullptr)
    {
        cv::Vec3f rgb_left(0, 0, 0), rgb_right(0, 0, 0);

        float pixel_dif = 0;

        for (int bias_idx = 1; bias_idx < ssd; bias_idx++ )
        {
            rgb_left += getSubPixel<cv::Vec3b>(rgb_image, v, u - bias_idx, layer);
            rgb_right += getSubPixel<cv::Vec3b>(rgb_image, v, u + bias_idx, layer);
            pixel_dif += 2 * bias_idx;
        }

        cv::Vec3f cv_rgb_dx = rgb_right - rgb_left;
        *rgb_dx = Eigen::Vector3d(cv_rgb_dx(0), cv_rgb_dx(1), cv_rgb_dx(2)) / pixel_dif;
    }

    if (rgb_dy != nullptr)
    {
        cv::Vec3f rgb_down(0, 0, 0), rgb_up(0, 0, 0);

        float pixel_dif = 0;

        for (int bias_idx = 1; bias_idx < ssd; bias_idx++)
        {
            rgb_down += getSubPixel<cv::Vec3b>(rgb_image, v - bias_idx, u, layer);
            rgb_up += getSubPixel<cv::Vec3b>(rgb_image, v + bias_idx, u, layer);
            pixel_dif += 2 * bias_idx;
        }

        cv::Vec3f cv_rgb_dy = rgb_up - rgb_down;
        *rgb_dy = Eigen::Vector3d(cv_rgb_dy(0), cv_rgb_dy(1), cv_rgb_dy(2)) / pixel_dif;
    }

    return Eigen::Vector3d(rgb(0), rgb(1), rgb(2));
}

bool cloudFrame::project3dTo2d(const pcl::PointXYZI &point_in, double &u, double &v, const double &scale)
{
    Eigen::Vector3d point_world(point_in.x, point_in.y, point_in.z);

    Eigen::Vector3d point_camera = p_state->q_camera_world.toRotationMatrix() * point_world + p_state->t_camera_world;

    // std::cout << "q_world_camera = " << q_world_camera.w() << " " << q_world_camera.x() << " " << q_world_camera.y() << " " << q_world_camera.z() << std::endl;
    // std::cout << "t_world_camera = " << t_world_camera.x() << " " << t_world_camera.y() << " " << t_world_camera.z() << std::endl;
    // std::cout << "point_world = " << point_world.x() << " " << point_world.y() << " " << point_world.z() << std::endl;

    if (point_camera(2, 0) < 0.001)
    {
        return false;
    }

    u = (point_camera(0) * p_state->fx / point_camera(2) + p_state->cx) * scale;
    v = (point_camera(1) * p_state->fy / point_camera(2) + p_state->cy) * scale;

    return true;
}

bool cloudFrame::project3dPointInThisImage(const pcl::PointXYZI &point_in, double &u, double &v, pcl::PointXYZRGB *rgb_point, double intrinsic_scale)
{
    if (project3dTo2d(point_in, u, v, intrinsic_scale) == false)
        return false;

    if (if2dPointsAvailable(u, v, intrinsic_scale) == false)
        return false;

    if (rgb_point != nullptr)
    {
        int r = 0;
        int g = 0;
        int b = 0;
        getRgb(u, v, r, g, b);
        rgb_point->x = point_in.x;
        rgb_point->y = point_in.y;
        rgb_point->z = point_in.z;
        rgb_point->r = r;
        rgb_point->g = g;
        rgb_point->b = b;
        rgb_point->a = 255;
    }

    return true;
}

bool cloudFrame::project3dPointInThisImage(const Eigen::Vector3d &point_in, double &u, double &v, pcl::PointXYZRGB *rgb_point, double intrinsic_scale)
{
    pcl::PointXYZI temp_point;
    temp_point.x = point_in(0);
    temp_point.y = point_in(1);
    temp_point.z = point_in(2);

    // std::cout << "temp_point = " << temp_point.x << " " << temp_point.y << " " << temp_point.z << std::endl;

    return project3dPointInThisImage(temp_point, u, v, rgb_point, intrinsic_scale);
}

void cloudFrame::refreshPoseForProjection()
{
    p_state->q_camera_world = p_state->q_world_camera.inverse();
    p_state->t_camera_world = - p_state->q_camera_world.toRotationMatrix() * p_state->t_world_camera;
}

estimationSummary::estimationSummary()
{

}

void estimationSummary::release()
{

}

lioOptimization::lioOptimization()
{
	allocateMemory();

    readParameters();

    initialValue();

    pub_cloud_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_current", 2);
    pub_cloud_world = nh.advertise<sensor_msgs::PointCloud2>("/cloud_global_map", 2);
    pub_odom = nh.advertise<nav_msgs::Odometry>("/Odometry_after_opt", 5);
    pub_path = nh.advertise<nav_msgs::Path>("/path", 5);
    pub_cloud_color = nh.advertise<sensor_msgs::PointCloud2>("/color_global_map", 2);
    pub_cloud_color_vec.resize(1000);

    if (cloud_pro->getLidarType() == LIVOX)
        sub_cloud_ori = nh.subscribe<livox_ros_driver::CustomMsg>(lidar_topic, 20, &lioOptimization::livoxHandler, this);
    else
        sub_cloud_ori = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 20, &lioOptimization::standardCloudHandler, this);

    sub_imu_ori = nh.subscribe<sensor_msgs::Imu>(imu_topic, 500, &lioOptimization::imuHandler, this);

    if (image_type == RGB8)
        sub_img_ori = nh.subscribe(image_topic, 20, &lioOptimization::imageHandler, this);
    else if (image_type == COMPRESSED)
        sub_img_ori = nh.subscribe(image_topic, 20, &lioOptimization::compressedImageHandler, this);

    path.header.stamp = ros::Time::now();
    path.header.frame_id ="camera_init";
    points_world.reset(new pcl::PointCloud<pcl::PointXYZI>());

    //odometry_options.recordParameters();
    //map_options.recordParameters();
}

void lioOptimization::readParameters()
{	
    int para_int;
    double para_double;
    bool para_bool;
    std::string str_temp;

    // common
    nh.param<std::string>("common/lidar_topic", lidar_topic, "/points_raw");
	nh.param<std::string>("common/imu_topic", imu_topic, "/imu_raw");
    nh.param<std::string>("common/image_topic", image_topic, "/image_raw");
    nh.param<std::string>("common/image_type", str_temp, "RGB8");
    if(str_temp == "RGB8") image_type = RGB8;
    else if(str_temp == "COMPRESSED") image_type = COMPRESSED;
    else std::cout << "The `image type` " << str_temp << " is not supported." << std::endl;

    nh.param<int>("common/point_filter_num", para_int, 1);  cloud_pro->setPointFilterNum(para_int);
    nh.param<std::vector<double>>("common/gravity_acc", v_G, std::vector<double>());
    nh.param<bool>("debug_output", debug_output, false);
    nh.param<std::string>("output_path", output_path, "");

    // LiDAR parameter
    nh.param<int>("lidar_parameter/lidar_type", para_int, LIVOX);  cloud_pro->setLidarType(para_int);
    nh.param<int>("lidar_parameter/N_SCANS", para_int, 16);  cloud_pro->setNumScans(para_int);
    nh.param<int>("lidar_parameter/SCAN_RATE", para_int, 10);  cloud_pro->setScanRate(para_int);
    nh.param<int>("lidar_parameter/time_unit", para_int, US);  cloud_pro->setTimeUnit(para_int);
    nh.param<double>("lidar_parameter/blind", para_double, 0.01);  cloud_pro->setBlind(para_double);
    nh.param<float>("lidar_parameter/det_range", det_range, 300.f);
    nh.param<double>("lidar_parameter/fov_degree", fov_deg, 180);

    // IMU parameter
    nh.param<double>("imu_parameter/acc_cov", para_double, 0.1);  eskf_pro->setAccCov(para_double);
    nh.param<double>("imu_parameter/gyr_cov", para_double, 0.1);  eskf_pro->setGyrCov(para_double);
    nh.param<double>("imu_parameter/b_acc_cov", para_double, 0.0001);  eskf_pro->setBiasAccCov(para_double);
    nh.param<double>("imu_parameter/b_gyr_cov", para_double, 0.0001);  eskf_pro->setBiasGyrCov(para_double);

    nh.param<bool>("imu_parameter/time_diff_enable", time_diff_enable, false);

    // camera parameter
    nh.param<int>("camera_parameter/image_width", para_int, 640); img_pro->setImageWidth(para_int);
    nh.param<int>("camera_parameter/image_height", para_int, 480); img_pro->setImageHeight(para_int);
    nh.param<std::vector<double>>("camera_parameter/camera_intrinsic", v_camera_intrinsic, std::vector<double>());
    nh.param<std::vector<double>>("camera_parameter/camera_dist_coeffs", v_camera_dist_coeffs, std::vector<double>());

    // extrinsic parameter
    nh.param<bool>("extrinsic_parameter/extrinsic_enable", extrin_enable, true);
    nh.param<std::vector<double>>("extrinsic_parameter/extrinsic_t_imu_lidar", v_extrin_t_imu_lidar, std::vector<double>());
    nh.param<std::vector<double>>("extrinsic_parameter/extrinsic_R_imu_lidar", v_extrin_R_imu_lidar, std::vector<double>());
    nh.param<std::vector<double>>("extrinsic_parameter/extrinsic_t_imu_camera", v_extrin_t_imu_camera, std::vector<double>());
    nh.param<std::vector<double>>("extrinsic_parameter/extrinsic_R_imu_camera", v_extrin_R_imu_camera, std::vector<double>());

    // state estimation parameters
    nh.param<double>("odometry_options/init_voxel_size", odometry_options.init_voxel_size, 0.2);
    nh.param<double>("odometry_options/init_sample_voxel_size", odometry_options.init_sample_voxel_size, 1.0);
    nh.param<int>("odometry_options/init_num_frames", odometry_options.init_num_frames, 20);
    nh.param<double>("odometry_options/voxel_size", odometry_options.voxel_size, 0.5);
    nh.param<double>("odometry_options/sample_voxel_size", odometry_options.sample_voxel_size, 1.5);
    nh.param<double>("odometry_options/max_distance", odometry_options.max_distance, 100.0);
    nh.param<int>("odometry_options/max_num_points_in_voxel", odometry_options.max_num_points_in_voxel, 20);
    nh.param<double>("odometry_options/min_distance_points", odometry_options.min_distance_points, 0.1);
    nh.param<double>("odometry_options/distance_error_threshold", odometry_options.distance_error_threshold, 5.0);

    nh.param<std::string>("odometry_options/motion_compensation", str_temp, "CONSTANT_VELOCITY");
    if(str_temp == "IMU") odometry_options.motion_compensation = IMU;
    else if(str_temp == "CONSTANT_VELOCITY") odometry_options.motion_compensation = CONSTANT_VELOCITY;
    else std::cout << "The `motion_compensation` " << str_temp << " is not supported." << std::endl;

    nh.param<std::string>("odometry_options/initialization", str_temp, "INIT_IMU");
    if(str_temp == "INIT_IMU") odometry_options.initialization = INIT_IMU;
    else if(str_temp == "INIT_CONSTANT_VELOCITY") odometry_options.initialization = INIT_CONSTANT_VELOCITY;
    else std::cout << "The `state_initialization` " << str_temp << " is not supported." << std::endl;


    icpOptions optimize_options;
    nh.param<int>("icp_options/threshold_voxel_occupancy", odometry_options.optimize_options.threshold_voxel_occupancy, 1);
    nh.param<double>("icp_options/size_voxel_map", odometry_options.optimize_options.size_voxel_map, 1.0);
    nh.param<int>("icp_options/num_iters_icp", odometry_options.optimize_options.num_iters_icp, 5);
    nh.param<int>("icp_options/min_number_neighbors", odometry_options.optimize_options.min_number_neighbors, 20);
    nh.param<int>("icp_options/voxel_neighborhood", odometry_options.optimize_options.voxel_neighborhood, 1);
    nh.param<double>("icp_options/power_planarity", odometry_options.optimize_options.power_planarity, 2.0);
    nh.param<bool>("icp_options/estimate_normal_from_neighborhood", odometry_options.optimize_options.estimate_normal_from_neighborhood, true);
    nh.param<int>("icp_options/max_number_neighbors", odometry_options.optimize_options.max_number_neighbors, 20);
    nh.param<double>("icp_options/max_dist_to_plane_icp", odometry_options.optimize_options.max_dist_to_plane_icp, 0.3);
    nh.param<double>("icp_options/threshold_orientation_norm", odometry_options.optimize_options.threshold_orientation_norm, 0.0001);
    nh.param<double>("icp_options/threshold_translation_norm", odometry_options.optimize_options.threshold_translation_norm, 0.001);
    nh.param<int>("icp_options/max_num_residuals", odometry_options.optimize_options.max_num_residuals, -1);
    nh.param<int>("icp_options/min_num_residuals", odometry_options.optimize_options.min_num_residuals, 100);
    nh.param<int>("icp_options/num_closest_neighbors", odometry_options.optimize_options.num_closest_neighbors, 1);
    nh.param<double>("icp_options/weight_alpha", odometry_options.optimize_options.weight_alpha, 0.9);
    nh.param<double>("icp_options/weight_neighborhood", odometry_options.optimize_options.weight_neighborhood, 0.1);
    nh.param<bool>("icp_options/debug_print", odometry_options.optimize_options.debug_print, true);
    nh.param<bool>("icp_options/debug_viz", odometry_options.optimize_options.debug_viz, false);

    nh.param<double>("map_options/size_voxel_map", map_options.size_voxel_map, 0.1);
    nh.param<int>("map_options/max_num_points_in_voxel", map_options.max_num_points_in_voxel, 20);
    nh.param<double>("map_options/min_distance_points", map_options.min_distance_points, 0.01);
    nh.param<int>("map_options/add_point_step", map_options.add_point_step, 4);
    nh.param<int>("map_options/pub_point_minimum_views", map_options.pub_point_minimum_views, 3);
}

void lioOptimization::allocateMemory()
{
    cloud_pro = new cloudProcessing();
    eskf_pro = new eskfEstimator();
    img_pro = new imageProcessing();

    down_cloud_body.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    down_cloud_world.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
}

void lioOptimization::initialValue()
{
    laser_point_cov = 0.001;

    G = vec3FromArray(v_G);
    G_norm = G.norm();
    R_imu_lidar = mat33FromArray(v_extrin_R_imu_lidar);
    t_imu_lidar = vec3FromArray(v_extrin_t_imu_lidar);
    R_imu_camera = mat33FromArray(v_extrin_R_imu_camera);
    t_imu_camera = vec3FromArray(v_extrin_t_imu_camera);

    cloud_pro->setExtrinR(R_imu_lidar);
    cloud_pro->setExtrinT(t_imu_lidar);

    img_pro->setCameraIntrinsic(v_camera_intrinsic);
    img_pro->setCameraDistCoeffs(v_camera_dist_coeffs);
    img_pro->setExtrinR(R_imu_camera);
    img_pro->setExtrinT(t_imu_camera);

    dt_sum = 0;

    last_time_lidar = -1.0;
    last_time_imu = -1.0;
    last_time_img = -1.0;
    last_get_measurement = -1.0;
    last_rendering = false;
    last_time_frame = -1.0;
    current_time = -1.0;

    index_frame = 1;

    fov_deg = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);

    odometry_options.optimize_options.init_num_frames = odometry_options.init_num_frames;

    img_pro->printParameter();
}

void lioOptimization::addPointToMap(voxelHashMap &map, rgbPoint &point, double voxel_size, int max_num_points_in_voxel, double min_distance_points, 
    int min_num_points, cloudFrame* p_frame)
{
    short kx = static_cast<short>(point.getPosition().x() / voxel_size);
    short ky = static_cast<short>(point.getPosition().y() / voxel_size);
    short kz = static_cast<short>(point.getPosition().z() / voxel_size);

    voxelHashMap::iterator search = map.find(voxel(kx, ky, kz));

    if(search != map.end())
    {
        auto &voxel_block = (search.value());

        if (!voxel_block.IsFull())
        {
            double sq_dist_min_to_points = 10 * voxel_size * voxel_size;

            for (int i(0); i < voxel_block.NumPoints(); ++i)
            {
                auto &_point = voxel_block.points[i];
                double sq_dist = (_point.getPosition() - point.getPosition()).squaredNorm();
                if (sq_dist < sq_dist_min_to_points)
                {
                    sq_dist_min_to_points = sq_dist;
                }
            }

            if(sq_dist_min_to_points > (min_distance_points * min_distance_points))
            {
                if(min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points)
                {
                    voxel_block.AddPoint(point);
                    addPointToPcl(points_world, point, p_frame);
                }
            }
        }
    }
    else
    {
        if (min_num_points <= 0) {
            voxelBlock voxel_block(max_num_points_in_voxel);
            voxel_block.AddPoint(point);
            map[voxel(kx, ky, kz)] = std::move(voxel_block);
        }

    }
}

void lioOptimization::addPointToColorMap(voxelHashMap &map, rgbPoint &point, double voxel_size, int max_num_points_in_voxel, double min_distance_points, 
    int min_num_points, cloudFrame* p_frame, std::vector<voxelId> &voxels_recent_visited_temp)
{
    bool add_point = true;

    int point_map_kx = static_cast<short>(point.getPosition().x() / min_distance_points);
    int point_map_ky = static_cast<short>(point.getPosition().y() / min_distance_points);
    int point_map_kz = static_cast<short>(point.getPosition().z() / min_distance_points);

    int kx =  static_cast<short>(point.getPosition().x() / voxel_size);
    int ky =  static_cast<short>(point.getPosition().y() / voxel_size);
    int kz =  static_cast<short>(point.getPosition().z() / voxel_size);

    // if (hashmap_3d_points.if_exist(point_map_kx, point_map_ky, point_map_kz))
    //     add_point = false;
    if (hashmap_3d_points.if_exist(point_map_kx, point_map_ky, point_map_kz)) {
        // std::cout << "Point exists." << std::endl;
        add_point = false;
    } else {
        // std::cout << "Point does not exist." << std::endl;
    }
    // std::cout << "Add point: " << (add_point ? "true" : "false") << std::endl;

    voxelHashMap::iterator search = map.find(voxel(kx, ky, kz));

    if(search != map.end())
    {
        auto &voxel_block = (search.value());

        if (!voxel_block.IsFull())
        { 
            if(min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points)
            {
                voxel_block.AddPoint(point);

                if (add_point)
                {
                    img_pro->map_tracker->mutex_rgb_points_vec->lock();
                    point.point_index = img_pro->map_tracker->rgb_points_vec.size();
                    img_pro->map_tracker->rgb_points_vec.push_back(&voxel_block.points.back());
                    hashmap_3d_points.insert(point_map_kx, point_map_ky, point_map_kz, img_pro->map_tracker->rgb_points_vec.back());
                    img_pro->map_tracker->mutex_rgb_points_vec->unlock();
                }
            }
        }

        if (fabs(p_frame->time_sweep_end - img_pro->time_last_process) > 1e-5 && fabs(voxel_block.last_visited_time - p_frame->time_sweep_end) > 1e-5)
        {
            voxel_block.last_visited_time = p_frame->time_sweep_end;
            voxels_recent_visited_temp.push_back(voxelId(kx, ky, kz));
        }
    }
    else
    {
        if (min_num_points <= 0)
        {
            voxelBlock voxel_block(max_num_points_in_voxel);
            voxel_block.AddPoint(point);
            map[voxel(kx, ky, kz)] = std::move(voxel_block);

            if (add_point)
            {
                img_pro->map_tracker->mutex_rgb_points_vec->lock();
                point.point_index = img_pro->map_tracker->rgb_points_vec.size();
                img_pro->map_tracker->rgb_points_vec.push_back(&map[voxel(kx, ky, kz)].points.back());
                hashmap_3d_points.insert(point_map_kx, point_map_ky, point_map_kz, img_pro->map_tracker->rgb_points_vec.back());
                img_pro->map_tracker->mutex_rgb_points_vec->unlock();
            }

            if (fabs(p_frame->time_sweep_end - img_pro->time_last_process) > 1e-5 && fabs(map[voxel(kx, ky, kz)].last_visited_time - p_frame->time_sweep_end) > 1e-5)
            {
                map[voxel(kx, ky, kz)].last_visited_time = p_frame->time_sweep_end;
                voxels_recent_visited_temp.push_back(voxelId(kx, ky, kz));
            }
        }

    }
}

void lioOptimization::addPointsToMap(voxelHashMap &map, cloudFrame* p_frame, double voxel_size, int max_num_points_in_voxel, 
    double min_distance_points, int min_num_points, bool to_rendering)
{
    if (to_rendering)
    {
        voxels_recent_visited_temp.clear();
        std::vector<voxelId>().swap(voxels_recent_visited_temp);
    }

    int number_of_voxels_before_add = voxels_recent_visited_temp.size();

    int point_idx = 0;

    for (const auto &point: p_frame->point_frame)
    {
        rgbPoint rgb_point(point.point);
        addPointToMap(map, rgb_point, voxel_size, max_num_points_in_voxel, min_distance_points, min_num_points, p_frame);

        if(point_idx % map_options.add_point_step == 0)
            addPointToColorMap(color_voxel_map, rgb_point, map_options.size_voxel_map, map_options.max_num_points_in_voxel, map_options.min_distance_points, 0, p_frame, voxels_recent_visited_temp);
        
        point_idx++;
    }

    if (to_rendering)
    {
        img_pro->map_tracker->voxels_recent_visited.clear();
        std::vector<voxelId>().swap(img_pro->map_tracker->voxels_recent_visited);
        img_pro->map_tracker->voxels_recent_visited = voxels_recent_visited_temp;
        img_pro->map_tracker->number_of_new_visited_voxel = img_pro->map_tracker->voxels_recent_visited.size() - number_of_voxels_before_add;
    }

    publishCLoudWorld(pub_cloud_world, points_world, p_frame);
    points_world->clear();
}

void lioOptimization::removePointsFarFromLocation(voxelHashMap &map, const Eigen::Vector3d &location, double distance)
{
    std::vector<voxel> voxels_to_erase;

    for (auto &pair: map) {
        rgbPoint rgb_point = pair.second.points[0];
        Eigen::Vector3d pt = rgb_point.getPosition();
        if ((pt - location).squaredNorm() > (distance * distance)) {
            voxels_to_erase.push_back(pair.first);
        }
    }

    for (auto &vox: voxels_to_erase)
        map.erase(vox);

    std::vector<voxel>().swap(voxels_to_erase);
}

size_t lioOptimization::mapSize(const voxelHashMap &map)
{
    size_t map_size(0);
    for (auto &itr_voxel_map: map) {
        map_size += (itr_voxel_map.second).NumPoints();
    }
    return map_size;
}

void lioOptimization::standardCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    assert(msg->header.stamp.toSec() > last_time_lidar);

    cloud_pro->process(msg, point_buffer);

    assert(msg->header.stamp.toSec() > last_time_lidar);
    last_time_lidar = msg->header.stamp.toSec();
}

void lioOptimization::livoxHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    assert(msg->header.stamp.toSec() > last_time_lidar);

    cloud_pro->livoxHandler(msg, point_buffer);

    assert(msg->header.stamp.toSec() > last_time_lidar);
    last_time_lidar = msg->header.stamp.toSec();
}

void lioOptimization::imuHandler(const sensor_msgs::Imu::ConstPtr &msg)
{
    // sensor_msgs::Imu::Ptr msg_temp(new sensor_msgs::Imu(*msg));
    sensor_msgs::Imu::Ptr msg_temp = boost::make_shared<sensor_msgs::Imu>(*msg);
    //std::cout << std::fixed << "current_time imu = " << msg->header.stamp.toSec() << std::endl;
     
    if (abs(time_diff) > 0.1 && time_diff_enable)
    {
        msg_temp->header.stamp = ros::Time().fromSec(time_diff + msg->header.stamp.toSec());
    }

    assert(msg_temp->header.stamp.toSec() > last_time_imu);

    imu_buffer.push(msg_temp);

    assert(msg_temp->header.stamp.toSec() > last_time_imu);
    last_time_imu = msg_temp->header.stamp.toSec();

    if (last_get_measurement < 0)
        last_get_measurement = last_time_imu;
}

void lioOptimization::imageHandler(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image.clone();

    assert(msg->header.stamp.toSec() > last_time_img);

    // std::cout << std::fixed << "current_time image = " << msg->header.stamp.toSec() << std::endl;

    img_buffer.push(image);
    time_img_buffer.push(msg->header.stamp.toSec());

    assert(msg->header.stamp.toSec() > last_time_img);
    last_time_img = msg->header.stamp.toSec();
}

void lioOptimization::compressedImageHandler(const sensor_msgs::CompressedImageConstPtr &msg)
{
    cv::Mat image;

    try
    {
        cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        image = cv_ptr_compressed->image;
        cv_ptr_compressed->image.release();
    }
    catch (cv_bridge::Exception &e)
    {
        printf("Could not convert from '%s' to 'bgr8' !!! ", msg->format.c_str());
    }

    assert(msg->header.stamp.toSec() > last_time_img);

    // std::cout << std::fixed << "current_time image = " << msg->header.stamp.toSec() << std::endl;

    img_buffer.push(image);
    time_img_buffer.push(msg->header.stamp.toSec());

    assert(msg->header.stamp.toSec() > last_time_img);
    last_time_img = msg->header.stamp.toSec();
}

std::vector<Measurements> lioOptimization::getMeasurements()
{
    std::vector<Measurements> measurements;

    while (true)
    {
        if (imu_buffer.empty() || img_buffer.empty() || time_img_buffer.empty() || point_buffer.empty())
            return measurements;

        if (!(point_buffer.back().timestamp > time_img_buffer.front()))
        {
            return measurements;
        }

        if (!(point_buffer.front().timestamp < time_img_buffer.front()))
        {
            time_img_buffer.pop();

            img_buffer.front().release();
            img_buffer.pop();

            continue;
        }

        if (!(imu_buffer.back()->header.stamp.toSec() > time_img_buffer.front()))
        {
            return measurements;
        }

        if (!(imu_buffer.front()->header.stamp.toSec() < time_img_buffer.front()))
        {
            time_img_buffer.pop();

            img_buffer.front().release();
            img_buffer.pop();

            continue;
        }

        Measurements measurement;

        if (last_get_measurement + cloud_pro->getSweepInterval() < time_img_buffer.front() - 0.5 * cloud_pro->getSweepInterval())
        {
            measurement.time_image = last_get_measurement + cloud_pro->getSweepInterval();

            while (imu_buffer.front()->header.stamp.toSec() < last_get_measurement + cloud_pro->getSweepInterval())
            {
                measurement.imu_measurements.emplace_back(imu_buffer.front());
                imu_buffer.pop();
            }

            measurement.imu_measurements.emplace_back(imu_buffer.front());

            while (point_buffer.front().timestamp < last_get_measurement + cloud_pro->getSweepInterval())
            {
                measurement.lidar_points.push_back(point_buffer.front());
                point_buffer.pop();
            }

            measurement.time_sweep.first = last_get_measurement;
            measurement.time_sweep.second = cloud_pro->getSweepInterval();

            measurement.rendering = false;

            if (measurement.lidar_points.size() > 0)
            {
                measurements.emplace_back(measurement);
                //assert(last_rendering != measurement.rendering);
                last_rendering = measurement.rendering;
            }

            last_get_measurement = last_get_measurement + cloud_pro->getSweepInterval();

            break;
        }
        else
        {
            measurement.time_image = time_img_buffer.front();
            measurement.image = img_buffer.front().clone();

            time_img_buffer.pop();

            img_buffer.front().release();
            img_buffer.pop();

            while (imu_buffer.front()->header.stamp.toSec() < measurement.time_image)
            {
                measurement.imu_measurements.emplace_back(imu_buffer.front());
                imu_buffer.pop();
            }

            measurement.imu_measurements.emplace_back(imu_buffer.front());

            while (point_buffer.front().timestamp < measurement.time_image)
            {
                measurement.lidar_points.push_back(point_buffer.front());
                point_buffer.pop();
            }

            measurement.time_sweep.first = last_get_measurement;
            measurement.time_sweep.second = measurement.time_image - last_get_measurement;

            measurement.rendering = true;

            if (measurement.lidar_points.size() > 0)
            {
                measurements.emplace_back(measurement);
                //assert(last_rendering != measurement.rendering);
                last_rendering = measurement.rendering;
            }

            last_get_measurement = measurement.time_image;

            break;
        }
    }

    return measurements;
}

void lioOptimization::makePointTimestamp(std::vector<point3D> &sweep, double time_begin, double time_end)
{
    if(cloud_pro->isPointTimeEnable())
    {
        double delta_t = time_end - time_begin;

        for (int i = 0; i < sweep.size(); i++)
        {
            sweep[i].relative_time = sweep[i].timestamp - time_begin;
            sweep[i].alpha_time = sweep[i].relative_time / delta_t;
            sweep[i].relative_time = sweep[i].relative_time * 1000.0;
            if(sweep[i].alpha_time > 1.0) sweep[i].alpha_time = 1.0 - 1e-5;
        }
    }
    else
    {
        double delta_t = time_end - time_begin;

        std::vector<point3D>::iterator iter = sweep.begin();

        while (iter != sweep.end())
        {
            if((*iter).timestamp > time_end) iter = sweep.erase(iter);
            else if((*iter).timestamp < time_begin) iter = sweep.erase(iter);
            else
            {
                (*iter).relative_time = (*iter).timestamp - time_begin;
                (*iter).alpha_time = (*iter).relative_time / delta_t;
                (*iter).relative_time = (*iter).relative_time * 1000.0;
                iter++;
            }
        }
    }
}

cloudFrame* lioOptimization::buildFrame(std::vector<point3D> &cut_sweep, state *cur_state, double timestamp_begin, double timestamp_offset)
{
    std::vector<point3D> frame(cut_sweep);

    double offset_begin = 0;
    double offset_end = timestamp_offset;

    double time_sweep_begin = timestamp_begin;
    double time_frame_begin = timestamp_begin;

    makePointTimestamp(frame, time_frame_begin, timestamp_begin + timestamp_offset);

    if (odometry_options.motion_compensation == CONSTANT_VELOCITY)
        distortFrameByConstant(frame, imu_states, time_frame_begin, R_imu_lidar, t_imu_lidar);
    else if (odometry_options.motion_compensation == IMU)
        distortFrameByImu(frame, imu_states, time_frame_begin, R_imu_lidar, t_imu_lidar);

    double sample_size = index_frame < odometry_options.init_num_frames ? odometry_options.init_voxel_size : odometry_options.voxel_size;

    boost::mt19937_64 seed;
    std::shuffle(frame.begin(), frame.end(), seed);

    if (odometry_options.voxel_size > 0)
    {
        subSampleFrame(frame, sample_size);

        std::shuffle(frame.begin(), frame.end(), seed);
    }

    transformAllImuPoint(frame, imu_states, R_imu_lidar, t_imu_lidar);

    double dt_offset = 0;

    if(index_frame > 1)
        dt_offset -= time_frame_begin - all_cloud_frame.back()->time_sweep_end;


    if (index_frame <= 2) {
        for (auto &point_temp: frame) {
            point_temp.alpha_time = 1.0;
        }
    }

    if (index_frame > 2) {
        for (auto &point_temp: frame) {
            transformPoint(point_temp, cur_state->rotation, cur_state->translation, R_imu_lidar, t_imu_lidar);
        }
    }
    else
    {
        for (auto &point_temp: frame) {
            Eigen::Quaterniond q_identity = Eigen::Quaterniond::Identity();
            Eigen::Vector3d t_zero = Eigen::Vector3d::Zero();
            transformPoint(point_temp, q_identity, t_zero, R_imu_lidar, t_imu_lidar);
        }
    }

    cloudFrame *p_frame = new cloudFrame(frame, cur_state);
    p_frame->time_sweep_begin = time_sweep_begin;
    p_frame->time_sweep_end = timestamp_begin + timestamp_offset;
    p_frame->time_frame_begin = time_frame_begin;
    p_frame->time_frame_end = p_frame->time_sweep_end;
    p_frame->offset_begin = offset_begin;
    p_frame->offset_end = offset_end;
    p_frame->dt_offset = dt_offset;
    p_frame->id = all_cloud_frame.size();
    p_frame->sub_id = 0;
    p_frame->frame_id = index_frame;

    all_cloud_frame.push_back(p_frame);

    return p_frame;
}

void lioOptimization::stateInitialization(state *cur_state)
{
    if (index_frame <= 2)
    {
        cur_state->rotation = Eigen::Quaterniond::Identity();
        cur_state->translation = Eigen::Vector3d::Zero();
    }
    else if (index_frame == 3)
    {
        if (odometry_options.initialization == INIT_CONSTANT_VELOCITY)
        {
            Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                    all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;

            Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation + 
                                         all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                                         all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * 
                                         (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation - 
                                         all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);

            cur_state->rotation = q_next_end;
            cur_state->translation = t_next_end;
        }
        else if (odometry_options.initialization == INIT_IMU)
        {
            if (initial_flag)
            {
                cur_state->rotation = eskf_pro->getRotation();
                cur_state->translation = eskf_pro->getTranslation();
            }
            else
            {
                Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                        all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;

                Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation + 
                                             all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                                             all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * 
                                             (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation - 
                                             all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);

                cur_state->rotation = q_next_end;
                cur_state->translation = t_next_end;
            }
        }
        else
        {
            cur_state->rotation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
            cur_state->translation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
        }
    }
    else
    {
        if (odometry_options.initialization == INIT_CONSTANT_VELOCITY)
        {
            Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                    all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;

            Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation + 
                                         all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                                         all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * 
                                         (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation - 
                                         all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);

            cur_state->rotation = q_next_end;
            cur_state->translation = t_next_end;
        }
        else if (odometry_options.initialization == INIT_IMU)
        {
            if (initial_flag)
            {
                cur_state->rotation = eskf_pro->getRotation();
                cur_state->translation = eskf_pro->getTranslation();
            }
            else
            {
                Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                        all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;

                Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation + 
                                             all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                                             all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * 
                                             (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation - 
                                             all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);

                cur_state->rotation = q_next_end;
                cur_state->translation = t_next_end;
            }
        }
        else
        {
            cur_state->rotation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
            cur_state->translation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
        }
    }
}

optimizeSummary lioOptimization::stateEstimation(cloudFrame *p_frame, bool to_rendering)
{
    icpOptions optimize_options = odometry_options.optimize_options;
    const double kSizeVoxelInitSample = odometry_options.voxel_size;

    const double kSizeVoxelMap = optimize_options.size_voxel_map;
    const double kMinDistancePoints = odometry_options.min_distance_points;
    const int kMaxNumPointsInVoxel = odometry_options.max_num_points_in_voxel;

    optimizeSummary optimize_summary;

    if(p_frame->frame_id > 1)
    {
        bool good_enough_registration = false;
        double sample_voxel_size = p_frame->frame_id < odometry_options.init_num_frames ? odometry_options.init_sample_voxel_size : odometry_options.sample_voxel_size;
        double min_voxel_size = std::min(odometry_options.init_voxel_size, odometry_options.voxel_size);

        optimize_summary = optimize(p_frame, optimize_options, sample_voxel_size);

        if(!optimize_summary.success)
        {
            return optimize_summary;
        }
    }
    else
    {
        p_frame->p_state->translation = eskf_pro->getTranslation();
        p_frame->p_state->rotation = eskf_pro->getRotation();
        p_frame->p_state->velocity = eskf_pro->getVelocity();
        p_frame->p_state->ba = eskf_pro->getBa();
        p_frame->p_state->bg = eskf_pro->getBg();
        G = eskf_pro->getGravity();
        G_norm = G.norm();
    }

    addPointsToMap(voxel_map, p_frame, kSizeVoxelMap, kMaxNumPointsInVoxel, kMinDistancePoints, 0, to_rendering);

    const double kMaxDistance = odometry_options.max_distance;
    const Eigen::Vector3d location = p_frame->p_state->translation;

    //removePointsFarFromLocation(voxel_map, location, kMaxDistance);

    return optimize_summary;
}

void lioOptimization::process(std::vector<point3D> &cut_sweep, double timestamp_begin, double timestamp_offset, cv::Mat &cur_image, bool to_rendering)
{
    state *cur_state = new state();

    stateInitialization(cur_state);

    std::vector<point3D> const_frame;

    const_frame.insert(const_frame.end(), cut_sweep.begin(), cut_sweep.end());

    cloudFrame *p_frame = buildFrame(const_frame, cur_state, timestamp_begin, timestamp_offset);

    optimizeSummary summary = stateEstimation(p_frame, to_rendering);

    dt_sum = 0;

    if (all_cloud_frame.size() < 3) {
        p_frame->p_state->fx = img_pro->getCameraIntrinsic()(0, 0);
        p_frame->p_state->fy = img_pro->getCameraIntrinsic()(1, 1);
        p_frame->p_state->cx = img_pro->getCameraIntrinsic()(0, 2);
        p_frame->p_state->cy = img_pro->getCameraIntrinsic()(1, 2);

        p_frame->p_state->R_imu_camera = R_imu_camera;
        p_frame->p_state->t_imu_camera = t_imu_camera;
    }
    else
    {
        p_frame->p_state->fx = all_cloud_frame[all_cloud_frame.size() - 2]->p_state->fx;
        p_frame->p_state->fy = all_cloud_frame[all_cloud_frame.size() - 2]->p_state->fy;
        p_frame->p_state->cx = all_cloud_frame[all_cloud_frame.size() - 2]->p_state->cx;
        p_frame->p_state->cy = all_cloud_frame[all_cloud_frame.size() - 2]->p_state->cy;

        p_frame->p_state->R_imu_camera = all_cloud_frame[all_cloud_frame.size() - 2]->p_state->R_imu_camera;
        p_frame->p_state->t_imu_camera = all_cloud_frame[all_cloud_frame.size() - 2]->p_state->t_imu_camera;
    }

    p_frame->p_state->q_world_camera = Eigen::Quaterniond(p_frame->p_state->rotation.toRotationMatrix() * p_frame->p_state->R_imu_camera);
    p_frame->p_state->t_world_camera = p_frame->p_state->rotation.toRotationMatrix() * p_frame->p_state->t_imu_camera + p_frame->p_state->translation;
    p_frame->refreshPoseForProjection();

    if (to_rendering)
    {
        p_frame->rgb_image = cur_image;
        p_frame->image_cols = cur_image.cols;
        p_frame->image_rows = cur_image.rows;

        img_pro->process(color_voxel_map, p_frame);

        // pubColorPoints(pub_cloud_color, p_frame);
    }

    publish_odometry(pub_odom, p_frame);
    publish_path(pub_path, p_frame);   

    if(debug_output)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr p_cloud_temp;
        p_cloud_temp.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
        point3DtoPCL(p_frame->point_frame, p_cloud_temp);

        std::string pcd_path(output_path + "/cloud_frame/" + std::to_string(index_frame) + std::string(".pcd"));
        saveCutCloud(pcd_path, p_cloud_temp);
    }

    int num_remove = 0;

    if (initial_flag)
    {
        if (index_frame > 1)
        {
            while (all_cloud_frame.size() > 2)
            {
                recordSinglePose(all_cloud_frame[0]);
                all_cloud_frame[0]->release();
                all_cloud_frame.erase(all_cloud_frame.begin());
                num_remove++;
            }
            assert(all_cloud_frame.size() == 2);
        }
    }
    else
    {
        while (all_cloud_frame.size() > odometry_options.num_for_initialization)
        {
            recordSinglePose(all_cloud_frame[0]);
            all_cloud_frame[0]->release();
            all_cloud_frame.erase(all_cloud_frame.begin());
            num_remove++;
        }
    }
    

    for(int i = 0; i < all_cloud_frame.size(); i++)
        all_cloud_frame[i]->id = all_cloud_frame[i]->id - num_remove;
}

void lioOptimization::recordSinglePose(cloudFrame *p_frame)
{
    std::ofstream foutC(std::string(output_path + "/pose.txt"), std::ios::app);

    foutC.setf(std::ios::scientific, std::ios::floatfield);
    foutC.precision(6);

    foutC << std::fixed << p_frame->time_sweep_end << " ";
    foutC << p_frame->p_state->translation.x() << " " << p_frame->p_state->translation.y() << " " << p_frame->p_state->translation.z() << " ";
    foutC << p_frame->p_state->rotation.x() << " " << p_frame->p_state->rotation.y() << " " << p_frame->p_state->rotation.z() << " " << p_frame->p_state->rotation.w();
    foutC << std::endl; 

    foutC.close();

    if (initial_flag)
    {
        std::ofstream foutC2(std::string(output_path + "/velocity.txt"), std::ios::app);

        foutC2.setf(std::ios::scientific, std::ios::floatfield);
        foutC2.precision(6);

        foutC2 << std::fixed << p_frame->time_sweep_end << " ";
        foutC2 << p_frame->p_state->velocity.x() << " " << p_frame->p_state->velocity.y() << " " << p_frame->p_state->velocity.z();
        foutC2 << std::endl; 

        foutC2.close();

        std::ofstream foutC3(std::string(output_path + "/bias.txt"), std::ios::app);

        foutC3.setf(std::ios::scientific, std::ios::floatfield);
        foutC3.precision(6);

        foutC3 << std::fixed << p_frame->time_sweep_end << " ";
        foutC3 << p_frame->p_state->ba.x() << " " << p_frame->p_state->ba.y() << " " << p_frame->p_state->ba.z() << " ";
        foutC3 << p_frame->p_state->bg.x() << " " << p_frame->p_state->bg.y() << " " << p_frame->p_state->bg.z();
        foutC3 << std::endl; 

        foutC3.close();
    }
}

void lioOptimization::set_posestamp(geometry_msgs::PoseStamped &body_pose_out,cloudFrame *p_frame)
{
    body_pose_out.pose.position.x = p_frame->p_state->translation.x();
    body_pose_out.pose.position.y = p_frame->p_state->translation.y();
    body_pose_out.pose.position.z = p_frame->p_state->translation.z();
    
    body_pose_out.pose.orientation.x = p_frame->p_state->rotation.x();
    body_pose_out.pose.orientation.y = p_frame->p_state->rotation.y();
    body_pose_out.pose.orientation.z = p_frame->p_state->rotation.z();
    body_pose_out.pose.orientation.w = p_frame->p_state->rotation.w();
}

void lioOptimization::publish_path(ros::Publisher pub_path,cloudFrame *p_frame)
{
    set_posestamp(msg_body_pose,p_frame);
    msg_body_pose.header.stamp = ros::Time().fromSec(p_frame->time_sweep_end);
    msg_body_pose.header.frame_id = "camera_init";

    static int i = 0;
    i++;
    if (i % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pub_path.publish(path);
    }
}

void lioOptimization::publishCLoudWorld(ros::Publisher &pub_cloud_world, pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points, cloudFrame* p_frame)
{
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_points, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(p_frame->time_sweep_end);
    laserCloudmsg.header.frame_id = "camera_init";
    pub_cloud_world.publish(laserCloudmsg);
}

void lioOptimization::pubColorPoints(ros::Publisher &pub_cloud_rgb, cloudFrame *p_frame)
{
    pcl::PointCloud<pcl::PointXYZRGB> points_rgb_vec;
    sensor_msgs::PointCloud2 ros_points_msg;

    //int num_publish = 0;

    for (int i = 0; i < img_pro->map_tracker->rgb_points_vec.size(); i++)
    {
        rgbPoint *p_point = img_pro->map_tracker->rgb_points_vec[i];

        if (p_point->N_rgb < map_options.pub_point_minimum_views) continue;

        pcl::PointXYZRGB rgb_point;

        rgb_point.x = p_point->getPosition()[0];
        rgb_point.y = p_point->getPosition()[1];
        rgb_point.z = p_point->getPosition()[2];
        rgb_point.r = p_point->getRgb()[2];
        rgb_point.g = p_point->getRgb()[1];
        rgb_point.b = p_point->getRgb()[0];

        points_rgb_vec.push_back(rgb_point);
    }

    pcl::toROSMsg(points_rgb_vec, ros_points_msg);

    ros_points_msg.header.stamp = ros::Time().fromSec(p_frame->time_sweep_end);
    ros_points_msg.header.frame_id = "camera_init";

    pub_cloud_rgb.publish(ros_points_msg);
}

void lioOptimization::threadPubColorPoints()
{
    int last_pub_map_index = -1000;
    int sleep_time_after_pub = 10;
    int number_of_points_per_topic = 1000;

    if (number_of_points_per_topic < 0) return;

    while (1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        pcl::PointCloud<pcl::PointXYZRGB> points_rgb_vec;
        sensor_msgs::PointCloud2 ros_points_msg;

        img_pro->map_tracker->mutex_rgb_points_vec->lock();
        int points_size = img_pro->map_tracker->rgb_points_vec.size();
        img_pro->map_tracker->mutex_rgb_points_vec->unlock();

        points_rgb_vec.resize(number_of_points_per_topic);

        int pub_index_size = 0;
        int cur_topic_index = 0;

        img_pro->map_tracker->mutex_frame_index->lock();
        int updated_frame_index = img_pro->map_tracker->updated_frame_index;
        img_pro->map_tracker->mutex_frame_index->unlock();

        if (last_pub_map_index == updated_frame_index) continue;

        last_pub_map_index = updated_frame_index;

        for (int i = 0; i < points_size; i++)
        {
            img_pro->map_tracker->mutex_rgb_points_vec->lock();
            int N_rgb = img_pro->map_tracker->rgb_points_vec[i]->N_rgb;
            img_pro->map_tracker->mutex_rgb_points_vec->unlock();

            if (N_rgb < map_options.pub_point_minimum_views)
            {
                continue;
            }

            img_pro->map_tracker->mutex_rgb_points_vec->lock();
            points_rgb_vec.points[pub_index_size].x = img_pro->map_tracker->rgb_points_vec[i]->getPosition()[0];
            points_rgb_vec.points[pub_index_size].y = img_pro->map_tracker->rgb_points_vec[i]->getPosition()[1];
            points_rgb_vec.points[pub_index_size].z = img_pro->map_tracker->rgb_points_vec[i]->getPosition()[2];
            points_rgb_vec.points[pub_index_size].r = img_pro->map_tracker->rgb_points_vec[i]->getRgb()[2];
            points_rgb_vec.points[pub_index_size].g = img_pro->map_tracker->rgb_points_vec[i]->getRgb()[1];
            points_rgb_vec.points[pub_index_size].b = img_pro->map_tracker->rgb_points_vec[i]->getRgb()[0];
            img_pro->map_tracker->mutex_rgb_points_vec->unlock();

            pub_index_size++;

            if (pub_index_size == number_of_points_per_topic)
            {
                pub_index_size = 0;

                pcl::toROSMsg(points_rgb_vec, ros_points_msg);
                ros_points_msg.header.frame_id = "camera_init";       
                ros_points_msg.header.stamp = ros::Time::now(); 

                if (pub_cloud_color_vec[cur_topic_index] == nullptr)
                {
                    pub_cloud_color_vec[cur_topic_index] =
                        std::make_shared<ros::Publisher>(nh.advertise<sensor_msgs::PointCloud2>(
                            std::string("/color_global_map_").append(std::to_string(cur_topic_index)), 100));
                }

                pub_cloud_color_vec[cur_topic_index]->publish(ros_points_msg);
                std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_after_pub));

                cur_topic_index++;
            }
        }

        points_rgb_vec.resize(pub_index_size);

        pcl::toROSMsg(points_rgb_vec, ros_points_msg);
        ros_points_msg.header.frame_id = "camera_init";       
        ros_points_msg.header.stamp = ros::Time::now();

        if (pub_cloud_color_vec[cur_topic_index] == nullptr)
        {
            pub_cloud_color_vec[cur_topic_index] =
                std::make_shared<ros::Publisher>(nh.advertise<sensor_msgs::PointCloud2>(
                    std::string("/color_global_map_").append(std::to_string(cur_topic_index)), 100));
        }

        std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_after_pub));

        pub_cloud_color_vec[cur_topic_index]->publish(ros_points_msg);

        cur_topic_index++;

        if (cur_topic_index >= 45)
        {
            number_of_points_per_topic *= 1.5;
            sleep_time_after_pub *= 1.5;
        }
    }
}

void lioOptimization::addPointToPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points, rgbPoint &point, cloudFrame *p_frame)
{
    pcl::PointXYZI cloudTemp;
    
    cloudTemp.x = point.getPosition().x();
    cloudTemp.y = point.getPosition().y();
    cloudTemp.z = point.getPosition().z();
    cloudTemp.intensity = 50*(cloudTemp.z - p_frame->p_state->translation.z());
    pcl_points->points.push_back(cloudTemp);
}

void lioOptimization::publish_odometry(const ros::Publisher & pubOdomAftMapped, cloudFrame *p_frame)
{
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(p_frame->p_state->rotation.z(), -p_frame->p_state->rotation.x(), -p_frame->p_state->rotation.y());

    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(p_frame->time_sweep_end);
    odomAftMapped.pose.pose.orientation.x = p_frame->p_state->rotation.x();
    odomAftMapped.pose.pose.orientation.y = p_frame->p_state->rotation.y();
    odomAftMapped.pose.pose.orientation.z = p_frame->p_state->rotation.z();
    odomAftMapped.pose.pose.orientation.w = p_frame->p_state->rotation.w();
    odomAftMapped.pose.pose.position.x = p_frame->p_state->translation.x();
    odomAftMapped.pose.pose.position.y = p_frame->p_state->translation.y();
    odomAftMapped.pose.pose.position.z = p_frame->p_state->translation.z();
    pubOdomAftMapped.publish(odomAftMapped);

    laserOdometryTrans.frame_id_ = "/camera_init";
    laserOdometryTrans.child_frame_id_ = "/laser_odom";
    laserOdometryTrans.stamp_ = ros::Time().fromSec(p_frame->time_sweep_end);
    laserOdometryTrans.setRotation(tf::Quaternion(p_frame->p_state->rotation.x(), 
                                                  p_frame->p_state->rotation.y(), 
                                                  p_frame->p_state->rotation.z(), 
                                                  p_frame->p_state->rotation.w()));
    laserOdometryTrans.setOrigin(tf::Vector3(p_frame->p_state->translation.x(), 
                                             p_frame->p_state->translation.y(), 
                                             p_frame->p_state->translation.z()));
    tfBroadcaster.sendTransform(laserOdometryTrans);
}

void lioOptimization::saveColorPoints()
{
    std::string pcd_path = std::string(output_path + "/rgb_map.pcd");
    std::cout << "Save colored points to " << pcd_path << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB> pcd_rgb;

    long point_size = img_pro->map_tracker->rgb_points_vec.size();
    pcd_rgb.resize(point_size);

    long point_count = 0;

    for (long i = point_size - 1; i > 0; i--)
    {
        img_pro->map_tracker->mutex_rgb_points_vec->lock();
        int N_rgb = img_pro->map_tracker->rgb_points_vec[i]->N_rgb;
        img_pro->map_tracker->mutex_rgb_points_vec->unlock();

        if (N_rgb < map_options.pub_point_minimum_views)
        {
            continue;
        }

        pcl::PointXYZRGB point;
        img_pro->map_tracker->mutex_rgb_points_vec->lock();
        pcd_rgb.points[point_count].x = img_pro->map_tracker->rgb_points_vec[i]->getPosition()[0];
        pcd_rgb.points[point_count].y = img_pro->map_tracker->rgb_points_vec[i]->getPosition()[1];
        pcd_rgb.points[point_count].z = img_pro->map_tracker->rgb_points_vec[i]->getPosition()[2];
        pcd_rgb.points[point_count].r = img_pro->map_tracker->rgb_points_vec[i]->getRgb()[2];
        pcd_rgb.points[point_count].g = img_pro->map_tracker->rgb_points_vec[i]->getRgb()[1];
        pcd_rgb.points[point_count].b = img_pro->map_tracker->rgb_points_vec[i]->getRgb()[0];
        img_pro->map_tracker->mutex_rgb_points_vec->unlock();
        point_count++;
    }

    pcd_rgb.resize(point_count);

    std::cout << "Total have " << point_count << " points." << std::endl;
    std::cout << "Now write to: " << pcd_path << std::endl; 
    pcl::io::savePCDFileBinary(pcd_path, pcd_rgb);
}

void lioOptimization::run()
{

    std::vector<Measurements> measurements = getMeasurements();

    for (auto &measurement : measurements)
    {
        // process
        double time_frame = measurement.time_image;
        double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;

        if (!initial_flag)
        {
            for (auto &imu_msg : measurement.imu_measurements)
            {
                double time_imu = imu_msg->header.stamp.toSec();

                if (time_imu <= time_frame)
                { 
                    current_time = time_imu;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    
                    imu_meas.emplace_back(current_time, std::make_pair(Eigen::Vector3d(rx, ry, rz), Eigen::Vector3d(dx, dy, dz)));
                }
                else
                {
                    double dt_1 = time_frame - current_time;
                    double dt_2 = time_imu - time_frame;
                    current_time = time_frame;
                    assert(dt_1 >= 0);
                    assert(dt_2 >= 0);
                    assert(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;

                    imu_meas.emplace_back(current_time, std::make_pair(Eigen::Vector3d(rx, ry, rz), Eigen::Vector3d(dx, dy, dz)));
                }
            }

            
            eskf_pro->tryInit(imu_meas);
            imu_meas.clear();

            last_time_frame = time_frame;

            std::vector<point3D>().swap(measurement.lidar_points);

            if(measurement.rendering) measurement.image.release();

            continue;
        }

        if (initial_flag)
        {
            imuState imu_state_temp;

            imu_state_temp.timestamp = current_time;

            imu_state_temp.un_acc = eskf_pro->getRotation().toRotationMatrix() * (eskf_pro->getLastAcc() - eskf_pro->getBa());
            imu_state_temp.un_gyr = eskf_pro->getLastGyr() - eskf_pro->getBg();
            imu_state_temp.trans = eskf_pro->getTranslation();
            imu_state_temp.quat = eskf_pro->getRotation();
            imu_state_temp.vel = eskf_pro->getVelocity();

            imu_states.push_back(imu_state_temp);
        }

        for (auto &imu_msg : measurement.imu_measurements)
        {
            double time_imu = imu_msg->header.stamp.toSec();

            if (time_imu <= time_frame)
            { 
                double dt = time_imu - current_time;

                if(dt < -1e-6) continue;
                assert(dt >= 0);
                current_time = time_imu;
                dx = imu_msg->linear_acceleration.x;
                dy = imu_msg->linear_acceleration.y;
                dz = imu_msg->linear_acceleration.z;
                rx = imu_msg->angular_velocity.x;
                ry = imu_msg->angular_velocity.y;
                rz = imu_msg->angular_velocity.z;
                
                imuState imu_state_temp;

                imu_state_temp.timestamp = current_time;

                imu_state_temp.un_acc = eskf_pro->getRotation().toRotationMatrix() * (0.5 * (eskf_pro->getLastAcc() + Eigen::Vector3d(dx, dy, dz)) - eskf_pro->getBa());
                imu_state_temp.un_gyr = 0.5 * (eskf_pro->getLastGyr() + Eigen::Vector3d(rx, ry, rz)) - eskf_pro->getBg();

                dt_sum = dt_sum + dt;
                eskf_pro->predict(dt, Eigen::Vector3d(dx, dy, dz), Eigen::Vector3d(rx, ry, rz));

                imu_state_temp.trans = eskf_pro->getTranslation();
                imu_state_temp.quat = eskf_pro->getRotation();
                imu_state_temp.vel = eskf_pro->getVelocity();

                imu_states.push_back(imu_state_temp);
            }
            else
            {
                double dt_1 = time_frame - current_time;
                double dt_2 = time_imu - time_frame;
                current_time = time_frame;
                assert(dt_1 >= 0);
                assert(dt_2 >= 0);
                assert(dt_1 + dt_2 > 0);
                double w1 = dt_2 / (dt_1 + dt_2);
                double w2 = dt_1 / (dt_1 + dt_2);
                dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                rz = w1 * rz + w2 * imu_msg->angular_velocity.z;

                imuState imu_state_temp;

                imu_state_temp.timestamp = current_time;

                imu_state_temp.un_acc = eskf_pro->getRotation().toRotationMatrix() * (0.5 * (eskf_pro->getLastAcc() + Eigen::Vector3d(dx, dy, dz)) - eskf_pro->getBa());
                imu_state_temp.un_gyr = 0.5 * (eskf_pro->getLastGyr() + Eigen::Vector3d(rx, ry, rz)) - eskf_pro->getBg();

                dt_sum = dt_sum + dt_1;
                eskf_pro->predict(dt_1, Eigen::Vector3d(dx, dy, dz), Eigen::Vector3d(rx, ry, rz));

                imu_state_temp.trans = eskf_pro->getTranslation();
                imu_state_temp.quat = eskf_pro->getRotation();
                imu_state_temp.vel = eskf_pro->getVelocity();

                imu_states.push_back(imu_state_temp);
            }
        }

        process(measurement.lidar_points, measurement.time_sweep.first, measurement.time_sweep.second, measurement.image, measurement.rendering);

        imu_states.clear();
        
        last_time_frame = time_frame;
        index_frame++;

        std::vector<point3D>().swap(measurement.lidar_points);

        if(measurement.rendering) measurement.image.release();
        // process
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "livo_node");
    ros::Time::init();
    
    lioOptimization LIO;

    std::thread visualization_map(&lioOptimization::threadPubColorPoints, &LIO);

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        LIO.run();

        rate.sleep();
    }

    LIO.saveColorPoints();

    visualization_map.join();

    return 0;
}
