#include "opticalFlowTracker.h"

opticalFlowTracker::opticalFlowTracker()
{
    cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.05);

    if (lk_optical_flow_kernel == nullptr)
        lk_optical_flow_kernel = std::make_shared<LKOpticalFlowKernel>(cv::Size(21, 21), 3, criteria, cv_OPTFLOW_LK_GET_MIN_EIGENVALS);

    maximum_tracked_points = 300;
}

void opticalFlowTracker::updateAndAppendTrackPoints(cloudFrame *p_frame, rgbMapTracker *map_tracker, double mini_distance, int minimum_frame_diff)
{
    double u_d, v_d;
    int u_i, v_i;

    double max_allow_reproject_error = 2.0 * p_frame->image_cols / 320.0;

    Hash_map_2d<int, float> map_2d_points_occupied;

    for (auto it = map_rgb_points_in_last_image_pose.begin(); it != map_rgb_points_in_last_image_pose.end();)
    {
        rgbPoint *rgb_point = ((rgbPoint*)it->first);
        Eigen::Vector3d point_3d = ((rgbPoint*)it->first)->getPosition();

        bool res = p_frame->project3dPointInThisImage(point_3d, u_d, v_d, nullptr, 1.0);

        u_i = std::round(u_d / mini_distance) * mini_distance;
        v_i = std::round(v_d / mini_distance) * mini_distance;

        double error = Eigen::Vector2d(u_d - it->second.x, v_d - it->second.y).norm();

        if (error > max_allow_reproject_error)
        {
            rgb_point->is_out_lier_count++;

            if ((rgb_point->is_out_lier_count > 1) || (error > max_allow_reproject_error * 2))
            {
                rgb_point->is_out_lier_count = 0;
                it = map_rgb_points_in_last_image_pose.erase(it);
                continue;
            }
        }
        else
        {
            rgb_point->is_out_lier_count = 0;
        }

        if (res)
        {
            double depth = (point_3d - p_frame->p_state->t_world_camera).norm();

            if (map_2d_points_occupied.if_exist(u_i, v_i) == false)
            {
                map_2d_points_occupied.insert(u_i, v_i, depth);
            }
        }

        it++;
    }

    if (map_tracker->points_rgb_vec_for_projection != nullptr)
    {
        int point_size = map_tracker->points_rgb_vec_for_projection->size();

        for (int i = 0; i < point_size; i++)
        {
            if (map_rgb_points_in_last_image_pose.find((*(map_tracker->points_rgb_vec_for_projection))[i]) !=
                 map_rgb_points_in_last_image_pose.end())
            {
                continue;
            }

            Eigen::Vector3d point_3d = (*(map_tracker->points_rgb_vec_for_projection))[i]->getPosition();

            bool res = p_frame->project3dPointInThisImage(point_3d, u_d, v_d, nullptr, 1.0);

            u_i = std::round(u_d / mini_distance) * mini_distance;
            v_i = std::round(v_d / mini_distance) * mini_distance;

            if (res)
            {
                double depth = (point_3d - p_frame->p_state->t_world_camera).norm();

                if (map_2d_points_occupied.if_exist(u_i, v_i) == false)
                {
                    map_2d_points_occupied.insert(u_i, v_i, depth);

                    map_rgb_points_in_last_image_pose[(*(map_tracker->points_rgb_vec_for_projection))[i]] = cv::Point2f(u_d, v_d);
                }
            }

            if (map_rgb_points_in_last_image_pose.size() >= maximum_tracked_points)
            {
                break;
            }
        }
    }

    updateLastTrackingVectorAndIds();
}

void opticalFlowTracker::setIntrinsic(Eigen::Matrix3d intrinsic_, Eigen::Matrix<double, 5, 1> dist_coeffs_, cv::Size image_size_)
{
    cv::eigen2cv(intrinsic_, intrinsic);
    cv::eigen2cv(dist_coeffs_, dist_coeffs);
    initUndistortRectifyMap(intrinsic, dist_coeffs, cv::Mat(), intrinsic, image_size_, CV_16SC2, m_ud_map1, m_ud_map2);
}

void opticalFlowTracker::trackImage(cloudFrame *p_frame, double distance)
{
    cur_image = p_frame->rgb_image;
    current_image_time = p_frame->time_sweep_end;
    map_rgb_points_in_cur_image_pose.clear();

    if (cur_image.empty()) return;

    cv::Mat gray_image = p_frame->gray_image;

    std::vector<uchar> status;
    std::vector<float> error;

    cur_tracked_points = last_tracked_points;

    int before_track = last_tracked_points.size();

    if (last_tracked_points.size() < 30)
    {
        last_image_time = current_image_time;
        return;
    }

    lk_optical_flow_kernel->trackImage(gray_image, last_tracked_points, cur_tracked_points, status, 2);

    reduce_vector(last_tracked_points, status);
    reduce_vector(old_ids, status);
    reduce_vector(cur_tracked_points, status);

    int after_track = last_tracked_points.size();
    cv::Mat mat_F;

    unsigned int points_before_F = last_tracked_points.size();
    mat_F = cv::findFundamentalMat(last_tracked_points, cur_tracked_points, cv::FM_RANSAC, 1.0, 0.997, status);

    unsigned int size_a = cur_tracked_points.size();
    reduce_vector(last_tracked_points, status);
    reduce_vector(old_ids, status);
    reduce_vector(cur_tracked_points, status);

    map_rgb_points_in_cur_image_pose.clear();

    double frame_time_diff = (current_image_time - last_image_time);

    for (uint i = 0; i < last_tracked_points.size(); i++)
    {
        if (p_frame->if2dPointsAvailable(cur_tracked_points[i].x, cur_tracked_points[i].y, 1.0, 0.05))
        {
            rgbPoint *rgb_point_ptr = ((rgbPoint*)rgb_points_ptr_vec_in_last_image[old_ids[i]]);
            
           // 检查指针有效性
            if (rgb_point_ptr == nullptr) {
            std::cerr << "Invalid rgbPoint pointer detected!" << std::endl;
            continue;
            }

            map_rgb_points_in_cur_image_pose[rgb_point_ptr] = cur_tracked_points[i];

            cv::Point2f point_image_velocity;

            if (frame_time_diff < 1e-5)
                point_image_velocity = cv::Point2f(1e-3, 1e-3);
            else
                point_image_velocity = (cur_tracked_points[i] - last_tracked_points[i]) / frame_time_diff;

            rgb_point_ptr->image_velocity = Eigen::Vector2d(point_image_velocity.x, point_image_velocity.y);
        }
    }

    if (distance > 0)
        rejectErrorTrackingPoints(p_frame, distance);

    old_gray = gray_image.clone();
    old_image = cur_image;
    
    std::vector<cv::Point2f>().swap(last_tracked_points);
    last_tracked_points = cur_tracked_points;

    updateLastTrackingVectorAndIds();

    image_idx++;
    last_image_time = current_image_time;
}

void opticalFlowTracker::init(cloudFrame *p_frame, std::vector<rgbPoint*> &rgb_points_vec, std::vector<cv::Point2f> &points_2d_vec)
{
    setTrackPoints(p_frame->rgb_image, rgb_points_vec, points_2d_vec);

    current_image_time = p_frame->time_sweep_end;
    last_image_time = current_image_time;

    std::vector<uchar> status;
    lk_optical_flow_kernel->trackImage(p_frame->gray_image, last_tracked_points, cur_tracked_points, status);
}

void opticalFlowTracker::setTrackPoints(cv::Mat &image, std::vector<rgbPoint*> &rgb_points_vec, std::vector<cv::Point2f> &points_2d_vec)
{
    old_image = image.clone();
    cv::cvtColor(old_image, old_gray, cv::COLOR_BGR2GRAY);
    map_rgb_points_in_last_image_pose.clear();

    for (unsigned int i = 0; i < rgb_points_vec.size(); i++)
    {
        map_rgb_points_in_last_image_pose[(void*)rgb_points_vec[i]] = points_2d_vec[i];
    }

    updateLastTrackingVectorAndIds();
}

void opticalFlowTracker::rejectErrorTrackingPoints(cloudFrame *p_frame, double distance)
{
    double u, v;

    int remove_count = 0;
    int total_count = map_rgb_points_in_cur_image_pose.size();

    scope_color(ANSI_COLOR_BLUE_BOLD);

    for (auto it = map_rgb_points_in_cur_image_pose.begin(); it != map_rgb_points_in_cur_image_pose.end(); it++)
    {
        cv::Point2f predicted_point = it->second;

        Eigen::Vector3d point_position = ((rgbPoint*)it->first)->getPosition();

        int res = p_frame->project3dPointInThisImage(point_position, u, v, nullptr, 1.0);

        if (res)
        {
            if ((fabs(u - predicted_point.x ) > distance) || (fabs(v - predicted_point.y) > distance))
            {
                // Remove tracking pts
                map_rgb_points_in_cur_image_pose.erase(it);
                remove_count++;
            }
        }
        else
        {
            map_rgb_points_in_cur_image_pose.erase(it);
            remove_count++;
        }
    }
}

void opticalFlowTracker::updateLastTrackingVectorAndIds()
{
    int idx = 0;

    last_tracked_points.clear();
    rgb_points_ptr_vec_in_last_image.clear();

    old_ids.clear();

    for (auto it = map_rgb_points_in_last_image_pose.begin(); it != map_rgb_points_in_last_image_pose.end(); it++)
    {
        rgb_points_ptr_vec_in_last_image.push_back(it->first);
        last_tracked_points.push_back(it->second);

        old_ids.push_back(idx);

        idx++;
    }
}

bool opticalFlowTracker::removeOutlierUsingRansacPnp(cloudFrame *p_frame, int if_remove_ourlier)
{
    cv::Mat cv_so3, cv_trans; //两个opencv矩阵对象
    Eigen::Vector3d eigen_so3, eigen_trans;

    std::vector<cv::Point3f> points_3d;
    std::vector<cv::Point2f> points_2d;

    std::vector<void *> map_ptr;

    for (auto it = map_rgb_points_in_cur_image_pose.begin(); it != map_rgb_points_in_cur_image_pose.end(); it++)
    {
        map_ptr.push_back(it->first);
        Eigen::Vector3d point_3d = ((rgbPoint*)it->first)->getPosition();

        points_3d.push_back(cv::Point3f(point_3d(0), point_3d(1), point_3d(2)));
        points_2d.push_back(it->second);
    }

    if (points_3d.size() < 10)
    {
        std::cerr << "三维点小于10个:" << points_3d.size() << std::endl;
        return false;
    }

    // std::vector<int> status;
    std::vector<int> status(points_2d.size(), 0); // 初始化为0，表示所有点初始状态为未知

    // try
    // {
    //     cv::solvePnPRansac(points_3d, points_2d, intrinsic, cv::Mat(), cv_so3, cv_trans, false, 200, 1.5, 0.99, status); // SOLVEPNP_ITERATIVE
    // }
    // catch (cv::Exception &e)
    // {
    //     scope_color(ANSI_COLOR_RED_BOLD);
    //     std::cout << "Catching a cv exception: " << e.msg << std::endl;
    //     return 0;
    // }

    try
    {
      if (points_3d.empty() || points_2d.empty())
       {
        std::cerr << "Error: Input points matrices are empty." << std::endl;
        return 0;
       }

      cv::Mat cv_so3, cv_trans;
      cv::solvePnPRansac(points_3d, points_2d, intrinsic, cv::Mat(), cv_so3, cv_trans, false, 200, 1.5, 0.99, status);
    }
    catch (cv::Exception &e)
    {
      std::cerr << "Catching a cv exception: " << e.msg << std::endl;
      return 0;
    }

    if (if_remove_ourlier)
    {
        // Remove outlier
        map_rgb_points_in_last_image_pose.clear();
        map_rgb_points_in_cur_image_pose.clear();

        for (unsigned int i = 0; i < status.size(); i++)
        {
            int inlier_idx = status[i];
            {
                map_rgb_points_in_last_image_pose[map_ptr[inlier_idx]] = points_2d[inlier_idx];
                map_rgb_points_in_cur_image_pose[map_ptr[inlier_idx]] = points_2d[inlier_idx];
            }
        }
    }

    updateLastTrackingVectorAndIds();

    return true;
}