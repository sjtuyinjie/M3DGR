#include "imageProcessing.h"
#include "utility.h"

imageProcessing::imageProcessing()
{
	time_last_process = -1e5;

	op_tracker = new opticalFlowTracker();
	map_tracker = new rgbMapTracker();

	image_resize_ratio = 1.0;
	image_scale_factor = image_resize_ratio;

	maximum_tracked_points = 300;
	track_windows_size = 40;

	tracker_minimum_depth = 0.1;
	tracker_maximum_depth = 200;

	num_iterations = 2;

	cam_measurement_weight =  1e-3;

	ifEstimateCameraIntrinsic = true;
	ifEstimateExtrinsic = true;

	first_data = true;

	setInitialCov();
}

void imageProcessing::setImageWidth(int &para)
{
	image_width = para;
}

void imageProcessing::setImageHeight(int &para)
{
	image_height = para;
}

void imageProcessing::setCameraIntrinsic(std::vector<double> &v_camera_intrinsic)
{
	camera_intrinsic << v_camera_intrinsic[0], v_camera_intrinsic[1], v_camera_intrinsic[2], 
						v_camera_intrinsic[3], v_camera_intrinsic[4], v_camera_intrinsic[5], 
						v_camera_intrinsic[6], v_camera_intrinsic[7], v_camera_intrinsic[8];
}

void imageProcessing::setCameraDistCoeffs(std::vector<double> &v_camera_dist_coeffs)
{
	camera_dist_coeffs << v_camera_dist_coeffs[0], v_camera_dist_coeffs[1], v_camera_dist_coeffs[2], 
						  v_camera_dist_coeffs[3], v_camera_dist_coeffs[4];
}

void imageProcessing::setExtrinR(Eigen::Matrix3d &R)
{
	R_imu_camera = R;
}

void imageProcessing::setExtrinT(Eigen::Vector3d &t)
{
	t_imu_camera = t;
}

void imageProcessing::setInitialCov()
{
    // Set cov
    covariance = Eigen::MatrixXd::Identity(11, 11) * INIT_COV;
    covariance(0, 0) = 0.00001;
    covariance.block<6, 6>(1, 1) = Eigen::MatrixXd::Identity(6, 6) * 1e-3; // extrinsic between camera and IMU
    covariance.block<4, 4>(7, 7) = Eigen::MatrixXd::Identity(4, 4) * 1e-3; // camera intrinsic
}

Eigen::Matrix3d imageProcessing::getCameraIntrinsic()
{
	return camera_intrinsic;
}

void imageProcessing::printParameter()
{
	std::cout << "image_width: " << image_width << std::endl;
	std::cout << "image_height: " << image_height << std::endl;
	std::cout << "camera_intrinsic: " << std::fixed << camera_intrinsic << std::endl;
	std::cout << "camera_dist_coeffs: " << std::fixed << camera_dist_coeffs.transpose() << std::endl;
	std::cout << "R_imu_camera: " << std::fixed << R_imu_camera << std::endl;
	std::cout << "t_imu_camera: " << std::fixed << t_imu_camera.transpose() << std::endl;
}

void imageProcessing::process(voxelHashMap &voxel_map, cloudFrame *p_frame)
{
	if (first_data)
	{
		image_scale_factor = image_width * image_resize_ratio / p_frame->rgb_image.cols;

		camera_intrinsic(0, 0) = camera_intrinsic(0, 0) / image_scale_factor;
		camera_intrinsic(0, 2) = camera_intrinsic(0, 2) / image_scale_factor;
		camera_intrinsic(1, 1) = camera_intrinsic(1, 1) / image_scale_factor;
		camera_intrinsic(1, 2) = camera_intrinsic(1, 2) / image_scale_factor;

		cv::eigen2cv(camera_intrinsic, intrinsic);
		cv::eigen2cv(camera_dist_coeffs, dist_coeffs);

		initUndistortRectifyMap(intrinsic, dist_coeffs, cv::Mat(), intrinsic, cv::Size(image_width/image_scale_factor, 
								image_height/image_scale_factor), CV_16SC2, m_ud_map1, m_ud_map2);

		op_tracker->setIntrinsic(camera_intrinsic, camera_dist_coeffs * 0, cv::Size(image_width/image_scale_factor, image_height/image_scale_factor));
		op_tracker->maximum_tracked_points = maximum_tracked_points;

		map_tracker->minimum_depth_for_projection = tracker_minimum_depth;
    	map_tracker->maximum_depth_for_projection = tracker_maximum_depth;
	}

	if (fabs(image_resize_ratio - 1.0) > 1e-6)
    {
    	cv::Mat temp_img;
        cv::resize(p_frame->rgb_image, temp_img, cv::Size(image_width/image_scale_factor, image_height/image_scale_factor));
        p_frame->rgb_image = temp_img;
    }

    cv::Mat image_undistort;
    cv::remap(p_frame->rgb_image, image_undistort, m_ud_map1, m_ud_map2, cv::INTER_LINEAR);

    p_frame->gray_image = initCubicInterpolation(image_undistort);
    imageEqualize(p_frame->gray_image, 3.0);
    p_frame->rgb_image = equalizeColorImageYcrcb(image_undistort);

    if (first_data)
    {
    	std::vector<cv::Point2f> points_2d_vec_temp;
        std::vector<rgbPoint*> rgb_points_vec_temp;
    	map_tracker->selectPointsForProjection(voxel_map, p_frame, &rgb_points_vec_temp, &points_2d_vec_temp, track_windows_size / image_scale_factor, 1);
        op_tracker->init(p_frame, rgb_points_vec_temp, points_2d_vec_temp);

		first_data = false;
    }
    

    // if (p_frame == nullptr) {
    //     std::cout << "p_frame is null." << std::endl;
    // } else {
    //     std::cout << "p_frame is not null." << std::endl;
    // }
    // op_tracker->trackImage(p_frame, -20);
    op_tracker->trackImage(p_frame, -20);

    bool enough_points = true;

    if (op_tracker->removeOutlierUsingRansacPnp(p_frame) == 0)
    {
    	enough_points = false;
    	std::cout << ANSI_COLOR_RED_BOLD << "****** Remove_outlier_using_ransac_pnp error*****" << ANSI_COLOR_RESET << std::endl;
    }
    

    bool res_esikf = true, res_photometric = true;

    if (enough_points)
    	res_esikf = vioEsikf(p_frame);

    if (enough_points)
    	res_photometric = vioPhotometric(p_frame);

    map_tracker->renderPointsInRecentVoxel(voxel_map, p_frame, &map_tracker->voxels_recent_visited, p_frame->time_sweep_end);

    map_tracker->updatePoseForProjection(p_frame, -0.4);

    map_tracker->refreshPointsForProjection(voxel_map);

    op_tracker->updateAndAppendTrackPoints(p_frame, map_tracker, track_windows_size / image_scale_factor, 1000000);

    time_last_process = p_frame->time_sweep_end;
}

void imageProcessing::imageEqualize(cv::Mat &image, int amp)
{
    cv::Mat image_temp;
    cv::Size eqa_image_size = cv::Size(std::max(image.cols * 32.0/640, 4.0), std::max(image.cols * 32.0/640, 4.0));
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(amp, eqa_image_size);
    clahe->apply(image, image_temp);
    image = image_temp;
}

cv::Mat imageProcessing::initCubicInterpolation(cv::Mat &image)
{
    calibrated_image_width = image.rows;
    calibrated_image_height = image.cols;
    cv::Mat image_gray;
    cv::cvtColor(image, image_gray, cv::COLOR_RGB2GRAY);

    return image_gray;
}

cv::Mat imageProcessing::equalizeColorImageYcrcb(cv::Mat &image)
{
    cv::Mat hist_equalized_image;
    cv::cvtColor(image, hist_equalized_image, cv::COLOR_BGR2YCrCb);

    //Split the image into 3 channels; Y, Cr and Cb channels respectively and store it in a std::vector
    std::vector<cv::Mat> vec_channels;
    cv::split(hist_equalized_image, vec_channels);

    //Equalize the histogram of only the Y channel
    imageEqualize(vec_channels[0], 1);
    cv::merge(vec_channels, hist_equalized_image);
    cv::cvtColor(hist_equalized_image, hist_equalized_image, cv::COLOR_YCrCb2BGR);

    return hist_equalized_image;
}

double getHuberLoss(double residual, double outlier_threshold = 1.0)
{
    double scale = 1.0;

    if (residual / outlier_threshold < 1.0)
    {
        scale = 1.0;
    }
    else
    {
        scale = (2 * sqrt(residual) / sqrt(outlier_threshold) - 1.0) / residual;
    }

    return scale;
}

const int minimum_iteration_points = 10;

bool imageProcessing::vioEsikf(cloudFrame *p_frame)
{
    scope_color(ANSI_COLOR_BLUE_BOLD);

    if (!ifEstimateCameraIntrinsic)
    {
        p_frame->p_state->fx = camera_intrinsic(0, 0);
        p_frame->p_state->fy = camera_intrinsic(1, 1);
        p_frame->p_state->cx = camera_intrinsic(0, 2);
        p_frame->p_state->cy = camera_intrinsic(1, 2);
    }

    // if (!ifEstimateExtrinsic)
    // {
    // 	p_frame->p_state->R_imu_camera = R_imu_camera;
    //     p_frame->p_state->t_imu_camera = t_imu_camera;
    // }

    if (!ifEstimateExtrinsic)
    {
    // 检查 p_frame 是否为空
      if (p_frame != nullptr)
      {
        // 检查 p_state 是否为空
        if (p_frame->p_state != nullptr)
        {
            p_frame->p_state->R_imu_camera = R_imu_camera;
            p_frame->p_state->t_imu_camera = t_imu_camera;
        }
        else
        {
            // 处理 p_state 为空的情况，例如输出错误信息
            std::cerr << "Error: p_state is null." << std::endl;
        }
      }
      else
      {
        // 处理 p_frame 为空的情况，例如输出错误信息
        std::cerr << "Error: p_frame is null." << std::endl;
      }
    }    


    Eigen::Matrix<double, -1, -1> H_mat;
    Eigen::Matrix<double, 11, 1> solution;
    Eigen::Matrix<double, -1, 1> residual_vec;
    Eigen::Matrix<double, 11, 1> HTr;
    Eigen::Matrix<double, 11, 11> HTH;

    Eigen::Matrix<double, 11, -1> K;
    Eigen::Matrix<double, 11, 1> d_x;

    int total_point_size = op_tracker->map_rgb_points_in_cur_image_pose.size();

    if (total_point_size < minimum_iteration_points)
    {
        return false;
    }

    H_mat.resize(total_point_size * 2, 11);
    residual_vec.resize(total_point_size * 2, 1);

    K.resize(11, total_point_size * 2);

    double t_predict = p_frame->p_state->time_td;
    Eigen::Vector3d p_predict = p_frame->p_state->t_imu_camera;
    Eigen::Quaterniond q_predict = Eigen::Quaterniond(p_frame->p_state->R_imu_camera);
    double fx_predict = p_frame->p_state->fx;
    double fy_predict = p_frame->p_state->fy;
    double cx_predict = p_frame->p_state->cx;
    double cy_predict = p_frame->p_state->cy;

    int num_used_point_count = 0;

    double acc_residual = 0;
    double last_acc_residual = 3e8;

    cam_measurement_weight = std::max(0.001, std::min(5.0 / map_tracker->number_of_new_visited_voxel, 0.01));

    for (int iter_count = 0; iter_count < num_iterations; iter_count++)
    {
        int point_idx = -1;
        acc_residual = 0;

        Eigen::Vector3d point_world, point_camera;
        Eigen::Vector2d pixel_match, pixel_projection, pixel_velocity;

        H_mat.setZero();
        solution.setZero();
        residual_vec.setZero();

        K.setZero();
        d_x.setZero();

        double d_t = p_frame->p_state->time_td - t_predict;
        Eigen::Vector3d d_p = p_frame->p_state->t_imu_camera - p_predict;
        Eigen::Quaterniond d_q = q_predict.inverse() * Eigen::Quaterniond(p_frame->p_state->R_imu_camera);
        Eigen::Vector3d d_so3 = numType::quatToSo3(d_q);
        double d_fx = p_frame->p_state->fx - fx_predict;
        double d_fy = p_frame->p_state->fy - fy_predict;
        double d_cx = p_frame->p_state->cx - cx_predict;
        double d_cy = p_frame->p_state->cy - cy_predict;

        d_x(0) = d_t;
        d_x.segment<3>(1) = d_so3;
        d_x.segment<3>(4) = d_p;
        d_x(7) = d_fx;
        d_x(8) = d_fy;
        d_x(9) = d_cx;
        d_x(10) = d_cy;

        num_used_point_count = 0;

		for (auto it = op_tracker->map_rgb_points_in_last_image_pose.begin(); it != op_tracker->map_rgb_points_in_last_image_pose.end(); it++)
		{
			point_world = ((rgbPoint*)it->first)->getPosition();
			pixel_velocity = ((rgbPoint*)it->first)->image_velocity;
			pixel_match = Eigen::Vector2d(it->second.x, it->second.y);

			point_camera = p_frame->p_state->q_camera_world.toRotationMatrix() * point_world + p_frame->p_state->t_camera_world;
			pixel_projection = Eigen::Vector2d(p_frame->p_state->fx * point_camera(0) / point_camera(2) + p_frame->p_state->cx, 
				p_frame->p_state->fy * point_camera(1) / point_camera(2) + p_frame->p_state->cy) + p_frame->p_state->time_td * pixel_velocity;

			double residual = (pixel_projection - pixel_match).norm();
			double huber_loss = getHuberLoss(residual);

			point_idx++;
			acc_residual += residual;

			residual_vec.block<2, 1>(point_idx * 2, 0) = (pixel_projection - pixel_match) * huber_loss;

			num_used_point_count++;

			Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_u_pc;

			J_u_pc << p_frame->p_state->fx / point_camera.z(), 0, - (p_frame->p_state->fx * point_camera.x()) / (point_camera.z() * point_camera.z()), 
				0, p_frame->p_state->fy / point_camera.z(), - (p_frame->p_state->fy * point_camera.y()) / (point_camera.z() * point_camera.z());

			Eigen::Matrix<double, 2, 4, Eigen::RowMajor> J_u_K;

			J_u_K << point_camera.x() / point_camera.z(), 0, 1, 0, 0, point_camera.y() / point_camera.z(), 0, 1;

			H_mat.block<2, 1>(point_idx * 2, 0) = pixel_velocity * huber_loss;

			if (ifEstimateExtrinsic)
			{
				H_mat.block<2, 3>(point_idx * 2, 1) = J_u_pc * numType::skewSymmetric(point_camera) * huber_loss;
				H_mat.block<2, 3>(point_idx * 2, 4) = - J_u_pc * p_frame->p_state->R_imu_camera.transpose() * huber_loss;
			}

			if (ifEstimateCameraIntrinsic)
			{
				H_mat.block<2, 4>(point_idx * 2, 7) = J_u_K * huber_loss;
			}
		}

		acc_residual /= total_point_size;

		if (num_used_point_count < minimum_iteration_points)
        {
            break;
        }

		Eigen::Matrix<double, 11, 11> J_zero = Eigen::MatrixXd::Identity(11, 11);
		J_zero.block<3, 3>(1, 1) = Eigen::Matrix3d::Identity() - 0.5 * numType::skewSymmetric(d_x.segment<3>(1));
                
         try {
           auto term = (J_zero * covariance * J_zero.transpose() * cam_measurement_weight);
           if (term.determinant() == 0) 
           {
            throw std::runtime_error("Matrix is singular and cannot be inverted.");
           }    
           K = (H_mat.transpose() * H_mat + term.inverse()).inverse() * H_mat.transpose();
           } 
         catch (const std::exception& e) {
           std::cerr << "Error: " << e.what() << std::endl;}

		// K = (H_mat.transpose() * H_mat + (J_zero * covariance * J_zero.transpose() * cam_measurement_weight).inverse()).inverse() * H_mat.transpose();
		solution = - K * residual_vec - (Eigen::Matrix<double, 11, 11>::Identity() - K * H_mat) * J_zero * d_x;

		updateCameraParameters(p_frame, solution);

        if (fabs(acc_residual - last_acc_residual) < 0.01)
        {
            break;
        }

        last_acc_residual = acc_residual;
	}

	Eigen::Matrix<double, 11, 11> J_k = Eigen::MatrixXd::Identity(11, 11);
	J_k.block<3, 3>(1, 1) = Eigen::Matrix3d::Identity() - 0.5 * numType::skewSymmetric(solution.segment<3>(1));

	covariance = J_k * (Eigen::Matrix<double, 11, 11>::Identity() - K * H_mat) * covariance * J_k.transpose();

	return true;
}

void imageProcessing::updateCameraParameters(cloudFrame *p_frame, Eigen::Matrix<double, 11, 1> &d_x)
{
	p_frame->p_state->time_td += d_x(0);

	Eigen::Quaterniond q_imu_camera = Eigen::Quaterniond(p_frame->p_state->R_imu_camera);
	q_imu_camera = (q_imu_camera * numType::so3ToQuat(d_x.segment<3>(1))).normalized();

	p_frame->p_state->R_imu_camera = q_imu_camera.toRotationMatrix();
	p_frame->p_state->t_imu_camera += d_x.segment<3>(4);
	p_frame->p_state->fx += d_x(7);
	p_frame->p_state->fy += d_x(8);
	p_frame->p_state->cx += d_x(9);
	p_frame->p_state->cy += d_x(10);

	p_frame->p_state->q_world_camera = Eigen::Quaterniond(p_frame->p_state->rotation.toRotationMatrix() * p_frame->p_state->R_imu_camera);
    p_frame->p_state->t_world_camera = p_frame->p_state->rotation.toRotationMatrix() * p_frame->p_state->t_imu_camera + p_frame->p_state->translation;

    p_frame->refreshPoseForProjection();
}

bool imageProcessing::vioPhotometric(cloudFrame *p_frame)
{
    Eigen::Matrix<double, -1, -1> H_mat, R_mat_inv, sqrt_info;
    Eigen::Matrix<double, 6, 1> solution;
    Eigen::Matrix<double, -1, 1> residual_vec;
    Eigen::Matrix<double, 6, 1> HTr;
    Eigen::Matrix<double, 6, 6> HTH;

    Eigen::Matrix<double, 6, -1> K;
    Eigen::Matrix<double, 6, 1> d_x;

    int total_point_size = op_tracker->map_rgb_points_in_cur_image_pose.size();

    if (total_point_size < minimum_iteration_points)
    {
        return false;
    }

    H_mat.resize(total_point_size * 3, 6);
    residual_vec.resize(total_point_size * 3, 1);
    R_mat_inv.resize(total_point_size * 3, total_point_size * 3);
    sqrt_info.resize(total_point_size * 3, total_point_size * 3);

    K.resize(6, total_point_size * 3);

    Eigen::Vector3d p_predict = p_frame->p_state->t_imu_camera;
    Eigen::Quaterniond q_predict = Eigen::Quaterniond(p_frame->p_state->R_imu_camera);

    int num_used_point_count = 0;

    double acc_residual = 0;
    double last_acc_residual = 3e8;

    cam_measurement_weight = std::max(0.001, std::min(5.0 / map_tracker->number_of_new_visited_voxel, 0.01));

    for (int iter_count = 0; iter_count < num_iterations; iter_count++)
    {
        int point_idx = -1;
        acc_residual = 0;

        Eigen::Vector3d point_world, point_camera;
        Eigen::Vector2d pixel_match, pixel_projection, pixel_velocity;

        H_mat.setZero();
        solution.setZero();
        residual_vec.setZero();
        R_mat_inv.setZero();
        sqrt_info.setZero();

        K.setZero();
        d_x.setZero();

        Eigen::Vector3d d_p = p_frame->p_state->t_imu_camera - p_predict;
        Eigen::Quaterniond d_q = q_predict.inverse() * Eigen::Quaterniond(p_frame->p_state->R_imu_camera);
        Eigen::Vector3d d_so3 = numType::quatToSo3(d_q);

        d_x.head<3>() = d_so3;
        d_x.tail<3>() = d_p;

        num_used_point_count = 0;

        for (auto it = op_tracker->map_rgb_points_in_last_image_pose.begin(); it != op_tracker->map_rgb_points_in_last_image_pose.end(); it++)
		{
			if (((rgbPoint*)it->first)->N_rgb < 3) continue;

			point_idx++;

			point_world = ((rgbPoint*)it->first)->getPosition();
			pixel_velocity = ((rgbPoint*)it->first)->image_velocity;

			point_camera = p_frame->p_state->q_camera_world.toRotationMatrix() * point_world + p_frame->p_state->t_camera_world;
			pixel_projection = Eigen::Vector2d(p_frame->p_state->fx * point_camera(0) / point_camera(2) + p_frame->p_state->cx, 
				p_frame->p_state->fy * point_camera(1) / point_camera(2) + p_frame->p_state->cy) + p_frame->p_state->time_td * pixel_velocity;

			Eigen::Vector3d point_color = ((rgbPoint*)it->first)->getRgb();
			Eigen::Matrix3d point_rgb_info = Eigen::Matrix3d::Zero();
            Eigen::Matrix3d point_rgb_cov = ((rgbPoint*)it->first)->getCovRgb();

            for (int i = 0; i < 3; i++)
            {
                point_rgb_info(i, i) = 1.0 / point_rgb_cov(i, i);
                R_mat_inv(point_idx * 3 + i, point_idx * 3 + i) = point_rgb_info(i, i);
                sqrt_info(point_idx * 3 + i, point_idx * 3 + i) = sqrt(R_mat_inv(point_idx * 3 + i, point_idx * 3 + i));
                // R_mat_inv(point_idx * 3 + i, point_idx * 3 + i) = 1.0;
            }

            Eigen::Vector3d obs_color_dx, obs_color_dy;
            Eigen::Vector3d obs_color = p_frame->getRgb(pixel_projection(0), pixel_projection(1), 0, &obs_color_dx, &obs_color_dy);
            Eigen::Vector3d residual = obs_color - point_color;

            double huber_loss = getHuberLoss(residual.norm());
            residual *= huber_loss;

            residual_vec.block<3, 1>(point_idx * 3, 0) = (obs_color - point_color) * huber_loss;

            acc_residual += residual.transpose() * point_rgb_info * residual;

            Eigen::Matrix<double, 3, 2, Eigen::RowMajor> J_color_u;

            J_color_u.block<3, 1>(0, 0) = obs_color_dx;
            J_color_u.block<3, 1>(0, 1) = obs_color_dy;

            num_used_point_count++;

            Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_u_pc;

			J_u_pc << p_frame->p_state->fx / point_camera.z(), 0, - (p_frame->p_state->fx * point_camera.x()) / (point_camera.z() * point_camera.z()), 
				0, p_frame->p_state->fy / point_camera.z(), - (p_frame->p_state->fy * point_camera.y()) / (point_camera.z() * point_camera.z());

			Eigen::Matrix3d J_color_pc = J_color_u * J_u_pc;

			if (ifEstimateExtrinsic)
			{
				H_mat.block<3, 3>(point_idx * 3, 0) = J_color_pc * numType::skewSymmetric(point_camera) * huber_loss;
				H_mat.block<3, 3>(point_idx * 3, 3) = - J_color_pc * p_frame->p_state->R_imu_camera.transpose() * huber_loss;
			}
		}

		if (num_used_point_count < minimum_iteration_points)
        {
            break;
        }

		Eigen::Matrix<double, 6, 6> J_zero = Eigen::MatrixXd::Identity(6, 6);
		J_zero.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() - 0.5 * numType::skewSymmetric(d_x.head<3>());

		K = (H_mat.transpose() * R_mat_inv * H_mat + (J_zero * covariance.block<6, 6>(1, 1) * J_zero.transpose() * cam_measurement_weight).inverse()).inverse() * H_mat.transpose() * R_mat_inv;
		solution = - K * residual_vec - (Eigen::Matrix<double, 6, 6>::Identity() - K * H_mat) * J_zero * d_x;

		updateCameraParameters(p_frame, solution);

		if ((acc_residual / total_point_size) < 10)
        {
            break;
        }

        if (fabs(acc_residual - last_acc_residual) < 0.01)
        {
            break;
        }

		last_acc_residual = acc_residual;
    }

	Eigen::Matrix<double, 6, 6> J_k = Eigen::MatrixXd::Identity(6, 6);
	J_k.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() - 0.5 * numType::skewSymmetric(solution.head<3>());

	covariance.block<6, 6>(1, 1) = J_k * (Eigen::Matrix<double, 6, 6>::Identity() - K * H_mat) * covariance.block<6, 6>(1, 1) * J_k.transpose();

    return true;
}

void imageProcessing::updateCameraParameters(cloudFrame *p_frame, Eigen::Matrix<double, 6, 1> &d_x)
{
	Eigen::Quaterniond q_imu_camera = Eigen::Quaterniond(p_frame->p_state->R_imu_camera);
	q_imu_camera = (q_imu_camera * numType::so3ToQuat(d_x.segment<3>(0))).normalized();

	p_frame->p_state->R_imu_camera = q_imu_camera.toRotationMatrix();
	p_frame->p_state->t_imu_camera += d_x.segment<3>(3);

	p_frame->p_state->q_world_camera = Eigen::Quaterniond(p_frame->p_state->rotation.toRotationMatrix() * p_frame->p_state->R_imu_camera);
    p_frame->p_state->t_world_camera = p_frame->p_state->rotation.toRotationMatrix() * p_frame->p_state->t_imu_camera + p_frame->p_state->translation;

    p_frame->refreshPoseForProjection();
}