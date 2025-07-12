#pragma once
// c++
#include <iostream>
#include <math.h>

// ros
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

// eigen 
#include <Eigen/Core>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include "lioOptimization.h"
#include "opticalFlowTracker.h"
#include "rgbMapTracker.h"

#define INIT_COV (0.0001)

class cloudFrame;
class opticalFlowTracker;
class rgbMapTracker;

class imageProcessing
{
private:

	int image_width;	// raw image width
	int image_height;	// raw image height

	int calibrated_image_width;		// processed image rows
	int calibrated_image_height;		// processed image cols

	Eigen::Matrix3d camera_intrinsic;
    Eigen::Matrix<double, 5, 1> camera_dist_coeffs;

    cv::Mat intrinsic, dist_coeffs;
    cv::Mat m_ud_map1, m_ud_map2;

	Eigen::Matrix3d R_imu_camera;
	Eigen::Vector3d t_imu_camera;

	bool ifEstimateCameraIntrinsic;
	bool ifEstimateExtrinsic;

	Eigen::Matrix<double, 11, 11> covariance;

	double image_resize_ratio;
	double image_scale_factor;

	bool first_data;

	int maximum_tracked_points;

	int track_windows_size;

	int num_iterations;

	double tracker_minimum_depth;
    double tracker_maximum_depth;

    double cam_measurement_weight;

public:

	double time_last_process;

	opticalFlowTracker *op_tracker;
	rgbMapTracker *map_tracker;

	imageProcessing();

	void setImageWidth(int &para);
	void setImageHeight(int &para);

	void setCameraIntrinsic(std::vector<double> &v_camera_intrinsic);
	void setCameraDistCoeffs(std::vector<double> &v_camera_dist_coeffs);

	void setExtrinR(Eigen::Matrix3d &R);
	void setExtrinT(Eigen::Vector3d &t);

	void setInitialCov();

	Eigen::Matrix3d getCameraIntrinsic();

	void process(voxelHashMap &voxel_map, cloudFrame *p_frame);

	void imageEqualize(cv::Mat &image, int amp);
	cv::Mat initCubicInterpolation(cv::Mat &image);
	cv::Mat equalizeColorImageYcrcb(cv::Mat &image);

	bool vioEsikf(cloudFrame *p_frame);

	bool vioPhotometric(cloudFrame *p_frame);

	void updateCameraParameters(cloudFrame *p_frame, Eigen::Matrix<double, 11, 1> &d_x);

	void updateCameraParameters(cloudFrame *p_frame, Eigen::Matrix<double, 6, 1> &d_x);

	void printParameter();
};