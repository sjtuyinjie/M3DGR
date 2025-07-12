#pragma once

// c++
#include <iostream>
#include <math.h>
#include <unordered_set>

// eigen 
#include <Eigen/Core>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>

#include "cloudMap.h"
#include "lioOptimization.h"

class cloudFrame;

class rgbMapTracker
{
public:

	std::vector<voxelId> voxels_recent_visited;

	std::vector<rgbPoint*> rgb_points_vec;
	std::vector<rgbPoint*> *points_rgb_vec_for_projection;

	std::shared_ptr<std::mutex> mutex_rgb_points_vec;

	cloudFrame *p_cloud_frame;

	double minimum_depth_for_projection;
    double maximum_depth_for_projection;

    double recent_visited_voxel_activated_time;

    int number_of_new_visited_voxel;

    int updated_frame_index;

    std::shared_ptr<std::mutex> mutex_frame_index;

    bool in_appending_points;

    rgbMapTracker();

    void threadRenderPointsInVoxel(voxelHashMap &map, const int &voxel_start, const int &voxel_end, cloudFrame *p_frame, 
    	const std::vector<voxelId> *voxels_for_render, const double obs_time);

    void renderPointsInRecentVoxel(voxelHashMap &map, cloudFrame *p_frame, std::vector<voxelId> *voxels_for_render, const double &obs_time);

    void refreshPointsForProjection(voxelHashMap &map);

    void selectPointsForProjection(voxelHashMap &map, cloudFrame *p_frame, std::vector<rgbPoint*> *pc_out_vec = nullptr, std::vector<cv::Point2f> *pc_2d_out_vec = nullptr, 
    	double minimum_dis = 5, int skip_step = 1, bool use_all_points = false);

    void updatePoseForProjection(cloudFrame *p_frame, double fov_margin = 0.0001);
};