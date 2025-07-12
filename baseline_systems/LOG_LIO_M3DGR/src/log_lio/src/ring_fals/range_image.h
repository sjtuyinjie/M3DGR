//
// Created by hk on 12/8/22.
//


#include <vector>
#include <array>
#include <opencv2/core.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

#include "point_type.h"

#include "Image_normals.hpp"

#ifndef LVI_SAM_RANGE_IMAGE_H
#define LVI_SAM_RANGE_IMAGE_H

//in lidar frame
class RangeImage
{
    typedef pcl::PointXYZI PointType;
    typedef cv::Matx<float, 3, 3> Mat33T;
    typedef cv::Vec<float, 9> Vec9T;
    typedef cv::Vec<float, 3> Vec3T;
public:
    explicit RangeImage()
    : num_bins_x(360)
    , num_bins_y(360)
    , lidar_fals()
    {
//        range_image = cv::Mat(num_bins_y, num_bins_x, CV_32F, cv::Scalar::all(FLT_MAX));
    }

    explicit RangeImage(const int& rows, const int& cols  )
            : rows_(rows)
            , cols_(cols)
            , lidar_fals(rows, cols, 3) //(row，col，window_size)
    {
//        range_image = cv::Mat(num_bins_y, num_bins_x, CV_32F, cv::Scalar::all(FLT_MAX));

        //储存计算M逆矩阵的中间变量
        cos_theta =  cv::Mat(rows_, cols_, CV_32F);
        sin_theta =  cv::Mat(rows_, cols_, CV_32F);
        cos_phi = cv::Mat(rows_, cols_, CV_32F);
        sin_phi =  cv::Mat(rows_, cols_, CV_32F);
        num_per_pixel.resize(rows_ * cols_, 0);
    }

    void createFromRings(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud);

    //create range imageg and build lookup table for normal computation
    void createAndBuildTableFromRings(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud);
    void createAndBuildTableFromRings(const int& num_rings, const int& num_horizon_scan
            , const pcl::PointCloud<PointXYZIRT>::Ptr& cloud);

    void buildTableFromRings(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud);
    void buildTableFromRings(const int& num_rings, const int& num_horizon_scan
            , const pcl::PointCloud<PointXYZIRT>::Ptr& cloud);

    void createFromCloud(const int& num_bins_v, const int& num_bins_h,
                         const pcl::PointCloud<PointType>::Ptr& cloud);
    void createFromCloud(const pcl::PointCloud<PointType>::Ptr& cloud);

    void buildTableFromCloud();

    void saveRangeImage(const std::string& file);

    void computeNormals();
    void computeNormals(cv::OutputArray normals_out);
    void computeNormals(const cv::Mat &range_image, cv::OutputArray normals_out);
    void computeNormals(const cv::Mat &range_image, cv::OutputArray normals_out, cv::Mat &residual);

    void computeLookupTable();

    bool loadLookupTable(std::string dir, std::string filename)
    {return lidar_fals.loadMInverse(dir, filename);}

    void saveLookupTable(std::string dir, std::string filename)
    {lidar_fals.saveMInverse(dir, filename);}

    void computeMInverse() {lidar_fals.computeMInverse(cos_theta, sin_theta, cos_phi, sin_phi);}

    enum CoordinateFrame
    {
        CAMERA_FRAME = 0,
        LASER_FRAME  = 1
    };

    // The actual range iamge.
    cv::Mat range_image;

    //only for visualization
    std::vector<int> cloud_id_image; //

private:

    int num_bins_x, num_bins_y;

    float resolution_x, resolution_y;

    float range_min = 1.0;
    float range_max = 200.0;

    float input_value_max = 0;
    int downsampleRate = 1; //

    cv::Scalar range_init = cv::Scalar::all(FLT_MAX);

    int num_cloud = 0;
    int min_table_cloud = 10;

    int rows_, cols_;
    int depth_ = CV_32F;
    cv::Mat K_;
    int window_size_;
    int method_;
//    mutable void* rgbd_normals_impl_;
    LIDAR_FALS lidar_fals;
    bool table_valid = false;

    cv::Mat cos_theta;
    cv::Mat sin_theta;
    cv::Mat cos_phi;
    cv::Mat sin_phi;
    std::vector<int> num_per_pixel; //
};


#endif //LVI_SAM_RANGE_IMAGE_H
