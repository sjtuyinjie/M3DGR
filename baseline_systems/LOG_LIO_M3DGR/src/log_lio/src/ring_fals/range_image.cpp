//
// Created by hk on 12/8/22.
//

#include "range_image.h"

void RangeImage::createFromRings(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud)
{
    if (rows_ == 0 || cols_ == 0) {
        std::cout << "rows_ or cols_ == 0, can't create range image.\n";
        return;
    }

    range_image = cv::Mat(rows_, cols_, CV_32F, range_init);
    cloud_id_image.resize(rows_ * cols_, -1);
    int cloudSize = (int)cloud->points.size();
    // range image projection
    for (int i = 0; i < cloudSize; ++i)
    {
//        PointType thisPoint;
//        thisPoint.x = laserCloudIn->points[i].x;
//        thisPoint.y = laserCloudIn->points[i].y;
//        thisPoint.z = laserCloudIn->points[i].z;
//        thisPoint.intensity = laserCloudIn->points[i].intensity;
        const PointXYZIRT & thisPoint = cloud->points[i];

        int rowIdn = cloud->points[i].ring; // ring
        if (rowIdn < 0 || rowIdn >= rows_)
            continue;

        if (downsampleRate > 0 && rowIdn % downsampleRate != 0) //采样行间隔
            continue;

//        float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
//
        static float ang_res_x = 360.0/float(cols_); // 360 / 1800 = 0.2

        float horizonAngle = atan2(thisPoint.y, -thisPoint.x);
        if (horizonAngle < 0)
            horizonAngle += 2 * M_PI;
        horizonAngle = horizonAngle  * 180 / M_PI;
        int columnIdn = round(horizonAngle/ ang_res_x);

        if (columnIdn < 0 || columnIdn >= cols_)
            continue;

        float range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y +thisPoint.z * thisPoint.z);//  sqrt(x^2 + y^2 + z^2)

        if (range < range_min || range > range_max)
            continue;

        if (range_image.at<float>(rowIdn, columnIdn) != FLT_MAX)
            continue;

        // for the amsterdam dataset
        // if (range < 6.0 && rowIdn <= 7 && (columnIdn >= 1600 || columnIdn <= 200))
        //     continue;
        // if (thisPoint.z < -2.0)
        //     continue;

        range_image.at<float>(rowIdn, columnIdn) = range; //
        if (input_value_max < range)
            input_value_max = range;


//        thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time); // Velodyne
//        // thisPoint = deskewPoint(&thisPoint, (float)laserCloudIn->points[i].t / 1000000000.0); // Ouster

        int index = columnIdn  + rowIdn * cols_; //
        cloud_id_image[index] = i;

    }
}

void RangeImage::saveRangeImage(const std::string &file) {
    cv::Mat output = cv::Mat(range_image.rows, range_image.cols, CV_8U , 255);
    for (int i = 0; i < range_image.rows; ++i) {
        for (int j = 0; j < range_image.cols; ++j) {
            if (range_image.at<float>(i, j) < 50.0)
                output.at<uchar>(i, j) = range_image.at<float>(i, j) / 50 * 255.0;
            else
                output.at<uchar>(i, j) = 255;
        }
    }
    cv::imwrite(file, output);
}

void RangeImage::computeNormals(cv::OutputArray normals_out) {

//    cv::Mat points3d = points3d_in.getMat();

    // Get the normals
    normals_out.create(rows_, cols_, CV_MAKETYPE(depth_, 3));
//    if (points3d_in.empty())
//        return;

    cv::Mat normals = normals_out.getMat();
    lidar_fals.compute(range_image, normals);

};

//in lidar frame
void RangeImage::createFromCloud(const int& num_bins_v, const int& num_bins_h,
                                 const pcl::PointCloud<PointType>::Ptr& cloud)
{
    float bin_res_v = 180.0 / (float)num_bins_v; // vertical
    float bin_res_h = 360.0 / (float)num_bins_h; // horizon

    range_image = cv::Mat(num_bins_v, num_bins_h, CV_32F, cv::Scalar::all(FLT_MAX));
    cloud_id_image.resize(rows_ * cols_, -1);

    //lidar系下点云
    for (int i = 0; i < (int)cloud->size(); ++i)
    {
        const PointType &p = cloud->points[i];

        // find row id in range image
        float row_angle = atan2(p.z, sqrt(p.x * p.x + p.y * p.y)); // degrees, bottom -> up, -90 -> 0 -> 90
        row_angle = -row_angle * 180.0 / M_PI + 90.0; //bottom -> up, 180 -> 90 -> 0
        int row_id = round(row_angle / bin_res_v);

        // find column id in range image
        float col_angle = atan2(p.y, -p.x) * 180.0 / M_PI; // degrees, back -> left -> front -> right, 0 -> 360
        if (col_angle < 0)
            col_angle += 360;
        int col_id = round(col_angle / bin_res_h);

        // id may be out of boundary
        if (row_id < 0 || row_id >= num_bins_v || col_id < 0 || col_id >= num_bins_h)
            continue;
        // only keep points that's closer
        float dist = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        if (dist < range_image.at<float>(row_id, col_id))
        {
            range_image.at<float>(row_id, col_id) = dist;

            int index = col_id  + row_id * cols_; //
            cloud_id_image[index] = i;
        }
    }
}

void RangeImage::computeLookupTable()
{
    if (range_image.empty())
        return;
}

void RangeImage::createAndBuildTableFromRings(const int &num_rings, const int &num_horizon_scan,
                                              const pcl::PointCloud<PointXYZIRT>::Ptr &cloud)
{
    if (rows_ != num_rings || cols_ != num_horizon_scan) {
        std::cout << "rows_ != num_rings || cols_ != num_horizon_scan. reset rows_ and cols\n";
        rows_ = num_rings;
        cols_ = num_horizon_scan;
        lidar_fals.setRowsCols(num_rings, num_horizon_scan);
    }
    createAndBuildTableFromRings(cloud);
}

void RangeImage::createAndBuildTableFromRings(const pcl::PointCloud<PointXYZIRT>::Ptr &cloud)
{
    if (rows_ == 0 || cols_ == 0) {
        std::cout << "rows_ == 0 or cols_ == 0, can't create range image.\n";
        return;
    }

    int num_valid_bins = 0;
    range_image = cv::Mat(rows_, cols_, CV_32F, range_init);
    cloud_id_image.resize(rows_ * cols_, -1);

    cv::Mat cos_theta =  cv::Mat(rows_, cols_, CV_32F);
    cv::Mat sin_theta =  cv::Mat(rows_, cols_, CV_32F);
    cv::Mat cos_phi = cv::Mat(rows_, cols_, CV_32F);
    cv::Mat sin_phi =  cv::Mat(rows_, cols_, CV_32F);
    int cloudSize = (int)cloud->points.size();
    // range image projection
    for (int i = 0; i < cloudSize; ++i)
    {
        const PointXYZIRT & thisPoint = cloud->points[i];

        int rowIdn = cloud->points[i].ring; // ring，
        if (rowIdn < 0 || rowIdn >= rows_)
            continue;

        if (downsampleRate > 0 && rowIdn % downsampleRate != 0) //
            continue;

        float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI; //theta

        static float ang_res_x = 360.0/float(cols_); // 360 / 1800 = 0.2
        int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + cols_/2; //
        if (columnIdn >= cols_)
            columnIdn -= cols_;

        if (columnIdn < 0 || columnIdn >= cols_)
            continue;

        float range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y +thisPoint.z * thisPoint.z);//  sqrt(x^2 + y^2 + z^2)

        if (range < range_min || range > range_max)
            continue;

        if (range_image.at<float>(rowIdn, columnIdn) != FLT_MAX) //
            continue;

        // for the amsterdam dataset
        // if (range < 6.0 && rowIdn <= 7 && (columnIdn >= 1600 || columnIdn <= 200))
        //     continue;
        // if (thisPoint.z < -2.0)
        //     continue;

        range_image.at<float>(rowIdn, columnIdn) = range; //
//        if (input_value_max < range)
//            input_value_max = range;

        //theta, phi used for building lookup table
        if (!table_valid)
        {
            float theta = (float) std::atan2(-thisPoint.y, thisPoint.x); //，-y / x
            float phi = (float) std::asin(thisPoint.z / range); //， z/r
            cos_theta.at<float>(rowIdn, columnIdn) = std::cos(theta);
            sin_theta.at<float>(rowIdn, columnIdn) = std::sin(theta);
            cos_phi.at<float>(rowIdn, columnIdn) = std::cos(phi);
            sin_phi.at<float>(rowIdn, columnIdn) = std::sin(phi);
        }

//        thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time); // Velodyne
//        // thisPoint = deskewPoint(&thisPoint, (float)laserCloudIn->points[i].t / 1000000000.0); // Ouster

        int index = columnIdn  + rowIdn * cols_; //点在深度图像中的索引
        cloud_id_image[index] = i;
//        fullCloud->points[index] = thisPoint;
    }

    //save mat for debug
    {
//        cv::Mat output = cv::Mat(rows_, cols_, CV_8U, 255);
//        for (int i = 0; i < rows_; ++i) {
//            for (int j = 0; j < cols_; ++j) {
//                if (range_image.at<float>(i, j) < 100.0)
//                    output.at<uchar>(i, j) = range_image.at<float>(i, j) / input_value_max * 255.0;
//            }
//        }
//        cv::imwrite("/tmp/cos_theta.jpg", cos_theta);
    }

    //todo compute M matrix and M inverse
    if (!table_valid) {
        lidar_fals.computeMInverse(cos_theta, sin_theta, cos_phi, sin_phi);
        table_valid = true;
    }
}

void RangeImage::buildTableFromRings(const pcl::PointCloud<PointXYZIRT>::Ptr &cloud) {

    if (rows_ == 0 || cols_ == 0) {
        std::cout << "rows_ == 0 or cols_ == 0, can't create range image.\n";
        return;
    }

//    int num_valid_bins = 0;
//    range_image = cv::Mat(rows_, cols_, CV_32F, range_init);
    cloud_id_image.resize(rows_ * cols_, -1);

    int cloudSize = (int)cloud->points.size();
    static float ang_res_x = 360.0 / float(cols_); // 360 / 1800 = 0.2
    // range image projection
    for (int i = 0; i < cloudSize; ++i)
    {
        const PointXYZIRT & thisPoint = cloud->points[i];

        int rowIdn = cloud->points[i].ring; // ring，
        if (rowIdn < 0 || rowIdn >= rows_)
            continue;

        float horizonAngle = atan2(thisPoint.y, -thisPoint.x); //theta
        if (horizonAngle < 0)
            horizonAngle += 2 * M_PI;
        horizonAngle = horizonAngle  * 180 / M_PI;
        int columnIdn = round(horizonAngle/ ang_res_x);

        if (columnIdn < 0 || columnIdn >= cols_)
            continue;

        float range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y +thisPoint.z * thisPoint.z);//  sqrt(x^2 + y^2 + z^2)

        if (range < range_min || range > range_max)
            continue;

        //theta, phi used for building lookup table
        float theta = (float) std::atan2(-thisPoint.y, thisPoint.x); //，-y / x
        float phi = (float) std::asin(thisPoint.z / range); //， z/r
        int index = columnIdn  + rowIdn * cols_; //
        float num_p = static_cast<float>(num_per_pixel[index]);
        float num_p_new = num_p + 1;
        cos_theta.at<float>(rowIdn, columnIdn) = (num_p * cos_theta.at<float>(rowIdn, columnIdn) + std::cos(theta)) / num_p_new;
        sin_theta.at<float>(rowIdn, columnIdn) = (num_p * sin_theta.at<float>(rowIdn, columnIdn) + std::sin(theta)) / num_p_new;
        cos_phi.at<float>(rowIdn, columnIdn) = (num_p * cos_phi.at<float>(rowIdn, columnIdn) + std::cos(phi)) / num_p_new;
        sin_phi.at<float>(rowIdn, columnIdn) = (num_p * sin_phi.at<float>(rowIdn, columnIdn) + std::sin(phi)) / num_p_new;

        ++num_per_pixel[index];
//        cloud_id_image[index] = i;
    }
    ++num_cloud;
    //todo compute M matrix and M inverse

    if (num_cloud < min_table_cloud)
        return;

//    lidar_fals.computeMInverse(cos_theta, sin_theta, cos_phi, sin_phi);
}

void RangeImage::buildTableFromRings(const int &num_rings, const int &num_horizon_scan,
                                     const pcl::PointCloud<PointXYZIRT>::Ptr &cloud) {
    if (rows_ != num_rings || cols_ != num_horizon_scan) {
        std::cout << "rows_ != num_rings || cols_ != num_horizon_scan. reset rows_ and cols\n";
        rows_ = num_rings;
        cols_ = num_horizon_scan;
        lidar_fals.setRowsCols(num_rings, num_horizon_scan);
    }
    buildTableFromRings(cloud);
}

void RangeImage::buildTableFromCloud() {
    float bin_res_v = M_PI / (float)rows_; // vertical(radian)
    float bin_res_h = 2.0 * M_PI / (float)cols_; // horizon(radian)

    //lidar系下点云
    for (int i = 0; i < rows_; ++i) {
        float phi =  0.5 * M_PI - ((float)i + 0.5) * bin_res_v;
        float c_phi = std::cos(phi);
        float s_phi = std::sin(phi);
        for (int j = 0; j < cols_; ++j) {
            float theta = ((float)j + 0.5) * bin_res_h - M_PI;
            cos_theta.at<float>(i, j) = std::cos(theta);
            sin_theta.at<float>(i, j) = std::sin(theta);
            cos_phi.at<float>(i, j) = c_phi;
            sin_phi.at<float>(i, j) = s_phi;
        }
    }
    lidar_fals.computeMInverse(cos_theta, sin_theta, cos_phi, sin_phi);
}

void RangeImage::createFromCloud(const pcl::PointCloud<RangeImage::PointType>::Ptr &cloud) {
    createFromCloud(rows_, cols_, cloud);
}

void RangeImage::computeNormals(const cv::Mat &r, const cv::_OutputArray &normals_out) {
    // Get the normals
    normals_out.create(rows_, cols_, CV_MAKETYPE(depth_, 3));

    cv::Mat normals = normals_out.getMat();
    lidar_fals.compute(r, normals);
}

void RangeImage::computeNormals(const cv::Mat &r, const cv::_OutputArray &normals_out,
                                cv::Mat &residual) {
    // Get the normals
    normals_out.create(rows_, cols_, CV_MAKETYPE(depth_, 3));
    residual = cv::Mat(rows_, cols_, CV_32F, cv::Scalar::all(FLT_MAX));


    cv::Mat normals = normals_out.getMat();
//    cv::Mat res = residual.getMat();
    lidar_fals.compute(r, normals, residual);
}
