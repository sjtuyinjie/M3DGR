/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/OccupancyGrid.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/package.h>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgcodecs.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "pose_graph.h"
#include "utility/CameraPoseVisualization.h"
#include "parameters.h"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/ply_io.h>

// pcl test lib
#include <pcl/point_types.h> //PCL中所有点类型定义的头文件
#include <pcl/io/pcd_io.h>   //打开关闭pcd文件的类定义的头文件
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h> //视觉化工具函式库（VTK，Visualization Toolkit）　模型
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h> //kd-tree搜索对象的类定义的头文件
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>          //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h>        //PCL对各种格式的点的支持头文件
#include <pcl/search/kdtree.h>      //kdtree搜索对象的类定义的头文件
#include <pcl/features/normal_3d.h> //法向量特征估计相关类定义的头文件
//重构
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>                  //最小二乘法平滑处理类定义头文件
#include <pcl/surface/marching_cubes_hoppe.h> // 移动立方体算法
#include <pcl/surface/marching_cubes_rbf.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/ply_io.h>

#define SKIP_FIRST_CNT 10
using namespace std;
using namespace pcl;

queue<sensor_msgs::ImageConstPtr> image_buf;
queue<sensor_msgs::ImageConstPtr> depth_buf;
queue<sensor_msgs::PointCloudConstPtr> point_buf;
queue<nav_msgs::Odometry::ConstPtr> pose_buf;
queue<Eigen::Vector3d> odometry_buf;
std::mutex m_buf;
std::mutex m_process;
int frame_index = 0;
int sequence = 1;
PoseGraph posegraph;
int skip_first_cnt = 0;
int SKIP_CNT;
int skip_cnt = 0;
bool load_flag = 0;
bool start_flag = 0;
double SKIP_DIS = 0;
int ESTIMATE_EXTRINSIC;

int DEPTH_DIST = 10;
int DEPTH_BOUNDARY = 10;
float RESOLUTION = 0.03;

int EQUALIZE; // new

int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int ROW;
int COL;
int DEBUG_IMAGE;
int DEPTH;

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
ros::Publisher pub_match_img;
ros::Publisher pub_camera_pose_visual;
ros::Publisher pub_odometry_rect;
ros::Publisher g_map_puber;

std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string VINS_RESULT_PATH;
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
double last_image_time = -1;

ros::Publisher pub_point_cloud, pub_margin_cloud;

typedef pcl::PointXYZRGB PoinT;
typedef pcl::PointXYZRGBNormal PoinTNormal;

void poisson_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud)
{
    // void poisson_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud)
    // {
    //       PointCloud<pcl::PointXYZRGB>::Ptr cloudx(new PointCloud<pcl::PointXYZRGB>());
    //       pcl::copyPointCloud(*object_cloud, *cloudx);

    // string name = "fusedCloud";
    // pcl::PointCloud<PoinT>::Ptr cloud(new pcl::PointCloud<PoinT>());
    // // 加载pcd文件
    // pcl::io::loadPCDFile(name + ".pcd", *cloud);
    PointCloud<pcl::PointXYZRGB>::Ptr cloud(new PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*object_cloud, *cloud);
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // for(int nIndex = 0; nIndex < cloud->points.size(); nIndex++)
    // {
    //     cloud->points[nIndex].x *= 100;
    //     cloud->points[nIndex].y *= 100;
    //     cloud->points[nIndex].z *= 100;
    // }

    // 计算法向量
    pcl::NormalEstimation<PoinT, pcl::Normal> n;                                 //法线估计对象
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //存储估计的法线
    pcl::search::KdTree<PoinT>::Ptr tree(new pcl::search::KdTree<PoinT>);        //定义kd树指针
    tree->setInputCloud(cloud);                                                  //用cloud构建tree对象
    n.setInputCloud(cloud);                                                      //为法线估计对象设置输入点云
    n.setSearchMethod(tree);                                                     //设置搜索方法
    n.setKSearch(20);                                                            //设置k搜索的k值为20
    n.compute(*normals);                                                         //估计法线存储结果到normals中
    //将点云和法线放到一起
    pcl::PointCloud<PoinTNormal>::Ptr cloud_with_normals(new pcl::PointCloud<PoinTNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals); //连接字段，cloud_with_normals存储有向点云
    std::cerr << "法线计算   完成" << std::endl;

    //创建搜索树
    pcl::search::KdTree<PoinTNormal>::Ptr tree2(new pcl::search::KdTree<PoinTNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // 曲面重建
    pcl::MovingLeastSquares<PoinTNormal, PoinTNormal> mls;
    mls.setComputeNormals(true);           //设置在最小二乘计算中需要进行法线估计
    mls.setInputCloud(cloud_with_normals); //设置参数
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree2);
    mls.setSearchRadius(0.03);
    pcl::PointCloud<PoinTNormal>::Ptr cloud_with_normals_msl(new pcl::PointCloud<PoinTNormal>);
    mls.process(*cloud_with_normals_msl);
    cloud_with_normals = cloud_with_normals_msl;
    std::cerr << "曲面重建   完成" << std::endl;

    // 开始表面重建 ********************************************************************

    //创建Poisson对象，并设置参数
    pcl::Poisson<pcl::PointXYZRGBNormal> pn;
    pn.setConfidence(false); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
    pn.setDegree(2);         //设置参数degree[1,5],值越大越精细，耗时越久。
    pn.setDepth(8);
    //树的最大深度，求解2^d x 2^d x 2^d立方体元。
    // 由于八叉树自适应采样密度，指定值仅为最大深度。
    pn.setIsoDivide(8);    //用于提取ISO等值面的算法的深度
    pn.setManifold(false); //是否添加多边形的重心，当多边形三角化时。
    // 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
    pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
    pn.setSamplesPerNode(3.0);   //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
    pn.setScale(1.25);           //设置用于重构的立方体直径和样本边界立方体直径的比率。
    pn.setSolverDivide(8);       //设置求解线性方程组的Gauss-Seidel迭代方法的深度
    // pn.setIndices();
    //设置搜索方法和输入点云
    pn.setSearchMethod(tree2);
    pn.setInputCloud(cloud_with_normals);
    //创建多变形网格，用于存储结果
    pcl::PolygonMesh mesh;
    //执行重构
    pn.performReconstruction(mesh);
    //保存网格图
    pcl::io::savePLYFile("/home/car/Downloads/newproject6/final_ws/src/RGBD-WINS-Fusion/z_mesh/poisson-nocolor.ply", mesh);
    std::cerr << "泊松重建   完成" << std::endl;

    // // 贪婪投影三角化算法
    // pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   // 定义三角化对象
    // //pcl::PolygonMesh mesh;                // 存储最终三角化的网络模型
    // // Set the maximum distance between connected points (maximum edge length)
    // gp3.setSearchRadius(0.025);  // 设置连接点之间的最大距离，（即是三角形最大边长）
    // // 设置各参数值
    // gp3.setMu(2.5);  // 设置被样本点搜索其近邻点的最远距离为2.5，为了使用点云密度的变化
    // gp3.setMaximumNearestNeighbors(100);// 设置样本点可搜索的邻域个数
    // gp3.setMaximumSurfaceAngle(M_PI / 4);  // 设置某点法线方向偏离样本点法线的最大角度45
    // gp3.setMinimumAngle(M_PI / 18);        // 设置三角化后得到的三角形内角的最小的角度为10
    // gp3.setMaximumAngle(2 * M_PI / 3);       // 设置三角化后得到的三角形内角的最大角度为120
    // gp3.setNormalConsistency(false);     // 设置该参数保证法线朝向一致
    // // Get result
    // gp3.setInputCloud(cloud_with_normals);// 设置输入点云为有向点云
    // gp3.setSearchMethod(tree2);               // 设置搜索方式
    // gp3.reconstruct(mesh);               // 重建提取三角化
    // // 附加顶点信息
    // std::vector<int> parts = gp3.getPartIDs();
    // std::vector<int> states = gp3.getPointStates();
    // // 保存mesh文件
    // pcl::io::saveVTKFile(name + "-quick.ply", mesh);
    // std::cerr << "快速三角化 完成" << std::endl;

    // //移动立方体算法
    // pcl::MarchingCubes<pcl::PointNormal> *mc;
    // mc = new pcl::MarchingCubesHoppe<pcl::PointNormal>();
    // /*
    //   if (hoppe_or_rbf == 0)
    //     mc = new pcl::MarchingCubesHoppe<pcl::PointNormal> ();
    //   else
    //   {
    //     mc = new pcl::MarchingCubesRBF<pcl::PointNormal> ();
    //     (reinterpret_cast<pcl::MarchingCubesRBF<pcl::PointNormal>*> (mc))->setOffSurfaceDisplacement (off_surface_displacement);
    //   }
    // */
    // //创建多变形网格，用于存储结果
    // //pcl::PolygonMesh mesh;
    // //设置MarchingCubes对象的参数
    // mc->setIsoLevel(0.0f);
    // mc->setGridResolution(50, 50, 50);
    // mc->setPercentageExtendGrid(0.0f);
    // //设置搜索方法
    // mc->setInputCloud(cloud_with_normals);
    // //执行重构，结果保存在mesh中
    // mc->reconstruct(mesh);
    // //保存网格图
    // pcl::io::saveVTKFile(name + "-cubes.ply", mesh);
    // std::cerr << "移动立方体 完成" << std::endl;

    //给mesh染色
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudx(new pcl::PointCloud<pcl::PointXYZRGB>());

    // // 加载pcd文件
    // pcl::io::loadPCDFile(name + ".pcd", *cloudx);

    pcl::PointCloud<pcl::PointXYZRGB> cloud_color_mesh;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud_color_mesh);

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud);
    // K nearest neighbor search
    int K = 5;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    for (int i = 0; i < cloud_color_mesh.points.size(); ++i)
    {
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        float dist = 0.0;
        int red = 0;
        int green = 0;
        int blue = 0;
        uint32_t rgb;

        if (kdtree.nearestKSearch(cloud_color_mesh.points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            for (int j = 0; j < pointIdxNKNSearch.size(); ++j)
            {

                r = cloud->points[pointIdxNKNSearch[j]].r;
                g = cloud->points[pointIdxNKNSearch[j]].g;
                b = cloud->points[pointIdxNKNSearch[j]].b;

                red += int(r);
                green += int(g);
                blue += int(b);
                dist += 1.0 / pointNKNSquaredDistance[j];

                std::cout << "red: " << int(r) << std::endl;
                std::cout << "green: " << int(g) << std::endl;
                std::cout << "blue: " << int(b) << std::endl;
                cout << "dis:" << dist << endl;
            }
        }

        cloud_color_mesh.points[i].r = int(red / pointIdxNKNSearch.size() + 0.5);
        cloud_color_mesh.points[i].g = int(green / pointIdxNKNSearch.size() + 0.5);
        cloud_color_mesh.points[i].b = int(blue / pointIdxNKNSearch.size() + 0.5);
    }
    toPCLPointCloud2(cloud_color_mesh, mesh.cloud);
    pcl::io::savePLYFile("/home/car/Downloads/newproject6/final_ws/src/RGBD-WINS-Fusion/z_mesh/rgb_mesh.ply", mesh);

  
}



void new_sequence()
{
    printf("new sequence\n");
    sequence++;
    printf("sequence cnt %d \n", sequence);
    if (sequence > 10)
    {
        ROS_WARN("only support 5 sequences since it's boring to copy code for more sequences.");
        ROS_BREAK();
    }
    posegraph.posegraph_visualization->reset();
    posegraph.publish();
    m_buf.lock();
    while (!image_buf.empty())
        image_buf.pop();
    while (!depth_buf.empty())
        depth_buf.pop();
    while (!point_buf.empty())
        point_buf.pop();
    while (!pose_buf.empty())
        pose_buf.pop();
    while (!odometry_buf.empty())
        odometry_buf.pop();
    m_buf.unlock();
}

void image_callback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::ImageConstPtr &depth_msg)
{
    // ROS_INFO("image_callback!");
    m_buf.lock();
    image_buf.push(image_msg);
    depth_buf.push(depth_msg);
    m_buf.unlock();
    // printf(" image time %f \n", image_msg->header.stamp.toSec());

    // detect unstable camera stream
    if (last_image_time == -1)
        last_image_time = image_msg->header.stamp.toSec();
    else if (image_msg->header.stamp.toSec() - last_image_time > 3.0)
    {
        ROS_WARN("image discontinue! >3 ! just warning!");
        // new_sequence();
    }
    else if (image_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! <0 ! just warning!");
        // new_sequence();
    }
    last_image_time = image_msg->header.stamp.toSec();
}

void point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    // ROS_INFO("point_callback!");
    m_buf.lock();
    point_buf.push(point_msg);
    m_buf.unlock();
    /*
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
        printf("%d, 3D point: %f, %f, %f 2D point %f, %f \n",i , point_msg->points[i].x,point_msg->points[i].y,point_msg->points[i].z,
                                                     point_msg->channels[i].values[0],point_msg->channels[i].values[1]);
    */

    // for visualization
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = point_msg->header;
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        cv::Point3f p_3d;
        p_3d.x = point_msg->points[i].x;
        p_3d.y = point_msg->points[i].y;
        p_3d.z = point_msg->points[i].z;
        Eigen::Vector3d tmp = posegraph.r_drift * Eigen::Vector3d(p_3d.x, p_3d.y, p_3d.z) + posegraph.t_drift;
        geometry_msgs::Point32 p;
        p.x = tmp(0);
        p.y = tmp(1);
        p.z = tmp(2);
        point_cloud.points.push_back(p);
    }
    pub_point_cloud.publish(point_cloud);
}

// only for visualization
void margin_point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = point_msg->header;
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        cv::Point3f p_3d;
        p_3d.x = point_msg->points[i].x;
        p_3d.y = point_msg->points[i].y;
        p_3d.z = point_msg->points[i].z;
        Eigen::Vector3d tmp = posegraph.r_drift * Eigen::Vector3d(p_3d.x, p_3d.y, p_3d.z) + posegraph.t_drift;
        geometry_msgs::Point32 p;
        p.x = tmp(0);
        p.y = tmp(1);
        p.z = tmp(2);
        point_cloud.points.push_back(p);
    }
    pub_margin_cloud.publish(point_cloud);
}

void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    // ROS_INFO("pose_callback!");
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
    /*
    printf("pose t: %f, %f, %f   q: %f, %f, %f %f \n", pose_msg->pose.pose.position.x,pose_msg->pose.pose.position.y,pose_msg->pose.pose.position.z,
                                                       pose_msg->pose.pose.orientation.w,pose_msg->pose.pose.orientation.x,
                                                       pose_msg->pose.pose.orientation.y,pose_msg->pose.pose.orientation.z);
    */
}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    // ROS_INFO("vio_callback!");
    Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;

    vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
    vio_q = posegraph.w_r_vio * vio_q;

    vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
    vio_q = posegraph.r_drift * vio_q;

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = vio_t.x();
    odometry.pose.pose.position.y = vio_t.y();
    odometry.pose.pose.position.z = vio_t.z();
    odometry.pose.pose.orientation.x = vio_q.x();
    odometry.pose.pose.orientation.y = vio_q.y();
    odometry.pose.pose.orientation.z = vio_q.z();
    odometry.pose.pose.orientation.w = vio_q.w();
    pub_odometry_rect.publish(odometry);

    Vector3d vio_t_cam;
    Quaterniond vio_q_cam;
    vio_t_cam = vio_t + vio_q * tic;
    vio_q_cam = vio_q * qic;

    cameraposevisual.reset();
    cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
    cameraposevisual.publish_by(pub_camera_pose_visual, pose_msg->header);
}

void extrinsic_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    m_process.lock();
    tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z)
              .toRotationMatrix();
    m_process.unlock();
}

void process()
{
    while (true)
    {
        sensor_msgs::ImageConstPtr image_msg = NULL;
        sensor_msgs::ImageConstPtr depth_msg = NULL;
        sensor_msgs::PointCloudConstPtr point_msg = NULL;
        nav_msgs::Odometry::ConstPtr pose_msg = NULL;

        // find out the messages with same time stamp
        m_buf.lock();
        if (!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (image_buf.front()->header.stamp.toSec() > pose_buf.front()->header.stamp.toSec())
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (image_buf.front()->header.stamp.toSec() > point_buf.front()->header.stamp.toSec())
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            else if (image_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec() && point_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec())
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while (image_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                {
                    image_buf.pop();
                    depth_buf.pop();
                }

                image_msg = image_buf.front();
                image_buf.pop();
                depth_msg = depth_buf.front();
                depth_buf.pop();

                while (point_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();

        if (pose_msg != NULL)
        {
            // printf(" depth time %f \n", depth_msg->header.stamp.toSec());
            // printf(" pose time %f \n", pose_msg->header.stamp.toSec());
            // printf(" point time %f \n", point_msg->header.stamp.toSec());
            // printf(" image time %f \n", image_msg->header.stamp.toSec());

            // skip fisrt few
            if (skip_first_cnt < SKIP_FIRST_CNT)
            {
                skip_first_cnt++;
                continue;
            }

            if (skip_cnt < SKIP_CNT)
            {
                skip_cnt++;
                continue;
            }
            else
            {
                skip_cnt = 0;
            }

            vector<Eigen::Matrix<float, 6, 1>> point_rgbd;
            if (DEPTH)
            {
                cv_bridge::CvImageConstPtr depth_ptr;
                // if (depth_msg->encoding == "8UC1")
                {
                    sensor_msgs::Image img;
                    img.header = depth_msg->header;
                    img.height = depth_msg->height;
                    img.width = depth_msg->width;
                    img.is_bigendian = depth_msg->is_bigendian;
                    img.step = depth_msg->step;
                    img.data = depth_msg->data;
                    img.encoding = sensor_msgs::image_encodings::MONO16;
                    depth_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
                }
                cv::Mat depth = depth_ptr->image;

                cv_bridge::CvImageConstPtr ptr;
                {
                    sensor_msgs::Image img;
                    img.header = image_msg->header;
                    img.height = image_msg->height;
                    img.width = image_msg->width;
                    img.is_bigendian = image_msg->is_bigendian;
                    img.step = image_msg->step;
                    img.data = image_msg->data;
                    img.encoding = "rgb8";
                    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
                }
                cv::Mat image = ptr->image;
                // cv::Mat image;
                // cout<< image.channels() <<endl;

                // cv::bilateralFilter(image2, image, 10, 10*2, 10/2);

                for (int v = DEPTH_BOUNDARY; v < ROW - DEPTH_BOUNDARY; v += DEPTH_DIST)
                {
                    for (int u = DEPTH_BOUNDARY; u < COL - DEPTH_BOUNDARY; u += DEPTH_DIST)
                    {
                        Eigen::Vector2d a(u, v);
                        Eigen::Vector3d a_3d;
                        m_camera->liftProjective(a, a_3d);
                        // float d = (depth.ptr<unsigned short>(v)[u]+ depth.ptr<unsigned short>(v-1)[u] +
                        //         depth.ptr<unsigned short>(v)[u-1] + depth.ptr<unsigned short>(v+1)[u] +
                        //         depth.ptr<unsigned short>(v)[u+1])/ 5000.0 * 1.15;
                        float d = (int)depth.at<unsigned short>(v, u);
                        d = d / 1000;
                        // cout<<"d vaule:"<<d<<endl;
                        float r = image.ptr<cv::Vec3b>(v)[u][0];
                        float g = image.ptr<cv::Vec3b>(v)[u][1];
                        float b = image.ptr<cv::Vec3b>(v)[u][2];
                        if (r > 240 && g > 240 && b > 240)
                            continue;
                        if (d < 0.1 || d >= 7.0)
                            continue;
                        Eigen::Matrix<float, 6, 1> point;
                        point << a_3d.x() * d, a_3d.y() * d, d, r, g, b;
                        point_rgbd.push_back(point);
                        // cout <<"r:"<< r <<" g:"<< g <<" b:"<< b <<" d:"<< d << endl;
                    }
                }
            }

            cv_bridge::CvImageConstPtr ptr;
            if (image_msg->encoding == "8UC1")
            {
                sensor_msgs::Image img;
                img.header = image_msg->header;
                img.height = image_msg->height;
                img.width = image_msg->width;
                img.is_bigendian = image_msg->is_bigendian;
                img.step = image_msg->step;
                img.data = image_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image, ptr->image);
                // ROS_INFO("Equalization Using!");
            }

            cv::Mat image = ptr->image;

            // build keyframe
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z)
                             .toRotationMatrix();
            if ((T - last_t).norm() > SKIP_DIS)
            {
                vector<cv::Point3f> point_3d;
                vector<cv::Point2f> point_2d_uv;
                vector<cv::Point2f> point_2d_normal;
                vector<double> point_id;

                for (unsigned int i = 0; i < point_msg->points.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);
                    // printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
                }

                if (DEPTH)
                {
                    KeyFrame *keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R, image, point_rgbd,
                                                      point_3d, point_2d_uv, point_2d_normal, point_id, sequence);
                    m_process.lock();
                    start_flag = 1;
                    posegraph.addKeyFrame(keyframe, 1);
                    m_process.unlock();
                }

                else
                {
                    KeyFrame *keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R, image,
                                                      point_3d, point_2d_uv, point_2d_normal, point_id, sequence);
                    m_process.lock();
                    start_flag = 1;
                    posegraph.addKeyFrame(keyframe, 1);
                    m_process.unlock();
                }

                frame_index++;
                last_t = T;
            }
        }
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void command()
{
    while (1)
    {
        char c = getchar();
        if (c == 's')
        {
            m_process.lock();
            posegraph.savePoseGraph();
            m_process.unlock();
            printf("save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
            printf("program shutting down...\n");
            ros::shutdown();
        }
        if (c == 'n')
            new_sequence();
        if (c == 'd')
        {
            cout << "begin to save pcd file";
            TicToc t_pcdfile;
            posegraph.save_cloud->width = posegraph.save_cloud->points.size();
            posegraph.save_cloud->height = 1;
            pcl::io::savePCDFileASCII("/home/car/Downloads/newproject6/final_ws/src/RGBD-WINS-Fusion/z_gridmap/pcd_file_" + to_string(frame_index) + "keyframes.pcd", *(posegraph.save_cloud));
            poisson_reconstruction((posegraph.save_cloud));

            printf("Save pcd file done! Time cost: %f", t_pcdfile.toc());
        }
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dense_map");
    ros::NodeHandle n("~");
    posegraph.registerPub(n);

    VISUALIZATION_SHIFT_X = 0;
    VISUALIZATION_SHIFT_Y = 0;
    SKIP_CNT = 0;
    SKIP_DIS = 0;

    if (argc != 2)
    {
        printf("please intput: rosrun dense_map dense_map_node [config file] \n"
               "for example: rosrun dense_map dense_map_node "
               "/home/tony-ws1/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 0;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    cameraposevisual.setScale(0.1);
    cameraposevisual.setLineWidth(0.01);

    std::string IMAGE_TOPIC;
    std::string DEPTH_TOPIC;
    int LOAD_PREVIOUS_POSE_GRAPH;
    int LOAD_GRID_MAP;

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    std::string pkg_path = ros::package::getPath("dense_map");
    string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";
    cout << "vocabulary_file" << vocabulary_file << endl;
    posegraph.loadVocabulary(vocabulary_file);

    BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
    cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;

    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    printf("cam calib path: %s\n", cam0Path.c_str());
    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(cam0Path.c_str());

    fsSettings["image0_topic"] >> IMAGE_TOPIC;

    DEPTH = fsSettings["depth"];
    fsSettings["image1_topic"] >> DEPTH_TOPIC;

    fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
    fsSettings["output_path"] >> VINS_RESULT_PATH;
    fsSettings["save_image"] >> DEBUG_IMAGE;

    DEPTH_DIST = fsSettings["depth_dist"];
    DEPTH_BOUNDARY = fsSettings["depth_boundary"];
    RESOLUTION = fsSettings["resolution"];

    EQUALIZE = fsSettings["equalize"];
    cout << "EQUALIZE USE:" << EQUALIZE << endl;
    // EQUALIZE = fsSettings["equalize"];
    // cout<<"EQUALIZE USE:"<<EQUALIZE<<endl;

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 0)
    {
        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        qic = (T.block<3, 3>(0, 0));
        tic = (T.block<3, 1>(0, 3));
    }

    LOAD_GRID_MAP = fsSettings["load_grid_map"];
    string GRID_MAP_PATH = fsSettings["grid_map_save_path"];

    LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
    VINS_RESULT_PATH = VINS_RESULT_PATH + "/vio_loop.txt";
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    int USE_IMU = fsSettings["imu"];
    posegraph.setIMUFlag(USE_IMU);
    fsSettings.release();

    // 读取先验地图（位姿图）
    if (LOAD_PREVIOUS_POSE_GRAPH)
    {
        printf("load pose graph\n");
        m_process.lock();
        posegraph.loadPoseGraph();
        m_process.unlock();
        printf("load pose graph finish\n");
        load_flag = 1;
    }
    else
    {
        printf("no previous pose graph\n");
        load_flag = 1;
    }

    // 读取先验地图（栅格图）
    if (LOAD_GRID_MAP)
    {
        printf("load grid map\n");
        g_map_puber = n.advertise<nav_msgs::OccupancyGrid>("grid_map", 1);
        cv::Mat grid_img = cv::imread(GRID_MAP_PATH + "map.png", cv::IMREAD_GRAYSCALE);
        cv::flip(grid_img, grid_img, 0);
        cv::Mat grid_img2;
        grid_img.convertTo(grid_img2, CV_32F, 1 / 255.0, 0.0);

        // 这些参数后续可以放到config中
        int size_x = 1500;
        int size_y = 500;
        int init_x = 750;
        int init_y = 250;
        double cell_size = 0.05;

        nav_msgs::OccupancyGrid occ_grid;
        occ_grid.header.frame_id = "world";
        occ_grid.header.stamp = ros::Time::now();
        occ_grid.info.width = size_x;
        occ_grid.info.height = size_y;
        // occ_grid.info.resolution = cell_size;
        occ_grid.info.resolution = RESOLUTION;
        occ_grid.info.origin.position.x = -init_x * cell_size;
        occ_grid.info.origin.position.y = -init_y * cell_size;

        vector<signed char> grid_data(size_x * size_y, -1);

        for (int j = 0; j < size_x; j++)
        {
            for (int i = 0; i < size_y; i++)
            {
                double value = 1.0 - 1.0 * grid_img2.at<float>(j, i);
                cout << value << " ";
                if (abs(value - 0.5) > 0.005)
                    grid_data[size_x * i + j] = value * 100;
            }
        }
        occ_grid.data = grid_data;
        g_map_puber.publish(occ_grid);
    }

    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 2000, vio_callback);
    // ros::Subscriber sub_image = n.subscribe(IMAGE_TOPIC, 100, image_callback);
    // ros::Subscriber sub_depth = n.subscribe(DEPTH_TOPIC, 100, depth_callback);
    message_filters::Subscriber<sensor_msgs::Image> sub_image(n, IMAGE_TOPIC, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth(n, DEPTH_TOPIC, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_image, sub_depth);
    sync.registerCallback(boost::bind(&image_callback, _1, _2));

    ros::Subscriber sub_pose = n.subscribe("/vins_estimator/keyframe_pose", 2000, pose_callback);
    ros::Subscriber sub_extrinsic = n.subscribe("/vins_estimator/extrinsic", 2000, extrinsic_callback);
    ros::Subscriber sub_point = n.subscribe("/vins_estimator/keyframe_point", 2000, point_callback);
    ros::Subscriber sub_margin_point = n.subscribe("/vins_estimator/margin_cloud", 2000, margin_point_callback);

    pub_match_img = n.advertise<sensor_msgs::Image>("match_image", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud_loop_rect", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("margin_cloud_loop_rect", 1000);
    pub_odometry_rect = n.advertise<nav_msgs::Odometry>("odometry_rect", 1000);

    std::thread measurement_process;
    std::thread keyboard_command_process;

    measurement_process = std::thread(process);
    keyboard_command_process = std::thread(command);

    ros::spin();

    return 0;
}
