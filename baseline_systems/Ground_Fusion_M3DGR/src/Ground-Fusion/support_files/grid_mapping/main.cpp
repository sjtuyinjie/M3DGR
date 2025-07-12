#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "grid_map.h"
#include "grid_mapper.h"

using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace cv;

GridMap* g_map;
GridMapper* g_gmapper;

string POSE_GRAPH_SAVE_PATH = "../pose_graph/";

int main (int argc, char** argv){
    Eigen::Matrix4d T;
    T << 0.007941668, 0.00510531, 0.99995543, 0.0424947,
            -0.99955139, -0.02883817,   0.00808569,0.02858,
            0.028878166,  -0.9995710558,  0.004874000,-0.0019294,
            0, 0, 0, 1;
    Eigen::Vector3d tic = (T.block<3, 1>(0, 3));
    Eigen::Matrix3d qic = (T.block<3, 3>(0, 0));

    g_map = new GridMap ( 1500, 500, 750, 250, 0.05 );
    g_gmapper = new GridMapper (g_map, 0.6, 0.4, 0.5);



    FILE * pFile;
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    printf("lode pose graph from: %s \n", file_path.c_str());
    printf("pose graph loading...\n");
    pFile = fopen (file_path.c_str(),"r");
    if (pFile == NULL)
    {
        printf("lode previous pose graph error: wrong previous pose graph path or no previous pose graph \n the system will start with new pose graph \n");
        return 1;
    }
    int index;
    double time_stamp;
    double VIO_Tx, VIO_Ty, VIO_Tz;
    double PG_Tx, PG_Ty, PG_Tz;
    double VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz;
    double PG_Qw, PG_Qx, PG_Qy, PG_Qz;
    double loop_info_0, loop_info_1, loop_info_2, loop_info_3;
    double loop_info_4, loop_info_5, loop_info_6, loop_info_7;
    int loop_index;
    int keypoints_num,densepoints_num;
    Eigen::Matrix<double, 8, 1 > loop_info;
    int cnt = 0;
    while (fscanf(pFile,"%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d %d", &index, &time_stamp,
                  &VIO_Tx, &VIO_Ty, &VIO_Tz,
                  &PG_Tx, &PG_Ty, &PG_Tz,
                  &VIO_Qw, &VIO_Qx, &VIO_Qy, &VIO_Qz,
                  &PG_Qw, &PG_Qx, &PG_Qy, &PG_Qz,
                  &loop_index,
                  &loop_info_0, &loop_info_1, &loop_info_2, &loop_info_3,
                  &loop_info_4, &loop_info_5, &loop_info_6, &loop_info_7,
                  &keypoints_num, &densepoints_num) != EOF)
    {
        cv::Mat image;
        std::string image_path, descriptor_path;

        Vector3d VIO_T(VIO_Tx, VIO_Ty, VIO_Tz);
        Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);
        Quaterniond VIO_Q;
        VIO_Q.w() = VIO_Qw;
        VIO_Q.x() = VIO_Qx;
        VIO_Q.y() = VIO_Qy;
        VIO_Q.z() = VIO_Qz;
        Quaterniond PG_Q;
        PG_Q.w() = PG_Qw;
        PG_Q.x() = PG_Qx;
        PG_Q.y() = PG_Qy;
        PG_Q.z() = PG_Qz;
        Matrix3d VIO_R, PG_R;
        VIO_R = VIO_Q.toRotationMatrix();
        PG_R = PG_Q.toRotationMatrix();
        Eigen::Matrix<double, 8, 1 > loop_info;
        loop_info << loop_info_0, loop_info_1, loop_info_2, loop_info_3, loop_info_4, loop_info_5, loop_info_6, loop_info_7;


        string densepoints_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_densepoints.txt";
        FILE *densepoints_file;
        densepoints_file = fopen(densepoints_path.c_str(), "r");
        vector<Eigen::Matrix<float ,6, 1>> points_rgbd;
        for (int i = 0; i < densepoints_num; i++){
            double p_x, p_y, p_z, p_r, p_g, p_b;
            Eigen::Matrix<float ,6, 1> point_rgbd;
            if(!fscanf(densepoints_file,"%lf %lf %lf %lf %lf %lf", &p_x, &p_y, &p_z, &p_r, &p_g, &p_b))
                printf(" fail to load pose graph \n");
            point_rgbd[0] = p_x;
            point_rgbd[1] = p_y;
            point_rgbd[2] = p_z;
            point_rgbd[3] = p_r;
            point_rgbd[4] = p_g;
            point_rgbd[5] = p_b;
            points_rgbd.push_back(point_rgbd);
        }
        fclose(densepoints_file);


        for(auto &point_rgbd : points_rgbd){
            Eigen::Vector3d point(point_rgbd[0], point_rgbd[1], point_rgbd[2]);
            Eigen::Vector3d pointWorld = PG_R * (qic * point + tic) + PG_T;
            if(pointWorld.z() > 0.5 || pointWorld.z() < 0.1)
                continue;
            pointWorld.z() = 0;
            g_gmapper->updateMap(pointWorld, PG_T);
        }
    }
    fclose (pFile);

    g_map->saveMap ( "map.png", "map.yaml" );

    return (0);

}
