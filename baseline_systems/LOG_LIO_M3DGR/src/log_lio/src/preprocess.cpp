#include "preprocess.h"
#include "point_type.h"
#include "tic_toc.h"

#define RETURN0     0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
  :feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;
  N_SCANS   = 6;
  SCAN_RATE = 10;
  group_size = 8;
  disA = 0.01;
  disA = 0.1; // B?
  p2l_ratio = 225;
  limit_maxmid =6.25;
  limit_midmin =6.25;
  limit_maxmin = 3.24;
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;
  given_offset_time = false;

  jump_up_limit = cos(jump_up_limit/180*M_PI);
  jump_down_limit = cos(jump_down_limit/180*M_PI);
  cos160 = cos(cos160/180*M_PI);
  smallp_intersect = cos(smallp_intersect/180*M_PI);
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
 avia_handler(msg);
 *pcl_out = pl_surf;
}

void Preprocess::saveNormalPCD(const std::string& file_name, const pcl::PointCloud<PointXYZIRT>::Ptr& cloud, const cv::Mat& normals_mat)
{
    ROS_ASSERT(rangeMat.size == normals_mat.size);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointXYZINormal>);
    for (int i = 0; i < normals_mat.rows; ++i) {
        for (int j = 0; j < normals_mat.cols; ++j) {
            if (rangeMat.at<float>(i, j) == FLT_MAX) //如果该珊格已经有点，则珊格内不再加入后续的点
                continue;

////            int index = j  + i * normals_mat.cols; //点在深度图像中的索引
            const int index = image2cloud[j + i * image_cols];
            const PointXYZIRT& thisPoint = cloud->points[index];
//            int index = j  + i * normals_mat.cols; //点在深度图像中的索引
//            const PointType& thisPoint = fullCloud->points[index];

            pcl::PointXYZINormal pn;
            pn.x = thisPoint.x;
            pn.y = thisPoint.y;
            pn.z = thisPoint.z;

            const cv::Vec3f & n_cv = normals_mat.at<cv::Vec3f>(i, j);
            pn.normal_x = n_cv(0);
            pn.normal_y = n_cv(1);
            pn.normal_z = n_cv(2);

            cloud_normal->push_back(pn);
        }
    }
//    if (cloud_normal->size() > 0)
//        pcl::io::savePCDFile(file_name, *cloud_normal);
}

void Preprocess::projectPointCloud(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud)
{
//    if (!compute_table)
        rangeMat = cv::Mat(N_SCANS, image_cols, CV_32F, cv::Scalar::all(FLT_MAX));
    image2cloud.clear();
    image2cloud.resize(N_SCANS * image_cols, -1);

    TicToc t_range_image;
    int cloudSize = (int)cloud->points.size();
    cloud2image.clear();
    cloud2image.resize(cloudSize, -1);

    // range image projection
    for (int i = 0; i < cloudSize; ++i)
    {
        const PointXYZIRT& thisPoint = cloud->points[i];

        int rowIdn, columnIdn;
        if (!pointInImage(thisPoint, rowIdn, columnIdn))
            continue;

        float range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);//  sqrt(x^2 + y^2 + z^2)


        rangeMat.at<float>(rowIdn, columnIdn) = range; //record range after correcting

        int index = columnIdn  + rowIdn * image_cols;
        image2cloud[index] = i;
        cloud2image[i] = index;
    }
}

void Preprocess::computeRingNormals(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud)
{
    // not test yet
    if (compute_table) {
        TicToc t_range_image;
        range_image.buildTableFromRings(cloud);
        ROS_WARN("build range image from rings cost: %fms", t_range_image.toc());
        return;
    }
    TicToc t_normal;
    range_image.computeNormals(rangeMat, normals, plane_residual); ///output: normalized normals
}



void Preprocess::flipNormalsTowardCenterAndNormalized(const cv::Vec3f& center, const pcl::PointCloud<PointXYZIRT>::Ptr& cloud, cv::Mat& image_normals)
{
    ROS_ASSERT(rangeMat.size == image_normals.size);
    for (int i = 0; i < N_SCANS; ++i)//row
        for (int j = 0; j < image_cols; ++j)
            if (rangeMat.at<float>(i, j) != FLT_MAX)
            {
                const int point_id = image2cloud[j + i * image_cols];
                const PointXYZIRT &thisPoint = cloud->points[point_id];

                // vector: from center to point
                cv::Vec3f vc2p(thisPoint.x - center(0), thisPoint.y - center(1), thisPoint.z - center(2));
                vc2p /= norm(vc2p);

                cv::Vec3f &n = image_normals.at<cv::Vec3f>(i, j); ///already normalized
                if (vc2p.dot(n) > 0)
                    n *= -1;
            }
}

void Preprocess::NormalizeNormals(cv::Mat& image_normals)
{
    ROS_ASSERT(rangeMat.size == image_normals.size);
    for (int i = 0; i < N_SCANS; ++i)
        for (int j = 0; j < image_cols; ++j)
            if (rangeMat.at<float>(i, j) != FLT_MAX)
            {
                cv::Vec3f &n = image_normals.at<cv::Vec3f>(i, j);
                n /= norm(n);
            }
}

void Preprocess::extractCloudAndNormals(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud)
{
    TicToc t_flip;
    cv::Vec3f lidar(0, 0, 0);
    flipNormalsTowardCenterAndNormalized(lidar, cloud, normals);
    // smooth normals after flipping
    TicToc t_smo;
    cv::medianBlur(normals, normals, 5);
    NormalizeNormals(normals);

return;

    cloud_with_normal.reset(new PointCloudXYZI());
    cloud_with_normal->resize(cloud->size());
    int p = 0;
    TicToc t_e;
    int num_filtered = 0;
    int count = 0;
    // extract segmented cloud for lidar odometry
    for (int i = 0; i < N_SCANS; ++i)
    {
        for (int j = 0; j < image_cols; ++j)
        {
            if (rangeMat.at<float>(i,j) != FLT_MAX)//深度图内有信息
            {
                ++count;
                if (count % point_filter_num != 0)
                    continue;

                // save extracted cloud
                const int point_id = image2cloud[j + i * image_cols];
                const cv::Vec3f & n_cv = normals.at<cv::Vec3f>(i, j);
                cloud_with_normal->points[point_id].normal_x = n_cv(0);
                cloud_with_normal->points[point_id].normal_y = n_cv(1);
                cloud_with_normal->points[point_id].normal_z = n_cv(2);
            }
        }
    }
}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (time_unit)
  {
    case SEC:
      time_unit_scale = 1.e3f;
      break;
    case MS:
      time_unit_scale = 1.f;
      break;
    case US:
      time_unit_scale = 1.e-3f;
      break;
    case NS:
      time_unit_scale = 1.e-6f;
      break;
    default:
      time_unit_scale = 1.f;
      break;
  }

  switch (lidar_type)
  {
  case OUSTER:
      ouster_handler(msg);
    break;

  case VELODYNE:
    velodyne_handler(msg);
    break;
  
  default:
    printf("Error LiDAR Type");
    break;
  }
    *pcl_out = pl_surf;

}


void Preprocess::estimateNormals(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud)
{
    TicToc t_1;
    projectPointCloud(cloud);
    proj_time = t_1.toc();
    TicToc t_2;
    computeRingNormals(cloud);
    compu_time = t_2.toc();
    TicToc t_3;
    if (!compute_table)
    {
        extractCloudAndNormals(cloud);
    }
    smooth_time = t_3.toc();
}


void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
 pl_surf.clear();
 pl_corn.clear();
 pl_full.clear();
 double t1 = omp_get_wtime();
 int plsize = msg->point_num;
 // cout<<"plsie: "<<plsize<<endl;

 pl_corn.reserve(plsize);
 pl_surf.reserve(plsize);
 pl_full.resize(plsize);

 for(int i=0; i<N_SCANS; i++)
 {
   pl_buff[i].clear();
   pl_buff[i].reserve(plsize);
 }
 uint valid_num = 0;

 if (feature_enabled)
 {
   for(uint i=1; i<plsize; i++)
   {
     if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
     {
       pl_full[i].x = msg->points[i].x;
       pl_full[i].y = msg->points[i].y;
       pl_full[i].z = msg->points[i].z;
       pl_full[i].intensity = msg->points[i].reflectivity;
       pl_full[i].curvature = msg->points[i].offset_time / float(1000000); //use curvature as time of each laser points

       bool is_new = false;
       if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7)
           || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
           || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
       {
         pl_buff[msg->points[i].line].push_back(pl_full[i]);
       }
     }
   }
   static int count = 0;
   static double time = 0.0;
   count ++;
   double t0 = omp_get_wtime();
   for(int j=0; j<N_SCANS; j++)
   {
     if(pl_buff[j].size() <= 5) continue;
     pcl::PointCloud<PointType> &pl = pl_buff[j];
     plsize = pl.size();
     vector<orgtype> &types = typess[j];
     types.clear();
     types.resize(plsize);
     plsize--;
     for(uint i=0; i<plsize; i++)
     {
       types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
       vx = pl[i].x - pl[i + 1].x;
       vy = pl[i].y - pl[i + 1].y;
       vz = pl[i].z - pl[i + 1].z;
       types[i].dista = sqrt(vx * vx + vy * vy + vz * vz);
     }
     types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
     give_feature(pl, types);
     // pl_surf += pl;
   }
   time += omp_get_wtime() - t0;
   printf("Feature extraction time: %lf \n", time / count);
 }
 else
 {
   for(uint i=1; i<plsize; i++)
   {
     if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
     {
       valid_num ++;
       if (valid_num % point_filter_num == 0)
       {
         pl_full[i].x = msg->points[i].x;
         pl_full[i].y = msg->points[i].y;
         pl_full[i].z = msg->points[i].z;
         pl_full[i].intensity = msg->points[i].reflectivity;
         pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms

         if(((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7)
             || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
             || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
             && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)))
         {
           pl_surf.push_back(pl_full[i]);
         }
       }
     }
   }
 }
}

void Preprocess::ouster2velodyne(const pcl::PointCloud<ouster_ros::Point>& cloud_in, const pcl::PointCloud<PointXYZIRT>::Ptr& cloud_out)
{
    int cloud_size = cloud_in.size();
    cloud_out->resize(cloud_size);
    for (int i = 0; i < cloud_size; ++i) {
        const ouster_ros::Point& p_in = cloud_in[i];
        PointXYZIRT& p_out = cloud_out->points[i];
        p_out.getVector4fMap() = p_in.getVector4fMap();
        p_out.ring = p_in.ring;
    }
}

void Preprocess::ouster_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);

  pcl::PointCloud<PointXYZIRT>::Ptr cloud_tmp(new pcl::PointCloud<PointXYZIRT>());
  ouster2velodyne(pl_orig, cloud_tmp);
    if (compute_normal)
    {
        TicToc t_nor;
        estimateNormals(cloud_tmp); // get cloud_with_normal
        if (runtime_log)
        {
            num_scans++;
            aver_normal_time = aver_normal_time * (num_scans - 1) / num_scans + t_nor.toc() / num_scans;
            aver_proj_time = aver_proj_time * (num_scans - 1) / num_scans + proj_time / num_scans;
            aver_compu_time = aver_compu_time * (num_scans - 1) / num_scans + compu_time / num_scans;
            aver_smooth_time = aver_smooth_time * (num_scans - 1) / num_scans + smooth_time / num_scans;
            ROS_INFO("[normal] mean project %0.3fms, compute %0.3fms, smooth %0.3fms, total %0.3fms",
                     aver_proj_time, aver_compu_time, aver_smooth_time, aver_normal_time);
        }
    }

  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (uint i = 0; i < plsize; i++)
    {
      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < (blind * blind)) continue;
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = cloud_with_normal->points[i].normal_x;// normal_x record ring
      added_pt.normal_y = cloud_with_normal->points[i].normal_y;
      added_pt.normal_z = cloud_with_normal->points[i].normal_z;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      if (yaw_angle >= 180.0)
        yaw_angle -= 360.0;
      if (yaw_angle <= -180.0)
        yaw_angle += 360.0;

      added_pt.curvature = pl_orig.points[i].t * time_unit_scale;
      if(pl_orig.points[i].ring < N_SCANS)
      {
        pl_buff[pl_orig.points[i].ring].push_back(added_pt);
      }
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    double time_stamp = msg->header.stamp.toSec();
    // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      if (i % point_filter_num != 0) continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      
      if (range < (blind * blind)) continue;
      
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].t * time_unit_scale; // curvature unit: ms

        const int & index =  cloud2image[i];
        if (compute_normal && index > 0)
        {
            int rowIdn, columnIdn;
            rowIdn = index / image_cols;
            columnIdn = index - rowIdn * image_cols;
            const cv::Vec3f &n_cv = normals.at<cv::Vec3f>(rowIdn, columnIdn);
            if (!std::isfinite(n_cv(0)) || !std::isfinite(n_cv(1)) || !std::isfinite(n_cv(2)))
                continue;
            added_pt.normal_x = n_cv(0);
            added_pt.normal_y = n_cv(1);
            added_pt.normal_z = n_cv(2);
        }

      pl_surf.points.push_back(added_pt);
    }
  }
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<PointXYZIRT>::Ptr pl_orig(new pcl::PointCloud<PointXYZIRT>());
//    pcl::PointCloud<PointXYZIRT>::Ptr pl_orig;
    pcl::fromROSMsg(*msg, *pl_orig);
    int plsize = pl_orig->points.size();
//    ROS_INFO("cloud input size: %d", plsize);

    if (compute_normal)
    {
        TicToc t_nor;
        estimateNormals(pl_orig); // get cloud_with_normal
        if (runtime_log)
        {
            num_scans++;
            aver_normal_time = aver_normal_time * (num_scans - 1) / num_scans + t_nor.toc() / num_scans;
            aver_proj_time = aver_proj_time * (num_scans - 1) / num_scans + proj_time / num_scans;
            aver_compu_time = aver_compu_time * (num_scans - 1) / num_scans + compu_time / num_scans;
            aver_smooth_time = aver_smooth_time * (num_scans - 1) / num_scans + smooth_time / num_scans;
            ROS_INFO("[normal] mean project %0.3fms, compute %0.3fms, smooth %0.3fms, total %0.3fms",
                     aver_proj_time, aver_compu_time, aver_smooth_time, aver_normal_time);
        }
    }

    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
    std::vector<bool> is_first(N_SCANS,true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig->points[plsize - 1].time > 0)
    {
      given_offset_time = true;
    }
    else
    {
      given_offset_time = false;
      double yaw_first = atan2(pl_orig->points[0].y, pl_orig->points[0].x) * 57.29578;
      double yaw_end  = yaw_first;
      int layer_first = pl_orig->points[0].ring;
      for (uint i = plsize - 1; i > 0; i--)
      {
        if (pl_orig->points[i].ring == layer_first)
        {
          yaw_end = atan2(pl_orig->points[i].y, pl_orig->points[i].x) * 57.29578;
          break;
        }
      }
    }

    if(feature_enabled)
    {
      for (int i = 0; i < N_SCANS; i++)
      {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize);
      }
      
      for (int i = 0; i < plsize; i++)
      {
        PointType added_pt;
        added_pt.normal_x = cloud_with_normal->points[i].normal_x;
        added_pt.normal_y = cloud_with_normal->points[i].normal_y;
        added_pt.normal_z = cloud_with_normal->points[i].normal_z;
        int layer  = pl_orig->points[i].ring;
        if (layer >= N_SCANS) continue;
        added_pt.x = pl_orig->points[i].x;
        added_pt.y = pl_orig->points[i].y;
        added_pt.z = pl_orig->points[i].z;
        added_pt.intensity = pl_orig->points[i].intensity;
        added_pt.curvature = pl_orig->points[i].time * time_unit_scale; // units: ms

        if (!given_offset_time)
        {
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
          if (is_first[layer])
          {
            // printf("layer: %d; is first: %d", layer, is_first[layer]);
              yaw_fp[layer]=yaw_angle;
              is_first[layer]=false;
              added_pt.curvature = 0.0;
              yaw_last[layer]=yaw_angle;
              time_last[layer]=added_pt.curvature;
              continue;
          }

          if (yaw_angle <= yaw_fp[layer])
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
          }
          else
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
          }

          if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

          yaw_last[layer] = yaw_angle;
          time_last[layer]=added_pt.curvature;
        }

        pl_buff[layer].points.push_back(added_pt);
      }

      for (int j = 0; j < N_SCANS; j++)
      {
        PointCloudXYZI &pl = pl_buff[j];
        int linesize = pl.size();
        if (linesize < 2) continue;
        vector<orgtype> &types = typess[j];
        types.clear();
        types.resize(linesize);
        linesize--;
        for (uint i = 0; i < linesize; i++)
        {
          types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
          vx = pl[i].x - pl[i + 1].x;
          vy = pl[i].y - pl[i + 1].y;
          vz = pl[i].z - pl[i + 1].z;
          types[i].dista = vx * vx + vy * vy + vz * vz;
        }
        types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
        give_feature(pl, types);
      }
    }
    else
    {
      for (int i = 0; i < plsize; i++)
      {
        PointType added_pt;
        const PointXYZIRT& orig_pt = pl_orig->points[i];
        // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;
        added_pt.x = orig_pt.x;
        added_pt.y = orig_pt.y;
        added_pt.z = orig_pt.z;
        added_pt.intensity = orig_pt.intensity;
        added_pt.curvature = orig_pt.time * time_unit_scale;  // curvature unit: ms // cout<<added_pt.curvature<<endl;

        if (!given_offset_time)
        {
          int layer = orig_pt.ring;
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

          if (is_first[layer])
          {
            // printf("layer: %d; is first: %d", layer, is_first[layer]);
              yaw_fp[layer]=yaw_angle;
              is_first[layer]=false;
              added_pt.curvature = 0.0;
              yaw_last[layer]=yaw_angle;
              time_last[layer]=added_pt.curvature;
              continue;
          }

          // compute offset time
          if (yaw_angle <= yaw_fp[layer])
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
          }
          else
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
          }

          if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

          yaw_last[layer] = yaw_angle;
          time_last[layer]=added_pt.curvature;
        }

        if (i % point_filter_num == 0)
        {
          if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind))
          {
//              if (!pointInImage(orig_pt, rowIdn, columnIdn))
//                continue;
              const int & index =  cloud2image[i];
              if (compute_normal && index > 0)
              {
                  int rowIdn, columnIdn;
                  rowIdn = index / image_cols;
                  columnIdn = index - rowIdn * image_cols;
                  const cv::Vec3f &n_cv = normals.at<cv::Vec3f>(rowIdn, columnIdn);
                  if (!std::isfinite(n_cv(0)) || !std::isfinite(n_cv(1)) || !std::isfinite(n_cv(2)))
                      continue;
                  added_pt.normal_x = n_cv(0);
                  added_pt.normal_y = n_cv(1);
                  added_pt.normal_z = n_cv(2);
              }
              pl_surf.points.push_back(added_pt);
          }
        }
      }
    }
    pl_surf.resize(pl_surf.points.size());
}

bool Preprocess::pointInImage(const PointXYZIRT& point, int& rowIdn, int& columnIdn)
{
    rowIdn = (int)point.ring; // ring，即行索引，此前以用normal_x记录
    if (rowIdn < 0 || rowIdn >= N_SCANS)
        return false;
    float horizonAngle = atan2(point.y, -point.x);
    if (horizonAngle < 0)
        horizonAngle += 2 * M_PI;
    horizonAngle = horizonAngle  * 180 / M_PI;
    columnIdn = round(horizonAngle/ ang_res_x); //计算列索引， x轴正向为中间列
    if (columnIdn < 0 || columnIdn >= image_cols)
        return false;
    return true;
}

void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  int plsize = pl.size();
  int plsize2;
  if(plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;

  while(types[head].range < blind)
  {
    head++;
  }

  // Surf
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex = 0, i2;
  uint last_i = 0; uint last_i_nex = 0;
  int last_state = 0;
  int plane_type;

  for(uint i=head; i<plsize2; i++)
  {
    if(types[i].range < blind)
    {
      continue;
    }

    i2 = i;

    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);
    
    if(plane_type == 1)
    {
      for(uint j=i; j<=i_nex; j++)
      { 
        if(j!=i && j!=i_nex)
        {
          types[j].ftype = Real_Plane;
        }
        else
        {
          types[j].ftype = Poss_Plane;
        }
      }
      
      // if(last_state==1 && fabs(last_direct.sum())>0.5)
      if(last_state==1 && last_direct.norm()>0.1)
      {
        double mod = last_direct.transpose() * curr_direct;
        if(mod>-0.707 && mod<0.707)
        {
          types[i].ftype = Edge_Plane;
        }
        else
        {
          types[i].ftype = Real_Plane;
        }
      }
      
      i = i_nex - 1;
      last_state = 1;
    }
    else // if(plane_type == 2)
    {
      i = i_nex;
      last_state = 0;
    }

    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }

  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for(uint i=head+3; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i].ftype>=Real_Plane)
    {
      continue;
    }

    if(types[i-1].dista<1e-16 || types[i].dista<1e-16)
    {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
    Eigen::Vector3d vecs[2];

    for(int j=0; j<2; j++)
    {
      int m = -1;
      if(j == 1)
      {
        m = 1;
      }

      if(types[i+m].range < blind)
      {
        if(types[i].range > inf_bound)
        {
          types[i].edj[j] = Nr_inf;
        }
        else
        {
          types[i].edj[j] = Nr_blind;
        }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i+m].x, pl[i+m].y, pl[i+m].z);
      vecs[j] = vecs[j] - vec_a;
      
      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
      if(types[i].angle[j] < jump_up_limit)
      {
        types[i].edj[j] = Nr_180;
      }
      else if(types[i].angle[j] > jump_down_limit)
      {
        types[i].edj[j] = Nr_zero;
      }
    }

    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_zero && types[i].dista>0.0225 && types[i].dista>4*types[i-1].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_zero && types[i].edj[Next]== Nr_nor && types[i-1].dista>0.0225 && types[i-1].dista>4*types[i].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_inf)
    {
      if(edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if(types[i].edj[Prev]==Nr_inf && types[i].edj[Next]==Nr_nor)
    {
      if(edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }
     
    }
    else if(types[i].edj[Prev]>Nr_nor && types[i].edj[Next]>Nr_nor)
    {
      if(types[i].ftype == Nor)
      {
        types[i].ftype = Wire;
      }
    }
  }

  plsize2 = plsize-1;
  double ratio;
  for(uint i=head+1; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i-1].range<blind || types[i+1].range<blind)
    {
      continue;
    }
    
    if(types[i-1].dista<1e-8 || types[i].dista<1e-8)
    {
      continue;
    }

    if(types[i].ftype == Nor)
    {
      if(types[i-1].dista > types[i].dista)
      {
        ratio = types[i-1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i-1].dista;
      }

      if(types[i].intersect<smallp_intersect && ratio < smallp_ratio)
      {
        if(types[i-1].ftype == Nor)
        {
          types[i-1].ftype = Real_Plane;
        }
        if(types[i+1].ftype == Nor)
        {
          types[i+1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }

  int last_surface = -1;
  for(uint j=head; j<plsize; j++)
  {
    if(types[j].ftype==Poss_Plane || types[j].ftype==Real_Plane)
    {
      if(last_surface == -1)
      {
        last_surface = j;
      }
    
      if(j == uint(last_surface+point_filter_num-1))
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    }
    else
    {
      if(types[j].ftype==Edge_Jump || types[j].ftype==Edge_Plane)
      {
        pl_corn.push_back(pl[j]);
      }
      if(last_surface != -1)
      {
        PointType ap;
        for(uint k=last_surface; k<j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j-last_surface);
        ap.y /= (j-last_surface);
        ap.z /= (j-last_surface);
        ap.intensity /= (j-last_surface);
        ap.curvature /= (j-last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}

int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  double group_dis = disA*types[i_cur].range + disB;
  group_dis = group_dis * group_dis;
  // i_nex = i_cur;

  double two_dis;
  vector<double> disarr;
  disarr.reserve(20);

  for(i_nex=i_cur; i_nex<i_cur+group_size; i_nex++)
  {
    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    disarr.push_back(types[i_nex].dista);
  }
  
  for(;;)
  {
    if((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx*vx + vy*vy + vz*vz;
    if(two_dis >= group_dis)
    {
      break;
    }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  double leng_wid = 0;
  double v1[3], v2[3];
  for(uint j=i_cur+1; j<i_nex; j++)
  {
    if((j >= pl.size()) || (i_cur >= pl.size())) break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    v2[0] = v1[1]*vz - vy*v1[2];
    v2[1] = v1[2]*vx - v1[0]*vz;
    v2[2] = v1[0]*vy - vx*v1[1];

    double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
    if(lw > leng_wid)
    {
      leng_wid = lw;
    }
  }


  if((two_dis*two_dis/leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;
  }

  uint disarrsize = disarr.size();
  for(uint j=0; j<disarrsize-1; j++)
  {
    for(uint k=j+1; k<disarrsize; k++)
    {
      if(disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  if(disarr[disarr.size()-2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  if(lidar_type==AVIA)
  {
    double dismax_mid = disarr[0]/disarr[disarrsize/2];
    double dismid_min = disarr[disarrsize/2]/disarr[disarrsize-2];

    if(dismax_mid>=limit_maxmid || dismid_min>=limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {
    double dismax_min = disarr[0] / disarr[disarrsize-2];
    if(dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  
  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}

bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  if(nor_dir == 0)
  {
    if(types[i-1].range<blind || types[i-2].range<blind)
    {
      return false;
    }
  }
  else if(nor_dir == 1)
  {
    if(types[i+1].range<blind || types[i+2].range<blind)
    {
      return false;
    }
  }
  double d1 = types[i+nor_dir-1].dista;
  double d2 = types[i+3*nor_dir-2].dista;
  double d;

  if(d1<d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

 
  if(d1>edgea*d2 || (d1-d2)>edgeb)
  {
    return false;
  }
  
  return true;
}

void Preprocess::initNormalEstimator() {
//    image_cols = Horizon_SCAN / point_filter_num;
    image_cols = Horizon_SCAN;
    range_image = RangeImage(N_SCANS, image_cols);
    ang_res_x = 360.0/float(image_cols);
    if (!compute_table) {
        if (!range_image.loadLookupTable(ring_table_dir, "ring" + std::to_string(N_SCANS))) {
            ROS_ERROR("Wrong path to ring normal M file, EXIT.");
        }
    }
    rangeMat = cv::Mat(N_SCANS, image_cols, CV_32F, cv::Scalar::all(FLT_MAX));
}
