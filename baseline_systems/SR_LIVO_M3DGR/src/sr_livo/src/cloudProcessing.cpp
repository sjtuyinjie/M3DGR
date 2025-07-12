#include "cloudProcessing.h"
#include "utility.h"

#define IS_VALID( a ) ( ( abs( a ) > 1e8 ) ? true : false )

cloudProcessing::cloudProcessing()
{
	point_filter_num = 1;
     last_end_time = -1;
	sweep_id = 0;
}

void cloudProcessing::setLidarType(int para)
{
	lidar_type = para;
}

void cloudProcessing::setNumScans(int para)
{
	N_SCANS = para;

	for(int i = 0; i < N_SCANS; i++){
		pcl::PointCloud<pcl::PointXYZINormal> v_cloud_temp;
		v_cloud_temp.clear();
		scan_cloud.push_back(v_cloud_temp);
	}

	assert(N_SCANS == scan_cloud.size());

	for(int i = 0; i < N_SCANS; i++){
		std::vector<extraElement> v_elem_temp;
		v_extra_elem.push_back(v_elem_temp);
	}

	assert(N_SCANS == v_extra_elem.size());
}

void cloudProcessing::setScanRate(int para)
{
	SCAN_RATE = para;
     time_interval_sweep = 1 / double(SCAN_RATE);
}

void cloudProcessing::setTimeUnit(int para)
{
	time_unit = para;

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
}

void cloudProcessing::setBlind(double para)
{
	blind = para;
}

void cloudProcessing::setExtrinR(Eigen::Matrix3d &R)
{
	R_imu_lidar = R;
}

void cloudProcessing::setExtrinT(Eigen::Vector3d &t)
{
	t_imu_lidar = t;
}

void cloudProcessing::setPointFilterNum(int para)
{
	point_filter_num = para;
}

void cloudProcessing::printfFieldName(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    std::cout << "Input pointcloud field names: [" << msg->fields.size() << "]: ";

    for (int i = 0; i < msg->fields.size(); i++)
    {
        std::cout << msg->fields[i].name << ", ";
    }

    std::cout << std::endl;
}

void cloudProcessing::process(const sensor_msgs::PointCloud2::ConstPtr &msg, std::queue<point3D> &point_buffer)
{
     switch (lidar_type)
     {
     case OUST:
          ousterHandler(msg, point_buffer);
          break;

     case VELO:
          velodyneHandler(msg, point_buffer);
          break;

     case ROBO:
          robosenseHandler(msg, point_buffer);
          break;

     default:
          ROS_ERROR("Only Velodyne LiDAR interface is supported currently.");
          printfFieldName(msg);
          break;
     }

    sweep_id++;
}

void cloudProcessing::livoxHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg, std::queue<point3D> &point_buffer)
{
     int point_size = msg->point_num;
     uint num_valid = 0;

     std::vector<point3D> v_point_temp;

     for(uint i = 1; i < point_size; i++)
     {

          // r3live process
          if ((msg->points[i].line < N_SCANS) && (!IS_VALID(msg->points[i].x)) && (!IS_VALID(msg->points[i].y)) && (!IS_VALID(msg->points[i].z)) && msg->points[i].x > 0.7)
          {
               if ((msg->points[i].x > 2.0) && (((msg->points[i].tag & 0x03) != 0x00) || ((msg->points[i].tag & 0x0C) != 0x00))) continue;

               point3D point_temp;

               point_temp.raw_point = Eigen::Vector3d(msg->points[i].x, msg->points[i].y, msg->points[i].z);
               point_temp.point = point_temp.raw_point;
               point_temp.relative_time = msg->points[i].offset_time * time_unit_scale;

               point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
               point_temp.alpha_time = 0.0;

               if ((std::abs(msg->points[i].x - msg->points[i - 1].x) > 1e-7) || (std::abs(msg->points[i].y - msg->points[i - 1].y) > 1e-7) || 
                    (std::abs(msg->points[i].z - msg->points[i - 1].z) > 1e-7))
               {
                    v_point_temp.push_back(point_temp);
               }
          }
          // r3live process
          
          // fast-lio process
          /*               
          if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
          {
               num_valid ++;

               if (num_valid % point_filter_num == 0)
               {
                    point3D point_temp;

                    point_temp.raw_point = Eigen::Vector3d(msg->points[i].x, msg->points[i].y, msg->points[i].z);
                    point_temp.point = point_temp.raw_point;
                    point_temp.relative_time = msg->points[i].offset_time * time_unit_scale;

                    point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
                    point_temp.alpha_time = 0.0;

                    v_point_temp.push_back(point_temp);
               }
          }
          */
          // fast-lio process
     }

     sort(v_point_temp.begin(), v_point_temp.end(), time_list);

     double dt_last_point = v_point_temp.back().relative_time;

     // r3live process
     for (int i = 0; i < v_point_temp.size(); i++)
     {
          num_valid ++;

          if (num_valid % point_filter_num != 0) continue;

          if (v_point_temp[i].raw_point.x() * v_point_temp[i].raw_point.x() + v_point_temp[i].raw_point.y() * v_point_temp[i].raw_point.y() 
               + v_point_temp[i].raw_point.z() * v_point_temp[i].raw_point.z() > (blind * blind))
          {
               point_buffer.push(v_point_temp[i]);
          }     
     }
     // r3live process

     // fast-lio process
     /*
     for (int i = 0; i < v_point_temp.size(); i++)
     {
          if (v_point_temp[i].raw_point.x() * v_point_temp[i].raw_point.x() + v_point_temp[i].raw_point.y() * v_point_temp[i].raw_point.y() 
               + v_point_temp[i].raw_point.z() * v_point_temp[i].raw_point.z() > (blind * blind))
          {
               point_buffer.push(v_point_temp[i]);
          }     
     }
     */
     // fast-lio process

    last_end_time = msg->header.stamp.toSec() + dt_last_point / 1000.0;
}

void cloudProcessing::ousterHandler(const sensor_msgs::PointCloud2::ConstPtr &msg, std::queue<point3D> &point_buffer)
{
     pcl::PointCloud<ouster_ros::Point> raw_cloud;
     pcl::fromROSMsg(*msg, raw_cloud);
     int size = raw_cloud.points.size();

     double dt_last_point;

     if (size == 0)
     {
          return;
     }

     if (raw_cloud.points[size - 1].t > 0)
          given_offset_time = true;
     else
          given_offset_time = false;

     if (given_offset_time)
     {
          sort(raw_cloud.points.begin(), raw_cloud.points.end(), time_list_ouster);
          dt_last_point = raw_cloud.points.back().t * time_unit_scale;
     }

     double omega = 0.361 * SCAN_RATE;

     std::vector<bool> is_first;
     is_first.resize(N_SCANS);
     fill(is_first.begin(), is_first.end(), true);

     std::vector<double> yaw_first_point;
     yaw_first_point.resize(N_SCANS);
     fill(yaw_first_point.begin(), yaw_first_point.end(), 0.0);

     std::vector<point3D> v_point_full;

     for (int i = 0; i < size; i++)
     {
          point3D point_temp;

          point_temp.raw_point = Eigen::Vector3d(raw_cloud.points[i].x, raw_cloud.points[i].y, raw_cloud.points[i].z);
          point_temp.point = point_temp.raw_point;
          point_temp.relative_time = raw_cloud.points[i].t * time_unit_scale;

          if (!given_offset_time)
          {
               int layer = raw_cloud.points[i].ring;
               double yaw_angle = atan2(point_temp.raw_point.y(), point_temp.raw_point.x()) * 57.2957;

               if (is_first[layer])
               {
                    yaw_first_point[layer] = yaw_angle;
                    is_first[layer] = false;
                    point_temp.relative_time = 0.0;

                    v_point_full.push_back(point_temp);

                    continue;
               }

               if (yaw_angle <= yaw_first_point[layer])
               {
                    point_temp.relative_time = (yaw_first_point[layer] - yaw_angle) / omega;
               }
               else
               {
                    point_temp.relative_time = (yaw_first_point[layer] - yaw_angle + 360.0) / omega;
               }

               point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
               v_point_full.push_back(point_temp);
          }

          if (given_offset_time && i % point_filter_num == 0)
          {
               if (point_temp.raw_point.x() * point_temp.raw_point.x() + point_temp.raw_point.y() * point_temp.raw_point.y() + point_temp.raw_point.z() * point_temp.raw_point.z() > (blind * blind))
               {
                    point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
                    point_temp.alpha_time = 0.0;

                    if (point_temp.timestamp > last_end_time)
                         point_buffer.push(point_temp);
               }
          }
     }

     if (!given_offset_time)
     {
          assert(v_point_full.size() == size);

          sort(v_point_full.begin(), v_point_full.end(), time_list);
          dt_last_point = v_point_full.back().relative_time;

          for (int i = 0; i < size; i++)
          {
               if (i % point_filter_num == 0)
               {
                    point3D point_temp = v_point_full[i];
                    point_temp.alpha_time = 0.0;

                    if (point_temp.timestamp > last_end_time)
                         point_buffer.push(point_temp);
               }
          }
     }

     last_end_time = msg->header.stamp.toSec() + dt_last_point / 1000.0;
}

void cloudProcessing::velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg, std::queue<point3D> &point_buffer)
{
     pcl::PointCloud<velodyne_ros::Point> raw_cloud;
     pcl::fromROSMsg(*msg, raw_cloud);
     int size = raw_cloud.points.size();

     double dt_last_point;

     if(size == 0)
     {
          return;
     }

     if (raw_cloud.points[size - 1].time > 0)
          given_offset_time = true;
     else
          given_offset_time = false;

     if(given_offset_time)
     {
          sort(raw_cloud.points.begin(), raw_cloud.points.end(), time_list_velodyne);
          dt_last_point = raw_cloud.points.back().time * time_unit_scale;
     }

     double omega = 0.361 * SCAN_RATE;

     std::vector<bool> is_first;
     is_first.resize(N_SCANS);
     fill(is_first.begin(), is_first.end(), true);

     std::vector<double> yaw_first_point;
     yaw_first_point.resize(N_SCANS);
     fill(yaw_first_point.begin(), yaw_first_point.end(), 0.0);

     std::vector<point3D> v_point_full;

     for(int i = 0; i < size; i++)
     {
          point3D point_temp;

          point_temp.raw_point = Eigen::Vector3d(raw_cloud.points[i].x, raw_cloud.points[i].y, raw_cloud.points[i].z);
          point_temp.point = point_temp.raw_point;
          point_temp.relative_time = raw_cloud.points[i].time * time_unit_scale;

          if(!given_offset_time)
          {
               int layer = raw_cloud.points[i].ring;
               double yaw_angle = atan2(point_temp.raw_point.y(), point_temp.raw_point.x()) * 57.2957;

               if (is_first[layer])
               {
                    yaw_first_point[layer] = yaw_angle;
                    is_first[layer] = false;
                    point_temp.relative_time = 0.0;

                    v_point_full.push_back(point_temp);

                    continue;
               }

               if (yaw_angle <= yaw_first_point[layer])
               {
                    point_temp.relative_time = (yaw_first_point[layer] - yaw_angle) / omega;
               }
               else
               {
                    point_temp.relative_time = (yaw_first_point[layer] - yaw_angle + 360.0) / omega;
               }

               point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
               v_point_full.push_back(point_temp);
          }

          if(given_offset_time && i % point_filter_num == 0)
          {
               if(point_temp.raw_point.x() * point_temp.raw_point.x() + point_temp.raw_point.y() * point_temp.raw_point.y()
                     + point_temp.raw_point.z() * point_temp.raw_point.z() > (blind * blind))
               {
                    point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
                    point_temp.alpha_time = 0.0;

                    if (point_temp.timestamp > last_end_time)
                         point_buffer.push(point_temp);
               }
          }
     }

     if(!given_offset_time)
     {
          assert(v_point_full.size() == size);

          sort(v_point_full.begin(), v_point_full.end(), time_list);
          dt_last_point = v_point_full.back().relative_time;

          for(int i = 0; i < size; i++)
          {
               if(i % point_filter_num == 0)
               {
                    point3D point_temp = v_point_full[i];
                    point_temp.alpha_time = 0.0;

                    if (point_temp.timestamp > last_end_time)
                         point_buffer.push(point_temp);
               }
         }
     }

     last_end_time = msg->header.stamp.toSec() + dt_last_point / 1000.0;
}

void cloudProcessing::robosenseHandler(const sensor_msgs::PointCloud2::ConstPtr &msg, std::queue<point3D> &point_buffer)
{
     pcl::PointCloud<robosense_ros::Point> raw_cloud;
     pcl::fromROSMsg(*msg, raw_cloud);
     int size = raw_cloud.points.size();

     double dt_last_point;

     if (size == 0)
     {
          return;
     }

     if (raw_cloud.points[size - 1].timestamp > 0)
          given_offset_time = true;
     else
          given_offset_time = false;

     if (given_offset_time)
     {
          sort(raw_cloud.points.begin(), raw_cloud.points.end(), time_list_robosense);
          dt_last_point = raw_cloud.points.back().timestamp * time_unit_scale;
     }

     double omega = 0.361 * SCAN_RATE;

     std::vector<bool> is_first;
     is_first.resize(N_SCANS);
     fill(is_first.begin(), is_first.end(), true);

     std::vector<double> yaw_first_point;
     yaw_first_point.resize(N_SCANS);
     fill(yaw_first_point.begin(), yaw_first_point.end(), 0.0);

     std::vector<point3D> v_point_full;

     for (int i = 0; i < size; i++)
     {
          point3D point_temp;

          point_temp.raw_point = Eigen::Vector3d(raw_cloud.points[i].x, raw_cloud.points[i].y, raw_cloud.points[i].z);
          point_temp.point = point_temp.raw_point;
          point_temp.relative_time = (raw_cloud.points[i].timestamp - raw_cloud.points.front().timestamp) * time_unit_scale;

          if (!given_offset_time)
          {
               int layer = raw_cloud.points[i].ring;
               double yaw_angle = atan2(point_temp.raw_point.y(), point_temp.raw_point.x()) * 57.2957;

               if (is_first[layer])
               {
                    yaw_first_point[layer] = yaw_angle;
                    is_first[layer] = false;
                    point_temp.relative_time = 0.0;

                    v_point_full.push_back(point_temp);

                    continue;
               }

               if (yaw_angle <= yaw_first_point[layer])
               {
                    point_temp.relative_time = (yaw_first_point[layer] - yaw_angle) / omega;
               }
               else
               {
                    point_temp.relative_time = (yaw_first_point[layer] - yaw_angle + 360.0) / omega;
               }

               point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
               v_point_full.push_back(point_temp);
          }

          if (given_offset_time && i % point_filter_num == 0)
          {
               if (point_temp.raw_point.x() * point_temp.raw_point.x() + point_temp.raw_point.y() * point_temp.raw_point.y() + point_temp.raw_point.z() * point_temp.raw_point.z() > (blind * blind))
               {
                    point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
                    point_temp.alpha_time = 0.0;

                    if (point_temp.timestamp > last_end_time)
                         point_buffer.push(point_temp);
               }
          }
     }

     if (!given_offset_time)
     {
          assert(v_point_full.size() == size);

          sort(v_point_full.begin(), v_point_full.end(), time_list);
          dt_last_point = v_point_full.back().relative_time;

          for (int i = 0; i < size; i++)
          {
               if (i % point_filter_num == 0)
               {
                    point3D point_temp = v_point_full[i];
                    point_temp.alpha_time = 0.0;

                    if (point_temp.timestamp > last_end_time)
                         point_buffer.push(point_temp);
               }
          }
     }

     last_end_time = msg->header.stamp.toSec() + dt_last_point / 1000.0;
}