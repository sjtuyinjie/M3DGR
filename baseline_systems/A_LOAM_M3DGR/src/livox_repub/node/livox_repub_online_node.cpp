#include <ros/ros.h>
#include "livox_repub/CustomMsg.h"
#include "lidar_data.h"

ros::Publisher pub_livox_points;
pcl::PointCloud<PointXYZIRT>::Ptr raw_cloud_;

void livox_raw_callback(livox_repub::CustomMsgConstPtr lidar_msg)
{
    raw_cloud_->clear();
    for(const auto& p : lidar_msg->points){
        RTPoint point;
        int line_num = (int)p.line;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.intensity = p.reflectivity;
        point.time = p.offset_time/1e9;
        point.ring = (line_num);

        Eigen::Vector3d epoint(p.x,p.y,p.z);
        raw_cloud_->push_back(point);
        
    }
    sensor_msgs::PointCloud2 pcd_msg;
    
    pcl::toROSMsg(*raw_cloud_,pcd_msg);
    pcd_msg.header.stamp = lidar_msg->header.stamp;
    pcd_msg.header.frame_id = "livox";
    pub_livox_points.publish(pcd_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "livox_repub_online_node");
    ros::NodeHandle nh("~");

    std::string livox_raw_message_name;
    nh.param<std::string>("livox_raw_message_name", livox_raw_message_name, "/livox/lidar");
    std::string livox_points_message_name;
    nh.param<std::string>("livox_points_message_name", livox_points_message_name, "/livox/points");

    raw_cloud_ = boost::make_shared<pcl::PointCloud<PointXYZIRT>>();
    pub_livox_points = nh.advertise<sensor_msgs::PointCloud2>(livox_points_message_name.c_str(), 10);
    ros::Subscriber sub_livox_raw = nh.subscribe(livox_raw_message_name.c_str(),1000,livox_raw_callback);

    
    ros::Rate rate(100);
    while(ros::ok())
    {   
        ros::spinOnce();
        rate.sleep();
    }
    return 1;
}