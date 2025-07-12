#include "imu_transformer/imu_transformer_nodelet.h"
#include "pluginlib/class_list_macros.h"
#include "geometry_msgs/Vector3Stamped.h"

//#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
//Remove this header when https://github.com/ros/geometry_experimental/pull/78 is released
#include "imu_transformer/tf2_sensor_msgs.h"

namespace imu_transformer
{

  void ImuTransformerNodelet::onInit(){

    nh_in_ = ros::NodeHandle(getNodeHandle(), "imu_in");
    nh_out_ = ros::NodeHandle(getNodeHandle(), "imu_out");
    private_nh_ = getPrivateNodeHandle();

    private_nh_.param<std::string>("target_frame", target_frame_, "base_link");

    tf2_.reset(new tf2_ros::Buffer());
    tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));

    imu_sub_.subscribe(nh_in_, "/camera/imu", 10);
    imu_filter_.reset(new ImuFilter(imu_sub_, *tf2_, target_frame_, 10, nh_in_));
    imu_filter_->registerCallback(boost::bind(&ImuTransformerNodelet::imuCallback, this, _1));
    imu_filter_->registerFailureCallback(boost::bind(&ImuTransformerNodelet::failureCb, this, _2));

    mag_sub_.subscribe(nh_in_, "mag", 10);
    // TF Message filter doesn't support ShapeShifter, which we use for Mag messages. Uncomment when IMU Rep goes into
    // effect and we don't need to support two types of Mag message.
//      mag_filter_.reset(new MagFilter(mag_sub_, *tf2_, target_frame_, 10, nh_in_));
//      mag_filter_->registerCallback(boost::bind(&ImuTransformerNodelet::magCallback, this, _1));
//      mag_filter_->registerFailureCallback(boost::bind(&ImuTransformerNodelet::failureCb, this, _2));
    mag_sub_.registerCallback(boost::bind(&ImuTransformerNodelet::magCallback, this, _1));
  }

  void ImuTransformerNodelet::imuCallback(const ImuMsg::ConstPtr &imu_in)
  {
    if(imu_pub_.getTopic().empty()){
      imu_pub_ = nh_out_.advertise<ImuMsg>("/rslidar/imu", 10);
    }

    try
    {
      ImuMsg imu_out;
      tf2_->transform(*imu_in, imu_out, target_frame_);
      imu_pub_.publish(imu_out);
    }
    catch (tf2::TransformException ex)
    {
      NODELET_ERROR_STREAM_THROTTLE(1.0, "IMU Transform failure: " << ex.what());
      return;
    }
  }

  // Need to support two types of magnemoter message for now, replace with MagMsg subscriber when IMU REP goes into
  // effect
  void ImuTransformerNodelet::magCallback(const topic_tools::ShapeShifter::ConstPtr &msg)
  {

    std::string error;
    try
    {
      MagMsg::ConstPtr mag_in = msg->instantiate<MagMsg>();

      if(tf2_->canTransform(target_frame_, mag_in->header.frame_id, mag_in->header.stamp, &error)){

        if(mag_pub_.getTopic().empty()){
          mag_pub_ = nh_out_.advertise<MagMsg>("mag", 10);
        }

        MagMsg out;
        tf2_->transform(*mag_in, out, target_frame_);
        mag_pub_.publish(out);
      }else{
        NODELET_WARN_STREAM_THROTTLE(1.0, error);
      }
      return;
    }
    catch (topic_tools::ShapeShifterException &e) {
      NODELET_DEBUG_STREAM(e.what());
    }

    try
    {
      geometry_msgs::Vector3Stamped::ConstPtr mag_in = msg->instantiate<geometry_msgs::Vector3Stamped>();

      if(tf2_->canTransform(target_frame_, mag_in->header.frame_id, mag_in->header.stamp, &error)){

        if(mag_pub_.getTopic().empty()){
          mag_pub_ = nh_out_.advertise<geometry_msgs::Vector3Stamped>("mag", 10);
        }

        MagMsg mag_temp_in, mag_temp_out;
        geometry_msgs::Vector3Stamped mag_out;

        mag_temp_in.header = mag_in->header;
        mag_temp_in.magnetic_field = mag_in->vector;
        tf2_->transform(mag_temp_in, mag_temp_out, target_frame_);
        mag_out.header = mag_temp_out.header;
        mag_out.vector = mag_temp_out.magnetic_field;

        mag_pub_.publish(mag_out);
      }else{
        NODELET_WARN_STREAM_THROTTLE(1.0, error);
      }
      return;
    }
    catch (topic_tools::ShapeShifterException &e) {
      NODELET_DEBUG_STREAM(e.what());
    }

    NODELET_ERROR_STREAM_THROTTLE(1.0, "imu_transformer only accepts sensor_msgs::MagneticField and "
          "geometry_msgs::Vector3Stamped message types on the imu_in/mag_in topic, received " << msg->getDataType());

  }

  void ImuTransformerNodelet::failureCb(tf2_ros::filter_failure_reasons::FilterFailureReason reason)
  {
    NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform incoming IMU data to " << target_frame_ << " " << 
        reason);
  }

}

PLUGINLIB_EXPORT_CLASS(imu_transformer::ImuTransformerNodelet, nodelet::Nodelet)
