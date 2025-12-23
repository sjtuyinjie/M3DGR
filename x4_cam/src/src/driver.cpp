// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/driver.h"
#include <string>

namespace
{
const double DEFAULT_RATE = 15.0;
const int32_t PUBLISHER_BUFFER_SIZE = 1;
}

namespace cv_camera
{

Driver::Driver(ros::NodeHandle &private_node, ros::NodeHandle &camera_node)
    : private_node_(private_node),
      camera_node_(camera_node)
{
}

void Driver::setup()
{
  double hz(DEFAULT_RATE);
  int32_t device_id(0);
  std::string device_path("");
  std::string frame_id("camera");
  std::string file_path("");
  std::string camera_name("");

  private_node_.getParam("device_id", device_id);
  private_node_.getParam("frame_id", frame_id);
  private_node_.getParam("rate", hz);
  if (!private_node_.getParam("camera_name", camera_name))
  {
    camera_name = frame_id;
  }

  int32_t image_width(640);
  int32_t image_height(480);

  camera_.reset(new Capture(camera_node_,
                            "image_raw",
                            PUBLISHER_BUFFER_SIZE,
                            frame_id,
                            camera_name));

  if (private_node_.getParam("file", file_path) && file_path != "")
  {
    camera_->openFile(file_path);
  }
  else if (private_node_.getParam("device_path", device_path) && device_path != "")
  {
    camera_->open(device_path);
  }
  else
  {
    camera_->open(device_id);
  }
  if (private_node_.getParam("image_width", image_width))
  {
    if (!camera_->setWidth(image_width))
    {
      ROS_WARN("fail to set image_width");
    }
  }
  if (private_node_.getParam("image_height", image_height))
  {
    if (!camera_->setHeight(image_height))
    {
      ROS_WARN("fail to set image_height");
    }
  }

  camera_->setPropertyFromParam(cv::CAP_PROP_POS_MSEC, "cv_cap_prop_pos_msec");
  camera_->setPropertyFromParam(cv::CAP_PROP_POS_AVI_RATIO, "cv_cap_prop_pos_avi_ratio");
  camera_->setPropertyFromParam(cv::CAP_PROP_FRAME_WIDTH, "cv_cap_prop_frame_width");
  camera_->setPropertyFromParam(cv::CAP_PROP_FRAME_HEIGHT, "cv_cap_prop_frame_height");
  camera_->setPropertyFromParam(cv::CAP_PROP_FPS, "cv_cap_prop_fps");
  camera_->setPropertyFromParam(cv::CAP_PROP_FOURCC, "cv_cap_prop_fourcc");
  camera_->setPropertyFromParam(cv::CAP_PROP_FRAME_COUNT, "cv_cap_prop_frame_count");
  camera_->setPropertyFromParam(cv::CAP_PROP_FORMAT, "cv_cap_prop_format");
  camera_->setPropertyFromParam(cv::CAP_PROP_MODE, "cv_cap_prop_mode");
  camera_->setPropertyFromParam(cv::CAP_PROP_BRIGHTNESS, "cv_cap_prop_brightness");
  camera_->setPropertyFromParam(cv::CAP_PROP_CONTRAST, "cv_cap_prop_contrast");
  camera_->setPropertyFromParam(cv::CAP_PROP_SATURATION, "cv_cap_prop_saturation");
  camera_->setPropertyFromParam(cv::CAP_PROP_HUE, "cv_cap_prop_hue");
  camera_->setPropertyFromParam(cv::CAP_PROP_GAIN, "cv_cap_prop_gain");
  camera_->setPropertyFromParam(cv::CAP_PROP_EXPOSURE, "cv_cap_prop_exposure");
  camera_->setPropertyFromParam(cv::CAP_PROP_CONVERT_RGB, "cv_cap_prop_convert_rgb");

  camera_->setPropertyFromParam(cv::CAP_PROP_RECTIFICATION, "cv_cap_prop_rectification");
  camera_->setPropertyFromParam(cv::CAP_PROP_ISO_SPEED, "cv_cap_prop_iso_speed");
  camera_->setPropertyFromParam(cv::CAP_PROP_WHITE_BALANCE_BLUE_U, "cv_cap_prop_white_balance_u");
  camera_->setPropertyFromParam(cv::CAP_PROP_WHITE_BALANCE_RED_V, "cv_cap_prop_white_balance_v");
  camera_->setPropertyFromParam(cv::CAP_PROP_BUFFERSIZE, "cv_cap_prop_buffersize");

  rate_.reset(new ros::Rate(hz));
}

void Driver::proceed()
{
  if (camera_->capture())
  {
    camera_->publish();
  }
  rate_->sleep();
}

Driver::~Driver()
{
}

} // namespace cv_camera
