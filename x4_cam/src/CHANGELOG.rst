^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cv_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2021-07-06)
------------------
* Adding MONO8 format if number of channels is 1
* Contributors: Servando

0.5.0 (2020-05-30)
------------------
* Use OpenCV 3.x+ enums instead of 2.x defines (#28)
* Allow specifying camera name for camera_info_manager (#26)
  I think this is reasonable. Thanks.
* Contributors: Alexey Rogachevskiy

0.4.0 (2019-05-08)
------------------
* Update for Melodic release
  Use libopencv-dev instead of opencv3
* Add link to ROS2 fork
* Improve README.md style
* Add capture_delay parameter
* Fix rescaling coefficient calculation (#15)
* Remove mail address of contributors
* Fix topic names
  Before this change, /image_raw, /camera_info is published.
  But that is not designed. As in README.md, it publishes
  ~image_raw, ~camera_info, that means it contains node name
  as prefix. This is broken so long time ago, (and test was
  failing.)
* Add contributers in README
* Format code
* Automatic rescaling camera info with rescale_camera_info parameter (#10) (#13)
* update to use non deprecated pluginlib macro (#9)
* Contributors: Mikael Arguedas, Oleg Kalachev, Takashi Ogura

0.3.0 (2017-10-03)
------------------
* Update README.md
* Add device_path support (`#8 <https://github.com/OTL/cv_camera/issues/8>`_)
  * Add device_path support
* Contributors: Maurice Meedendorp, Takashi Ogura

0.2.1 (2017-05-05)
------------------
* use OpenCV3
* rostest is optional

0.1.0 (2015-10-17)
------------------
* Fix opencv2 to libopencv-dev
* Contributors: Takashi Ogura

0.0.3 (2015-10-17)
------------------
* Enable any prop code
* Fix coding style using roslint
* Support CV_CAP_PROP_* params
* Contributors: Takashi Ogura

0.0.2 (2013-11-08)
------------------
* rostest should be build_depend (for binary package build)
  see http://docs.ros.org/api/catkin/html/howto/rostest_configuration.html
  for more information.

0.0.1 (2013-11-07)
------------------
* change behavior with calibration yaml.
  overwrite image size with yaml data if yaml is specified.
  if no calib is provided, use image size.
* add offline mode and tests
* use camera_info_manager for CameraInfo.
* add parameters frame_id/image_width/image_height
* ROS Camera node by cv::VideoCapture
* Initial commit
