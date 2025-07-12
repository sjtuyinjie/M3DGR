
# ROS-Wrapper for DM-VIO: Delayed Marginalization Visual-Inertial Odometry

For more information see https://vision.in.tum.de/dm-vio and https://github.com/lukasvst/dm-vio.

This is a ROS-Wrapper for DM-VIO, inspired by the [ROS-Wrapper for DSO](https://github.com/JakobEngel/dso_ros).

It interfaces with ROS topics to input images and IMU data and to output poses.
Alternatively, it can read images and IMU data directly from a rosbag dataset.

#### Related Papers
* **[DM-VIO: Delayed Marginalization Visual-Inertial Odometry](https://vision.in.tum.de/dm-vio)**, L. von Stumberg and D. Cremers, In IEEE Robotics and Automation Letters (RA-L), volume 7, 2022
* **[Direct Sparse Visual-Inertial Odometry using Dynamic Marginalization](https://vision.in.tum.de/vi-dso)**, L. von Stumberg, V. Usenko and D. Cremers, In International Conference on Robotics and Automation (ICRA), 2018
* **[Direct Sparse Odometry](https://vision.in.tum.de/dso)**, *J. Engel, V. Koltun, D. Cremers*, In  TPAMI, vol. 40, 2018

### Installation
First install DM-VIO as described [here](https://github.com/lukasvst/dm-vio) and make sure it works.
This version of the ROS wrapper was tested with commit `d18fa15ba086043561361941b8e5298074b34b47` of DM-VIO.

You need to set the environment variable `DMVIO_BUILD` to point to the folder where you built DM-VIO (which is expected to be 
a direct subfolder of DM-VIO).
The easiest way is to put the following into your `.bashrc`

    export DMVIO_BUILD=/PATH/TO/dm-vio/cmake-build-relwithdebinfo

Then you can clone this project into your workspace and build normally with `catkin_make`.

### Running

You can run with 

    rosrun dmvio_ros node calib=/PATH/TO/camera.txt \
                          imuCalib=/PATH_TO/camchain.yaml \
                          settingsFile=/PATH_TO/dm-vio-public/configs/t265_noise_tumvi.yaml \
                          mode=1 nogui=0 preset=1 quiet=1 
                          # vignette=PATH_TO/vignette.png # highly recommended, especially for fisheye cameras (use mode=3 if vignette is set).

Most commandline arguments are the same as for [DM-VIO](https://github.com/lukasvst/dm-vio#3-running), you will at least need to pass camera calibration, IMU camchain, preset and mode.

The node will listen for images at `cam0/image_raw` and for IMU messages at `/imu0`.

Settings file: I recommend starting with `t265_noise_tumvi`, and in general with TUM-VI noise values. For more details and tips see [here](https://github.com/lukasvst/dm-vio/blob/master/doc/RealsenseLiveVersion.md#adjusting-the-config-file).

Notes on exposures and photometric calibration: The standard ROS image messages do not include exposure times. If you have a custom message with exposures you can modify the method `vidCb` and pass the exposure to `undistorter->undistort`.

If you do not have exposure times you cannot use `mode=0`!
Note that `mode=1` completely disables photometric calibration, including the vignette, even if it is passed.
Hence, I have added `mode=3`, which uses photometric calibration, but does not assume that there are exposures.

#### Example 1: Using EuRoC Bags
To test your installation you can download this EuRoC bag: http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room2/V2_01_easy/V2_01_easy.bag

You need to create the `camera.txt` file using

    echo -e "458.654 457.296 367.215 248.375 -0.28340811 0.07395907 0.00019359 1.76187114e-05\n752 480\ncrop\n640 480\n" > camera.txt


Then you can run the following commands in separate terminals

    rosrun dmvio_ros node calib=/PATH/TO/camera.txt settingsFile=/PATH/TO/dm-vio/configs/euroc.yaml mode=1 nogui=0 preset=1 useimu=1 quiet=1 init_requestFullResetNormalizedErrorThreshold=0.8 init_pgba_skipFirstKFs=1

    rosbag play V2_01_easy.bag


#### New: Now you can also run in non-realtime mode on rosbags

For EuRoC, simply run the following command in a terminal. (Make sure that roscore is running in a separate terminal.)

    rosrun dmvio_ros node calib=/PATH/TO/camera.txt settingsFile=/PATH/TO/dm-vio/configs/euroc.yaml mode=1 nogui=0 preset=1 useimu=1 quiet=1 init_requestFullResetNormalizedErrorThreshold=0.8 init_pgba_skipFirstKFs=1 rosbag=/PATH/TO/V2_01_easy.bag loadRosbagThread=1


This command will run as fast as possible and exit once execution is complete, which makes it useful to run on rosbag 
datasets. If a dataset is available as a rosbag, this is more convenient than extracting the rosbag and running the non-ros 
version of DM-VIO. However, keep in mind that ROS does not support exposure times out of the box, which can result in 
slightly worse accuracy compared to a dataset with full photometric calibration (including exposure times).

#### Example 2: Running on Realense T265
You can run on the Realsense T265 with their provided ROS driver. Install it with

    sudo apt-get install ros-$ROS_DISTRO-realsense2-camera

Then you can run the following commands in separate terminals

    rosrun dmvio_ros node nogui=0 useimu=1 quiet=1 mode=3 `#we use mode 3 because we have vignette but no expusures` \
                          calib=/PATH_TO/RealsenseCalibration/camera.txt \
                          imuCalib=/PATH_TO/RealsenseCalibration/factory_camchain.yaml \
                          gamma=PATH_TO/dm-vio/configs/pcalib_linear_8bit.txt \
                          vignette=PATH_TO/dm-vio/configs/realsense/vignette_t265.png \
                          settingsFile=/PATH_TO/dm-vio/configs/t265_noise_tumvi.yaml \
                          resultsPrefix=/PATH_TO_RESULTS/ \
                          cam0/image_raw:=/camera/fisheye1/image_raw imu0:=/camera/imu # Remap topics

    roslaunch realsense2_camera rs_t265.launch unite_imu_method:=linear_interpolation enable_fisheye1:=true enable_fisheye2:=true enable_pose:=false

The `camera.txt` and `factory_camchain.txt` can be obtained by calibrating yourself or by first running the normal [Realsense demo](https://github.com/lukasvst/dm-vio/blob/master/doc/RealsenseLiveVersion.md) which will save the factory calibration.

Note that the ROS driver does **not** read out exposures, hence we need to use `mode=3` instead of `mode=1` and the accuracy can be a bit worse than with the normal Realsense demo.

### Published topics
When using this ROS wrapper you might want to change `src/ROSOutputWrapper.cpp` to publish information in a format suited to your application.

We publish the following topics by default:

### `dmvio/frame_tracked`: DMVIOPoseMsg
Published for every tracked frame (except the inital two frames). 
It includes
* `geometry_msgs/Pose` pose: Transformation from camera to world in DSO frame (meaning it can have an arbitrary scale).

and all fields in `transformDSOToIMU`, which enables you to convert the pose (and all published poses prior to this) to the metric frame:
* `float64` scale
* `geometry_msgs/Quaternion` rotationMetricToDSO: Gravity direction encoded as the rotation to convert from metric world to DSO world
* `geometry_msgs/Pose` imuToCam: Transformation between camera and IMU

### `dmvio/unscaled_pose`: PoseStamped
This is just a copy of `/dmvio/frame_tracked/pose`. It can be useful for visualizing in Rviz as PoseStamped is a standard message.
**Note** that the used coordinate system is camera-based (see below), which is why it can look strange in Rviz.

### `dmvio/metric_pose`: PoseStamped
These poses are the transformation from IMU to world (in metric scale).
We provide this for convenience but for many applications you should **not** directly subscribe to this topic, but rather convert the poses yourself using the newest scale.

The reason is that these metric poses are all converted with the scale estimated at the time they were published.
But when you perform computations on the entire trajectory you want to **convert all poses with the newest scale** instead. 

### `dmvio/system_status`: Int32 
This is a number, indicating the status the system is in (0 means visual initializer, 1 means visual-only mode and 2 means visual-inertial mode). 
For details see the enum [`dmvio::SystemStatus`](https://github.com/lukasvst/dm-vio/blob/master/src/dso/IOWrapper/Output3DWrapper.h#L46).
Whenever a full reset happens, 0 is sent to this topic again.

#### Coordinate system
As mentioned the poses are either in metric or in DSO frame, for converting between the two see [TransformDSOToIMU::transformPose](https://github.com/lukasvst/dm-vio/blob/master/src/GTSAMIntegration/PoseTransformationIMU.cpp) or equation (5) in the [DM-VIO paper](https://vision.in.tum.de/_media/research/vslam/dm-vio/dm-vio.pdf).

We use the typical camera-based coordinate frame, where x points to the right of the image, y points down and z points forward (into the camera).

#### What is not published / implemented and what you might want to implement yourself:
* The **keyframes** after bundle adjustment: These will be more accurate than the initially tracked frame poses. You can access them as well as the **pointcloud** by overriding `publishKeyframes` in `ROSOutputWrapper.cpp`. You could take inspiration from how it was implemented in [LSD-SLAM](https://github.com/tum-vision/lsd_slam/blob/master/lsd_slam_core/src/IOWrapper/ROS/ROSOutput3DWrapper.cpp).
* TF frames
* Dynamic reconfigure

### License
This ROS wrapper is licensed under the GNU General Public License 3 (GPLv3).
The main.cpp file reuses code from the main file of DSO.
