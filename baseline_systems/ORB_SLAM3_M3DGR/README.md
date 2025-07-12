# ORB-SLAM3
## M3DGR
```
./build.sh

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM3/Examples/ROS

./build_ros.sh

roscore

source Examples/ROS/ORB_SLAM3/build/devel/setup.bash

rosrun ORB_SLAM3 Mono_Inertial Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/m3dgr.yaml

rosrun image_transport republish compressed in:=/camera/color/image_raw  raw  out:=/camera/color/image_raw

```
