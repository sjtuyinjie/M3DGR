# ORB-SLAM2
## M3DGR
```
./build.sh

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM2/Examples/ROS

./build_ros.sh

roscore

source Examples/ROS/ORB_SLAM2/build/devel/setup.bash

rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/m3dgr.yaml

rosrun image_transport republish compressed in:=/camera/color/image_raw  raw  out:=/camera/color/image_raw

rosbag play dynamic_person_1.bag
```
