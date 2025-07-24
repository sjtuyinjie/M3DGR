# ORB-SLAM2
Quickly run on M3DGR:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/ORB_SLAM2_M3DGR

cd baseline_systems/ORB_SLAM2_M3DGR

./build.sh

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM2/Examples/ROS

./build_ros.sh

```

## 2. Run M3DGR example
```
roscore

source Examples/ROS/ORB_SLAM2/build/devel/setup.bash
rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/m3dgr.yaml

rosrun image_transport republish compressed in:=/camera/color/image_raw  raw  out:=/camera/color/image_raw

rosbag play Dynamic01.bag
```
