# TartanVO: A Generalizable Learning-based VO

Quickly run on M3DGR:

## 1. Compile
```
cd ~/catkin_ws/src
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/TartanVO_M3DGR

```

## 2. Rum M3DGR example
We have prepared the Dynamic01 sequence, which you can run directly：
```
roscore

rviz -d ./config.rviz

python tartanvo_node.py

python publish_image_from_folder.py
```

## 3. Prepare other M3DGR data
Save the image in the "data" folder:

```
rosrun image_view extract_images _sec_per_frame:=0.0096 image:=/camera/color/image_raw

rosrun image_transport republish compressed in:=/camera/color/image_raw raw out:=/camera/color/image_raw

rosbag play Dynamic02.bag
```
