# TartanVO: A Generalizable Learning-based VO

## 1. Compile
```
cd ~/catkin_ws/src
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/TartanVO_M3DGR
cd ../..
catkin_make
```

## 2. Rum example
```
roscore

rviz -d ./config.rviz

python tartanvo_node.py

python publish_image_from_folder.py
```
