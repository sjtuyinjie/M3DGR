# Coco-LIC
Quickly run on M3DGR:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/Coco_LIC_M3DGR

cd Coco_LIC_M3DGR

catkin_make
```
## 2. Run M3DGR example
```
source devel/setup.bash
roslaunch cocolic m3dgr_avia_odometry.launch

rosbag play Dynamic01.bag
```
