# GNSS SPP Solution
This is a GNSS SPP solution demo：

## 1. Compile:
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/SPP_M3DGR

cd baseline_systems/SPP_M3DGR

catkin_make
```
## 2. Run M3DGR or m2dgr-Plus example
```
roscore

source devel/setup.bash
rosrun gnss_spp spp

rosbag play Dynamic01.bag
```

