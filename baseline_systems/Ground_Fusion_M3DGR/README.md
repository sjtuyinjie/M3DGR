# Ground-Fusion: A Low-cost Ground SLAM System Robust to Corner Cases (ICRA2024)
Quickly run on M3DGR and M2DGR-Plus:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/Ground_Fusion_M3DGR

cd Ground_Fusion_M3DGR

catkin_make
```

## 2. Run M3DGR example
```
source devel/setup.bash
roslaunch vins groundfusion.launch

source devel/setup.bash
rosrun vins vins_node src/Ground-Fusion/config/realsense/m3dgr.yaml

source devel/setup.bash
rosrun dense_map dense_map_node src/Ground-Fusion/config/realsense/m3dgr.yaml
```

## 3. Run M2DGR-Plus example
```
source devel/setup.bash
roslaunch vins groundfusion.launch

source devel/setup.bash
rosrun vins vins_node src/Ground-Fusion/config/realsense/m2dgrp.yaml

source devel/setup.bash
rosrun dense_map dense_map_node src/Ground-Fusion/config/realsense/m2dgrp.yaml
```
