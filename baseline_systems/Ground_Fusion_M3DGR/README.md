# Ground-Fusion: A Low-cost Ground SLAM System Robust to Corner Cases (ICRA2024)
## M2DGR-Plus
```
source devel/setup.bash
roslaunch vins groundfusion.launch

source devel/setup.bash
rosrun vins vins_node src/Ground-Fusion/config/realsense/m2dgrp.yaml

source devel/setup.bash
rosrun dense_map dense_map_node src/Ground-Fusion/config/realsense/m2dgrp.yaml
```

## M3DGR
```
source devel/setup.bash
roslaunch vins groundfusion.launch

source devel/setup.bash
rosrun vins vins_node src/Ground-Fusion/config/realsense/m3dgr.yaml

source devel/setup.bash
rosrun dense_map dense_map_node src/Ground-Fusion/config/realsense/m3dgr.yaml
```
