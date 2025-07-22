# VIW-Fusion
Quickly run on M3DGR and M2DGR-Plus:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/VIW_Fusion_M3DGR

cd VIW_Fusion_M3DGR

catkin_make
```

## 2. Run M3DGR example
```
source devel/setup.bash
roslaunch vins vins_rviz.launch

source devel/setup.bash
rosrun vins viwo_node src/config/realsense_d435i/m3dgr_realsense_stereo_imu_config_ridgeback.yaml

source devel/setup.bash
(optional) rosrun loop_fusion loop_fusion_node src/config/realsense_d435i/m3dgr_realsense_stereo_imu_config_ridgeback.yaml

```

## 3. Run M2DGR-Plus example

```
source devel/setup.bash
roslaunch vins vins_rviz.launch

source devel/setup.bash
rosrun vins viwo_node src/config/realsense_d435i/m2dgr_plus_realsense_stereo_imu_config_ridgeback.yaml

source devel/setup.bash
(optional) rosrun loop_fusion loop_fusion_node src/config/realsense_d435i/m2dgr_plus_realsense_stereo_imu_config_ridgeback.yaml
```

