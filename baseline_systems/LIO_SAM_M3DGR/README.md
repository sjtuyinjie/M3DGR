# LIO-SAM
Quickly run on M3DGR, M2DGR-Plus and M2DGR:

## 1.Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/LIO_SAM_M3DGR

cd baseline_systems/LIO_SAM_M3DGR

catkin_make
```

## 2. Run M3DGR example
```
# outdoor:
source devel/setup.bash
roslaunch lio_sam_6axis avia_m3dgr_run.launch
(roslaunch lio_sam_6axis cavia_m3dgr_run.launch  # for camera imu)

source devel/setup.bash
roslaunch livox_repub avia_repub.launch

# indoor:
source devel/setup.bash
roslaunch lio_sam_6axis mid360_m3dgr_run.launch
(roslaunch lio_sam_6axis cmid360_m3dgr_run.launch # for camera imu)

source devel/setup.bash
roslaunch livox_repub mid360_repub.launch

rosbag play Dynamic01.bag
```

## 3. Run M2DGR-Plus example

```
source devel/setup.bash
roslaunch lio_sam m2dgrplus_run.launch

rosbag play tree3.bag
```

## 4. Run M2DGR example
```
source devel/setup.bash
roslaunch lio_sam_6axis m2dgr_run.launch

rosbag play door_02.bag
```



