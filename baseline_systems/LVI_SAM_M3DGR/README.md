# LVI_SAM
Quickly run on M3DGR, M2DGR-Plus and M2DGR:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/LVI_SAM_M3DGR

cd LVI_SAM_M3DGR

catkin_make
```

## 2. Run M3DGR example
```
# outdoor
source devel/setup.bash
roslaunch lvi_sam m3dgr_run.launch

source devel/setup.bash
roslaunch livox_repub avia_repub.launch


# indoor
source devel/setup.bash
roslaunch lvi_sam 360m3dgr_run.launch

source devel/setup.bash
roslaunch livox_repub mid360_repub.launch

rosbag play Dynamic01.bag

```

## 3. Run M2DGR-Plus example 
```
source devel/setup.bash
roslaunch lvi_sam m2dgr_plus_run.launch

rosbag play tree3.bag
```

## 4. Run M2DGR example
```
source devel/setup.bash
roslaunch lvi_sam m2dgr_run.launch

rosbag play door_02.bag

```
