# LIO-SAM-MID360

本代码为LIO-SAM适配Livox MID360版本。
支持使用六轴和九轴IMU。

## Dependency

- 与LIO-SAM有相同依赖 ( [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/) ）
- 请安装LIVOX ROS包用于发布点云数据 ( [览沃 ROS 驱动程序](https://github.com/Livox-SDK/livox_ros_driver/) )

如果使用Livox-ros-driver2 驱动可以切换分支为 [Livox-ros-driver2](https://github.com/nkymzsy/LIO-SAM-MID360/tree/Livox-ros-driver2) 。

## Prepare lidar data

Livox MID360雷达数据请使用览沃自定义点云数据`CustomMsg`，即LIVOX ROS驱动请运行`livox_lidar_msg.launch`。

## Run

使用自带的6轴IMU请运行

```
roslaunch lio_sam run6axis.launch
```

使用9轴IMU请运行

```
roslaunch lio_sam run9axis.launch
```

## Test

- 室内环境旋转测试

<img src="doc/indoorTest.gif" alt="indoorTest.gif" title="Indoor Test" style="zoom: 100%;"/>

- 室外环境 倾斜手持

<img src="doc/Outdoor.gif" alt="Outdoor.gif" title="Outdoor" style="zoom: 100%;"/>

- 室内环境

<img src="doc/indoor.gif" alt="indoor.gif" title="indoor" style="zoom: 100%;"/>

## Note

- 注意对齐Lidar与IMU的时间戳。
- 使用非自带IMU注意修改Lidai与IMU的外参。(param.yaml文件)
- 六轴IMU默认使用MID360内置的IMU，加速度单位为g，处理时乘重力加速度转换为m/s^2，如果使用其他六轴IMU需要注释这条语句。
- 如果观察到IMU里程计抖动的话，请调节IMU参数

## TODO

- [ ] 提取特征优化

## Acknowledgement

- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/)
- [LIO-SAM-DetailedNote](https://github.com/smilefacehh/LIO-SAM-DetailedNote)
- [LIO_SAM_6AXIS](https://github.com/JokerJohn/LIO_SAM_6AXIS)
