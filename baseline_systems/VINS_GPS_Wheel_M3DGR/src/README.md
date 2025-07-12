# VINS-GPS-Wheel
## Visual-Inertial Odometry Coupled with Wheel Encoder and GNSS
This repo couples wheel encoder data and GPS data on the basis of [VINS_Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono). The project is tested on [KAIST](https://irap.kaist.ac.kr/dataset/) dataset and is suitable for automatic driving scenario.

<img src="https://github.com/Wallong/VINS-GPS-Wheel/blob/master/support_files/image/kaist.png" width = 80% height = 80% div align=center />

The wheel encoder data is tightly coupled, referred to the paper[[1]](https://ieeexplore.ieee.org/abstract/document/8967607). GPS fusion adopts loose coupling, and the fusion method is consistent with [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion).

Detailed derivations can be found in: https://blog.csdn.net/ewtewtewrt/article/details/117249295
The method has tested in [KAIST](https://irap.kaist.ac.kr/dataset/) dataset (urban28-pankyo) [video](https://www.bilibili.com/video/BV1q64y1R7hW/)

# Install
## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu  16.04.
ROS Kinetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)
additional ROS pacakge
```
    sudo apt-get install ros-YOUR_DISTRO-cv-bridge ros-YOUR_DISTRO-tf ros-YOUR_DISTRO-message-filters ros-YOUR_DISTRO-image-transport
```
### 1.2 **Dependencies**
Follow [Ceres Installation](http://ceres-solver.org/installation.html), remember to **make install**.
(Our testing environment: Ubuntu 16.04, ROS Kinetic, OpenCV 3.1.0, Eigen 3.3.7) 
## 2. Build VINS-Mono on ROS
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/Wallong/VINS-GPS-Wheel.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Dataset
**The method is tested on KAIST dataset.** https://irap.kaist.ac.kr/dataset/

## 4. Example
Open four terminals, launch the vins_estimator, rviz and pubish the data file respectively. Take urban28-pankyo for example
```
    roslaunch vins_estimator kaist.launch 
    rosrun multisensor_fusion multisensor_fusion_node (optional, for GPS)
    rosrun vins_estimator kaist_pub YOUR_PATH_TO_DATASET/KAIST/urban28/urban28-pankyo
    roslaunch vins_estimator vins_rviz.launch
```
## 5. Plan
|  Module   | Status  |
|  ----  | ----  |
| Encoder Pre-integration  | Done |
| Initialization with encoder  | Done |
| Optimization with encoder  | Done |
| Online Extrinsic Calibration about encoder  | Doing |
| Loosely coupled with GNSS  | Done |
| Initialization with GNSS  | Will do |
| Tightly coupled with GNSS  | Will do |

## References
* J. Liu, W. Gao and Z. Hu, "Visual-Inertial Odometry Tightly Coupled with Wheel Encoder Adopting Robust Initialization and Online Extrinsic Calibration," 2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2019, pp. 5391-5397, doi: 10.1109/IROS40897.2019.8967607.

# Contact us
For any issues, please feel free to contact **[Longlong Wang](https://github.com/Wallong)**: <wanglonglong@tju.edu.cn>