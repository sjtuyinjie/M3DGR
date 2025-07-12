# LOG-LIO

----------------------------

Our recent work [LOG-LIO2](https://github.com/tiev-tongji/LOG-LIO2) is now available!


The trajectory file will be saved in **TUM** format in the file named "/Log/target_path.txt".

And a more detailed readme is coming soon!

**Related video:**  [Ring FALS](https://youtu.be/cxTLywI7X7M).


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
**Ubuntu >= 16.04**

For **Ubuntu 18.04 or higher**, the **default** PCL and Eigen is enough for FAST-LIO to work normally.

[//]: # ()
[//]: # (ROS    >= Melodic. [ROS Installation]&#40;http://wiki.ros.org/ROS/Installation&#41;)

### 1.2 Ring FALS Normal Estimator
Compile follow [Ring FALS normal estimator](https://github.com/tiev-tongji/RingFalsNormal).


## 2. Build
Clone the repository and catkin_make:

```
    cd ~/$A_ROS_DIR$/src
    git clone https://github.com/tiev-tongji/LOG-LIO.git
    cd ..
    catkin_make
    source devel/setup.bash
```

## 3.  run
We conduct experiments on the [M2DGR](https://github.com/SJTU-ViSYS/M2DGR) and [NTU VIRAL](https://github.com/ntu-aris/ntu_viral_dataset) datasets.
The corresponding launch and yaml file is provided.

Note that the timestamp in the NTU VARIL dataset is the **_end time_**.

```
    cd ~/$LOG_LIO_ROS_DIR$
    source devel/setup.bash
    # for the M2DGR dataset
    roslaunch log_lio mapping_m2dgr.launch
    # for the NTU VIRAL dataset
    roslaunch log_lio mapping_viral.launch
```

## 4. Acknowledgments

Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), 
[Fast-LIO2](https://github.com/hku-mars/FAST_LIO), 
[ikd-Tree](https://github.com/hku-mars/ikd-Tree) 
and [VoxelMap](https://github.com/hku-mars/VoxelMap).

## Citation

```
    @article{huang2023log,
    title={LOG-LIO: A LiDAR-Inertial Odometry with Efficient Local Geometric Information Estimation},
    author={Huang, Kai and Zhao, Junqiao and Zhu, Zhongyang and Ye, Chen and Feng, Tiantian},
    journal={IEEE Robotics and Automation Letters},
    volume={9},
    number={1},
    pages={459--466},
    year={2023},
    publisher={IEEE}
    }
```
