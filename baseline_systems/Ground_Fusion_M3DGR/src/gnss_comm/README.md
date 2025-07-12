# gnss_comm

**Authors/Maintainers:** CAO Shaozu (shaozu.cao AT gmail.com)

The *gnss_comm* package contains basic definitions and utility functions for GNSS raw measurement processing. 

## 1. Prerequisites

### 1.1 C++11 Compiler
This package requires some features of C++11.

### 1.2 ROS
This package is developed under [ROS Kinetic](http://wiki.ros.org/kinetic) environment.

### 1.3 Eigen
Our code uses [Eigen 3.3.3](https://gitlab.com/libeigen/eigen/-/archive/3.3.3/eigen-3.3.3.zip) for matrix manipulation. After downloading and unzipping the Eigen source code package, you may install it with the following commands:

```
cd eigen-3.3.3/
mkdir build
cd build
cmake ..
sudo make install
```

### 1.4 Glog
We use google's glog library for message output. If you are using Ubuntu, install it by:
```
sudo apt-get install libgoogle-glog-dev
```
If you are on other OS or just want to build it from source, please follow [these instructions](https://github.com/google/glog#building-glog-with-cmake) to install it.


## 2. Build gnss_comm library
Clone the repository to your catkin workspace (for example `~/catkin_ws/`):
```
cd ~/catkin_ws/src/
git clone https://github.com/HKUST-Aerial-Robotics/gnss_comm.git
```
Then build the package with:
```
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
```
If you encounter any problem during the building of *gnss_comm*, try with docker in [the next section](#docker_section).

## 3. <a name="docker_section"></a>Docker Support
To simplify the building process, we add docker in our code. Docker is like a sandbox so it can isolate our code from your local environment. To run with docker, first make sure [ros](http://wiki.ros.org/ROS/Installation) and [docker](https://docs.docker.com/get-docker/) are installed on your machine. Then add your account to `docker` group by `sudo usermod -aG docker $USER`. **Relaunch the terminal or logout and re-login if you get `Permission denied` error**, type:
```
cd ~/catkin_ws/src/gnss_comm/docker
make build
```
The docker image `gnss_comm:latest` should be successfully built after a while. Then you can check all available docker images in your local machine by `docker image ls` command.

## 4. Acknowledgements
Many of the definitions and utility functions in this package are adapted from [RTKLIB](http://www.rtklib.com/).

## 5. License
The source code is released under [GPLv3](https://www.gnu.org/licenses/gpl-3.0.html) license.
