FROM ros:kinetic-perception

ENV EIGEN_VERSION="3.3.3"
ENV CATKIN_WS=/root/catkin_ws

      # set up thread number for building
RUN   apt-get update && apt-get install -y \
      git \
      cmake \
      python-catkin-tools \
      libgoogle-glog-dev && \
      rm -rf /var/lib/apt/lists/* && \
      # install eigen
      git clone https://gitlab.com/libeigen/eigen.git && \
      cd eigen && \
      git checkout tags/${EIGEN_VERSION} && \
      mkdir build && cd build && \
      cmake .. && make install && \
      cd ../.. && rm -rf eigen && \
      # create gnss_comm directory
      mkdir -p $CATKIN_WS/src/gnss_comm/

# Copy the local replica of gnss_comm
COPY ./ $CATKIN_WS/src/gnss_comm/
# use the following line if you only have this dockerfile
# RUN git clone https://github.com/HKUST-Aerial-Robotics/gnss_comm.git

# Build gnss_comm
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release && \
    catkin build && \
    sed -i '/exec "$@"/i \
            source "/root/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh
