FROM ubuntu:20.04

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release software-properties-common \
    build-essential git cmake \
    python3-pip \
    libceres-dev libeigen3-dev \
    libpcl-dev \
    nlohmann-json3-dev \
    tmux \
    libusb-1.0-0-dev \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros1.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt

RUN wget https://github.com/Kitware/CMake/releases/download/v3.26.4/cmake-3.26.4.tar.gz && \
    tar -xf cmake-3.26.4.tar.gz && cd cmake-3.26.4 && \
    ./bootstrap --prefix=/opt/cmake-3.26.4 && \
    make -j$(nproc)

RUN git clone https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && git fetch --all --tags && git checkout tags/2.1.0 && \
    mkdir build && cd build && \
    /opt/cmake-3.26.4/bin/cmake .. -DCMAKE_INSTALL_PREFIX=/opt/eigen-3.4 && \
    make -j$(nproc) && make install

RUN git clone https://gitlab.com/libeigen/eigen.git && \
    cd eigen && git checkout 3.4.0 && \
    mkdir build && cd build && \
    /opt/cmake-3.26.4/bin/cmake .. -DCMAKE_INSTALL_PREFIX=/opt/eigen-3.4 && \
    make -j$(nproc) && make install

RUN git clone https://github.com/strasdat/Sophus && \
    cd Sophus && mkdir build && cd build && \
    /opt/cmake-3.26.4/bin/cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON && make -j$(nproc)&& make install

RUN git clone https://github.com/Livox-SDK/Livox-SDK.git && \
    cd Livox-SDK && \
    mkdir -p build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && \
    mkdir -p build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

RUN mkdir -p /ws_livox/src

WORKDIR /ws_livox/src

RUN git clone https://github.com/Livox-SDK/livox_ros_driver.git

WORKDIR /ws_livox

RUN source /opt/ros/noetic/setup.bash && \
    catkin build

RUN mkdir -p /ws_livox2/src

WORKDIR /ws_livox2/src

RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git

WORKDIR /ws_livox2/src/livox_ros_driver2

WORKDIR /ws_livox2

RUN source /opt/ros/noetic/setup.bash && \
    ./src/livox_ros_driver2/build.sh ROS1

WORKDIR /ws_livox

RUN source /opt/ros/noetic/setup.bash && \
    catkin init && \
    catkin config --extend /ws_livox2/devel && \
    catkin clean -y && \
    catkin build

WORKDIR /ros_ws

COPY ./src ./src

RUN sed -i 's|<!-- <remap from="/livox/lidar"  to="/livox/lidar_hybrid"/> -->|<remap from="/livox/lidar_ouster"  to="/livox/pointcloud"/>|' /ros_ws/src/slict/launch/run_mcdviral.launch
RUN sed -i 's|"/livox/lidar_ouster"|"/livox/pointcloud"|' /ros_ws/src/slict/config/mcdviral_livox_hhs.yaml && \
    sed -i 's|"/vn200/imu"|"/livox/imu"|' /ros_ws/src/slict/config/mcdviral_livox_hhs.yaml


RUN source /opt/ros/noetic/setup.bash && \
    source /ws_livox2/devel/setup.bash && \
    source /ws_livox/devel/setup.bash && \
    catkin init && \
    catkin build

ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID ros && \
    useradd -m -u $UID -g $GID -s /bin/bash ros
WORKDIR /ros_ws

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /ws_livox/devel/setup.bash" >> ~/.bashrc && \
    echo "source /ws_livox2/devel/setup.bash" >> ~/.bashrc && \
    echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc

CMD ["bash"]
