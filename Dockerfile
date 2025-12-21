ARG ROS_DISTRO=noetic
FROM ros:${ROS_DISTRO}-ros-base
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    git \
    build-essential \
    cmake \
    python3-pip \
    nlohmann-json3-dev \
    libpcl-dev \
    libtbb-dev \
    libceres-dev \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-tf-conversions \
    ros-${ROS_DISTRO}-sophus \
    ros-${ROS_DISTRO}-rosbag
RUN pip3 install rosbags

# Install Livox SDK
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git /tmp/Livox-SDK && \
    cd /tmp/Livox-SDK/build && \
    cmake .. && make -j$(nproc) && make install && \
    rm -rf /tmp/Livox-SDK

# Install Livox SDK2
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 && \
    cd /tmp/Livox-SDK2 && \
    mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && \
    rm -rf /tmp/Livox-SDK2

RUN mkdir -p /test_ws/src
COPY src/ /test_ws/src

# Clone slict if submodule is empty
RUN if [ ! -f /test_ws/src/slict/package.xml ]; then \
      rm -rf /test_ws/src/slict && \
      git clone --depth 1 https://github.com/MapsHD/slict.git /test_ws/src/slict; \
    fi

# Clone ufomap if submodule is empty
RUN if [ ! -f /test_ws/src/ufomap/ufomap/CMakeLists.txt ]; then \
      rm -rf /test_ws/src/ufomap && \
      git clone --depth 1 https://github.com/brytsknguyen/ufomap /test_ws/src/ufomap; \
    fi

# Clone livox_ros_driver2 if submodule is empty
RUN if [ ! -f /test_ws/src/livox_ros_driver2/package_ROS1.xml ]; then \
      rm -rf /test_ws/src/livox_ros_driver2 && \
      git clone --depth 1 https://github.com/brytsknguyen/livox_ros_driver2 /test_ws/src/livox_ros_driver2; \
    fi

# Clone LASzip for converter
RUN if [ ! -f /test_ws/src/slict-to-hdmapping/src/3rdparty/LASzip/CMakeLists.txt ]; then \
      mkdir -p /test_ws/src/slict-to-hdmapping/src/3rdparty && \
      rm -rf /test_ws/src/slict-to-hdmapping/src/3rdparty/LASzip && \
      git clone --depth 1 https://github.com/LASzip/LASzip.git /test_ws/src/slict-to-hdmapping/src/3rdparty/LASzip; \
    fi

RUN if [ ! -f /test_ws/src/livox_ros_driver/package.xml ]; then rm -rf /test_ws/src/livox_ros_driver && git clone https://github.com/brytsknguyen/livox_ros_driver /test_ws/src/livox_ros_driver; fi
RUN if [ -f /test_ws/src/livox_ros_driver2/package_ROS1.xml ]; then cp /test_ws/src/livox_ros_driver2/package_ROS1.xml /test_ws/src/livox_ros_driver2/package.xml; fi
RUN cd /test_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y || true
# Build livox_ros_driver packages first (slict depends on them)
RUN cd /test_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --packages-select livox_ros_driver livox_ros_driver2
# Build remaining packages
RUN cd /test_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && source install/setup.bash && colcon build
