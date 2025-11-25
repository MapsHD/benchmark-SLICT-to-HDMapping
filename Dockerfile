ARG ROS_DISTRO=noetic
FROM ros:${ROS_DISTRO}-ros-base
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y python3-colcon-common-extensions ros-${ROS_DISTRO}-pcl-ros git nlohmann-json3-dev libpcl-dev python3-pip libtbb-dev ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-tf-conversions libceres-dev ros-${ROS_DISTRO}-sophus
RUN pip3 install rosbags
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git /tmp/Livox-SDK && \
    cd /tmp/Livox-SDK/build && \
    cmake .. && make -j$(nproc) && make install && \
    rm -rf /tmp/Livox-SDK
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 && \
    cd /tmp/Livox-SDK2 && \
    mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && \
    rm -rf /tmp/Livox-SDK2
RUN mkdir -p /test_ws/src
COPY src/ /test_ws/src
RUN if [ ! -f /test_ws/src/livox_ros_driver/package.xml ]; then rm -rf /test_ws/src/livox_ros_driver && git clone https://github.com/Livox-SDK/livox_ros_driver.git /test_ws/src/livox_ros_driver; fi
RUN if [ -f /test_ws/src/livox_ros_driver2/package_ROS1.xml ]; then cp /test_ws/src/livox_ros_driver2/package_ROS1.xml /test_ws/src/livox_ros_driver2/package.xml; fi
RUN cd /test_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y || true
# Build livox_ros_driver packages first (slict depends on them)
RUN cd /test_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --packages-select livox_ros_driver livox_ros_driver2
# Build remaining packages
RUN cd /test_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && source install/setup.bash && colcon build
