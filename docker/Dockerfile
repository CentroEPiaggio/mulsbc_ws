# Docker image for ARM64 cross-compilation testing of mulsbc_ws
# Builds on x86_64 via QEMU emulation using docker buildx
FROM --platform=linux/arm64 arm64v8/ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    libeigen3-dev \
    python3-colcon-common-extensions \
    ros-$ROS_DISTRO-controller-interface \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-pluginlib \
    ros-$ROS_DISTRO-realtime-tools \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-rosbag2-cpp \
    ros-$ROS_DISTRO-rosidl-default-generators \
    ros-$ROS_DISTRO-rosidl-default-runtime \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-std-srvs \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-xacro \
    && rm -rf /var/lib/apt/lists/*

# bcm_host stub library
# moteus_pi3hat links against -lbcm_host but never calls any symbols from it.
# The real library is RPi-specific; we provide an empty stub so the linker is happy.
RUN echo 'void __bcm_host_stub(void) {}' > /tmp/bcm_stub.c \
    && gcc -shared -o /usr/lib/libbcm_host.so /tmp/bcm_stub.c \
    && rm /tmp/bcm_stub.c \
    && ldconfig

WORKDIR /ws

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo 'source /ws/install/local_setup.bash' >> /root/.bashrc

CMD ["/bin/bash"]
