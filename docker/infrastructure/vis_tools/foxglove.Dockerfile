# ================= Dependencies ===================
FROM ros:humble AS base

RUN apt-get update && apt-get install -y curl ros-humble-ros2bag ros-humble-rosbag2* ros-humble-foxglove-msgs&& \
    rm -rf /var/lib/apt/lists/*

# Set up apt repo
RUN apt-get update && apt-get install -y lsb-release software-properties-common apt-transport-https && \
    apt-add-repository universe

# Install Dependencies
RUN apt-get update && \
    apt-get install -y \ 
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-rosbridge-server \
    ros-$ROS_DISTRO-topic-tools

ENV DEBIAN_FRONTEND noninteractive
RUN sudo chsh -s /bin/bash
ENV SHELL=/bin/bash

# ================= Repositories ===================
FROM base as repo

RUN mkdir -p ~/ament_ws/src
WORKDIR /root/ament_ws/src

WORKDIR /root/ament_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh /root/wato_ros_entrypoint.sh
COPY docker/.bashrc /root/.bashrc
ENTRYPOINT ["/root/wato_ros_entrypoint.sh"]