FROM osrf/ros:foxy-desktop

RUN apt update && apt-get install --no-install-recommends -y \
    vim \
    git \
    ros-$ROS_DISTRO-can-msgs

WORKDIR /workspace

COPY . tamagawa_imu_driver/

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

CMD ["/bin/bash"] 