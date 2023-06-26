FROM osrf/ros:foxy-desktop

RUN apt update && apt-get install --no-install-recommends -y \
    vim \
    git \
    ros-$ROS_DISTRO-can-msgs

WORKDIR /workspace

COPY imu/tamagawa_imu_driver /workspace/tamagawa_imu_driver/

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

COPY imu/tamagawa_imu_driver/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["ros2", "launch", "./tamagawa_imu_driver/launch/tag_can_driver.launch.py"]