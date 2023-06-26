import os
from launch import LaunchDescription
from launch_ros import actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    param = os.path.join(
        get_package_share_directory('tamagawa_imu_driver'),
        'config',
        'param.yaml'
    )
    
    tag_can_driver = actions.Node(
        package='tamagawa_imu_driver',
        executable='tag_can_driver',
        parameters=[param],
        output='screen'
    )
    return LaunchDescription([tag_can_driver])