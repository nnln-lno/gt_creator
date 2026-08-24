import os
import launch

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    uwb_config_dir = get_package_share_directory('sobang_navigation')
    uwb_config_    = os.path.join(uwb_config_dir, 'param', 'nav_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'uwb_config_',
            default_value=uwb_config_,
            description="Path to the uwb anchor configuration"
        ),
        Node(
            package='gt_creator',
            executable='gt_creator_node',
            output='screen',
            parameters=[LaunchConfiguration('uwb_config_')]
        ),
    ])
    
