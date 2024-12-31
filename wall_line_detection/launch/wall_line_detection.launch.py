import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    launch_description = LaunchDescription()

    # get pkg path
    wall_line_pkg = get_package_share_directory("wall_line_detection")
    
    # create launch configuration variables
    params_file_path = LaunchConfiguration('wall_line_params_files', default=os.path.join(wall_line_pkg, 'param', 'config.yaml'))
    motion_control_log_level = LaunchConfiguration('wall_line_log_level', default="info")
    
    # manual dock node
    wall_line_node = Node(
        executable='wall_line_detection',
        package='wall_line_detection',
        name='wall_line_detection',
        namespace='',
        output='screen',
        parameters=[params_file_path],
        
    )
   
    launch_description.add_action(wall_line_node)

    return launch_description
