import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'sbrobot'
    file_subpath = 'description/2linkarm.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]  # add other parameters here if required
    )

    # Configure the joint_state_publisher_gui node
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    # Configure RViz node
    rviz_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',  # Adjust this path based on your package structure
        'sbrobot_config.rviz'  # Replace with your RViz config file name
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    # Run the nodes
    return LaunchDescription([
        node_robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
    ])

