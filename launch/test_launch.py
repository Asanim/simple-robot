import os

from ament_index_python.packages import get_package_share_directory

import launch_ros
from launch_ros.actions.node import Node

from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.launch_description import LaunchDescription
from launch.substitutions.launch_configuration import LaunchConfiguration


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="kinect_ros2").find(
        "kinect_ros2"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz/pointcloud.rviz")
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'spider.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('urdf_tutorial_r2d2'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation (Gazebo) clock if true'),

            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            Node(
                package="kinect_ros2",
                executable="kinect_ros2_node",
                name="kinect_ros2",
                namespace="kinect",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rvizconfig")],
            ),
            launch_ros.actions.Node(
                package="image_tools",
                executable="showimage",
                name="rgb_showimage",
                parameters=[{"window_name": "RGB"}],
                remappings=[("image", "kinect/image_raw")],
            ),
            launch_ros.actions.Node(
                package="image_tools",
                executable="showimage",
                name="depth_showimage",
                parameters=[{"window_name": "Depth"}],
                remappings=[("image", "kinect/depth/image_raw")],
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
                arguments=[urdf]),
            Node(
                package='urdf_tutorial_r2d2',
                executable='state_publisher',
                name='state_publisher',
                output='screen'),

        ]
    )