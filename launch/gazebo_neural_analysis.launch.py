#!/usr/bin/env python3

"""
Launch file for Gazebo Neural Analysis Node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Generate launch description for Gazebo Neural Analysis Node"""
    
    # Declare launch arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/my_camera/image_raw',
        description='Camera topic name'
    )
    
    num_steps_arg = DeclareLaunchArgument(
        'num_steps',
        default_value='150',
        description='Number of processing steps'
    )
    
    initial_x_arg = DeclareLaunchArgument(
        'initial_x',
        default_value='0.4',
        description='Initial X position'
    )
    
    initial_y_arg = DeclareLaunchArgument(
        'initial_y',
        default_value='0.25',
        description='Initial Y position'
    )
    
    initial_hd_arg = DeclareLaunchArgument(
        'initial_hd',
        default_value='0.0',
        description='Initial heading direction'
    )
    
    processing_rate_arg = DeclareLaunchArgument(
        'processing_rate',
        default_value='10.0',
        description='Processing rate in Hz'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='./output',
        description='Output directory for results'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz'
    )
    
    # Set environment variables
    matlab_path_env = SetEnvironmentVariable(
        'MATLABPATH',
        os.path.join(os.path.dirname(__file__), '..', 'matlab')
    )
    
    # Gazebo Neural Analysis Node
    neural_analysis_node = Node(
        package='neural_rate_maps',
        executable='gazebo_neural_analysis.py',
        name='gazebo_neural_analysis',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'num_steps': LaunchConfiguration('num_steps'),
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_hd': LaunchConfiguration('initial_hd'),
            'processing_rate': LaunchConfiguration('processing_rate'),
            'output_dir': LaunchConfiguration('output_dir'),
        }],
        remappings=[
            ('/my_camera/image_raw', LaunchConfiguration('camera_topic')),
        ]
    )
    
    # Static transform publisher for coordinate frames
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    # RViz node (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('neural_rate_maps'),
            'config',
            'neural_analysis.rviz'
        ])],
        condition=LaunchConfiguration('use_rviz'),
        output='screen'
    )
    
    return LaunchDescription([
        camera_topic_arg,
        num_steps_arg,
        initial_x_arg,
        initial_y_arg,
        initial_hd_arg,
        processing_rate_arg,
        output_dir_arg,
        use_rviz_arg,
        matlab_path_env,
        neural_analysis_node,
        static_transform_publisher,
        rviz_node,
    ])