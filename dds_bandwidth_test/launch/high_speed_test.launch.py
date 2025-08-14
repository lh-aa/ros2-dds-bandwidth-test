#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Launch arguments for high-speed streaming
    stream_type_arg = DeclareLaunchArgument(
        'stream_type',
        default_value='pointcloud',
        description='Type of stream (pointcloud, laserscan, odometry, twist)'
    )
    
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='1000.0',
        description='Streaming frequency in Hz'
    )
    
    point_count_arg = DeclareLaunchArgument(
        'point_count',
        default_value='10000',
        description='Number of points in pointcloud (if applicable)'
    )
    
    duration_arg = DeclareLaunchArgument(
        'duration',
        default_value='30.0',
        description='Test duration in seconds'
    )
    
    enable_multiple_streams_arg = DeclareLaunchArgument(
        'enable_multiple_streams',
        default_value='false',
        description='Enable multiple concurrent streams'
    )
    
    enable_subscriber_arg = DeclareLaunchArgument(
        'enable_subscriber',
        default_value='true',
        description='Enable the subscriber for monitoring'
    )

    # High-speed streamer node
    streamer_node = Node(
        package='dds_bandwidth_test',
        executable='high_speed_streamer',
        name='high_speed_streamer',
        parameters=[{
            'stream_type': LaunchConfiguration('stream_type'),
            'frequency': LaunchConfiguration('frequency'),
            'point_count': LaunchConfiguration('point_count'),
            'duration': LaunchConfiguration('duration'),
            'enable_multiple_streams': LaunchConfiguration('enable_multiple_streams'),
        }],
        output='screen'
    )

    # Subscriber for monitoring (subscribe to the main stream)
    subscriber_node = Node(
        package='dds_bandwidth_test',
        executable='bandwidth_subscriber',
        name='high_speed_subscriber',
        parameters=[{
            'topic_name': LaunchConfiguration('stream_type'),
            'window_size': 1000,
            'report_interval': 2.0,
        }],
        condition=IfCondition(LaunchConfiguration('enable_subscriber')),
        output='screen'
    )

    # Log info about the high-speed test configuration
    log_info = LogInfo(msg=[
        'Starting high-speed streaming test with: ',
        'Stream type: ', LaunchConfiguration('stream_type'), ', ',
        'Frequency: ', LaunchConfiguration('frequency'), ' Hz, ',
        'Point count: ', LaunchConfiguration('point_count'), ', ',
        'Duration: ', LaunchConfiguration('duration'), ' seconds, ',
        'Multiple streams: ', LaunchConfiguration('enable_multiple_streams')
    ])

    return LaunchDescription([
        stream_type_arg,
        frequency_arg,
        point_count_arg,
        duration_arg,
        enable_multiple_streams_arg,
        enable_subscriber_arg,
        log_info,
        streamer_node,
        subscriber_node,
    ]) 