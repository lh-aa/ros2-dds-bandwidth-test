#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Launch arguments
    message_type_arg = DeclareLaunchArgument(
        'message_type',
        default_value='string',
        description='Type of message to test (string, image, pose_array)'
    )
    
    message_size_arg = DeclareLaunchArgument(
        'message_size',
        default_value='1024',
        description='Size of message in KB'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='Publishing rate in Hz'
    )
    
    duration_arg = DeclareLaunchArgument(
        'duration',
        default_value='60.0',
        description='Test duration in seconds'
    )
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='bandwidth_test',
        description='Topic name for testing'
    )
    
    enable_publisher_arg = DeclareLaunchArgument(
        'enable_publisher',
        default_value='true',
        description='Enable the publisher node'
    )
    
    enable_subscriber_arg = DeclareLaunchArgument(
        'enable_subscriber',
        default_value='true',
        description='Enable the subscriber node'
    )

    # Publisher node
    publisher_node = Node(
        package='dds_bandwidth_test',
        executable='bandwidth_publisher',
        name='bandwidth_publisher',
        parameters=[{
            'message_type': LaunchConfiguration('message_type'),
            'message_size': LaunchConfiguration('message_size'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'duration': LaunchConfiguration('duration'),
            'topic_name': LaunchConfiguration('topic_name'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_publisher')),
        output='screen'
    )

    # Subscriber node
    subscriber_node = Node(
        package='dds_bandwidth_test',
        executable='bandwidth_subscriber',
        name='bandwidth_subscriber',
        parameters=[{
            'topic_name': LaunchConfiguration('topic_name'),
            'window_size': 1000,
            'report_interval': 5.0,
        }],
        condition=IfCondition(LaunchConfiguration('enable_subscriber')),
        output='screen'
    )

    # Log info about the test configuration
    log_info = LogInfo(msg=[
        'Starting bandwidth test with: ',
        'Message type: ', LaunchConfiguration('message_type'), ', ',
        'Message size: ', LaunchConfiguration('message_size'), ' KB, ',
        'Publish rate: ', LaunchConfiguration('publish_rate'), ' Hz, ',
        'Duration: ', LaunchConfiguration('duration'), ' seconds, ',
        'Topic: ', LaunchConfiguration('topic_name')
    ])

    return LaunchDescription([
        message_type_arg,
        message_size_arg,
        publish_rate_arg,
        duration_arg,
        topic_name_arg,
        enable_publisher_arg,
        enable_subscriber_arg,
        log_info,
        publisher_node,
        subscriber_node,
    ]) 