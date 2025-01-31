import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node( 
            package='food_code',
            executable='pyqt_customer_1',
            name='customer_part_1',
            output='screen'),
        Node( 
            package='food_code',
            executable='pyqt_customer_2',
            name='customer_part_2',
            output='screen'),
        Node( 
            package='food_code',
            executable='pyqt_customer_3',
            name='customer_part_3',
            output='screen'),
        Node( 
            package='food_code',
            executable='pyqt_customer_4',
            name='customer_part_4',
            output='screen'),
        Node( 
            package='food_code',
            executable='pyqt_customer_5',
            name='customer_part_5',
            output='screen'),
        Node( 
            package='food_code',
            executable='pyqt_customer_6',
            name='customer_part_6',
            output='screen'),
        Node( 
            package='food_code',
            executable='pyqt_customer_7',
            name='customer_part_7',
            output='screen'),
        Node( 
            package='food_code',
            executable='pyqt_customer_8',
            name='customer_part_8',
            output='screen'),
        Node( 
            package='food_code',
            executable='pyqt_customer_9',
            name='customer_part_9',
            output='screen'),
        Node(
            package='food_code',
            executable='data',
            name='data_part',
            output='screen'),
        Node(
            package='food_code',
            executable='calcul',
            name='calcul_part',
            output='screen'),
        Node(
            package='food_code',
            executable='pyqt_kitch',
            name='kitch_part',
            output='screen'),
        Node(
            package='food_code',
            executable='navigation',
            name='navigation_part',
            output='screen'),
    ])
