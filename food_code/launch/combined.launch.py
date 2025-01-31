import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 경로 설정
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_navigation2_dir = get_package_share_directory('turtlebot3_navigation2')
    food_code_dir = get_package_share_directory('food_code')

    # 지도 파일 경로
    map_file_path = '/home/happy/Desktop/map_file/hexa_map.yaml'

    # TURTLEBOT3_MODEL 환경 변수 설정
    os.environ['TURTLEBOT3_MODEL'] = 'burger'

    # Gazebo 시뮬레이터 실행
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # Navigation 실행
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_navigation2_dir, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={'map': map_file_path}.items()
    )

    # Food Code 실행 (all_code.launch.py 포함)
    food_code_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(food_code_dir, 'launch', 'all_code.launch.py')
        )
    )

    return LaunchDescription([
        # 환경 변수 설정
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        # Launch 파일 실행
        gazebo_launch,
        navigation_launch,
        food_code_launch,
    ])
