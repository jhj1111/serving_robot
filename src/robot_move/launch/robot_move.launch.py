import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 환경 변수 설정
    SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),

    # 기본 map.yaml 경로 설정
    default_map_path = os.path.join(
        get_package_share_directory('robot_move'),
        'map',
        'map.yaml'
    )

    # map 경로를 launch argument로 선언
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to the map YAML file'
    )

    # 환경 변수 선언
    turtlebot3_model_arg = DeclareLaunchArgument(
        'turtlebot_model',
        default_value='burger',
        description='TurtleBot3 model type'
    )

    turtlebot3_model = LaunchConfiguration('turtlebot_model')

    turtlebot3_model_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', turtlebot3_model)


    # map 경로 설정
    map_path = LaunchConfiguration('map')

    # turtlebot3_navigation2의 launch 파일 포함
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_navigation2'),
                'launch',
                'navigation2.launch.py'
            )
        ),
        launch_arguments={'map': map_path}.items(),
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                # get_package_share_directory('turtlebot3_gazebo'),
                get_package_share_directory('robot_move'),
                'launch',
                'turtlebot3_world.launch.py'
            )
        )
    )

    # robot_move 노드 정의
    robot_move_node = Node(
        package='robot_move',
        executable='robot_move',
        output='screen',
    )

    # LaunchDescription에 추가
    ld = LaunchDescription()
    #ld.add_action()
    ld.add_action(turtlebot3_model_arg)
    ld.add_action(turtlebot3_model_env)
    ld.add_action(map_arg)
    ld.add_action(navigation_launch)
    ld.add_action(robot_move_node)
    ld.add_action(gazebo_launch)

    return ld