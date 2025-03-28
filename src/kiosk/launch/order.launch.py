import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 런치 디스크립션 생성
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'table_num',
            default_value='2',  # 기본값은 문자열로 설정
            description='Number of tables (1-9)'),
        ]
    )

    # table_num을 처리하는 함수를 호출
    ld.add_action(OpaqueFunction(function=add_table_nodes))

    return ld


def add_table_nodes(context):
    # table_num 값을 가져와 문자열에서 정수로 변환
    table_num_str = LaunchConfiguration('table_num').perform(context)
    try:
        table_num = int(table_num_str)
        if table_num < 1 or table_num > 9:
            raise ValueError("table_num must be between 1 and 9.")
    except ValueError as e:
        raise RuntimeError(f"Invalid table_num: {table_num_str}. {e}")

    # 노드 생성 및 추가
    nodes = []
    for i in range(1, table_num + 1):
        namespace = f"table_{i}"
        
        order_system_gui = Node(
            package='kiosk',
            executable='order_system_gui',
            namespace=namespace,
            parameters=[{'table_id': str(i)}],
            arguments=[
                str(i),
            ],
            output='screen',
        )

        order_checker = Node(
            package='kiosk',
            executable='order_checker',
            namespace=namespace,
            parameters=[{'table_id': str(i)}],
            arguments=[
                str(i),
            ],
            output='screen',
        )

        nodes.append(order_system_gui)
        nodes.append(order_checker)

    return nodes
