# multi_aruco_recognition.launch.py
# Starter 6 marker_recognition-noder; én pr. robot-namespace tb3_0–tb3_5.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    tb3_0 = Node(
        package='multi_robot_challenge_23',
        executable='marker_recognition',
        name='aruco_recognition',
        namespace='tb3_0',
        output='screen',
        parameters=[
            {'namespace': 'tb3_0'},
            {'global_frame': 'tb3_0/map'},
        ],
    )

    tb3_1 = Node(
        package='multi_robot_challenge_23',
        executable='marker_recognition',
        name='aruco_recognition',
        namespace='tb3_1',
        output='screen',
        parameters=[
            {'namespace': 'tb3_1'},
            {'global_frame': 'tb3_1/map'},
        ],
    )

    tb3_2 = Node(
        package='multi_robot_challenge_23',
        executable='marker_recognition',
        name='aruco_recognition',
        namespace='tb3_2',
        output='screen',
        parameters=[
            {'namespace': 'tb3_2'},
            {'global_frame': 'tb3_2/map'},
        ],
    )

    tb3_3 = Node(
        package='multi_robot_challenge_23',
        executable='marker_recognition',
        name='aruco_recognition',
        namespace='tb3_3',
        output='screen',
        parameters=[
            {'namespace': 'tb3_3'},
            {'global_frame': 'tb3_3/map'},
        ],
    )

    return LaunchDescription([
        tb3_0, tb3_1, tb3_2, tb3_3
    ])
