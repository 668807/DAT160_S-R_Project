from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('multi_robot_challenge_23')
    delay = 1.0  # sekunder mellom hver gruppe

    def launch_aruco(ns, delay_offset):
        return TimerAction(
            period=delay_offset,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg, 'launch', 'aruco_recognition.launch.py')
                    ),
                    launch_arguments={'namespace': ns}.items()
                )
            ]
        )

    def run_node(pkg_name, exe, ns=None, params=None, delay_offset=0.0, remap=None):
        kwargs = {
            'package': pkg_name,
            'executable': exe,
            'output': 'screen',
            'respawn': True
        }
        if ns:
            kwargs['namespace'] = ns
        if params:
            kwargs['parameters'] = [params]
        if remap:
            kwargs['remappings'] = remap
        return TimerAction(period=delay_offset, actions=[Node(**kwargs)])

    return LaunchDescription([
        # --- ArUco recognition for tb3_0..tb3_3 ---
        launch_aruco('tb3_0', 0.0),
        launch_aruco('tb3_1', delay),
       # launch_aruco('tb3_2', delay * 2),
       # launch_aruco('tb3_3', delay * 3),

        # --- Scoring node ---
        run_node('scoring', 'scoring', delay_offset=delay * 4),

        # --- Marker detection ---
        run_node('multi_robot_challenge_23', 'marker_detection', params={'namespace': 'tb3_0'}, delay_offset=delay * 5),
        run_node('multi_robot_challenge_23', 'marker_detection', params={'namespace': 'tb3_1'}, delay_offset=delay * 5.2),
        run_node('multi_robot_challenge_23', 'marker_detection', params={'namespace': 'tb3_2'}, delay_offset=delay * 5.4),
        run_node('multi_robot_challenge_23', 'marker_detection', params={'namespace': 'tb3_3'}, delay_offset=delay * 5.6),

        # --- tb3_0 wall follower + handlers ---
        run_node('multi_robot_challenge_23', 'wall_follower',
                 ns='tb3_0',
                 params={'service_name': 'wall_follower_enable', 'start_enabled': True},
                 delay_offset=delay * 6.0),
        run_node('multi_robot_challenge_23', 'robot_handler',
                 ns='tb3_0',
                 params={'target_robot_ns': 'tb3_2'},
                 delay_offset=delay * 6.2),
        run_node('multi_robot_challenge_23', 'robot_handler',
                 ns='tb3_0',
                 params={'target_robot_ns': 'tb3_3'},
                 delay_offset=delay * 6.4),

        # --- tb3_1 wall follower left + handlers ---
        run_node('multi_robot_challenge_23', 'wall_follower_left',
                 ns='tb3_1',
                 params={'service_name': 'wall_follower_enable', 'start_enabled': True},
                 delay_offset=delay * 6.6),
        run_node('multi_robot_challenge_23', 'robot_handler',
                 ns='tb3_1',
                 params={'target_robot_ns': 'tb3_2'},
                 delay_offset=delay * 6.8),
        run_node('multi_robot_challenge_23', 'robot_handler',
                 ns='tb3_1',
                 params={'target_robot_ns': 'tb3_3'},
                 delay_offset=delay * 7.0),

        # --- tb3_2 bug2: wall_follower + go_to_point ---
        run_node('multi_robot_challenge_23', 'wall_follower',
                 ns='tb3_2',
                 params={'service_name': 'wall_follower_enable', 'start_enabled': False},
                 delay_offset=delay * 7.2),
        
        run_node('multi_robot_challenge_23', 'bug2_controller',
                 ns='tb3_2',
                 params={'wall_follower_service_name': 'wall_follower_enable',
                         'go_to_point_service_name': 'go_to_point_switch'},
                 delay_offset=delay * 7.4),
        
        run_node('multi_robot_challenge_23', 'go_to_point',
                 ns='tb3_2',
                 params={'service_name': 'go_to_point_switch'},
                 delay_offset=delay * 7.6),

        # --- tb3_3 bug2: wall_follower + go_to_point ---
        run_node('multi_robot_challenge_23', 'wall_follower',
                ns='tb3_3',
                params={'service_name': 'wall_follower_enable', 'start_enabled': False},
                delay_offset=delay * 7.8),
        
        run_node('multi_robot_challenge_23', 'bug2_controller',
                 ns='tb3_3',
                 params={'wall_follower_service_name': 'wall_follower_enable',
                         'go_to_point_service_name': 'go_to_point_switch'},
                 delay_offset=delay * 8.0),
        
        run_node('multi_robot_challenge_23', 'go_to_point',
                 ns='tb3_3',
                 params={'service_name': 'go_to_point_switch'},
                 delay_offset=delay * 8.2),
        
    ])
