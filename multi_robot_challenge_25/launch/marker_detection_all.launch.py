from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch.substitutions import TextSubstitution

def generate_launch_description():
    robot_names = ['tb3_0', 'tb3_1']
    def launch_detection_node(robot_name):
        return GroupAction(
            actions=[
                Node(
                    package='multi_robot_challenge_23',
                    executable='marker_detection',
                    name='marker_detection',
                    namespace=robot_name,
                    parameters=[
                        {'namespace': robot_name}
                    ]
                ),
            ]
        )

    robot_launchers = [
        launch_detection_node(name) for name in robot_names
    ]

    return LaunchDescription(robot_launchers)
