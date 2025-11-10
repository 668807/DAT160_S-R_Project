# four_robots.launch.py
# ROS2 Foxy
from launch import LaunchDescription
from launch_ros.actions import Node

def make_wall_follower(ns: str):
    return Node(
        package='bug2_navigation',
        executable='wall_follower',
        namespace=ns,
        name='wall_follower',
        parameters=[{
            'service_name': 'wall_follower_enable',   # relativt navn
        }],
        output='screen'
    )

def make_go_to_point(ns: str):
    return Node(
        package='bug2_navigation',
        executable='go_to_point',
        namespace=ns,
        name='go_to_point',
        parameters=[{
            'service_name': 'go_to_point_switch',     # relativt navn
            # valgfritt: init-fart/terskler
            'lin_speed': 0.6,
            'ang_speed': 0.8,
        }],
        
        output='screen'
    )

def make_bug2_controller(ns: str, goal_x: float = 10.0, goal_y: float = 10.0):
    return Node(
        package='multi_robot_challenge_23',
        executable='bug2_controller',
        namespace=ns,
        name='bug2_controller',
        parameters=[{
            'wall_follower_service_name': 'wall_follower_enable',  # relativt navn
            'go_to_point_service_name':  'go_to_point_switch',     # relativt navn
            'goal_x': goal_x,
            'goal_y': goal_y,
        }],
        
        output='screen'
    )

def make_robot_handler(src_ns: str, target_ns: str, trigger_ids=None):
    params = {
        'target_robot_ns': target_ns,   # hvor målet skal sendes
    }
    if trigger_ids is not None:
        params['trigger_marker_ids'] = trigger_ids  # f.eks. [4,7,12]

    return Node(
        package='multi_robot_challenge_23',
        executable='robot_handler',
        namespace=src_ns,
        name='robot_handler',
        parameters=[params],
        output='screen'
    )

def generate_launch_description():
    ld = LaunchDescription()

    # ——— Wall-follower roboter (oppdager ArUco og ruter mål) ———
    # tb3_0 sender mål til tb3_2
    ld.add_action(make_wall_follower('tb3_0'))
    ld.add_action(make_robot_handler('tb3_0', 'tb3_2'))
    ld.add_action(make_robot_handler('tb3_0', 'tb3_3'))
    # tb3_1 sender mål til tb3_3
 #   ld.add_action(make_wall_follower('tb3_1'))
  #  ld.add_action(make_robot_handler('tb3_1', 'tb3_3'))
  #  ld.add_action(make_robot_handler('tb3_1', 'tb3_2'))

    # ——— Bug2 roboter (kjører controller+goto+wall_follower) ———
    # Merk: controlleren forventer at wall_follower og go_to_point finnes i samme NS
    for ns, gx, gy in [('tb3_2', 10.0, 10.0), ('tb3_3', -8.0, 6.0)]:
        ld.add_action(make_wall_follower(ns))            # slik at controller kan enable/disable
        ld.add_action(make_go_to_point(ns))
        ld.add_action(make_bug2_controller(ns, gx, gy))

    return ld
