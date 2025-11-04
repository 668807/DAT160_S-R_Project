import launch
import launch_ros.actions

def generate_launch_description():
    # 1. Robot Handler Node for tb3_0
    robot_handler_0 = launch_ros.actions.Node(
        package='multi_robot_challenge_23',
        executable='robot_handler',
        namespace='tb3_0',
        name='robot_handler_node'
    )
    
    # 2. Robot Handler Node for tb3_1
    robot_handler_1 = launch_ros.actions.Node(
        package='multi_robot_challenge_23',
        executable='robot_handler',
        namespace='tb3_1',
        name='robot_handler_node'
    )
    
    leader_node = launch_ros.actions.Node(
        package='multi_robot_challenge_23',
        executable='leader',
        name='robot_leader_node'
    )
    
    return launch.LaunchDescription([
        robot_handler_0,
        robot_handler_1,
        leader_node,
    ])