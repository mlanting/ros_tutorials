from launch import LaunchDescription
import launch.actions
# import launch.sustitutions
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='turtlesim', node_executable='turtlesim_node',
            node_name= 'sim', node_namespace= 'turtlesim1',
        ),
        launch_ros.actions.Node(
            package='turtlesim', node_executable='turtlesim_node',
            node_name= 'sim', node_namespace= 'turtlesim2',
        )
    ])
