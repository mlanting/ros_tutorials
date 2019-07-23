from launch import LaunchDescription
import launch.actions
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package= "turtlesim", node_executable= "turtlesim_node",
            name= "sim", node_namespace= "turtlesim1"
        ),
        launch_ros.actions.Node(
            package= "turtlesim", node_executable= "turtlesim_node",
            name= "sim", node_namespace= "turtlesim2"
        ),
        launch_ros.actions.Node(
            package= "turtlesim", node_executable= "mimic",
            name= "mimic",
            remappings= [('input/pose', 'turtlesim1/turtle1/pose'), ('output/cmd_vel', 'turtlesim2/turtle1/cmd_vel')]
        )
    ])
