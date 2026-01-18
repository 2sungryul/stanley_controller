from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        
        # Circular path publisher node
        Node(
            package='path_publisher',
            executable='circular_path_publisher_node',
            name='circular_path_publisher',
            output='screen',
            parameters=[{
                'center_x': 250.0,
                'center_y': 250.0,
                'radius': 50.0,
                'num_points': 100,
                'frame_id': 'map',
                'publish_rate': 1.0,
            }]
        ),
        
        # Stanley controller node
        Node(
            package='stanley_controller',
            executable='stanley_controller_node',
            name='stanley_controller',
            output='screen',
            parameters=[{
                'k_gain': 1.5,
                'k_soft': 0.5,
                'max_linear_vel': 2.0,
                'min_linear_vel': 0.5,
                'max_angular_vel': 2.0,
                'lookahead_distance': 0.5,
                'goal_tolerance': 0.1,
                'wheelbase': 0.1,
            }]
        ),
        
        # Turtle tracker node
        Node(
            package='turtle_visualizer',
            executable='turtle_visualizer_node',
            name='turtle_visualizer',
            output='screen'
        )
    ])
