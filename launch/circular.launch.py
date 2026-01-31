from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Turtlesim node1
        Node(
            package='turtlesim',
            namespace='stanley1',
            executable='turtlesim_node',
            name='turtlesim_node',
            output='screen',
            remappings=[
                ('/turtle1/cmd_vel', '/stanley1/turtle1/cmd_vel'),
                ('/turtle1/pose', '/stanley1/turtle1/pose'),             
            ]
        ),
        
        # Circular path publisher node
        Node(
            package='path_publisher',
            namespace='stanley1',
            executable='circular_path_publisher_node',
            name='circular_path_publisher_node',
            output='screen',
            parameters=[{
                'center_x': 5.5,
                'center_y': 5.5,
                'radius': 3.0,
                'num_points': 100,
                'frame_id': 'map',
                'publish_rate': 10.0,
            }],
            remappings=[            
                ('/desired_path', '/stanley1/desired_path'),             
            ]
        ),
        
        # Stanley controller node
        Node(
            package='stanley_controller',
            namespace='stanley1',
            executable='stanley_controller_node',
            name='stanley_controller_node',
            output='screen',
            parameters=[{
                'k_gain': 2.5,
                'k_soft': 0.5,
                'max_linear_vel': 2.0,
                'min_linear_vel': 0.5,
                'max_angular_vel': 2.0,
                'lookahead_distance': 0.5,
                'goal_tolerance': 0.1,
                'wheelbase': 0.1,
            }],
            remappings=[
                ('/turtle1/pose', '/stanley1/turtle1/pose'),
                ('/desired_path', '/stanley1/desired_path'),
                ('/turtle1/cmd_vel', '/stanley1/turtle1/cmd_vel'),
            ]
        ),
        
        # Turtle tracker node
        Node(
            package='turtle_visualizer',
            namespace='stanley1',
            executable='turtle_visualizer_node',
            name='turtle_visualizer_node',
            output='screen',
             remappings=[
                ('/turtle1/pose', '/stanley1/turtle1/pose'),
                ('/desired_path', '/stanley1/desired_path'),             
            ]
        )        
    ])
