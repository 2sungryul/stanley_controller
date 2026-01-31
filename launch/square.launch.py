from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # Turtlesim node2
        Node(
            package='turtlesim',
            namespace='stanley2',
            executable='turtlesim_node',
            name='turtlesim_node',
            output='screen',
            remappings=[
                ('/turtle1/cmd_vel', '/stanley2/turtle1/cmd_vel'),
                ('/turtle1/pose', '/stanley2/turtle1/pose'),             
            ]
        ),
        
        # square path publisher node2
        Node(
            package='path_publisher',
            namespace='stanley2',
            #executable='square_path_publisher_node',
            #name='square_path_publisher_node',
            executable='rounded_square_path_publisher_node',
            name='rounded_square_path_publisher_node',
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
                ('/desired_path', '/stanley2/desired_path'),             
            ]
        ),
        
        # Stanley controller node2
        Node(
            package='stanley_controller',
            namespace='stanley2',
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
                ('/turtle1/pose', '/stanley2/turtle1/pose'),
                ('/desired_path', '/stanley2/desired_path'),
                ('/turtle1/cmd_vel', '/stanley2/turtle1/cmd_vel'),
            ]
        ),
        
        # Turtle tracker node2
        Node(
            package='turtle_visualizer',
            namespace='stanley2',
            executable='turtle_visualizer_node',
            name='turtle_visualizer_node',
            output='screen',
             remappings=[
                ('/turtle1/pose', '/stanley2/turtle1/pose'),
                ('/desired_path', '/stanley2/desired_path'),             
            ]
        )
    ])
