from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Stanley controller node                 
        Node(
            package='stanley_controller',
            #namespace='stanley2',
            executable='stanley_controller_node',
            name='stanley_controller_node',
            output='screen',
            parameters=[{
                'k_gain': 2.5,
                'k_soft': 0.5,
                'max_linear_vel': 1.0,  # turtle2은 느리게
                'max_angular_vel': 2.5,     
            }],
            remappings=[
                ('/turtle1/pose', '/turtle2/pose'),
                #('/plan1/desired_path', '/stanley1/desired_path'),
                #('/plan2/desired_path', '/stanley2/desired_path'),
                #('/desired_path', '/plan1/desired_path'),
                ('/turtle1/cmd_vel', '/turtle2/cmd_vel'),
            ]
        ),
        
        # Stanley controller node1                 
        Node(
            package='stanley_controller',
            #namespace='stanley2',
            executable='overtaking_stanley_controller_node',
            name='overtaking_stanley_controller_node',
            output='screen',
            parameters=[{
                'k_gain': 2.0,
                'k_soft': 0.5,
                'max_linear_vel': 2.5,  # turtle1은 빠르게
                'max_angular_vel': 2.0,
                'safe_distance' : 1.5,
                'lookahead_distance' : 1.0,
                'overtake_complete_distance' : 2.5
            }],
            remappings=[
                #('/turtle1/pose', '/turtle2/pose'),
                #('/plan1/desired_path', '/stanley1/desired_path'),
                #('/plan2/desired_path', '/stanley2/desired_path'),
                #('/desired_path', '/plan1/desired_path'),
                #('/turtle1/cmd_vel', '/turtle2/cmd_vel'),
            ]
        ),
        
    ])
