from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    """
    Turtlesim에 2개의 거북이를 생성하는 Launch 파일
    - turtle1: (8.0, 8.0)
    - turtle2: (2.0, 5.8)
    """
    return LaunchDescription([
        # 1. Turtlesim 노드 실행
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        
        # 2. 기본 turtle1 제거 (1초 후)
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/kill',
                        'turtlesim/srv/Kill',
                        "{name: 'turtle1'}"
                    ],
                    output='screen',
                    shell=False
                )
            ]
        ),
        
        # 3. turtle1 생성 - (8.0, 8.0) (2초 후)
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/spawn',
                        'turtlesim/srv/Spawn',
                        "{x: 8.5, y: 5.5, theta: 1.5, name: 'turtle1'}"
                    ],
                    output='screen',
                    shell=False
                )
            ]
        ),
        
        # 4. turtle2 생성 - (2.0, 5.8) (3초 후)
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/spawn',
                        'turtlesim/srv/Spawn',
                        "{x: 2.5, y: 5.5, theta: 4.64, name: 'turtle2'}"
                    ],
                    output='screen',
                    shell=False
                )
            ]
        ),

         # Circular path publisher node
        Node(
            package='path_publisher',
            namespace='plan1',
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
                ('/desired_path', '/plan1/desired_path'),             
            ]
        ),
        
        Node(
            package='path_publisher',
            namespace='plan2',
            executable='circular_path_publisher_node',
            name='circular_path_publisher_node',
            output='screen',
            parameters=[{
                'center_x': 5.5,
                'center_y': 5.5,
                'radius': 4.0,
                'num_points': 100,
                'frame_id': 'map',
                'publish_rate': 10.0,
            }],
            remappings=[            
                ('/desired_path', '/plan2/desired_path'),             
            ]
        ),

         # Stanley controller node : turtle2제어                 
        Node(
            package='stanley_controller',
            #namespace='stanley2',
            executable='stanley_controller_node',
            name='stanley_controller_node',
            output='screen',
            parameters=[{
                'k_gain': 1.5,
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
        
        # overtaking_stanley_controller_node : turtle1 제어              
        Node(
            package='stanley_controller',
            #namespace='stanley2',
            executable='overtaking_stanley_controller_node',
            name='overtaking_stanley_controller_node',
            output='screen',
            parameters=[{
                'k_gain': 3.0,
                'k_soft': 0.1,
                'max_linear_vel': 2.0,  # turtle1은 빠르게
                'max_angular_vel': 5.0,
                'lookahead_distance' : 0.05,
                'safe_distance' : 1.5,
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