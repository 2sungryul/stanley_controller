from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    k_gain_arg = DeclareLaunchArgument(
        'k_gain',
        default_value='1.5',
        description='Cross-track error gain'
    )
    
    k_soft_arg = DeclareLaunchArgument(
        'k_soft',
        default_value='0.5',
        description='Softening gain for steering'
    )
    
    max_linear_vel_arg = DeclareLaunchArgument(
        'max_linear_vel',
        default_value='2.0',
        description='Maximum linear velocity (m/s)'
    )
    
    min_linear_vel_arg = DeclareLaunchArgument(
        'min_linear_vel',
        default_value='0.5',
        description='Minimum linear velocity (m/s)'
    )
    
    max_angular_vel_arg = DeclareLaunchArgument(
        'max_angular_vel',
        default_value='2.0',
        description='Maximum angular velocity (rad/s)'
    )
    
    lookahead_distance_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='0.5',
        description='Lookahead distance (m)'
    )
    
    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value='0.1',
        description='Goal reached tolerance (m)'
    )
    
    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase',
        default_value='0.1',
        description='Vehicle wheelbase (m)'
    )
    
    # Node
    stanley_controller_node = Node(
        package='stanley_controller',
        executable='stanley_controller_node',
        name='stanley_controller',
        output='screen',
        parameters=[{
            'k_gain': LaunchConfiguration('k_gain'),
            'k_soft': LaunchConfiguration('k_soft'),
            'max_linear_vel': LaunchConfiguration('max_linear_vel'),
            'min_linear_vel': LaunchConfiguration('min_linear_vel'),
            'max_angular_vel': LaunchConfiguration('max_angular_vel'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'wheelbase': LaunchConfiguration('wheelbase'),
        }]
    )
    
    return LaunchDescription([
        k_gain_arg,
        k_soft_arg,
        max_linear_vel_arg,
        min_linear_vel_arg,
        max_angular_vel_arg,
        lookahead_distance_arg,
        goal_tolerance_arg,
        wheelbase_arg,
        stanley_controller_node
    ])
