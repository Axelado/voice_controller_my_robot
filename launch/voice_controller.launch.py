from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'voice_controller_my_robot'
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    keyboard_activator = Node(
        package=package_name,
        executable='keyboard_activator',
        name='keyboard_activator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    voice_recorder = Node(
        package=package_name,
        executable='voice_recorder',
        name='voice_recorder',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    pose_publish_from_room_number = Node(
        package=package_name,
        executable='pose_publish_from_room_number',
        name='pose_publish_from_room_number',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Return the launch description
    return LaunchDescription([
        use_sim_time_arg,
        keyboard_activator,
        voice_recorder,
        pose_publish_from_room_number,
    ])