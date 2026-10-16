import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    """
    Launch file for spawning and tracking a human worker actor in the vineyard simulation.
    """
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='vineyard_variable_slope',
        description='Name of the Gazebo world for set_pose service'
    )
    worker_name_arg = DeclareLaunchArgument(
        'worker_name',
        default_value='human_worker',
        description='Name/frame of the human worker actor'
    )
    speed_arg = DeclareLaunchArgument(
        'walk_speed',
        default_value='0.65',
        description='Human walking speed in m/s along the slope'
    )
    slope_deg_arg = DeclareLaunchArgument(
        'slope_deg',
        default_value='15.0',
        description='Hillside slope angle in degrees for Z trajectory estimation'
    )

    tracker_node = launch_ros.actions.Node(
        package='gazebo_actors',
        executable='human_worker_tracker',
        name='human_worker_tracker',
        output='screen',
        parameters=[
            {'world_name': LaunchConfiguration('world_name')},
            {'worker_name': LaunchConfiguration('worker_name')},
            {'walk_speed_mps': LaunchConfiguration('walk_speed')},
            {'slope_deg': LaunchConfiguration('slope_deg')},
            {'x_start': 4.5},
            {'x_end': 14.0},
            {'y_lane': 0.0},
            {'publish_rate_hz': 20.0},
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        world_name_arg,
        worker_name_arg,
        speed_arg,
        slope_deg_arg,
        tracker_node
    ])
