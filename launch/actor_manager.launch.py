import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    """
    Launch file for starting the Generic Actor Manager & Waypoint Sequencer with embedded Web UI.
    Provides web interface at http://localhost:5000 for managing human/animated actors in any Gazebo simulation.
    """
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='auto',
        description='Name of the Gazebo world ("auto" auto-detects active simulation world)'
    )
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='5000',
        description='Port for the embedded Web Dashboard and REST API'
    )
    terrain_type_arg = DeclareLaunchArgument(
        'terrain_type',
        default_value='auto',
        description='Type of terrain: "auto" (auto-adapts to world), "flat", or "slope"'
    )
    slope_deg_arg = DeclareLaunchArgument(
        'slope_deg',
        default_value='0.0',
        description='Hillside slope angle in degrees (auto-detected if terrain_type is "auto")'
    )
    x_min_arg = DeclareLaunchArgument(
        'x_min',
        default_value='-15.0',
        description='Minimum X coordinate for 2D canvas workspace bounds'
    )
    x_max_arg = DeclareLaunchArgument(
        'x_max',
        default_value='30.0',
        description='Maximum X coordinate for 2D canvas workspace bounds'
    )
    y_min_arg = DeclareLaunchArgument(
        'y_min',
        default_value='-15.0',
        description='Minimum Y coordinate for 2D canvas workspace bounds'
    )
    y_max_arg = DeclareLaunchArgument(
        'y_max',
        default_value='15.0',
        description='Maximum Y coordinate for 2D canvas workspace bounds'
    )
    environment_name_arg = DeclareLaunchArgument(
        'environment_name',
        default_value='Gazebo Simulation',
        description='Human-readable label for the simulated environment'
    )
    robot_odom_topic_arg = DeclareLaunchArgument(
        'robot_odom_topic',
        default_value='odom',
        description='Odometry topic of the main robot in simulation'
    )
    default_actor_name_arg = DeclareLaunchArgument(
        'default_actor_name',
        default_value='human_worker',
        description='Name of the initial spawned actor'
    )
    default_actor_x_arg = DeclareLaunchArgument(
        'default_actor_x',
        default_value='4.5',
        description='Initial X position of default actor'
    )
    default_actor_y_arg = DeclareLaunchArgument(
        'default_actor_y',
        default_value='0.0',
        description='Initial Y position of default actor'
    )
    spawn_initial_actor_arg = DeclareLaunchArgument(
        'spawn_initial_actor',
        default_value='true',
        description='Whether to spawn an initial actor automatically on startup (default: true)'
    )
    publish_rate_hz_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='20.0',
        description='Gazebo pose synchronization and TF broadcast frequency in Hz'
    )
    sync_gazebo_arg = DeclareLaunchArgument(
        'sync_gazebo',
        default_value='true',
        description='Whether to synchronize poses directly with Gazebo Sim'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock from /clock'
    )
    scenario_file_arg = DeclareLaunchArgument(
        'scenario_file',
        default_value='',
        description='Ruta o nombre de archivo de configuración de escenario (JSON o YAML) para cargar actores y secuencias'
    )

    actor_manager_node = launch_ros.actions.Node(
        package='gazebo_actors',
        executable='actor_manager',
        name='actor_manager',
        output='screen',
        parameters=[
            {'world_name': LaunchConfiguration('world_name')},
            {'web_port': LaunchConfiguration('web_port')},
            {'terrain_type': LaunchConfiguration('terrain_type')},
            {'slope_deg': LaunchConfiguration('slope_deg')},
            {'x_min': LaunchConfiguration('x_min')},
            {'x_max': LaunchConfiguration('x_max')},
            {'y_min': LaunchConfiguration('y_min')},
            {'y_max': LaunchConfiguration('y_max')},
            {'environment_name': LaunchConfiguration('environment_name')},
            {'robot_odom_topic': LaunchConfiguration('robot_odom_topic')},
            {'spawn_initial_actor': LaunchConfiguration('spawn_initial_actor')},
            {'default_actor_name': LaunchConfiguration('default_actor_name')},
            {'default_actor_x': LaunchConfiguration('default_actor_x')},
            {'default_actor_y': LaunchConfiguration('default_actor_y')},
            {'publish_rate_hz': LaunchConfiguration('publish_rate_hz')},
            {'sync_gazebo': LaunchConfiguration('sync_gazebo')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'scenario_file': LaunchConfiguration('scenario_file')}
        ]
    )

    return LaunchDescription([
        world_name_arg,
        web_port_arg,
        terrain_type_arg,
        slope_deg_arg,
        x_min_arg,
        x_max_arg,
        y_min_arg,
        y_max_arg,
        environment_name_arg,
        robot_odom_topic_arg,
        spawn_initial_actor_arg,
        default_actor_name_arg,
        default_actor_x_arg,
        default_actor_y_arg,
        publish_rate_hz_arg,
        sync_gazebo_arg,
        use_sim_time_arg,
        scenario_file_arg,
        actor_manager_node
    ])

