import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
import launch_ros.actions


def generate_launch_description():
    """
    Launch file for starting a complete standalone flat Gazebo simulation with actor management.
    Launches:
      1. Gazebo Sim (Harmonic) with flat ground world (flat_world.sdf)
      2. ROS <-> Gazebo Clock Bridge (/clock)
      3. Actor Manager node with integrated Web Dashboard (http://localhost:5000)
    """
    pkg_gazebo_actors = get_package_share_directory('gazebo_actors')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    world_path = os.path.join(pkg_gazebo_actors, 'worlds', 'flat_world.sdf')
    models_path = os.path.join(pkg_gazebo_actors, 'models')
    plugin_path = os.path.abspath(os.path.join(pkg_gazebo_actors, '..', '..', 'lib', 'gazebo_actors'))

    # Launch Arguments
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode without GUI'
    )
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='5000',
        description='Port for the embedded Web Dashboard'
    )
    spawn_initial_actor_arg = DeclareLaunchArgument(
        'spawn_initial_actor',
        default_value='false',
        description='Whether to spawn an initial actor automatically on startup (default: false)'
    )
    x_min_arg = DeclareLaunchArgument('x_min', default_value='-15.0', description='Minimum X coordinate')
    x_max_arg = DeclareLaunchArgument('x_max', default_value='15.0', description='Maximum X coordinate')
    y_min_arg = DeclareLaunchArgument('y_min', default_value='-15.0', description='Minimum Y coordinate')
    y_max_arg = DeclareLaunchArgument('y_max', default_value='15.0', description='Maximum Y coordinate')
    environment_name_arg = DeclareLaunchArgument(
        'environment_name',
        default_value='Simulación General (Plano)',
        description='Human-readable label for the simulated environment'
    )

    # Environment Variables for Gazebo
    env_gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{models_path}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}".rstrip(':')
    )
    env_gz_plugin = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=f"{plugin_path}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}".rstrip(':')
    )

    # Gazebo Sim Launch
    gz_args = PythonExpression([
        "('-r -s ' if '", LaunchConfiguration('headless'), "' == 'true' else '-r ') + '",
        world_path, "'"
    ])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': gz_args,
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ROS <-> Gazebo Clock Bridge
    clock_bridge = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    )

    # Actor Manager Node with Web UI
    actor_manager_node = launch_ros.actions.Node(
        package='gazebo_actors',
        executable='actor_manager',
        name='actor_manager',
        output='screen',
        parameters=[
            {'world_name': 'flat_world'},
            {'web_port': LaunchConfiguration('web_port')},
            {'terrain_type': 'flat'},
            {'slope_deg': 0.0},
            {'x_min': LaunchConfiguration('x_min')},
            {'x_max': LaunchConfiguration('x_max')},
            {'y_min': LaunchConfiguration('y_min')},
            {'y_max': LaunchConfiguration('y_max')},
            {'environment_name': LaunchConfiguration('environment_name')},
            {'spawn_initial_actor': LaunchConfiguration('spawn_initial_actor')},
            {'sync_gazebo': True},
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        headless_arg,
        web_port_arg,
        spawn_initial_actor_arg,
        x_min_arg,
        x_max_arg,
        y_min_arg,
        y_max_arg,
        environment_name_arg,
        env_gz_resource,
        env_gz_plugin,
        gz_sim,
        clock_bridge,
        actor_manager_node
    ])

