"""
gazebo_actors package.
Generic actor management, physical collision synchronization, skeletal animation coupling,
and interactive waypoint sequencing dashboard for Gazebo Sim and ROS 2.
"""

from .actor import ManagedActor
from .node import ActorManagerNode
from .terrain import compute_terrain_elevation, detect_terrain_profile
from .utils import (
    configure_gazebo_environment,
    free_port,
    normalize_angle,
    resolve_asset_paths,
    sanitize_identifier
)
from .waypoint import Waypoint
from .web_app import create_flask_app

__all__ = [
    "ActorManagerNode",
    "ManagedActor",
    "Waypoint",
    "compute_terrain_elevation",
    "detect_terrain_profile",
    "create_flask_app",
    "free_port",
    "normalize_angle",
    "resolve_asset_paths",
    "sanitize_identifier",
    "configure_gazebo_environment",
]

__version__ = "1.0.0"

