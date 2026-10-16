"""
Utility functions and path resolution for Gazebo Actors.
"""

import math
import os
import re
import socket
import subprocess
import time
import unicodedata
from typing import Tuple

from ament_index_python.packages import get_package_prefix, get_package_share_directory


def sanitize_identifier(raw_name: str) -> str:
    """
    Sanitizes an input string into a valid ROS 2 and Gazebo identifier:
    - Normalizes accented/Unicode characters to ASCII (e.g. 'maría' -> 'maria').
    - Converts to lowercase.
    - Replaces whitespace and invalid punctuation with underscores.
    - Strips leading digits or underscores to adhere to ROS naming rules.
    """
    if not raw_name:
        return "actor"

    normalized = unicodedata.normalize('NFKD', raw_name)
    ascii_str = normalized.encode('ASCII', 'ignore').decode('utf-8')
    cleaned = re.sub(r'[^a-zA-Z0-9_]', '_', ascii_str.lower())
    cleaned = re.sub(r'_+', '_', cleaned).strip('_')

    if not cleaned or not cleaned[0].isalpha():
        cleaned = f"actor_{cleaned}"

    return cleaned


def normalize_angle(angle: float) -> float:
    """Normalizes an angle in radians to the interval [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def free_port(port: int):
    """
    Frees the specified TCP port by terminating any lingering zombie processes,
    preventing 'Address already in use' errors.
    """
    try:
        subprocess.run(
            ["fuser", "-k", f"{port}/tcp"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        time.sleep(0.3)
    except Exception:
        pass

    # Verify if port is freed
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.2)
            if s.connect_ex(("127.0.0.1", port)) == 0:
                # Still occupied, try with lsof if available
                subprocess.run(
                    f"lsof -ti:{port} | xargs -r kill -9",
                    shell=True,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                time.sleep(0.2)
    except Exception:
        pass


def get_or_create_colored_skin(base_mesh_path: str, color_hex: str) -> str:
    """
    Generates a COLLADA (.dae) mesh with the shirt/sweater phong material diffuse
    and ambient colors replaced with the specified hex color.
    Uses cached meshes in ~/.gazebo_actors/meshes/ to avoid regenerating duplicate files.
    """
    if not os.path.exists(base_mesh_path):
        return base_mesh_path

    try:
        clean_hex = color_hex.lstrip('#').lower()
        if len(clean_hex) != 6:
            clean_hex = "10b981"

        r = int(clean_hex[0:2], 16) / 255.0
        g = int(clean_hex[2:4], 16) / 255.0
        b = int(clean_hex[4:6], 16) / 255.0

        cache_dir = os.path.expanduser("~/.gazebo_actors/meshes")
        os.makedirs(cache_dir, exist_ok=True)
        dst_path = os.path.join(cache_dir, f"human_worker_skin_{clean_hex}.dae")

        if os.path.exists(dst_path) and os.path.getsize(dst_path) > 1000:
            return dst_path

        with open(base_mesh_path, "r", encoding="utf-8") as f:
            content = f.read()

        # Regex matching sweater material in COLLADA phong profile
        pat = r'(<effect id="sweater-[^"]+-effect">.*?<color sid="ambient">)[^<]+(</color>.*?<color sid="diffuse">)[^<]+(</color>)'
        replacement = rf'\g<1>{r:.4f} {g:.4f} {b:.4f} 1\g<2>{r:.4f} {g:.4f} {b:.4f} 1\g<3>'
        new_content = re.sub(pat, replacement, content, flags=re.DOTALL)

        with open(dst_path, "w", encoding="utf-8") as f:
            f.write(new_content)

        return dst_path
    except Exception:
        return base_mesh_path


def resolve_actor_assets(color: str = "#10b981") -> Tuple[str, str, str, str, str, str]:
    """
    Resolves absolute file paths for actor meshes, including a dynamically colored
    skin mesh matching the actor's shirt color, and the follower C++ plugin.
    Returns:
        (skin_mesh, walk_mesh, stand_mesh, dance_mesh, talk_mesh, plugin_so)
    """
    try:
        pkg_share = get_package_share_directory("gazebo_actors")
        pkg_prefix = get_package_prefix("gazebo_actors")
        mesh_walk = os.path.join(pkg_share, "models", "human_worker", "meshes", "walk.dae")
        mesh_stand = os.path.join(pkg_share, "models", "human_worker", "meshes", "stand.dae")
        mesh_dance = os.path.join(pkg_share, "models", "human_worker", "meshes", "dance.dae")
        mesh_talk = os.path.join(pkg_share, "models", "human_worker", "meshes", "talk.dae")
        plugin_so = os.path.join(pkg_prefix, "lib", "gazebo_actors", "libGazeboActorFollower.so")
    except Exception:
        mesh_walk = "model://human_worker/meshes/walk.dae"
        mesh_stand = "model://human_worker/meshes/stand.dae"
        mesh_dance = "model://human_worker/meshes/dance.dae"
        mesh_talk = "model://human_worker/meshes/talk.dae"
        plugin_so = "libGazeboActorFollower.so"

    if not os.path.exists(mesh_dance):
        mesh_dance = mesh_stand
    if not os.path.exists(mesh_talk):
        mesh_talk = mesh_stand

    skin_mesh = get_or_create_colored_skin(mesh_walk, color)
    return skin_mesh, mesh_walk, mesh_stand, mesh_dance, mesh_talk, plugin_so


def resolve_asset_paths() -> Tuple[str, str, str]:
    """
    Backwards compatibility: resolves (walk_mesh, stand_mesh, plugin_so).
    """
    _, mesh_walk, mesh_stand, _, _, plugin_so = resolve_actor_assets("#10b981")
    return mesh_walk, mesh_stand, plugin_so


def configure_gazebo_environment():
    """
    Configures GZ_SIM_RESOURCE_PATH and GZ_SIM_SYSTEM_PLUGIN_PATH dynamically
    for the current process.
    """
    try:
        pkg_share = get_package_share_directory("gazebo_actors")
        models_dir = os.path.join(pkg_share, "models")
        if os.path.isdir(models_dir):
            curr_res = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
            if models_dir not in curr_res:
                os.environ["GZ_SIM_RESOURCE_PATH"] = f"{models_dir}:{curr_res}" if curr_res else models_dir

        pkg_prefix = get_package_prefix("gazebo_actors")
        plugin_dir = os.path.join(pkg_prefix, "lib", "gazebo_actors")
        if os.path.isdir(plugin_dir):
            curr_plug = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
            if plugin_dir not in curr_plug:
                os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_dir}:{curr_plug}" if curr_plug else plugin_dir
    except Exception:
        pass

