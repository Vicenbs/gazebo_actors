"""
Gazebo Sim SDF generation, world discovery, and service utilities.
"""

import re
import subprocess
import time
from typing import Tuple


def hex_to_rgb(hex_str: str) -> Tuple[float, float, float]:
    """Converts a hex color code to normalized RGB floats in [0.0, 1.0]."""
    cleaned = hex_str.lstrip('#')
    if len(cleaned) == 6:
        try:
            r = int(cleaned[0:2], 16) / 255.0
            g = int(cleaned[2:4], 16) / 255.0
            b = int(cleaned[4:6], 16) / 255.0
            return r, g, b
        except ValueError:
            pass
    return 0.2, 0.7, 0.4


def generate_collider_sdf(
    name: str,
    x: float,
    y: float,
    z: float,
    color: str = "#10b981"
) -> str:
    """
    Generates SDF 1.9 for a kinematic capsule collider representing an actor.
    Includes physics contact/friction properties and a colored visual safety envelope.
    """
    col_name = f"{name}_collision"
    r, g, b = hex_to_rgb(color)

    return f"""<sdf version='1.9'>
  <model name='{col_name}'>
    <pose>{x:.2f} {y:.2f} {z:.3f} 0 0 0.0</pose>
    <link name='collision_link'>
      <gravity>false</gravity>
      <kinematic>true</kinematic>
      <collision name='human_body_collision'>
        <pose>0 0 0.88 0 0 0</pose>
        <geometry>
          <capsule><radius>0.24</radius><length>1.20</length></capsule>
        </geometry>
        <surface>
          <contact>
            <collide_without_contact>true</collide_without_contact>
          </contact>
        </surface>
      </collision>
      <visual name='safety_envelope_visual'>
        <pose>0 0 0.88 0 0 0</pose>
        <geometry>
          <capsule><radius>0.25</radius><length>1.20</length></capsule>
        </geometry>
        <material>
          <ambient>{r:.2f} {g:.2f} {b:.2f} 0.25</ambient>
          <diffuse>{r:.2f} {g:.2f} {b:.2f} 0.25</diffuse>
          <specular>0.1 0.1 0.1 0.25</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def generate_actor_sdf(
    name: str,
    collider_name: str,
    skin_mesh: str,
    walk_mesh: str,
    stand_mesh: str,
    dance_mesh: str,
    talk_mesh: str,
    plugin_file: str
) -> str:
    """
    Generates SDF 1.9 for the animated visual actor coupled to the collider
    via GazeboActorFollower C++ system plugin.
    Includes walk, stand, dance, and talk animations.
    """
    return f"""<sdf version='1.9'>
  <actor name='{name}'>
    <pose>0 0 0 0 0 0</pose>
    <skin>
      <filename>{skin_mesh}</filename>
      <scale>1.0</scale>
    </skin>
    <animation name='walk'>
      <filename>{walk_mesh}</filename>
      <scale>1.0</scale>
    </animation>
    <animation name='stand'>
      <filename>{stand_mesh}</filename>
      <scale>1.0</scale>
    </animation>
    <animation name='dance'>
      <filename>{dance_mesh}</filename>
      <scale>1.0</scale>
    </animation>
    <animation name='talk'>
      <filename>{talk_mesh}</filename>
      <scale>1.0</scale>
    </animation>
    <plugin filename='{plugin_file}' name='gazebo_actors::ActorFollower'>
      <actor_name>{name}</actor_name>
      <target>{collider_name}</target>
      <animation_walk>walk</animation_walk>
      <animation_stand>stand</animation_stand>
      <z_offset>0.86</z_offset>
      <animation_x_vel>2.0</animation_x_vel>
    </plugin>
  </actor>
</sdf>"""


def detect_gazebo_world(
    max_retries: int = 3,
    timeout: float = 4.0,
    current_world: str = ""
) -> str:
    """
    Auto-detects the active Gazebo world name by querying gz service -l.
    """
    for attempt in range(max_retries):
        try:
            res = subprocess.run(
                ['gz', 'service', '-l'],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                timeout=timeout,
                text=True
            )
            matches = re.findall(r'/world/([^/]+)/(?:create|set_pose)', res.stdout)
            if matches:
                unique = []
                for m in matches:
                    if m not in unique and m not in ('auto', 'default'):
                        unique.append(m)
                if unique:
                    if current_world and current_world in unique:
                        return current_world
                    return unique[0]
        except Exception:
            pass
        if attempt < max_retries - 1:
            time.sleep(0.5)
    return ""

