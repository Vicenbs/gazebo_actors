"""
ManagedActor class representing an actor's kinematic state, waypoints, and state machine.
"""

import math
import time
from typing import Any, Dict, List, Optional, Tuple

from .terrain import compute_terrain_elevation
from .utils import normalize_angle, sanitize_identifier
from .waypoint import Waypoint


class ManagedActor:
    """
    Manages an animated actor's 3D kinematics, state machine, waypoints sequence,
    and interactive teleoperation.
    """

    def __init__(
        self,
        name: str,
        x: float,
        y: float,
        z: Optional[float] = None,
        color: str = "#10b981",
        speed: float = 0.65,
        slope_deg: float = 0.0,
        terrain_type: str = "auto",
        is_default: bool = False,
        display_name: str = "",
        world_name: str = ""
    ):
        self.name = sanitize_identifier(name)
        self.display_name = display_name if display_name else name
        self.col_model_name = f"{self.name}_collision"
        self.color = color
        self.default_speed = float(speed)
        self.slope_deg = float(slope_deg)
        self.terrain_type = str(terrain_type).lower()
        self.world_name = world_name
        self.is_default = is_default
        self.is_spawned_in_gz = False

        # 3D Kinematics State
        self.x = float(x)
        self.y = float(y)
        base_z = compute_terrain_elevation(
            self.x, self.y, self.slope_deg, self.terrain_type, self.world_name
        )
        if z is not None:
            self.z = float(z)
            self.manual_z_offset = self.z - base_z
        else:
            self.z = base_z
            self.manual_z_offset = 0.0

        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0

        # Waypoint Sequencer
        self.waypoints: List[Waypoint] = []
        self.current_wp_idx = 0
        self.mode = "loop"             # 'loop', 'ping_pong', 'once'
        self.state = "NAVIGATING"       # 'IDLE', 'NAVIGATING', 'WAITING', 'PAUSED', 'MANUAL'
        self.wait_start_time = 0.0
        self.ping_pong_dir = 1         # 1: forward, -1: reverse
        self.next_wp_id = 1

        # State & Action tracking
        self.current_action = "stand"
        self.teleop_idle_action = "idle"  # Idle animation: 'idle', 'dance', 'talk'

        # Interactive manual teleoperation (3D with Q/E)
        self.teleop_lin = 0.0
        self.teleop_ang = 0.0
        self.teleop_vert = 0.0         # m/s vertical
        self.last_teleop_time = 0.0

        # Initial default waypoints if is_default
        if is_default:
            if self.has_terrain_slope():
                self.add_waypoint(self.x, self.y, wait_time=4.0, speed=self.default_speed, label="Inicio Hilera", action="talk")
                self.add_waypoint(14.0, self.y, wait_time=5.0, speed=self.default_speed, label="Cima Hilera", action="dance")
            else:
                self.add_waypoint(self.x, self.y, wait_time=3.0, speed=self.default_speed, label="Punto A", action="talk")
                self.add_waypoint(self.x + 5.0, self.y, wait_time=5.0, speed=self.default_speed, label="Punto B", action="dance")

    def has_terrain_slope(self) -> bool:
        """Checks if the active terrain has a noticeable incline."""
        if self.terrain_type in ("flat", "plane", "warehouse") or "warehouse" in self.world_name.lower():
            return False
        return self.slope_deg > 0.1

    def add_waypoint(
        self,
        x: float,
        y: float,
        z: Optional[float] = None,
        wait_time: float = 0.0,
        speed: Optional[float] = None,
        label: str = "",
        action: str = "idle"
    ) -> Waypoint:
        """Adds a waypoint to the sequencer route with terrain elevation fallback and action."""
        if z is None:
            base_z = compute_terrain_elevation(x, y, self.slope_deg, self.terrain_type, self.world_name)
            z = base_z + self.manual_z_offset

        wp_speed = float(speed) if (speed is not None and speed > 0.01) else self.default_speed
        wp = Waypoint(self.next_wp_id, x, y, z=z, wait_time=wait_time, speed=wp_speed, label=label, action=action)
        self.waypoints.append(wp)
        self.next_wp_id += 1
        return wp

    def update_waypoint(
        self,
        wp_id: int,
        wait_time: Any = None,
        speed: Any = None,
        action: Any = None,
        label: Any = None
    ) -> Optional[Waypoint]:
        """Updates an existing waypoint's wait time, speed, action or label."""
        for wp in self.waypoints:
            if wp.id == wp_id:
                wp.update(wait_time=wait_time, speed=speed, action=action, label=label)
                return wp
        return None

    def remove_waypoint(self, wp_id: int) -> bool:
        """Removes a waypoint by ID and adjusts current route index."""
        target_idx = None
        for i, wp in enumerate(self.waypoints):
            if wp.id == wp_id:
                target_idx = i
                break
        if target_idx is not None:
            self.waypoints.pop(target_idx)
            if self.current_wp_idx >= len(self.waypoints):
                self.current_wp_idx = max(0, len(self.waypoints) - 1)
            return True
        return False

    def clear_waypoints(self):
        """Clears all waypoints and resets index."""
        self.waypoints.clear()
        self.current_wp_idx = 0
        self.next_wp_id = 1

    def load_preset(self, preset: str):
        """Loads a predefined patrol pattern relative to current position."""
        self.clear_waypoints()
        cx, cy = self.x, self.y

        if preset in ("linear_patrol", "row_patrol"):
            self.add_waypoint(cx, cy, wait_time=3.0, speed=self.default_speed, label="Punto A", action="talk")
            self.add_waypoint(cx + 6.0, cy, wait_time=5.0, speed=self.default_speed, label="Punto B", action="dance")
            self.mode = "loop"
        elif preset in ("square_patrol", "cross_inspect"):
            self.add_waypoint(cx - 3.0, cy - 3.0, wait_time=2.0, speed=self.default_speed, label="Esquina SO", action="idle")
            self.add_waypoint(cx + 3.0, cy - 3.0, wait_time=4.0, speed=self.default_speed, label="Esquina SE", action="dance")
            self.add_waypoint(cx + 3.0, cy + 3.0, wait_time=3.0, speed=self.default_speed, label="Esquina NE", action="talk")
            self.add_waypoint(cx - 3.0, cy + 3.0, wait_time=2.0, speed=self.default_speed, label="Esquina NO", action="idle")
            self.mode = "loop"
        elif preset == "zigzag":
            self.add_waypoint(cx, cy - 2.0, wait_time=2.0, speed=self.default_speed, label="Z1", action="idle")
            self.add_waypoint(cx + 3.0, cy + 2.0, wait_time=5.0, speed=self.default_speed, label="Z2", action="dance")
            self.add_waypoint(cx + 6.0, cy - 2.0, wait_time=3.0, speed=self.default_speed, label="Z3", action="talk")
            self.add_waypoint(cx + 9.0, cy + 2.0, wait_time=2.0, speed=self.default_speed, label="Z4", action="idle")
            self.mode = "ping_pong"

        self.state = "NAVIGATING"

    def step_sequencer(self, dt: float, current_sim_time: float):
        """Advances the state machine and kinematics by time step dt."""
        if self.state in ("PAUSED", "IDLE"):
            self.vx = 0.0
            self.wz = 0.0
            self.current_action = self.teleop_idle_action if getattr(self, "teleop_idle_action", None) else "idle"
            return

        if self.state == "MANUAL":
            # Safety timeout: stop moving if no teleop command received for 0.8s (monotonic wall-clock)
            now_mono = time.monotonic()
            if self.last_teleop_time > 0.0 and (now_mono - self.last_teleop_time > 0.8):
                self.teleop_lin = 0.0
                self.teleop_ang = 0.0
                self.teleop_vert = 0.0

            # Orientation update
            self.yaw += self.teleop_ang * dt
            self.yaw = normalize_angle(self.yaw)

            # Horizontal translation
            self.x += self.teleop_lin * math.cos(self.yaw) * dt
            self.y += self.teleop_lin * math.sin(self.yaw) * dt

            # Vertical elevation offset via Q/E keys
            if abs(self.teleop_vert) > 0.001:
                self.manual_z_offset += self.teleop_vert * dt

            # Ground elevation at current (X, Y)
            base_z = compute_terrain_elevation(
                self.x, self.y, self.slope_deg, self.terrain_type, self.world_name
            )
            # Total elevation is always terrain base height + manual clearance offset
            self.z = base_z + self.manual_z_offset

            self.vx = self.teleop_lin
            self.wz = self.teleop_ang
            is_moving = (abs(self.vx) > 0.05 or abs(self.wz) > 0.05)
            if is_moving:
                self.current_action = "walk"
            else:
                self.current_action = self.teleop_idle_action if self.teleop_idle_action else "idle"
            return

        base_z = compute_terrain_elevation(
            self.x, self.y, self.slope_deg, self.terrain_type, self.world_name
        )
        self.z = base_z + self.manual_z_offset

        if not self.waypoints or self.state in ("IDLE", "PAUSED"):
            self.vx = 0.0
            self.wz = 0.0
            if not self.waypoints:
                self.state = "IDLE"
                self.current_action = "stand"
            return

        if self.current_wp_idx >= len(self.waypoints):
            self.current_wp_idx = 0

        target_wp = self.waypoints[self.current_wp_idx]

        if self.state == "WAITING":
            self.vx = 0.0
            self.wz = 0.0
            self.current_action = target_wp.action if target_wp.action else "stand"
            if current_sim_time < self.wait_start_time:
                self.wait_start_time = current_sim_time
            elapsed_wait = current_sim_time - self.wait_start_time
            if elapsed_wait >= target_wp.wait_time:
                self._advance_to_next_waypoint()
            return

        if self.state == "NAVIGATING":
            self.current_action = "walk"
            dx = target_wp.x - self.x
            dy = target_wp.y - self.y
            dist_xy = math.hypot(dx, dy)

            # Arrival threshold: 18 cm horizontal
            if dist_xy < 0.18:
                self.x = target_wp.x
                self.y = target_wp.y
                base_z = compute_terrain_elevation(
                    self.x, self.y, self.slope_deg, self.terrain_type, self.world_name
                )
                self.z = base_z + self.manual_z_offset
                if target_wp.wait_time > 0.05:
                    self.state = "WAITING"
                    self.wait_start_time = current_sim_time
                    self.vx = 0.0
                    self.wz = 0.0
                else:
                    self._advance_to_next_waypoint()
                return

            # Target heading and smooth rotation
            target_yaw = math.atan2(dy, dx)
            yaw_diff = normalize_angle(target_yaw - self.yaw)

            # Turn rate limit: 2.2 rad/s
            max_turn = 2.2 * dt
            if abs(yaw_diff) > max_turn:
                turn_step = math.copysign(max_turn, yaw_diff)
            else:
                turn_step = yaw_diff
            self.yaw = normalize_angle(self.yaw + turn_step)
            self.wz = turn_step / dt

            # Translational progress
            wp_speed = target_wp.speed if target_wp.speed > 0.05 else self.default_speed
            move_factor = max(0.2, math.cos(yaw_diff))
            step_xy = min(dist_xy, wp_speed * move_factor * dt)

            self.x += step_xy * math.cos(self.yaw)
            self.y += step_xy * math.sin(self.yaw)

            # Continuous exact terrain elevation tracking with manual ground margin
            base_z = compute_terrain_elevation(
                self.x, self.y, self.slope_deg, self.terrain_type, self.world_name
            )
            self.z = base_z + self.manual_z_offset
            self.vx = wp_speed * move_factor

    def _advance_to_next_waypoint(self):
        """Advances route index according to execution mode."""
        n = len(self.waypoints)
        if n == 0:
            self.state = "IDLE"
            return

        if self.mode == "loop":
            self.current_wp_idx = (self.current_wp_idx + 1) % n
            self.state = "NAVIGATING"
        elif self.mode == "ping_pong":
            if self.current_wp_idx >= n - 1 and self.ping_pong_dir > 0:
                self.ping_pong_dir = -1
            elif self.current_wp_idx <= 0 and self.ping_pong_dir < 0:
                self.ping_pong_dir = 1
            self.current_wp_idx += self.ping_pong_dir
            self.current_wp_idx = max(0, min(n - 1, self.current_wp_idx))
            self.state = "NAVIGATING"
        elif self.mode == "once":
            if self.current_wp_idx < n - 1:
                self.current_wp_idx += 1
                self.state = "NAVIGATING"
            else:
                self.state = "IDLE"
                self.vx = 0.0

    def to_dict(
        self,
        robot_pos: Optional[Tuple[float, float, float]] = None,
        current_sim_time: float = 0.0
    ) -> Dict[str, Any]:
        """Serializes actor state for the REST API and Web Dashboard."""
        dist_to_robot = None
        if robot_pos:
            rx, ry, _ = robot_pos
            dist_to_robot = round(math.hypot(self.x - rx, self.y - ry), 2)

        wait_remaining = 0.0
        if self.state == "WAITING" and self.waypoints and self.current_wp_idx < len(self.waypoints):
            target_wp = self.waypoints[self.current_wp_idx]
            elapsed = current_sim_time - self.wait_start_time
            wait_remaining = max(0.0, target_wp.wait_time - elapsed)

        current_wp_data = None
        if self.waypoints and 0 <= self.current_wp_idx < len(self.waypoints):
            current_wp_data = self.waypoints[self.current_wp_idx].to_dict()

        return {
            "name": self.name,
            "display_name": self.display_name,
            "col_model_name": self.col_model_name,
            "color": self.color,
            "is_default": self.is_default,
            "is_spawned_in_gz": self.is_spawned_in_gz,
            "x": round(self.x, 2),
            "y": round(self.y, 2),
            "z": round(self.z, 3),
            "manual_z_offset": round(self.manual_z_offset, 3),
            "yaw": round(self.yaw, 3),
            "yaw_deg": round(math.degrees(self.yaw), 1),
            "vx": round(self.vx, 2),
            "wz": round(self.wz, 2),
            "state": self.state,
            "mode": self.mode,
            "current_action": self.current_action,
            "teleop_idle_action": self.teleop_idle_action,
            "current_wp_idx": self.current_wp_idx,
            "current_waypoint": current_wp_data,
            "waypoints": [wp.to_dict() for wp in self.waypoints],
            "wait_remaining": round(wait_remaining, 1),
            "distance_to_robot": dist_to_robot,
            "default_speed": round(self.default_speed, 2)
        }

    def to_config_dict(self) -> Dict[str, Any]:
        """Serializes actor configuration (properties and waypoints) for saving scenarios."""
        return {
            "name": self.name,
            "display_name": self.display_name,
            "color": self.color,
            "speed": round(self.default_speed, 2),
            "mode": self.mode,
            "teleop_idle_action": self.teleop_idle_action,
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "z": round(self.z, 3),
            "manual_z_offset": round(self.manual_z_offset, 3),
            "yaw": round(self.yaw, 3),
            "waypoints": [
                {
                    "id": wp.id,
                    "x": round(wp.x, 3),
                    "y": round(wp.y, 3),
                    "z": round(wp.z, 3),
                    "wait_time": round(wp.wait_time, 2),
                    "speed": round(wp.speed, 2),
                    "action": wp.action,
                    "label": wp.label
                }
                for wp in self.waypoints
            ]
        }

    @classmethod
    def from_config_dict(
        cls,
        data: Dict[str, Any],
        slope_deg: float = 0.0,
        terrain_type: str = "auto",
        world_name: str = ""
    ) -> "ManagedActor":
        """Reconstructs ManagedActor from configuration dictionary."""
        name = data.get("name", "actor")
        display_name = data.get("display_name", name)
        color = data.get("color", "#10b981")
        speed = float(data.get("speed", data.get("default_speed", 0.65)))

        pose_data = data.get("pose", {})
        x = float(data.get("x", pose_data.get("x", 4.5)))
        y = float(data.get("y", pose_data.get("y", 0.0)))
        raw_z = data.get("z", pose_data.get("z", None))
        z = float(raw_z) if (raw_z is not None and str(raw_z).strip() != "") else None

        actor = cls(
            name=name,
            x=x,
            y=y,
            z=z,
            color=color,
            speed=speed,
            slope_deg=slope_deg,
            terrain_type=terrain_type,
            is_default=False,
            display_name=display_name,
            world_name=world_name
        )

        if "manual_z_offset" in data:
            actor.manual_z_offset = float(data["manual_z_offset"])
        elif "manual_z_offset" in pose_data:
            actor.manual_z_offset = float(pose_data["manual_z_offset"])

        if "yaw" in data:
            actor.yaw = float(data["yaw"])
        elif "yaw" in pose_data:
            actor.yaw = float(pose_data["yaw"])

        actor.mode = data.get("mode", "loop")
        actor.teleop_idle_action = data.get("teleop_idle_action", "idle")

        actor.clear_waypoints()
        wps = data.get("waypoints", [])
        for wp in wps:
            wp_x = float(wp.get("x", x))
            wp_y = float(wp.get("y", y))
            wp_raw_z = wp.get("z", None)
            wp_z = float(wp_raw_z) if (wp_raw_z is not None and str(wp_raw_z).strip() != "") else None
            wp_wait = float(wp.get("wait_time", 0.0))
            wp_spd = float(wp.get("speed", speed))
            wp_act = str(wp.get("action", "idle"))
            wp_lbl = str(wp.get("label", ""))
            actor.add_waypoint(wp_x, wp_y, z=wp_z, wait_time=wp_wait, speed=wp_spd, label=wp_lbl, action=wp_act)

        return actor


