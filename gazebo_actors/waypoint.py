"""
Waypoint data class for Gazebo Actors sequencer.
"""

from typing import Dict, Any


class Waypoint:
    """Represents a 3D navigational waypoint with dwell time, speed, and idle action."""

    def __init__(
        self,
        wp_id: int,
        x: float,
        y: float,
        z: float = 0.0,
        wait_time: float = 0.0,
        speed: float = 0.65,
        label: str = "",
        action: str = "idle"
    ):
        self.id = int(wp_id)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.wait_time = max(0.0, float(wait_time))
        self.speed = max(0.05, float(speed))
        self.label = str(label) if label else f"WP {wp_id}"
        self.action = str(action).lower().strip() if action else "idle"

    def update(
        self,
        wait_time: Any = None,
        speed: Any = None,
        action: Any = None,
        label: Any = None
    ):
        """Updates waypoint properties."""
        if wait_time is not None:
            self.wait_time = max(0.0, float(wait_time))
        if speed is not None:
            self.speed = max(0.05, float(speed))
        if action is not None:
            self.action = str(action).lower().strip()
        if label is not None:
            self.label = str(label)

    def to_dict(self) -> Dict[str, Any]:
        """Serializes waypoint to JSON-friendly dictionary."""
        return {
            "id": self.id,
            "x": round(self.x, 2),
            "y": round(self.y, 2),
            "z": round(self.z, 3),
            "wait_time": round(self.wait_time, 1),
            "speed": round(self.speed, 2),
            "label": self.label,
            "action": self.action
        }

