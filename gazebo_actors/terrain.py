"""
Terrain profile detection and elevation computation for Gazebo Actors.
Supports flat planes, vineyard hillsides with variable slope, warehouses, and custom worlds.
"""

import math
from typing import Any, Dict


def compute_terrain_elevation(
    x: float,
    y: float,
    slope_deg: float = 0.0,
    terrain_type: str = "flat",
    world_name: str = ""
) -> float:
    """
    Computes exact terrain elevation Z at coordinates (X, Y) for any simulation world.
    - If terrain is flat, warehouse, or slope_deg is 0: returns 0.0.
    - For vineyard worlds: computes the exact mathematical surface of the 3D ground mesh,
      including the polynomial hillside ramp (x^1.8) and agricultural vine row furrows.
    """
    ttype = str(terrain_type).lower()
    wname = str(world_name).lower()

    if ttype in ("flat", "plane", "warehouse") or "warehouse" in wname:
        return 0.0

    eff_slope = slope_deg
    if eff_slope <= 0.01:
        if "steep" in wname:
            eff_slope = 18.0
        elif "variable" in wname:
            eff_slope = 15.0
        elif "gentle" in wname:
            eff_slope = 8.0
        else:
            return 0.0

    # Outside the hillside bounds (x <= 0): ground plane is flat at z = 0.0
    if x <= 0.0:
        return 0.0

    # Hillside longitudinal span: 0 to 30 meters
    norm_x = min(1.0, max(0.0, x / 30.0))
    nominal_max_height = 30.0 * math.tan(math.radians(eff_slope))
    ramp_z = nominal_max_height * (norm_x ** 1.8)

    # Vine row furrows (present on agricultural vineyard terrain meshes)
    if "vineyard" in wname or "slope" in ttype or eff_slope > 1.0:
        furrows = 0.06 * math.cos(2.0 * math.pi * x / 3.5)
        return float(ramp_z + furrows)

    return float(ramp_z)


def detect_terrain_profile(
    world_name: str,
    terrain_type: str = "auto",
    slope_deg: float = 0.0
) -> Dict[str, Any]:
    """
    Detects and returns the workspace boundaries and terrain parameters
    for a given Gazebo simulation world.
    """
    w = world_name.lower()
    profile = {
        "world_name": world_name,
        "terrain_type": "flat",
        "slope_deg": 0.0,
        "x_min": -15.0,
        "x_max": 15.0,
        "y_min": -15.0,
        "y_max": 15.0,
        "environment_name": f"Gazebo ({world_name})" if world_name and world_name not in ("auto", "default") else "Simulación General",
        "row_spacing": 5.0,
        "rows_y": []
    }

    if terrain_type == "auto":
        if "gentle_slope" in w:
            profile.update({
                "terrain_type": "slope",
                "slope_deg": slope_deg if slope_deg > 0.01 else 8.0,
                "x_min": 0.0,
                "x_max": 32.0,
                "y_min": -8.0,
                "y_max": 8.0,
                "environment_name": "Viñedo (8° Suave)",
                "row_spacing": 3.0,
                "rows_y": [-6.0, -3.0, 0.0, 3.0, 6.0]
            })
        elif "steep" in w:
            profile.update({
                "terrain_type": "slope",
                "slope_deg": slope_deg if slope_deg > 0.01 else 18.0,
                "x_min": 0.0,
                "x_max": 32.0,
                "y_min": -8.0,
                "y_max": 8.0,
                "environment_name": "Viñedo (18° Ladera Crítica)",
                "row_spacing": 3.0,
                "rows_y": [-6.0, -3.0, 0.0, 3.0, 6.0]
            })
        elif "variable" in w:
            profile.update({
                "terrain_type": "slope",
                "slope_deg": slope_deg if slope_deg > 0.01 else 15.0,
                "x_min": 0.0,
                "x_max": 32.0,
                "y_min": -8.0,
                "y_max": 8.0,
                "environment_name": "Viñedo en Ladera (15°)",
                "row_spacing": 3.0,
                "rows_y": [-6.0, -3.0, 0.0, 3.0, 6.0]
            })
        elif "warehouse" in w:
            profile.update({
                "terrain_type": "flat",
                "slope_deg": 0.0,
                "x_min": -12.0,
                "x_max": 12.0,
                "y_min": -12.0,
                "y_max": 12.0,
                "environment_name": "Almacén (Warehouse - Plano)",
                "row_spacing": 4.0,
                "rows_y": []
            })
        elif "flat" in w or terrain_type == "flat":
            profile.update({
                "terrain_type": "flat",
                "slope_deg": 0.0,
                "x_min": -15.0,
                "x_max": 15.0,
                "y_min": -15.0,
                "y_max": 15.0,
                "environment_name": "Simulación General (Plano)",
                "row_spacing": 5.0,
                "rows_y": []
            })
    elif terrain_type == "slope":
        profile.update({
            "terrain_type": "slope",
            "slope_deg": slope_deg if slope_deg > 0.01 else 15.0,
            "x_min": 0.0,
            "x_max": 32.0,
            "y_min": -8.0,
            "y_max": 8.0,
            "environment_name": f"Terreno en Pendiente ({slope_deg}°)",
            "row_spacing": 3.0,
            "rows_y": [-6.0, -3.0, 0.0, 3.0, 6.0]
        })

    return profile

