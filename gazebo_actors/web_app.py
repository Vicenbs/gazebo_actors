"""
Embedded Flask Web Application and REST API for Gazebo Actors.
"""

import json
import logging
import os
import time
from urllib.parse import unquote
from flask import Flask, jsonify, render_template, request, Response
from ament_index_python.packages import get_package_share_directory

from .actor import ManagedActor
from .node import ActorManagerNode
from .terrain import compute_terrain_elevation
from .utils import sanitize_identifier


def create_flask_app(node: ActorManagerNode) -> Flask:
    """Creates and configures the Flask application for the Actor Web Dashboard."""
    # Suppress werkzeug request logs
    logging.getLogger('werkzeug').setLevel(logging.ERROR)

    # Locate templates and static directories
    pkg_share = None
    try:
        pkg_share = get_package_share_directory("gazebo_actors")
    except Exception:
        pass

    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates_template = []
    candidates_static = []

    if pkg_share:
        candidates_template.append(os.path.join(pkg_share, "web", "templates"))
        candidates_static.append(os.path.join(pkg_share, "web", "static"))

    candidates_template.append(os.path.abspath(os.path.join(script_dir, "..", "web", "templates")))
    candidates_static.append(os.path.abspath(os.path.join(script_dir, "..", "web", "static")))

    template_dir = next((d for d in candidates_template if os.path.isdir(d)), candidates_template[-1])
    static_dir = next((d for d in candidates_static if os.path.isdir(d)), candidates_static[-1])

    app = Flask(__name__, template_folder=template_dir, static_folder=static_dir)

    @app.route("/")
    def index():
        return render_template(
            "index.html",
            port=node.web_port,
            environment_name=node.environment_name,
            terrain_type=node.terrain_type
        )

    @app.route("/api/status")
    def get_status():
        with node.lock:
            actors_list = [
                actor.to_dict(node.robot_pos, node.get_sim_time())
                for actor in node.actors.values()
            ]
            robot_data = None
            if node.robot_pos:
                robot_data = {
                    "x": round(node.robot_pos[0], 2),
                    "y": round(node.robot_pos[1], 2),
                    "z": round(node.robot_pos[2], 2),
                    "yaw": round(node.robot_yaw, 3)
                }
            return jsonify({
                "actors": actors_list,
                "robot": robot_data,
                "sim_time": round(node.get_sim_time(), 1)
            })

    @app.route("/api/terrain")
    def get_terrain():
        is_slope = (node.terrain_type == "slope" and node.slope_deg > 0.1)
        return jsonify({
            "environment_name": node.environment_name,
            "terrain_type": node.terrain_type,
            "x_min": node.x_min,
            "x_max": node.x_max,
            "y_min": node.y_min,
            "y_max": node.y_max,
            "row_spacing": 3.0 if is_slope else 5.0,
            "rows_y": [-6.0, -3.0, 0.0, 3.0, 6.0] if is_slope else [],
            "slope_deg": node.slope_deg
        })

    @app.route("/api/actors", methods=["POST"])
    def add_actor():
        data = request.json or {}
        raw_name = data.get("name", "").strip()

        with node.lock:
            if not raw_name:
                raw_name = f"actor_{len(node.actors) + 1}"

            base_name = sanitize_identifier(raw_name)
            name = base_name
            counter = 2
            while name in node.actors:
                name = f"{base_name}_{counter}"
                counter += 1

            x = float(data.get("x", 4.5))
            y = float(data.get("y", 0.0))
            raw_z = data.get("z", None)
            z = float(raw_z) if (raw_z is not None and str(raw_z).strip() != "") else None
            color = str(data.get("color", "#3b82f6"))
            speed = float(data.get("speed", 0.65))

            spawn_ok = True
            if node.sync_gazebo:
                spawn_ok = node.spawn_gazebo_actor(name, x, y, z=z, color=color)

            actor = ManagedActor(
                name=name,
                x=x,
                y=y,
                z=z,
                color=color,
                speed=speed,
                slope_deg=node.slope_deg,
                terrain_type=node.terrain_type,
                is_default=False,
                display_name=raw_name,
                world_name=node.world_name
            )
            actor.is_spawned_in_gz = spawn_ok
            node.actors[name] = actor
            node._setup_actor_publishers(name)

            return jsonify({
                "success": True,
                "spawn_gazebo": spawn_ok,
                "actor": actor.to_dict(node.robot_pos, node.get_sim_time())
            })

    @app.route("/api/actors/<name>", methods=["DELETE"])
    def delete_actor(name: str):
        clean_name = unquote(name).strip()
        with node.lock:
            target_key = None
            if clean_name in node.actors:
                target_key = clean_name
            else:
                for k in node.actors.keys():
                    if k.lower() == clean_name.lower() or sanitize_identifier(k) == sanitize_identifier(clean_name):
                        target_key = k
                        break

            if not target_key:
                return jsonify({"success": False, "error": f"Actor '{name}' no encontrado"}), 404

            name = target_key

            if node.sync_gazebo:
                node.delete_gazebo_actor(name)

            if name in node.actor_pubs:
                try:
                    pubs = node.actor_pubs[name]
                    node.destroy_publisher(pubs["pose"])
                    node.destroy_publisher(pubs["odom"])
                except Exception:
                    pass
                del node.actor_pubs[name]

            del node.actors[name]
            return jsonify({"success": True, "deleted": name})

    @app.route("/api/actors/reset_default", methods=["POST"])
    def reset_default_actor():
        """Cleanly respawns the default actor."""
        def_name = node.default_actor_name
        with node.lock:
            if def_name in node.actors:
                if node.sync_gazebo:
                    node.delete_gazebo_actor(def_name)
                if def_name in node.actor_pubs:
                    try:
                        pubs = node.actor_pubs[def_name]
                        node.destroy_publisher(pubs["pose"])
                        node.destroy_publisher(pubs["odom"])
                    except Exception:
                        pass
                    del node.actor_pubs[def_name]
                del node.actors[def_name]

            time.sleep(0.2)
            spawn_ok = True
            if node.sync_gazebo:
                spawn_ok = node.spawn_gazebo_actor(
                    def_name, node.default_actor_x, node.default_actor_y, color="#10b981"
                )

            actor = ManagedActor(
                name=def_name,
                x=node.default_actor_x,
                y=node.default_actor_y,
                color="#10b981",
                speed=0.65,
                slope_deg=node.slope_deg,
                terrain_type=node.terrain_type,
                is_default=True,
                display_name=def_name,
                world_name=node.world_name
            )
            actor.is_spawned_in_gz = spawn_ok
            node.actors[def_name] = actor
            node._setup_actor_publishers(def_name)

            return jsonify({
                "success": True,
                "spawn_gazebo": spawn_ok,
                "actor": actor.to_dict(node.robot_pos, node.get_sim_time())
            })

    @app.route("/api/actors/<name>/waypoints", methods=["POST"])
    def modify_waypoints(name: str):
        data = request.json or {}
        action = data.get("action", "add")

        with node.lock:
            if name not in node.actors:
                return jsonify({"success": False, "error": "Actor no encontrado"}), 404
            actor = node.actors[name]

            if action == "add":
                x = float(data.get("x", actor.x))
                y = float(data.get("y", actor.y))
                raw_z = data.get("z", None)
                z = float(raw_z) if (raw_z is not None and str(raw_z).strip() != "") else None
                wait_time = float(data.get("wait_time", 0.0))
                speed = float(data.get("speed", actor.default_speed))
                label = str(data.get("label", ""))
                wp_action = str(data.get("wp_action") or data.get("action_type") or data.get("action_name") or "idle")
                wp = actor.add_waypoint(x, y, z=z, wait_time=wait_time, speed=speed, label=label, action=wp_action)
                return jsonify({"success": True, "waypoint": wp.to_dict()})

            elif action == "update":
                wp_id = int(data.get("id", 0))
                wait_time = data.get("wait_time")
                speed = data.get("speed")
                wp_action = data.get("wp_action") or data.get("action_type") or data.get("action_name")
                label = data.get("label")
                wp = actor.update_waypoint(wp_id, wait_time=wait_time, speed=speed, action=wp_action, label=label)
                if wp:
                    return jsonify({"success": True, "waypoint": wp.to_dict()})
                return jsonify({"success": False, "error": f"Waypoint {wp_id} no encontrado"}), 404

            elif action == "clear":
                actor.clear_waypoints()
                return jsonify({"success": True})

            elif action == "set_all":
                wps_list = data.get("waypoints", [])
                actor.clear_waypoints()
                for item in wps_list:
                    item_z = item.get("z", None)
                    parsed_z = float(item_z) if (item_z is not None and str(item_z).strip() != "") else None
                    actor.add_waypoint(
                        float(item.get("x", 0.0)),
                        float(item.get("y", 0.0)),
                        z=parsed_z,
                        wait_time=float(item.get("wait_time", 0.0)),
                        speed=float(item.get("speed", actor.default_speed)),
                        label=str(item.get("label", "")),
                        action=str(item.get("action", "idle"))
                    )
                return jsonify({"success": True, "count": len(actor.waypoints)})

            return jsonify({"success": False, "error": "Acción no reconocida"}), 400

    @app.route("/api/actors/<name>/waypoints/<int:wp_id>", methods=["DELETE"])
    def delete_single_waypoint(name: str, wp_id: int):
        with node.lock:
            if name not in node.actors:
                return jsonify({"success": False, "error": "Actor no encontrado"}), 404
            ok = node.actors[name].remove_waypoint(wp_id)
            return jsonify({"success": ok})

    @app.route("/api/actors/<name>/sequence", methods=["POST"])
    def control_sequence(name: str):
        data = request.json or {}
        cmd = data.get("command", "")

        with node.lock:
            if name not in node.actors:
                return jsonify({"success": False, "error": "Actor no encontrado"}), 404
            actor = node.actors[name]

            if cmd == "start":
                actor.state = "NAVIGATING" if actor.waypoints else "IDLE"
            elif cmd == "pause":
                actor.state = "PAUSED"
                actor.vx = 0.0
                actor.wz = 0.0
            elif cmd == "stop":
                actor.state = "IDLE"
                actor.current_wp_idx = 0
                actor.vx = 0.0
                actor.wz = 0.0
            elif cmd == "step":
                actor._advance_to_next_waypoint()
            elif cmd == "set_mode":
                new_mode = data.get("mode", "loop")
                if new_mode in ("loop", "ping_pong", "once"):
                    actor.mode = new_mode
            elif cmd == "load_preset":
                preset = data.get("preset", "linear_patrol")
                actor.load_preset(preset)

            return jsonify({
                "success": True,
                "state": actor.state,
                "mode": actor.mode,
                "current_wp_idx": actor.current_wp_idx
            })

    @app.route("/api/actors/<name>/teleop", methods=["POST"])
    def teleop_actor(name: str):
        data = request.json or {}
        lin = float(data.get("linear", 0.0))
        ang = float(data.get("angular", 0.0))
        vert = float(data.get("vertical", 0.0))
        idle_action = data.get("idle_action")

        with node.lock:
            if name not in node.actors:
                import urllib.parse
                name = urllib.parse.unquote(name)
            if name not in node.actors:
                return jsonify({"success": False, "error": "Actor no encontrado"}), 404
            actor = node.actors[name]
            actor.state = "MANUAL"
            actor.teleop_lin = lin
            actor.teleop_ang = ang
            actor.teleop_vert = vert
            actor.last_teleop_time = time.monotonic()
            if idle_action:
                actor.teleop_idle_action = str(idle_action).lower()

        return jsonify({"success": True})

    @app.route("/api/actors/<name>/teleop_idle_action", methods=["POST"])
    def set_teleop_idle_action(name: str):
        data = request.json or {}
        action = str(data.get("action", "idle")).lower()

        with node.lock:
            if name not in node.actors:
                import urllib.parse
                name = urllib.parse.unquote(name)
            if name not in node.actors:
                return jsonify({"success": False, "error": "Actor no encontrado"}), 404
            actor = node.actors[name]
            actor.teleop_idle_action = action

            ANIM_MAP = {"idle": "stand", "stand": "stand", "dance": "dance", "talk": "talk"}
            desired_anim = ANIM_MAP.get(action, "stand")

            # Always switch to MANUAL mode, halt any movement, and play the gesture immediately
            actor.state = "MANUAL"
            actor.vx = 0.0
            actor.wz = 0.0
            actor.teleop_lin = 0.0
            actor.teleop_ang = 0.0
            actor.last_teleop_time = time.monotonic()
            actor.current_action = action
            node.actor_last_action[name] = desired_anim
            node.actor_last_anim_time[name] = node.get_sim_time()

            # Always notify Gazebo plugin of the selected idle animation
            if node.sync_gazebo:
                node.send_gazebo_animation(name, desired_anim)

            return jsonify({"success": True, "teleop_idle_action": actor.teleop_idle_action})

    @app.route("/api/actors/<name>/adjust_z", methods=["POST"])
    def adjust_actor_z(name: str):
        data = request.json or {}
        step = data.get("step", None)
        target_z = data.get("z", None)
        reset_ground = data.get("reset_ground", False)

        with node.lock:
            if name not in node.actors:
                return jsonify({"success": False, "error": "Actor no encontrado"}), 404
            actor = node.actors[name]
            if step is not None:
                dz = float(step)
                actor.z += dz
                actor.manual_z_offset += dz
            elif target_z is not None:
                actor.z = float(target_z)
                base_z = compute_terrain_elevation(
                    actor.x, actor.y, actor.slope_deg, actor.terrain_type, actor.world_name
                )
                actor.manual_z_offset = actor.z - base_z
            elif reset_ground:
                base_z = compute_terrain_elevation(
                    actor.x, actor.y, actor.slope_deg, actor.terrain_type, actor.world_name
                )
                actor.z = base_z
                actor.manual_z_offset = 0.0

            return jsonify({
                "success": True,
                "z": round(actor.z, 3),
                "manual_z_offset": round(actor.manual_z_offset, 3)
            })

    # =========================================================================
    # GESTIÓN Y PERSISTENCIA DE CONFIGURACIONES (ESCENARIOS)
    # =========================================================================

    @app.route("/api/config/export", methods=["GET"])
    def export_config():
        """Exports the active simulation scenario as JSON."""
        data = node.export_scenario_data()
        download = request.args.get("download", "false").lower() == "true"
        if download:
            json_str = json.dumps(data, indent=2, ensure_ascii=False)
            filename = f"escenario_actores_{int(time.time())}.json"
            return Response(
                json_str,
                mimetype="application/json",
                headers={"Content-Disposition": f"attachment; filename={filename}"}
            )
        return jsonify(data)

    @app.route("/api/config/import", methods=["POST"])
    def import_config():
        """Imports a scenario configuration from JSON payload."""
        data = request.json or {}
        ok, msg = node.import_scenario_data(data)
        if not ok:
            return jsonify({"success": False, "error": msg}), 400
        return jsonify({
            "success": True,
            "message": msg,
            "actors_count": len(node.actors)
        })

    @app.route("/api/config/presets", methods=["GET"])
    def get_presets():
        """Returns list of available preset configuration files."""
        presets = node.list_preset_files()
        return jsonify({"presets": presets})

    @app.route("/api/config/presets/load", methods=["POST"])
    def load_preset():
        """Loads a preset configuration file from server storage."""
        data = request.json or {}
        filename = data.get("filename", data.get("preset", ""))
        if not filename:
            return jsonify({"success": False, "error": "Nombre de preset no especificado"}), 400
        ok, msg = node.load_scenario_file(filename)
        if not ok:
            return jsonify({"success": False, "error": msg}), 400
        return jsonify({
            "success": True,
            "message": msg,
            "actors_count": len(node.actors)
        })

    @app.route("/api/config/presets/save", methods=["POST"])
    def save_preset():
        """Saves current scenario configuration as a named preset file."""
        data = request.json or {}
        filename = str(data.get("filename", "")).strip()
        if not filename:
            filename = f"escenario_{int(time.time())}.json"
        if not filename.endswith((".json", ".yaml", ".yml")):
            filename += ".json"

        safe_fname = os.path.basename(filename)
        save_dir = node.get_presets_directory()
        filepath = os.path.join(save_dir, safe_fname)

        scenario_data = node.export_scenario_data()
        try:
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(scenario_data, f, indent=2, ensure_ascii=False)
            node.get_logger().info(f"💾 Preset guardado en: {filepath}")

            # Also mirror to package share config dir if distinct
            try:
                pkg_share = get_package_share_directory("gazebo_actors")
                share_config = os.path.join(pkg_share, "config")
                if os.path.abspath(share_config) != os.path.abspath(save_dir):
                    os.makedirs(share_config, exist_ok=True)
                    share_filepath = os.path.join(share_config, safe_fname)
                    with open(share_filepath, "w", encoding="utf-8") as sf:
                        json.dump(scenario_data, sf, indent=2, ensure_ascii=False)
            except Exception:
                pass

            return jsonify({
                "success": True,
                "message": f"Preset guardado como '{safe_fname}'",
                "filename": safe_fname,
                "filepath": filepath
            })
        except Exception as e:
            err = f"Error al guardar preset: {e}"
            node.get_logger().error(err)
            return jsonify({"success": False, "error": err}), 500

    return app


