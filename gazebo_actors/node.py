"""
ROS 2 ActorManagerNode managing multiple actors, Gazebo synchronization, and TF broadcast.
"""

import json
import math
import os
import subprocess
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster

from .actor import ManagedActor
from .gazebo_interface import (
    detect_gazebo_world,
    generate_actor_sdf,
    generate_collider_sdf
)
from .terrain import compute_terrain_elevation, detect_terrain_profile
from .utils import resolve_actor_assets, resolve_asset_paths, sanitize_identifier

try:
    from ros_gz_interfaces.srv import SetEntityPose
    from ros_gz_interfaces.msg import Entity
    HAS_ROS_GZ = True
except ImportError:
    HAS_ROS_GZ = False

try:
    import gz.transport13 as gz_transport
    import gz.msgs10 as gz_msgs
    HAS_GZ_PYTHON = True
except ImportError:
    HAS_GZ_PYTHON = False


class ActorManagerNode(Node):
    """
    ROS 2 Node for multi-actor simulation, waypoint sequencing,
    and real-time Gazebo Sim synchronization.
    """

    def __init__(self):
        super().__init__("actor_manager")

        # Declare parameters
        self.declare_parameter("world_name", "auto")
        self.declare_parameter("terrain_type", "auto")
        self.declare_parameter("slope_deg", 0.0)
        self.declare_parameter("x_min", -15.0)
        self.declare_parameter("x_max", 15.0)
        self.declare_parameter("y_min", -15.0)
        self.declare_parameter("y_max", 15.0)
        self.declare_parameter("environment_name", "Simulación General")
        self.declare_parameter("robot_odom_topic", "odom")
        self.declare_parameter("spawn_initial_actor", True)
        self.declare_parameter("default_actor_name", "human_worker")
        self.declare_parameter("default_actor_x", 4.5)
        self.declare_parameter("default_actor_y", 0.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("web_port", 5000)
        self.declare_parameter("sync_gazebo", True)
        self.declare_parameter("scenario_file", "")

        # Retrieve parameters
        self.world_name = str(self.get_parameter("world_name").value)
        self.terrain_type = str(self.get_parameter("terrain_type").value).lower()
        self.slope_deg = float(self.get_parameter("slope_deg").value)
        self.x_min = float(self.get_parameter("x_min").value)
        self.x_max = float(self.get_parameter("x_max").value)
        self.y_min = float(self.get_parameter("y_min").value)
        self.y_max = float(self.get_parameter("y_max").value)
        self.environment_name = str(self.get_parameter("environment_name").value)
        self.default_actor_x = float(self.get_parameter("default_actor_x").value)
        self.default_actor_y = float(self.get_parameter("default_actor_y").value)
        self.robot_odom_topic = str(self.get_parameter("robot_odom_topic").value)
        self.spawn_initial_actor = bool(self.get_parameter("spawn_initial_actor").value)
        self.default_actor_name = str(self.get_parameter("default_actor_name").value)
        self.publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.web_port = int(self.get_parameter("web_port").value)
        self.sync_gazebo = bool(self.get_parameter("sync_gazebo").value)
        self.scenario_file = str(self.get_parameter("scenario_file").value).strip()

        # Auto-detect active world
        det_world = detect_gazebo_world(current_world=self.world_name)
        if det_world:
            self.world_name = det_world
            self.get_logger().info(f"🔍 Mundo de Gazebo auto-detectado: '{self.world_name}'")

        # Auto-configure terrain bounds and profile
        profile = detect_terrain_profile(self.world_name, self.terrain_type, self.slope_deg)
        if self.terrain_type == "auto":
            self.terrain_type = profile["terrain_type"]
            self.slope_deg = profile["slope_deg"]
            self.x_min = profile["x_min"]
            self.x_max = profile["x_max"]
            self.y_min = profile["y_min"]
            self.y_max = profile["y_max"]
            self.environment_name = profile["environment_name"]

        self.tf_broadcaster = TransformBroadcaster(self)
        self.lock = threading.Lock()

        # Service client for SetEntityPose and parameter bridge
        self.set_pose_client = None
        self.bridge_proc = None
        self.gz_node = None
        self.gz_msgs = None
        if HAS_GZ_PYTHON:
            try:
                self.gz_node = gz_transport.Node()
                self.gz_msgs = gz_msgs
            except Exception:
                pass

        if self.sync_gazebo and self.world_name not in ("auto", "default", ""):
            self._ensure_set_pose_bridge()

        # Actors registry and ROS 2 publishers
        self.actors: Dict[str, ManagedActor] = {}
        self.actor_pubs: Dict[str, Dict[str, Any]] = {}
        self.actor_last_action: Dict[str, str] = {}
        self.actor_last_anim_time: Dict[str, float] = {}
        self.gz_anim_pubs: Dict[str, Any] = {}
        self._cleaned_up = False
        self._scenario_loaded = False

        # Load scenario file if specified via launch argument / parameter
        if self.scenario_file:
            ok, msg = self.load_scenario_file(self.scenario_file)
            if ok:
                self._scenario_loaded = True
                self.get_logger().info(f"📂 Configuración inicial cargada: {msg}")
            else:
                self.get_logger().warn(f"⚠️ No se pudo cargar escenario desde '{self.scenario_file}': {msg}")

        # Default initial actor fallback if no scenario loaded and spawn_initial_actor is true
        if not self._scenario_loaded and self.spawn_initial_actor:
            default_worker = ManagedActor(
                name=self.default_actor_name,
                x=self.default_actor_x,
                y=self.default_actor_y,
                color="#10b981",
                speed=0.65,
                slope_deg=self.slope_deg,
                terrain_type=self.terrain_type,
                is_default=True,
                world_name=self.world_name
            )
            self.actors[self.default_actor_name] = default_worker
            self._setup_actor_publishers(self.default_actor_name)

        if self.sync_gazebo and (self.actors or self.spawn_initial_actor):
            threading.Thread(target=self._initial_gazebo_spawn, daemon=True).start()
        elif not self.actors:
            self.get_logger().info("ℹ️ Sin actores iniciales. Spawnea o carga un escenario desde la Web UI.")

        # Subscriptions
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        clock_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.clock_sub = self.create_subscription(Clock, "/clock", self.clock_callback, clock_qos)
        self.robot_odom_sub = self.create_subscription(Odometry, self.robot_odom_topic, self.robot_odom_callback, 10)
        self.human_cmd_sub = self.create_subscription(Twist, "human/cmd_vel", self.human_cmd_callback, 10)
        self.actor_cmd_sub = self.create_subscription(Twist, "actor/cmd_vel", self.human_cmd_callback, 10)

        self.sim_time_sec = 0.0
        self.last_sim_time = 0.0
        self.robot_pos = None
        self.robot_yaw = 0.0

        # Terminate any lingering tracker processes
        try:
            subprocess.run(["pkill", "-f", "human_worker_tracker"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(["pkill", "-f", "actor_tracker"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

        # 20 Hz update timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.update_loop)

        self.get_logger().info("============================================================")
        self.get_logger().info("🤖 Gazebo Actor Manager & Waypoint Sequencer")
        self.get_logger().info(f"   Entorno: {self.environment_name} | Mundo: {self.world_name}")
        self.get_logger().info(f"   Terreno: {self.terrain_type.upper()} ({self.slope_deg}°) | Rango: X[{self.x_min}, {self.x_max}] Y[{self.y_min}, {self.y_max}]")
        self.get_logger().info(f"   Web UI disponible en: http://localhost:{self.web_port}")
        self.get_logger().info("============================================================")

    def destroy_node(self):
        """Shuts down the node, cleans up Gazebo actors, and terminates bridge processes."""
        try:
            self.cleanup_all_actors()
        except Exception as e:
            self.get_logger().warn(f"Aviso al limpiar actores en destroy_node: {e}")

        if self.bridge_proc:
            try:
                self.bridge_proc.terminate()
                self.bridge_proc.wait(timeout=1.0)
            except Exception:
                pass
        super().destroy_node()

    def _ensure_set_pose_bridge(self):
        """
        Ensures the ROS 2 service /world/{world_name}/set_pose is available.
        If an external simulation does not provide it, launches a lightweight
        ros_gz_bridge parameter_bridge automatically in the background.
        """
        if not self.world_name or self.world_name in ("auto", "default"):
            return

        srv_name = f"/world/{self.world_name}/set_pose"

        has_srv = False
        try:
            for srv_tuple in self.get_service_names_and_types():
                if srv_tuple[0] == srv_name:
                    has_srv = True
                    break
        except Exception:
            pass

        if not has_srv and not self.bridge_proc:
            try:
                bridge_args = [
                    'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    f'{srv_name}@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean'
                ]
                self.bridge_proc = subprocess.Popen(
                    bridge_args,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                self.get_logger().info(f"🌉 Puente ros_gz_bridge iniciado automáticamente para {srv_name}")
                time.sleep(0.4)
            except Exception as e:
                self.get_logger().warn(f"Aviso al iniciar ros_gz_bridge: {e}")

        if HAS_ROS_GZ and self.sync_gazebo:
            self.set_pose_client = self.create_client(SetEntityPose, srv_name)
            self.get_logger().info(f"Cliente de servicio Gazebo configurado: {srv_name}")

    def get_sim_time(self) -> float:
        """Returns exact simulation time from clock or monotonic time fallback."""
        if self.sim_time_sec > 0.0:
            return self.sim_time_sec
        now = self.get_clock().now()
        sec = now.nanoseconds * 1e-9
        if sec > 0.0:
            return sec
        return time.monotonic()

    def _initial_gazebo_spawn(self):
        """Background thread to spawn initial actors after Gazebo is ready."""
        for attempt in range(25):
            time.sleep(1.0)
            if self.world_name in ("auto", "default", ""):
                det = detect_gazebo_world(current_world=self.world_name)
                if det:
                    self.world_name = det
                    profile = detect_terrain_profile(self.world_name, "auto", self.slope_deg)
                    self.terrain_type = profile["terrain_type"]
                    self.slope_deg = profile["slope_deg"]
                    self.x_min = profile["x_min"]
                    self.x_max = profile["x_max"]
                    self.y_min = profile["y_min"]
                    self.y_max = profile["y_max"]
                    self.environment_name = profile["environment_name"]
                    with self.lock:
                        for act in self.actors.values():
                            act.world_name = self.world_name
                            act.terrain_type = self.terrain_type
                            act.slope_deg = self.slope_deg
                    self._ensure_set_pose_bridge()
                    self.get_logger().info(f"🔍 Conectado a mundo Gazebo: '{self.world_name}' ({self.environment_name})")

            if self.world_name and self.world_name not in ("auto", "default"):
                with self.lock:
                    actors_to_spawn = [act for act in self.actors.values() if not act.is_spawned_in_gz]

                if not actors_to_spawn:
                    return

                all_ok = True
                for actor in actors_to_spawn:
                    ok = self.spawn_gazebo_actor(
                        actor.name, actor.x, actor.y, z=actor.z, color=actor.color
                    )
                    with self.lock:
                        actor.is_spawned_in_gz = ok
                    if ok:
                        self.get_logger().info(f"✅ Actor '{actor.name}' generado en Gazebo.")
                    else:
                        all_ok = False

                if all_ok and actors_to_spawn:
                    self.get_logger().info(f"🎉 Todos los actores ({len(self.actors)}) generados en Gazebo.")
                    return
                else:
                    self.get_logger().debug(f"Intento {attempt + 1}/25: Esperando a Gazebo...")

    def _resolve_config_path(self, filepath: str) -> Optional[str]:
        """Resolves configuration file path checking multiple standard locations."""
        if not filepath:
            return None

        fp = filepath.strip().strip("'\"")

        candidates = [
            fp,
            os.path.abspath(fp),
            os.path.join(os.getcwd(), fp)
        ]
        if not fp.endswith((".json", ".yaml", ".yml")):
            candidates.append(f"{fp}.json")
            candidates.append(os.path.join(os.getcwd(), f"{fp}.json"))

        # Check package share directory
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory("gazebo_actors")
            candidates.append(os.path.join(share_dir, "config", fp))
            candidates.append(os.path.join(share_dir, fp))
            if not fp.endswith((".json", ".yaml", ".yml")):
                candidates.append(os.path.join(share_dir, "config", f"{fp}.json"))
        except Exception:
            pass

        # Check source tree directory
        this_dir = os.path.dirname(os.path.abspath(__file__))
        src_config = os.path.abspath(os.path.join(this_dir, "..", "config"))
        candidates.append(os.path.join(src_config, fp))
        if not fp.endswith((".json", ".yaml", ".yml")):
            candidates.append(os.path.join(src_config, f"{fp}.json"))

        for c in candidates:
            if os.path.isfile(c):
                return os.path.abspath(c)

        return None

    def get_presets_directory(self) -> str:
        """Returns the primary writable presets directory."""
        this_dir = os.path.dirname(os.path.abspath(__file__))
        src_config = os.path.abspath(os.path.join(this_dir, "..", "config"))
        if os.path.isdir(os.path.abspath(os.path.join(this_dir, "..", "gazebo_actors"))):
            os.makedirs(src_config, exist_ok=True)
            return src_config

        try:
            from ament_index_python.packages import get_package_share_directory
            share_config = os.path.join(get_package_share_directory("gazebo_actors"), "config")
            os.makedirs(share_config, exist_ok=True)
            return share_config
        except Exception:
            pass

        fallback = os.path.join(os.getcwd(), "config")
        os.makedirs(fallback, exist_ok=True)
        return fallback

    def list_preset_files(self) -> List[Dict[str, Any]]:
        """Lists all available JSON/YAML scenario preset files."""
        dirs = []
        this_dir = os.path.dirname(os.path.abspath(__file__))
        dirs.append(os.path.abspath(os.path.join(this_dir, "..", "config")))
        try:
            from ament_index_python.packages import get_package_share_directory
            dirs.append(os.path.join(get_package_share_directory("gazebo_actors"), "config"))
        except Exception:
            pass
        dirs.append(os.path.join(os.getcwd(), "config"))

        presets = {}
        for d in dirs:
            if not os.path.isdir(d):
                continue
            for fname in sorted(os.listdir(d)):
                if fname.endswith((".json", ".yaml", ".yml")):
                    fpath = os.path.join(d, fname)
                    if fname not in presets:
                        desc = ""
                        count = 0
                        try:
                            with open(fpath, "r", encoding="utf-8") as f:
                                if fname.endswith((".yaml", ".yml")):
                                    import yaml
                                    pdata = yaml.safe_load(f) or {}
                                else:
                                    pdata = json.load(f) or {}
                                desc = pdata.get("description", pdata.get("name", ""))
                                count = len(pdata.get("actors", []))
                        except Exception:
                            pass
                        presets[fname] = {
                            "filename": fname,
                            "path": fpath,
                            "description": desc,
                            "actors_count": count
                        }
        return list(presets.values())

    def export_scenario_data(self) -> Dict[str, Any]:
        """Exports full scenario configuration as a JSON-serializable dictionary."""
        with self.lock:
            actors_data = [act.to_config_dict() for act in self.actors.values()]
            return {
                "version": "1.0",
                "environment_name": self.environment_name,
                "world_name": self.world_name,
                "terrain_type": self.terrain_type,
                "slope_deg": self.slope_deg,
                "actors_count": len(actors_data),
                "actors": actors_data
            }

    def import_scenario_data(self, scenario_data: Dict[str, Any]) -> Tuple[bool, str]:
        """
        Loads a full scenario: cleans up current actors in Gazebo,
        and spawns/configures all actors defined in scenario_data.
        """
        if not isinstance(scenario_data, dict) or "actors" not in scenario_data:
            return False, "Estructura de escenario inválida (debe contener la clave 'actors')"

        raw_actors = scenario_data.get("actors", [])
        if not isinstance(raw_actors, list):
            return False, "La clave 'actors' debe ser una lista de actores"

        # 1. Clean up existing actors from Gazebo
        with self.lock:
            existing_names = list(self.actors.keys())
        for name in existing_names:
            try:
                self.delete_gazebo_actor(name)
            except Exception as e:
                self.get_logger().warn(f"Aviso al limpiar actor '{name}' previo a importación: {e}")

        with self.lock:
            self.actors.clear()
            self.actor_last_action.clear()
            self.actor_last_anim_time.clear()
            self._cleaned_up = False

        # 2. Re-create each actor from scenario data
        spawned_count = 0
        for act_data in raw_actors:
            try:
                actor = ManagedActor.from_config_dict(
                    act_data,
                    slope_deg=self.slope_deg,
                    terrain_type=self.terrain_type,
                    world_name=self.world_name
                )
                with self.lock:
                    self.actors[actor.name] = actor
                    self._setup_actor_publishers(actor.name)

                # Spawn in Gazebo if sync enabled and world ready
                if self.sync_gazebo and self.world_name not in ("auto", "default", ""):
                    ok = self.spawn_gazebo_actor(actor.name, actor.x, actor.y, z=actor.z, color=actor.color)
                    actor.is_spawned_in_gz = ok
                    if ok:
                        spawned_count += 1
                        self.sync_gazebo_pose(actor.col_model_name, actor.x, actor.y, actor.z, actor.yaw)
                else:
                    spawned_count += 1
            except Exception as e:
                self.get_logger().error(f"Error al instanciar actor desde configuración: {e}")

        msg = f"Escenario cargado con éxito: {len(self.actors)} actores configurados ({spawned_count} en Gazebo)."
        self.get_logger().info(f"📂 {msg}")
        return True, msg

    def load_scenario_file(self, filepath: str) -> Tuple[bool, str]:
        """Resolves file path and imports scenario configuration from JSON or YAML."""
        resolved_path = self._resolve_config_path(filepath)
        if not resolved_path:
            return False, f"No se pudo encontrar el archivo de configuración: '{filepath}'"

        try:
            with open(resolved_path, "r", encoding="utf-8") as f:
                if resolved_path.endswith((".yaml", ".yml")):
                    import yaml
                    data = yaml.safe_load(f)
                else:
                    data = json.load(f)
            self.get_logger().info(f"📂 Cargando archivo de configuración: {resolved_path}")
            return self.import_scenario_data(data)
        except Exception as e:
            err = f"Error al leer archivo de configuración '{resolved_path}': {e}"
            self.get_logger().error(err)
            return False, err

    def _setup_actor_publishers(self, name: str):
        """Creates Pose and Odometry publishers for a given actor identifier."""
        if name not in self.actor_pubs:
            topic_base = sanitize_identifier(name)
            try:
                self.actor_pubs[name] = {
                    "pose": self.create_publisher(PoseStamped, f"{topic_base}/pose", 10),
                    "odom": self.create_publisher(Odometry, f"{topic_base}/odom", 10)
                }
            except Exception as e:
                self.get_logger().error(f"Error creando publicadores ROS 2 para '{name}': {e}")

            # Pre-advertise Gazebo animation topic
            if getattr(self, "gz_node", None) and HAS_GZ_PYTHON:
                try:
                    from gz.msgs10.stringmsg_pb2 import StringMsg
                    anim_topic = f"/actor/{name}/animation_cmd"
                    if name not in self.gz_anim_pubs:
                        self.gz_anim_pubs[name] = self.gz_node.advertise(anim_topic, StringMsg)
                except Exception:
                    pass

    def clock_callback(self, msg: Clock):
        self.sim_time_sec = msg.clock.sec + msg.clock.nanosec * 1e-9

    def robot_odom_callback(self, msg: Odometry):
        self.robot_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.robot_yaw = 2.0 * math.atan2(qz, qw)

    def human_cmd_callback(self, msg: Twist):
        with self.lock:
            target_actor = None
            if "human_worker" in self.actors:
                target_actor = self.actors["human_worker"]
            elif len(self.actors) > 0:
                target_actor = next(iter(self.actors.values()))
            if target_actor:
                if (abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01 or abs(msg.linear.z) > 0.01):
                    target_actor.state = "MANUAL"
                    target_actor.teleop_lin = msg.linear.x
                    target_actor.teleop_ang = msg.angular.z
                    target_actor.teleop_vert = msg.linear.z
                    target_actor.last_teleop_time = time.monotonic()
                elif target_actor.state == "MANUAL":
                    if time.monotonic() - target_actor.last_teleop_time > 0.8:
                        target_actor.teleop_lin = 0.0
                        target_actor.teleop_ang = 0.0
                        target_actor.teleop_vert = 0.0
                        target_actor.vx = 0.0
                        target_actor.wz = 0.0

    def sync_gazebo_pose(self, entity_name: str, x: float, y: float, z: float, yaw: float):
        """Sends entity pose to Gazebo Sim via direct gz.transport topic and ROS 2 service."""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        # 1. Direct high-frequency topic publication to GazeboActorFollower
        pose_msg = None
        if getattr(self, "gz_node", None) and HAS_GZ_PYTHON:
            try:
                from gz.msgs10.pose_pb2 import Pose
                pose_msg = Pose()
                pose_msg.name = entity_name
                pose_msg.position.x = float(x)
                pose_msg.position.y = float(y)
                pose_msg.position.z = float(z)
                pose_msg.orientation.x = 0.0
                pose_msg.orientation.y = 0.0
                pose_msg.orientation.z = float(qz)
                pose_msg.orientation.w = float(qw)

                if not hasattr(self, "gz_pose_pubs"):
                    self.gz_pose_pubs = {}
                base_name = entity_name.replace("_collision", "")
                topic = f"/actor/{base_name}/pose_cmd"
                if base_name not in self.gz_pose_pubs:
                    self.gz_pose_pubs[base_name] = self.gz_node.advertise(topic, Pose)
                self.gz_pose_pubs[base_name].publish(pose_msg)
            except Exception as e:
                self.get_logger().debug(f"Aviso gz.transport pose {entity_name}: {e}")

        # 2. Primary path for physical collision model: ROS 2 service
        if self.set_pose_client and self.set_pose_client.service_is_ready():
            req = SetEntityPose.Request()
            req.entity.name = entity_name
            req.entity.type = Entity.MODEL
            req.pose.position.x = float(x)
            req.pose.position.y = float(y)
            req.pose.position.z = float(z)
            req.pose.orientation.x = 0.0
            req.pose.orientation.y = 0.0
            req.pose.orientation.z = float(qz)
            req.pose.orientation.w = float(qw)
            self.set_pose_client.call_async(req)
            return

        # 3. In-memory gz.transport fallback
        if getattr(self, "gz_node", None) and HAS_GZ_PYTHON and self.world_name and pose_msg:
            try:
                from gz.msgs10.pose_pb2 import Pose
                from gz.msgs10.boolean_pb2 import Boolean
                srv_name = f"/world/{self.world_name}/set_pose"
                self.gz_node.request(srv_name, pose_msg, Pose, Boolean, 50)
            except Exception:
                pass

    def _check_gazebo_world(self, timeout: float = 0.8, max_retries: int = 1) -> str:
        """Throttled check to detect active Gazebo world without blocking repeatedly."""
        now = time.monotonic()
        last_t = getattr(self, "_last_world_check_t", 0.0)
        if now - last_t < 2.0 and getattr(self, "_last_world_check_val", None) is not None:
            return self._last_world_check_val

        self._last_world_check_t = now
        det = detect_gazebo_world(max_retries=max_retries, timeout=timeout, current_world=self.world_name)
        if det:
            self.world_name = det
            self._last_world_check_val = det
            self._ensure_set_pose_bridge()
            return det

        self._last_world_check_val = ""
        return ""

    def spawn_gazebo_actor(
        self,
        name: str,
        x: float,
        y: float,
        z: Optional[float] = None,
        color: str = "#10b981"
    ) -> bool:
        """Dynamically spawns collider model and animated actor in Gazebo Sim."""
        if self.world_name in ("auto", "default", ""):
            self._check_gazebo_world(timeout=0.8, max_retries=1)

        if z is None:
            z = compute_terrain_elevation(x, y, self.slope_deg, self.terrain_type, self.world_name)

        col_name = f"{name}_collision"
        col_sdf = generate_collider_sdf(name, x, y, z, color)

        skin_mesh, walk_mesh, stand_mesh, dance_mesh, talk_mesh, plugin_file = resolve_actor_assets(color=color)

        actor_sdf = generate_actor_sdf(
            name, col_name, skin_mesh, walk_mesh, stand_mesh, dance_mesh, talk_mesh, plugin_file
        )

        try:
            req1 = f'sdf: "{col_sdf.replace(chr(10), " ").replace(chr(34), chr(92) + chr(34))}"'
            res1 = subprocess.run([
                'gz', 'service', '-s', f'/world/{self.world_name}/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '3000',
                '--req', req1
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5.0)

            req2 = f'sdf: "{actor_sdf.replace(chr(10), " ").replace(chr(34), chr(92) + chr(34))}"'
            res2 = subprocess.run([
                'gz', 'service', '-s', f'/world/{self.world_name}/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '3000',
                '--req', req2
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5.0)
            return res1.returncode == 0 and res2.returncode == 0
        except Exception as e:
            self.get_logger().warn(f"Aviso al spawnear actor en Gazebo: {e}")
            return False

    def send_gazebo_animation(self, name: str, anim_name: str, force_cli: bool = False):
        """Sends animation clip command to GazeboActorFollower via gz.transport and gz topic."""
        topic = f"/actor/{name}/animation_cmd"
        sent_via_gz = False
        if self.gz_node and HAS_GZ_PYTHON:
            try:
                from gz.msgs10.stringmsg_pb2 import StringMsg
                if name not in self.gz_anim_pubs:
                    self.gz_anim_pubs[name] = self.gz_node.advertise(topic, StringMsg)
                msg = StringMsg()
                msg.data = anim_name
                sent_via_gz = self.gz_anim_pubs[name].publish(msg)
            except Exception as e:
                self.get_logger().debug(f"Aviso gz.transport animación {name}: {e}")

        # CLI fallback to ensure guaranteed delivery if not sent in-memory or on explicit state change
        if not sent_via_gz or force_cli:
            try:
                subprocess.Popen([
                    'gz', 'topic', '-t', topic,
                    '-m', 'gz.msgs.StringMsg',
                    '-p', f'data: "{anim_name}"'
                ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass

    def delete_gazebo_actor(self, name: str) -> bool:
        """Deletes actor and physical collider model from Gazebo Sim."""
        col_name = f"{name}_collision"
        if not self.world_name or self.world_name in ("auto", "default"):
            self._check_gazebo_world(timeout=0.8, max_retries=1)

        if not self.world_name or self.world_name in ("auto", "default"):
            return False

        # 1. Direct delete topic for GazeboActorFollower
        if getattr(self, "gz_node", None) and HAS_GZ_PYTHON:
            try:
                from gz.msgs10.boolean_pb2 import Boolean
                if not hasattr(self, "gz_del_pubs"):
                    self.gz_del_pubs = {}
                for topic_name in (f"/actor/{name}/delete", f"/actor/{col_name}/delete"):
                    if topic_name not in self.gz_del_pubs:
                        self.gz_del_pubs[topic_name] = self.gz_node.advertise(topic_name, Boolean)
                    b_msg = Boolean()
                    b_msg.data = True
                    self.gz_del_pubs[topic_name].publish(b_msg)
            except Exception:
                pass

        try:
            subprocess.Popen([
                'gz', 'topic', '-t', f'/actor/{name}/delete',
                '-m', 'gz.msgs.Boolean',
                '-p', 'data: true'
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

        # 2. In-memory service removal via gz_node (fast, < 5ms)
        removed_in_memory = False
        if getattr(self, "gz_node", None) and HAS_GZ_PYTHON:
            try:
                from gz.msgs10.entity_pb2 import Entity
                from gz.msgs10.boolean_pb2 import Boolean
                srv_remove = f"/world/{self.world_name}/remove"

                req_col = Entity()
                req_col.name = col_name
                req_col.type = Entity.MODEL
                self.gz_node.request(srv_remove, req_col, Entity, Boolean, 1000)

                req_act = Entity()
                req_act.name = name
                req_act.type = Entity.ACTOR
                self.gz_node.request(srv_remove, req_act, Entity, Boolean, 1000)
                removed_in_memory = True
            except Exception as e:
                self.get_logger().debug(f"Aviso al eliminar entidades via gz_node: {e}")

        # 3. CLI fallback if in-memory removal was not available
        if not removed_in_memory:
            try:
                subprocess.run([
                    'gz', 'service', '-s', f'/world/{self.world_name}/remove',
                    '--reqtype', 'gz.msgs.Entity',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '1000',
                    '--req', f'name: "{col_name}", type: MODEL'
                ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.5)

                subprocess.run([
                    'gz', 'service', '-s', f'/world/{self.world_name}/remove',
                    '--reqtype', 'gz.msgs.Entity',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '1000',
                    '--req', f'name: "{name}", type: ACTOR'
                ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.5)
            except Exception:
                pass

        self.get_logger().info(f"🗑️ Actor '{name}' y colisionador '{col_name}' eliminados de Gazebo.")
        return True

    def cleanup_all_actors(self):
        """Removes all spawned actors and their collision models from Gazebo Sim on node shutdown."""
        if getattr(self, "_cleaned_up", False):
            return
        self._cleaned_up = True

        if not self.sync_gazebo:
            return

        with self.lock:
            actor_names = list(self.actors.keys())
            if not actor_names and getattr(self, "default_actor_name", None):
                actor_names = [self.default_actor_name]

        if not actor_names:
            return

        self.get_logger().info(f"🧹 Limpiando {len(actor_names)} actores del entorno de Gazebo...")
        for name in actor_names:
            try:
                self.delete_gazebo_actor(name)
            except Exception as e:
                self.get_logger().debug(f"Aviso al limpiar actor '{name}': {e}")

    def update_loop(self):
        """20 Hz periodic sequencing and synchronization loop."""
        now = self.get_clock().now()
        stamp = now.to_msg()
        t = self.get_sim_time()

        if self.last_sim_time <= 0.0:
            self.last_sim_time = t
            return

        dt = t - self.last_sim_time
        if dt < 0.0 or dt > 0.5:
            dt = 1.0 / self.publish_rate
        else:
            dt = max(0.001, min(0.1, dt))
        self.last_sim_time = t

        ANIM_MAP = {
            "idle": "stand",
            "stand": "stand",
            "dance": "dance",
            "moonwalk": "dance",
            "talk": "talk"
        }

        with self.lock:
            for name, actor in self.actors.items():
                # 1. Advance sequencer kinematics
                actor.step_sequencer(dt, t)

                # Determine desired animation name for Gazebo
                desired_anim = "stand"
                if actor.state == "NAVIGATING":
                    desired_anim = "walk"
                elif actor.state == "WAITING":
                    desired_anim = ANIM_MAP.get(actor.current_action, "stand")
                elif actor.state == "MANUAL":
                    is_moving = (abs(actor.vx) > 0.05 or abs(actor.wz) > 0.05)
                    if is_moving:
                        desired_anim = "walk"
                    else:
                        idle_act = getattr(actor, "teleop_idle_action", "idle") or "idle"
                        desired_anim = ANIM_MAP.get(idle_act, "stand")
                elif actor.state in ("IDLE", "PAUSED"):
                    idle_act = getattr(actor, "teleop_idle_action", "idle") or "idle"
                    desired_anim = ANIM_MAP.get(idle_act, "stand")

                # 2. Synchronize pose and animation in Gazebo
                if self.sync_gazebo:
                    self.sync_gazebo_pose(actor.col_model_name, actor.x, actor.y, actor.z, actor.yaw)

                    last_anim_time = self.actor_last_anim_time.get(name, 0.0)
                    anim_changed = (desired_anim != self.actor_last_action.get(name))
                    if anim_changed or (t - last_anim_time > 2.0):
                        self.actor_last_action[name] = desired_anim
                        self.actor_last_anim_time[name] = t
                        self.send_gazebo_animation(name, desired_anim, force_cli=anim_changed)

                # 3. Publish ROS 2 topics
                self._setup_actor_publishers(name)
                pubs = self.actor_pubs[name]

                qz = math.sin(actor.yaw / 2.0)
                qw = math.cos(actor.yaw / 2.0)

                # PoseStamped
                p_msg = PoseStamped()
                p_msg.header.stamp = stamp
                p_msg.header.frame_id = "map"
                p_msg.pose.position.x = float(actor.x)
                p_msg.pose.position.y = float(actor.y)
                p_msg.pose.position.z = float(actor.z + 0.85)
                p_msg.pose.orientation.z = float(qz)
                p_msg.pose.orientation.w = float(qw)
                pubs["pose"].publish(p_msg)

                # Odometry
                o_msg = Odometry()
                o_msg.header.stamp = stamp
                o_msg.header.frame_id = "map"
                o_msg.child_frame_id = name
                o_msg.pose.pose = p_msg.pose
                o_msg.twist.twist.linear.x = float(actor.vx)
                o_msg.twist.twist.angular.z = float(actor.wz)
                pubs["odom"].publish(o_msg)

                # TF transform
                tf_msg = TransformStamped()
                tf_msg.header.stamp = stamp
                tf_msg.header.frame_id = "map"
                tf_msg.child_frame_id = name
                tf_msg.transform.translation.x = float(actor.x)
                tf_msg.transform.translation.y = float(actor.y)
                tf_msg.transform.translation.z = float(actor.z)
                tf_msg.transform.rotation.z = float(qz)
                tf_msg.transform.rotation.w = float(qw)
                self.tf_broadcaster.sendTransform(tf_msg)

