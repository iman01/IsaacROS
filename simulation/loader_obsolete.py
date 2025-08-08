from isaacsim import SimulationApp
import os

import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
import threading
from sensor_msgs.msg import Image
import numpy as np
import yaml
import math
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description="Isaac Sim URDF Loader")

    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run in headless mode (no UI). Default: False"
    )
    parser.add_argument(
        "--ghost-opacity",
        type=float,
        default=0,
        help="Opacity of the ghost robot (0.0–1.0). If 0, the robot will not spawn. Default: 0"
    )
    parser.add_argument(
        "--robot-urdf",
        type=str,
        default="agrorob/agrorob_visualization.urdf",
        help="Path to main robot URDF. Default: agrorob/agrorob_visualization.urdf"
    )
    parser.add_argument(
        "--ghost-urdf",
        type=str,
        default="agrorob/agrorob_visualization_ghost.urdf",
        help="Path to ghost robot URDF. Default: agrorob/agrorob_visualization_ghost.urdf"
    )

    return parser.parse_args()

args = parse_args()
simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.utils.stage as stage_utils
import isaacsim.core.utils.prims as prim_utils
import isaacsim.core.utils.extensions as extensions
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.core.api import World
from isaacsim.core.api.objects import GroundPlane

if args.ghost_opacity:
    from agrorob_msgs.msg import RobotState



extensions.enable_extension("isaacsim.ros2.bridge")
extensions.enable_extension("isaacsim.replicator.synthetic_recorder")
extensions.enable_extension("omni.kit.window.script_editor")


from isaacsim.asset.importer.urdf import _urdf

import omni.kit.commands
import omni.usd

import omni.replicator.core as rep

from isaacsim.core.utils.semantics import add_update_semantics

from pxr import UsdPhysics, Gf, UsdGeom
from isaacsim.sensors.camera import Camera
import omni.syntheticdata._syntheticdata as sd
import omni.replicator.core as rep
import omni.graph.core as og



from pxr import UsdShade, Sdf, UsdGeom, Gf, Vt, Usd

import omni.usd
from pxr import UsdPhysics
import isaacsim.core.utils.prims as prim_utils
from isaacsim.sensors.camera import Camera
import carb.settings






def ros_spin(node):
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        print("ROS2 node has been shut down.")


class URDFLoaderApp:
    def __init__(self):
        self.world = None
        self.args = None

    def setup_scene(self): 
        def create_ground():
            self.world = World(stage_units_in_meters=1.0)
            GroundPlane("/World/defaultGroundPlane", size=100.0)
            plane_path = "/World/TexturedPlane"
            material_path = "/World/Materials/TiledMaterial"
            texture_file_path = os.path.join(
                os.path.dirname(__file__), "dirt.png"
            )  # ← Set this to your actual image file path
            texture_scale = (100.0, 100.0)  # Tiling factor

            size = 100.0
            # === Create a simple quad plane (4 vertices, 1 face) ===
            plane = UsdGeom.Mesh.Define(stage, plane_path)
            points = Vt.Vec3fArray(
                [
                    Gf.Vec3f(-size, -size, 0.01),
                    Gf.Vec3f(size, -size, 0.01),
                    Gf.Vec3f(size, size, 0.01),
                    Gf.Vec3f(-size, size, 0.01),
                ]
            )
            faceVertexCounts = Vt.IntArray([4])
            faceVertexIndices = Vt.IntArray([0, 1, 2, 3])
            plane.CreatePointsAttr(points)
            plane.CreateFaceVertexCountsAttr(faceVertexCounts)
            plane.CreateFaceVertexIndicesAttr(faceVertexIndices)

            # Add UVs to the plane
            uvs = Vt.Vec2fArray(
                [
                    Gf.Vec2f(0.0, 0.0),
                    Gf.Vec2f(texture_scale[0], 0.0),
                    Gf.Vec2f(texture_scale[0], texture_scale[1]),
                    Gf.Vec2f(0.0, texture_scale[1]),
                ]
            )
            primvars_api = UsdGeom.PrimvarsAPI(plane)
            st = primvars_api.CreatePrimvar(
                "st", Sdf.ValueTypeNames.Float2Array, UsdGeom.Tokens.vertex
            )
            st.Set(uvs)

            # === Create material with a texture ===
            material = UsdShade.Material.Define(stage, material_path)
            shader = UsdShade.Shader.Define(stage, material_path + "/Shader")
            shader.CreateIdAttr("UsdPreviewSurface")

            # Texture shader
            texture_shader = UsdShade.Shader.Define(stage, material_path + "/Texture")
            texture_shader.CreateIdAttr("UsdUVTexture")
            texture_shader.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
                texture_file_path
            )
            texture_shader.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
            texture_shader.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")

            # Primvar reader to use the UVs we defined
            st_reader = UsdShade.Shader.Define(stage, material_path + "/PrimVarReader")
            st_reader.CreateIdAttr("UsdPrimvarReader_float2")
            st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")

            # Connect reader to texture
            texture_shader.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
                st_reader.ConnectableAPI(), "result"
            )

            # Connect texture to surface shader
            shader.CreateInput(
                "diffuseColor", Sdf.ValueTypeNames.Color3f
            ).ConnectToSource(texture_shader.ConnectableAPI(), "rgb")

            # Finish material
            material.CreateSurfaceOutput().ConnectToSource(
                shader.ConnectableAPI(), "surface"
            )

            # Bind material to the plane
            UsdShade.MaterialBindingAPI(plane).Bind(material)
        
        def create_objects():
            usdc_path = "simulation/crops.usdc"

            prim_path = "/World/Crops"

            stage = omni.usd.get_context().get_stage()

            omni.kit.commands.execute(
                "CreateReference",
                path_to=prim_path,
                asset_path=usdc_path,
                usd_context=omni.usd.get_context()
            )   

            xform = UsdGeom.Xform(stage.GetPrimAtPath(prim_path))
            xform.AddTranslateOp().Set(Gf.Vec3d(0.0, -1.0, 0.02))  # Position
            xform.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 0))        # Orientation

            root_prim = stage.GetPrimAtPath(prim_path)



            with open("simulation/agrorob_crops.yaml", 'r') as f:
                config = yaml.safe_load(f)

            beds_config = config.get("field", {}).get("beds", {})
            weeds_config = config.get("field", {}).get("weeds", {})

            # Build label lookup dictionaries
            bed_labels = {name: data.get("plant_type", "unknown") for name, data in beds_config.items()}
            weed_labels = {name: "weed" for name in weeds_config.keys()}

            # Combine for fast lookup
            all_labels = {**bed_labels, **weed_labels}

            # Hardcoded exclusions
            EXCLUDE_NAMES = {"_materials", "stones", "ground"}

            if not root_prim.IsValid():
                print(f"[ERROR] Prim '{prim_path}' not found.")
                return

            for child in root_prim.GetChildren():
                name = child.GetName()

                if name in EXCLUDE_NAMES:
                    continue

                if name not in all_labels:
                    print(f"[WARNING] UNKNOWN OBJECT FOUND: {name}")
                    continue

                label = all_labels[name]

                if not child.IsValid():
                    continue

                print(f"[INFO] Labeling group '{name}' with class '{label}': {child.GetPath()}")

                for sub in child.GetChildren():
                    if sub.IsValid() and child.GetTypeName() == 'Xform':
                        add_update_semantics(sub, label)



        def create_robot():
            import_config = _urdf.ImportConfig()
            import_config.convex_decomp = False
            import_config.fix_base = False
            import_config.make_default_prim = True
            import_config.self_collision = False
            import_config.distance_scale = 1
            import_config.density = 0.0

            urdf_path = self.args.robot_urdf

            result, robot_model = omni.kit.commands.execute(
                "URDFParseFile", urdf_path=urdf_path, import_config=import_config
            )

            result, prim_path = omni.kit.commands.execute(
                "URDFImportRobot",
                urdf_robot=robot_model,
                import_config=import_config,
            )

            robot_prim = prim_utils.get_prim_at_path(prim_path)
            if robot_prim:
                xform_api = robot_prim.GetAttribute("xformOp:translate")
                if not xform_api:
                    xform = UsdGeom.Xformable(robot_prim)
                    xform.AddTranslateOp().Set((0, 0, 2.2))
                else:
                    xform_api.Set((0, 0, 2.2))
        
        def create_ghost_robot():
            import_config = _urdf.ImportConfig()
            import_config.convex_decomp = False
            import_config.fix_base = False
            import_config.make_default_prim = False
            import_config.self_collision = False
            import_config.density = 0.0  # no physics

            urdf_path = self.args.ghost_urdf

            result, robot_model = omni.kit.commands.execute(
                "URDFParseFile", urdf_path=urdf_path, import_config=import_config
            )

            result, prim_path = omni.kit.commands.execute(
                "URDFImportRobot",
                urdf_robot=robot_model,
                import_config=import_config,
            )

            stage = omni.usd.get_context().get_stage()

            ghost_path = "/ghost_robot"

            move_prim(prim_path, ghost_path)
            ghost_prim = prim_utils.get_prim_at_path(ghost_path)


            if ghost_prim:
                xform_api = ghost_prim.GetAttribute("xformOp:translate")
                if not xform_api:
                    xform = UsdGeom.Xformable(ghost_prim)
                    xform.AddTranslateOp().Set((0.001, 0.001, 2.2))
                else:
                    xform_api.Set((0.001, 0.001, 2.2))

            
            apply_transparency_to_materials(ghost_path+"/Looks", self.args.ghost_opacity)
                            
        def move_prim(old_path, new_path):
            omni.kit.commands.execute(
                "MovePrim",
                path_from=old_path,
                path_to=new_path
            )


                    


        def apply_transparency_to_materials(root_path, opacity=0.25):
            stage = omni.usd.get_context().get_stage()
            root_prim = stage.GetPrimAtPath(root_path)
            carb.settings.get_settings().set("/rtx/raytracing/fractionalCutoutOpacity", True)

            if not root_prim.IsValid():
                print(f"[ERROR] Material root '{root_path}' not found.")
                return

            print(f"[INFO] Applying transparency to materials under: {root_path}")

            for mat_prim in root_prim.GetChildren():
                if not mat_prim.IsA(UsdShade.Material):
                    continue

                found_shader = False

                for child in mat_prim.GetChildren():
                    if child.GetTypeName() != "Shader":
                        continue

                    shader = UsdShade.Shader(child)
                    shader_id = shader.GetIdAttr().Get()
                    shader_path = child.GetPath()
                    found_shader = True


                    shader.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
                    shader.CreateInput("opacity_constant", Sdf.ValueTypeNames.Float).Set(opacity)

                if not found_shader:
                    print(f"[WARNING] No shader found under material: {mat_prim.GetPath()}")

        def configure_collision_groups():
            stage = omni.usd.get_context().get_stage()
            group_names = ["CollisionGroupA", "CollisionGroupB"]
            group_objs = {}
            for name in group_names:
                path = f"/World/{name}"
                cg = UsdPhysics.CollisionGroup.Define(stage, path)
                coll_api = Usd.CollectionAPI.Apply(cg.GetPrim(), UsdPhysics.Tokens.colliders)
                includes_rel = coll_api.CreateIncludesRel()
                filt_rel = cg.CreateFilteredGroupsRel()
                group_objs[name] = {"cg": cg, "includes": includes_rel, "filters": filt_rel}

            # Set filtering bidirectionally
            group_objs["CollisionGroupA"]["filters"].AddTarget(
                Sdf.Path("/World/CollisionGroupB"))
            group_objs["CollisionGroupB"]["filters"].AddTarget(
                Sdf.Path("/World/CollisionGroupA"))

            
            group_objs["CollisionGroupA"]["includes"].AddTarget("/agrorob_visualization")
            group_objs["CollisionGroupB"]["includes"].AddTarget("/ghost_robot")

                                


    

            


        def configure_cameras():
            def publish_rgb(camera: Camera, cam_name, freq):
                render_product = camera._render_product_path
                step_size = int(60 / freq)
                topic_name = cam_name + "_rgb"
                queue_size = 1
                node_namespace = ""
                frame_id = camera.prim_path.split("/")[-1]

                rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                    sd.SensorType.Rgb.name
                )

                writer = rep.writers.get(rv + "ROS2PublishImage")
                writer.initialize(
                    frameId=frame_id,
                    nodeNamespace=node_namespace,
                    queueSize=queue_size,
                    topicName=topic_name,
                )
                writer.attach([render_product])

                gate_path = omni.syntheticdata.SyntheticData._get_node_path(
                    rv + "IsaacSimulationGate", render_product
                )
                og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

                return
            self.cameras = {}
            camera_configs = [
            {
                "name": "camera_front",
                "prim_path": "/agrorob_visualization/base_link/camera_front",
                "position": np.array([1.45, 0, 2]),
                "orientation": rot_utils.euler_angles_to_quats(
                    np.array([0, 15, 0]), degrees=True
                ),
            },
            {
                "name": "camera_left",
                "prim_path": "/agrorob_visualization/base_link/camera_left",
                "position": np.array([1.58, 0.5, 1.95]),
                "orientation": rot_utils.euler_angles_to_quats(
                    np.array([0, 90, 0]), degrees=True
                ),

            },
            {
                "name": "camera_right",
                "prim_path": "/agrorob_visualization/base_link/camera_right",
                "position": np.array([1.58, -0.52, 1.95]),
                "orientation": rot_utils.euler_angles_to_quats(
                    np.array([0, 90, 0]), degrees=True
                ),
            },
        ]

            for cam in camera_configs:
                camera = Camera(
                    prim_path=cam["prim_path"],
                    position=cam["position"],
                    frequency=60,
                    resolution=(256, 256),
                    orientation=cam["orientation"],
                )
                camera.initialize()
                publish_rgb(camera, cam["name"], freq=60)
                self.cameras[cam["name"]] = camera




        
        stage = omni.usd.get_context().get_stage()
        rep.orchestrator.set_capture_on_play(True)

        
        light_prim = stage.DefinePrim("/World/lightDistant1", "DistantLight")
        light_prim.GetAttribute("inputs:intensity").Set(1000.0)
        light_prim.GetAttribute("inputs:color").Set(Gf.Vec3f(0.75, 0.75, 0.75))

        light_prim2 = stage.DefinePrim("/World/lightDistant2", "DistantLight")
        light_prim2.GetAttribute("inputs:intensity").Set(1000.0)
        light_prim2.GetAttribute("inputs:angle").Set(10)
        light_prim2.GetAttribute("inputs:color").Set(Gf.Vec3f(0.75, 0.75, 0.75))

        create_ground()
        create_objects()
        create_robot()
        if (self.args.ghost_opacity):
            create_ghost_robot()
            configure_collision_groups()
        configure_cameras()


        self.world.reset()

    

    def run(self, args):
        self.args = args
        rclpy.init()

        node = JointStateListener()
        nodes = [node]

        if args.ghost_opacity:
            ghost_node = GhostJointStateListener()
            nodes.append(ghost_node)
        else:
            ghost_node = None

        executor = MultiThreadedExecutor()
        for n in nodes:
            executor.add_node(n)

        # Spin in one thread
        threading.Thread(target=executor.spin, daemon=True).start()


        self.setup_scene()
        

        drive_apis, body_shin_drive_apis = self.configure_joints()
        # ghost_drive_apis, ghost_body_shin_drive_apis = None, None
        

        if args.ghost_opacity != 0:
            ghost_drive_apis, ghost_body_shin_drive_apis = self.configure_joints("/ghost_robot", "_ghost")

        try:
            while simulation_app.is_running():
                
                self.apply_joint_values(node, drive_apis, body_shin_drive_apis)
                if args.ghost_opacity != 0:
                    self.apply_joint_values(ghost_node, ghost_drive_apis, ghost_body_shin_drive_apis)

                

                self.world.step(render=True)
                simulation_app.update()

        except KeyboardInterrupt:
            print("Simulation interrupted.")

        simulation_app.close()
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()


    def apply_joint_values(self, _node, _drive_apis, _body_shin_drive_apis):
        drive_velocities = {
                    "FL": -(_node.fl_vel * 180./math.pi),  # isaak sim takes deg/s for angular joints
                    "FR": _node.fr_vel * 180./math.pi,
                    "RL": -(_node.rl_vel * 180./math.pi),  
                    "RR": _node.rr_vel * 180./math.pi
                }

        for key in _drive_apis:
            _drive_apis[key].GetTargetVelocityAttr().Set(drive_velocities[key])

        # Steering angles (degrees)
        steer_angles = {
            "FL": _node.fl * 180 / math.pi,
            "FR": _node.fr * 180 / math.pi,
            "RL": _node.rl * 180 / math.pi,
            "RR": _node.rr * 180 / math.pi,
        }

        for key in _body_shin_drive_apis:
            _body_shin_drive_apis[key].GetTargetPositionAttr().Set(steer_angles[key])


    def configure_joints(self, prefix="/agrorob_visualization", postfix=""):
        # WHEELS
        wheel_joints = {
                "FL": f"{prefix}/joints/shin_wheel_FL{postfix}",
                "FR": f"{prefix}/joints/shin_wheel_FR{postfix}",
                "RL": f"{prefix}/joints/shin_wheel_RL{postfix}",
                "RR": f"{prefix}/joints/shin_wheel_RR{postfix}",
            }
        stage = omni.usd.get_context().get_stage()
        drive_apis = {
            k: UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(v), "angular")
            for k, v in wheel_joints.items()
            if stage.GetPrimAtPath(v).IsValid()
        }

        for key in drive_apis:
            drive_apis[key].GetMaxForceAttr().Set(1e6) 
            drive_apis[key].GetDampingAttr().Set(1000.0)
            drive_apis[key].GetStiffnessAttr().Set(0.0)

        # Steering joints
        body_shin_joints = {
            "FL": f"{prefix}/joints/body_shin_FL{postfix}",
            "FR": f"{prefix}/joints/body_shin_FR{postfix}",
            "RL": f"{prefix}/joints/body_shin_RL{postfix}",
            "RR": f"{prefix}/joints/body_shin_RR{postfix}",
        }
        body_shin_drive_apis = {
            k: UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(v), "angular")
            for k, v in body_shin_joints.items()
            if stage.GetPrimAtPath(v).IsValid()
        }
        for key in body_shin_drive_apis:
            body_shin_drive_apis[key].GetDampingAttr().Set(100.0)
            body_shin_drive_apis[key].GetStiffnessAttr().Set(200.0)

        return drive_apis, body_shin_drive_apis

    


class JointStateListener(Node):
    def __init__(self):
        super().__init__("joint_state_listener")
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.listener_callback, 10
        )

        # Default values for steering and speed
        self.speed = 0.0
        self.fl = 0.0  # front left
        self.fr = 0.0  # front right
        self.rl = 0.0  # rear left
        self.rr = 0.0  # rear right

        self.fl_vel = 0.0
        self.fr_vel = 0.0
        self.rl_vel = 0.0
        self.rr_vel = 0.0


    def listener_callback(self, msg):
        joint_positions = dict(zip(msg.name, msg.position))
        joint_velocities = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}

        # Steering angles
        self.fl = joint_positions.get("front_left", joint_positions.get("front", 0.0))
        self.fr = joint_positions.get("front_right", joint_positions.get("front", 0.0))
        self.rl = joint_positions.get("rear_left", joint_positions.get("back", 0.0))
        self.rr = joint_positions.get("rear_right", joint_positions.get("back", 0.0))

        # Velocity (per-wheel or fallback to average)
        self.fl_vel = joint_velocities.get("front_left", 0.0)
        self.fr_vel = joint_velocities.get("front_right", 0.0)
        self.rl_vel = joint_velocities.get("rear_left", 0.0)
        self.rr_vel = joint_velocities.get("rear_right", 0.0)

        # You can store individual ones if needed later
        self.speed = (self.fl_vel + self.fr_vel + self.rl_vel + self.rr_vel) / 4.0

class GhostJointStateListener(Node):
    def __init__(self):
        super().__init__("ghost_joint_state_listener")
        self.subscription = self.create_subscription(
            RobotState, "/agrorob/robot_state", self.listener_callback, 10
        )

        self.fl = 0.0  # front left
        self.fr = 0.0  # front right
        self.rl = 0.0  # rear left
        self.rr = 0.0  # rear right

        self.fl_vel = 0.0
        self.fr_vel = 0.0
        self.rl_vel = 0.0
        self.rr_vel = 0.0
        

    def listener_callback(self, msg):
        self.fl = msg.left_front_wheel_turn_angle_rad
        self.fr = msg.right_front_wheel_turn_angle_rad
        self.rl = msg.left_rear_wheel_turn_angle_rad 
        self.rr = msg.right_rear_wheel_turn_angle_rad  

        # self.fl_vel = msg.left_front_wheel_rotational_speed_rad_s
        # self.fr_vel = msg.right_front_wheel_rotational_speed_rad_s
        # self.rl_vel = msg.left_rear_wheel_rotational_speed_rad_s
        # self.rr_vel = msg.right_rear_wheel_rotational_speed_rad_s

        self.fl_vel = self.impulses_to_rad_s(msg.left_front_wheel_encoder_imp)
        self.fr_vel = self.impulses_to_rad_s(msg.right_front_wheel_encoder_imp)
        self.rl_vel = self.impulses_to_rad_s(msg.left_rear_wheel_encoder_imp)
        self.rr_vel = self.impulses_to_rad_s(msg.right_rear_wheel_encoder_imp)

    def impulses_to_rad_s(self, impulses_scaled):
        impulses_per_sec = impulses_scaled / 100.0
        revolutions_per_sec = impulses_per_sec / 54.0
        rad_per_sec = revolutions_per_sec * 2 * math.pi
        return rad_per_sec






if __name__ == "__main__":

    app = URDFLoaderApp()
    app.run(args)
