#!/usr/bin/env python3
from isaacsim import SimulationApp
import os, sys, threading, yaml, numpy as np

simulation_app = SimulationApp({"headless": "--headless" in sys.argv})

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import omni.usd
import omni.kit.commands
import omni.graph.core as og
import omni.replicator.core as rep

import omni.syntheticdata as syn
import omni.syntheticdata._syntheticdata as sd

from pxr import UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Vt

import isaacsim.core.utils.extensions as extensions
import isaacsim.core.utils.prims as prim_utils
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.core.api import World
from isaacsim.core.api.objects import GroundPlane
from isaacsim.sensors.camera import Camera
from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.utils.semantics import add_update_semantics

extensions.enable_extension("isaacsim.ros2.bridge")
extensions.enable_extension("isaacsim.replicator.synthetic_recorder")
extensions.enable_extension("omni.kit.window.script_editor")

def ros_spin(node):
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        print("[ROS2] node shut down")

class JointStateListener(Node):
    def __init__(self):
        super().__init__("joint_state_listener")
        self.subscription = self.create_subscription(JointState, "/joint_states",
                                                     self.listener_callback, 10)
        self.speed = 0.0
        self.fl = self.fr = self.rl = self.rr = 0.0

    def listener_callback(self, msg):
        jp = dict(zip(msg.name, msg.position))
        self.speed = jp.get("speed", 0.0)
        self.fl = jp.get("front_left",  jp.get("front", 0.0))
        self.fr = jp.get("front_right", jp.get("front", 0.0))
        self.rl = jp.get("rear_left",   jp.get("back", 0.0))
        self.rr = jp.get("rear_right",  jp.get("back", 0.0))

class URDFLoaderApp:
    def __init__(self):
        self.world = None
        self.cameras = {}

    def _create_ground(self):
        self.world = World(stage_units_in_meters=1.0)
        GroundPlane("/World/defaultGroundPlane", size=100.0)

        stage = omni.usd.get_context().get_stage()
        try:
            plane_path = "/World/TexturedPlane"
            material_path = "/World/Materials/TiledMaterial"
            texture_file_path = os.path.join(os.path.dirname(__file__), "dirt.png")
            if not os.path.isfile(texture_file_path):
                print(f"[ground] texture missing: {texture_file_path} (skipping)")
                return

            size = 100.0
            mesh = UsdGeom.Mesh.Define(stage, plane_path)
            points = Vt.Vec3fArray([
                Gf.Vec3f(-size, -size, 0.01),
                Gf.Vec3f( size, -size, 0.01),
                Gf.Vec3f( size,  size, 0.01),
                Gf.Vec3f(-size,  size, 0.01),
            ])
            mesh.CreatePointsAttr(points)
            mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
            mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))

            texture_scale = (100.0, 100.0)
            uvs = Vt.Vec2fArray([
                Gf.Vec2f(0.0, 0.0),
                Gf.Vec2f(texture_scale[0], 0.0),
                Gf.Vec2f(texture_scale[0], texture_scale[1]),
                Gf.Vec2f(0.0, texture_scale[1]),
            ])
            primvars_api = UsdGeom.PrimvarsAPI(mesh.GetPrim())
            st = primvars_api.CreatePrimvar("st", Sdf.ValueTypeNames.Float2Array,
                                            UsdGeom.Tokens.vertex)
            st.Set(uvs)

            material = UsdShade.Material.Define(stage, material_path)
            shader = UsdShade.Shader.Define(stage, material_path + "/Shader")
            shader.CreateIdAttr("UsdPreviewSurface")

            tex = UsdShade.Shader.Define(stage, material_path + "/Texture")
            tex.CreateIdAttr("UsdUVTexture")
            tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture_file_path)
            tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
            tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")

            reader = UsdShade.Shader.Define(stage, material_path + "/PrimVarReader")
            reader.CreateIdAttr("UsdPrimvarReader_float2")
            reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
                reader.ConnectableAPI(), "result"
            )

            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f
                               ).ConnectToSource(tex.ConnectableAPI(), "rgb")
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

            UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(material)
            print("[ground] textured plane created")
        except Exception as e:
            print(f"[ground] textured plane skipped: {e}")

    def _create_crops(self):
        stage = omni.usd.get_context().get_stage()
        usdc_path = "simulation/crops.usdc"
        if not os.path.isfile(usdc_path):
            print(f"[crops] {usdc_path} not found; skipping.")
            return
        prim_path = "/World/Crops"
        try:
            omni.kit.commands.execute(
                "CreateReference", path_to=prim_path, asset_path=usdc_path,
                usd_context=omni.usd.get_context()
            )
            xform = UsdGeom.Xform(stage.GetPrimAtPath(prim_path))
            xform.AddTranslateOp().Set(Gf.Vec3d(0.0, -1.0, 0.02))
            xform.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 0))
        except Exception as e:
            print(f"[crops] reference failed: {e}")
            return

        try:
            cfg_path = "simulation/agrorob_crops.yaml"
            if not os.path.isfile(cfg_path):
                print(f"[crops] {cfg_path} not found; skipping semantics.")
                return
            import yaml
            with open(cfg_path, "r") as f:
                config = yaml.safe_load(f)
            beds = (config.get("field", {}) or {}).get("beds", {})
            weeds = (config.get("field", {}) or {}).get("weeds", {})
            bed_labels = {n: d.get("plant_type", "unknown") for n, d in beds.items()}
            weed_labels = {n: "weed" for n in weeds.keys()}
            labels = {**bed_labels, **weed_labels}
            exclude = {"_materials", "stones", "ground"}

            root = stage.GetPrimAtPath(prim_path)
            if not root.IsValid():
                print(f"[crops] prim {prim_path} invalid; skip labeling.")
                return
            for child in root.GetChildren():
                name = child.GetName()
                if name in exclude:
                    continue
                label = labels.get(name)
                if not label:
                    print(f"[crops] unknown group '{name}', skipping")
                    continue
                for sub in child.GetChildren():
                    if sub.IsValid() and child.GetTypeName() == "Xform":
                        add_update_semantics(sub, label)
            print("[crops] semantics applied")
        except Exception as e:
            print(f"[crops] labeling skipped: {e}")

    def _create_robot(self):
        stage = omni.usd.get_context().get_stage()
        from isaacsim.asset.importer.urdf import _urdf
        import_config = _urdf.ImportConfig()
        import_config.convex_decomp = False
        import_config.fix_base = False
        import_config.make_default_prim = True
        import_config.self_collision = False
        import_config.distance_scale = 1
        import_config.density = 0.0

        urdf_path = "agrorob/agrorob_visualization.urdf"
        result, robot_model = omni.kit.commands.execute(
            "URDFParseFile", urdf_path=urdf_path, import_config=import_config
        )
        result, prim_path = omni.kit.commands.execute(
            "URDFImportRobot", urdf_robot=robot_model, import_config=import_config
        )
        robot_prim = prim_utils.get_prim_at_path(prim_path)
        if robot_prim:
            xform_api = robot_prim.GetAttribute("xformOp:translate")
            if not xform_api:
                x = UsdGeom.Xformable(robot_prim)
                x.AddTranslateOp().Set((0, 0, 2.2))
            else:
                xform_api.Set((0, 0, 2.2))
        print(f"[robot] imported at {prim_path}")

    def _configure_lights(self):
        stage = omni.usd.get_context().get_stage()
        light1 = stage.DefinePrim("/World/lightDistant1", "DistantLight")
        light1.GetAttribute("inputs:intensity").Set(1000.0)
        light1.GetAttribute("inputs:color").Set(Gf.Vec3f(0.75, 0.75, 0.75))
        light2 = stage.DefinePrim("/World/lightDistant2", "DistantLight")
        light2.GetAttribute("inputs:intensity").Set(1000.0)
        light2.GetAttribute("inputs:angle").Set(10)
        light2.GetAttribute("inputs:color").Set(Gf.Vec3f(0.75, 0.75, 0.75))

    def _configure_cameras(self):
        def publish_rgb(camera: Camera, cam_name: str, freq_hz: int):
            render_product = camera._render_product_path
            step_size = max(1, int(60 / max(1, freq_hz)))
            frame_id = camera.prim_path.split("/")[-1]
            topic_name = cam_name + "_rgb"
            rv = syn.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
            writer = rep.writers.get(rv + "ROS2PublishImage")
            writer.initialize(frameId=frame_id, nodeNamespace="", queueSize=1, topicName=topic_name)
            writer.attach([render_product])
            gate_path = syn.SyntheticData._get_node_path(rv + "IsaacSimulationGate", render_product)
            og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

        cam_cfgs = [
            {"name":"camera_front","prim_path":"/agrorob_visualization/base_link/camera_front",
             "position":np.array([1.45,0.0,2.0]),
             "orientation":rot_utils.euler_angles_to_quats(np.array([0,15,0]),degrees=True)},
            {"name":"camera_left","prim_path":"/agrorob_visualization/base_link/camera_left",
             "position":np.array([1.58,0.5,1.95]),
             "orientation":rot_utils.euler_angles_to_quats(np.array([0,90,0]),degrees=True)},
            {"name":"camera_right","prim_path":"/agrorob_visualization/base_link/camera_right",
             "position":np.array([1.58,-0.52,1.95]),
             "orientation":rot_utils.euler_angles_to_quats(np.array([0,90,0]),degrees=True)},
        ]
        for cfg in cam_cfgs:
            cam = Camera(prim_path=cfg["prim_path"], position=cfg["position"],
                         frequency=60, resolution=(256,256), orientation=cfg["orientation"])
            cam.initialize(); publish_rgb(cam, cfg["name"], 60)

    def setup_scene(self):
        rep.orchestrator.set_capture_on_play(True)
        self._configure_lights()
        self._create_ground()
        self._create_crops()
        self._create_robot()
        self._configure_cameras()
        self.world.reset()
        print("[scene] setup complete")

    def run(self):
        rclpy.init()
        node = JointStateListener()
        import threading
        ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
        ros_thread.start()

        self.setup_scene()

        stage = omni.usd.get_context().get_stage()
        wheel_joints = {
            "FL": "/agrorob_visualization/joints/shin_wheel_FL",
            "FR": "/agrorob_visualization/joints/shin_wheel_FR",
            "RL": "/agrorob_visualization/joints/shin_wheel_RL",
            "RR": "/agrorob_visualization/joints/shin_wheel_RR",
        }
        drive = {k: UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(v), "angular")
                 for k,v in wheel_joints.items() if stage.GetPrimAtPath(v).IsValid()}
        for k in drive: drive[k].GetDampingAttr().Set(6000.0); drive[k].GetStiffnessAttr().Set(0.0)

        steer_joints = {
            "FL": "/agrorob_visualization/joints/body_shin_FL",
            "FR": "/agrorob_visualization/joints/body_shin_FR",
            "RL": "/agrorob_visualization/joints/body_shin_RL",
            "RR": "/agrorob_visualization/joints/body_shin_RR",
        }
        steer = {k: UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(v), "angular")
                 for k,v in steer_joints.items() if stage.GetPrimAtPath(v).IsValid()}
        for k in steer: steer[k].GetDampingAttr().Set(100.0); steer[k].GetStiffnessAttr().Set(200.0)

        try:
            while simulation_app.is_running():
                vel = node.speed * 400.0
                for k in drive:
                    drive[k].GetTargetVelocityAttr().Set( vel if k in ["FR","RR"] else -vel )
                for k,deg in {
                    "FL": -node.fl*180.0/np.pi, "FR": -node.fr*180.0/np.pi,
                    "RL": -node.rl*180.0/np.pi, "RR": -node.rr*180.0/np.pi}.items():
                    steer[k].GetTargetPositionAttr().Set(deg)
                self.world.step(render=True)
        except KeyboardInterrupt:
            pass

        simulation_app.close(); node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    URDFLoaderApp().run()

