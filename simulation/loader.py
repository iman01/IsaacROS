from isaacsim import SimulationApp
import os

import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
from sensor_msgs.msg import Image
import numpy as np


simulation_app = SimulationApp({"headless": "--headless" in sys.argv})


import isaacsim.core.utils.stage as stage_utils
import isaacsim.core.utils.prims as prim_utils
import isaacsim.core.utils.extensions as extensions
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.core.api import World
from isaacsim.core.api.objects import GroundPlane



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

extensions.enable_extension("isaacsim.ros2.bridge")
extensions.enable_extension("isaacsim.replicator.synthetic_recorder")
extensions.enable_extension("omni.kit.window.script_editor")


from pxr import UsdShade, Sdf, UsdGeom, Gf, Vt

import omni.usd
from pxr import UsdPhysics
import isaacsim.core.utils.prims as prim_utils
from isaacsim.sensors.camera import Camera


def ros_spin(node):
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        print("ROS2 node has been shut down.")


class URDFLoaderApp:
    def __init__(self):
        self.world = None

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
            usdc_path = "cropcraft/crops.usdc"

            prim_path = "/World/Crops"

            stage = omni.usd.get_context().get_stage()

            omni.kit.commands.execute(
                "CreateReference",
                path_to=prim_path,
                asset_path=usdc_path,
                usd_context=omni.usd.get_context()
            )   

            xform = UsdGeom.Xform(stage.GetPrimAtPath(prim_path))
            xform.AddTranslateOp().Set(Gf.Vec3d(0.0, -1.0, 0.0))  # Position
            xform.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 0))        # Orientation

            root_prim = stage.GetPrimAtPath(prim_path)



            if not root_prim.IsValid():
                print(f"[ERROR] Prim '{prim_path}' not found.")
                return

            current_class = "crop"

            
            for child in root_prim.GetChildren():
                name = child.GetName()
                if name == "_materials" or name == "stones":
                    continue  
                elif name.startswith("bed"):
                    current_class = "maize"
                elif name == "taraxacum" or name == "polygonum" or name == "portulaca":
                    current_class = "weed"
                else:
                    print(f"[WARNING] UNKNOWN OBJECT FOUND: {name}")
                    continue

                if not child.IsValid():
                    continue

                print(f"[INFO] Labeling bed group: {child.GetPath()}")

                for sub in child.GetChildren():
                    if sub.IsValid():
                        
                        add_update_semantics(sub, current_class)



        def create_robot():
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
        configure_cameras()


        self.world.reset()

    def run(self):
        rclpy.init()
        node = JointStateListener()
        ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
        ros_thread.start()




        self.setup_scene()
        

        # WHEELS
        wheel_joints = {
            "FL": "/agrorob_visualization/joints/shin_wheel_FL",
            "FR": "/agrorob_visualization/joints/shin_wheel_FR",
            "RL": "/agrorob_visualization/joints/shin_wheel_RL",
            "RR": "/agrorob_visualization/joints/shin_wheel_RR",
        }
        stage = omni.usd.get_context().get_stage()
        drive_apis = {}
        for key, path in wheel_joints.items():
            joint_prim = stage.GetPrimAtPath(path)
            if joint_prim:
                drive_apis[key] = UsdPhysics.DriveAPI.Get(joint_prim, "angular")

        for key in ["FR", "RR", "FL", "RL"]:
            if key not in drive_apis:
                print(f"Warning: Drive API for {key} not found.")
                continue
            drive_apis[key].GetDampingAttr().Set(6000.0)
            drive_apis[key].GetStiffnessAttr().Set(0.0)

        # ROTATION JOINTS
        body_shin_joints = {
            "FL": "/agrorob_visualization/joints/body_shin_FL",
            "FR": "/agrorob_visualization/joints/body_shin_FR",
            "RL": "/agrorob_visualization/joints/body_shin_RL",
            "RR": "/agrorob_visualization/joints/body_shin_RR",
        }
        body_shin_drive_apis = {}
        for key, path in body_shin_joints.items():
            joint_prim = stage.GetPrimAtPath(path)
            if joint_prim:
                body_shin_drive_apis[key] = UsdPhysics.DriveAPI.Get(
                    joint_prim, "angular"
                )
        for key in ["FR", "RR", "FL", "RL"]:
            if key not in body_shin_drive_apis:
                print(f"Warning: Body_shin Drive API for {key} not found.")
                continue
            body_shin_drive_apis[key].GetDampingAttr().Set(100.0)
            body_shin_drive_apis[key].GetStiffnessAttr().Set(200.0)

        try:
            # Main simulation loop
            while simulation_app.is_running():

                velocity = node.speed * 400

                for key in ["FR", "RR", "FL", "RL"]:
                    drive_apis[key].GetTargetVelocityAttr().Set(
                        velocity if key in ["FR", "RR"] else -velocity
                    )
                    body_shin_drive_apis[key].GetTargetPositionAttr().Set(
                        -node.front * 180 / 3.14
                        if key in ["FR", "FL"]
                        else -node.back * 180 / 3.14
                    )

                self.world.step(render=True)
                simulation_app.update()
        except KeyboardInterrupt:
            print("Simulation interrupted by user.")

        simulation_app.close()
        node.destroy_node()
        rclpy.shutdown()


class JointStateListener(Node):
    def __init__(self):
        super().__init__("joint_state_listener")
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

        self.speed = 0
        self.front = 0
        self.back = 0

    def listener_callback(self, msg):
        joint_positions = dict(zip(msg.name, msg.position))
        self.front = joint_positions.get("front", 0)
        self.back = joint_positions.get("back", 0)
        self.speed = joint_positions.get("speed", 0)


if __name__ == "__main__":
    URDFLoaderApp().run()
