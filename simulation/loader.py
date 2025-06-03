from isaacsim import SimulationApp


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import numpy as np



def ros_spin(node):
    rclpy.spin(node)

class URDFLoaderApp:
    def __init__(self):
        self.simulation_app = SimulationApp({"headless": False})
        self.world = None

    def setup_scene(self):
        import isaacsim.core.utils.stage as stage_utils
        import isaacsim.core.utils.prims as prim_utils
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import GroundPlane
        from isaacsim.asset.importer.urdf import _urdf
        import omni.kit.commands
        import omni.usd
        from pxr import UsdPhysics, Gf, UsdGeom,UsdShade, Sdf, UsdGeom

        # Create world and ground plane
        self.world = World(stage_units_in_meters=1.0)
        GroundPlane("/World/defaultGroundPlane", size=100.0)

        # Add a distant light
        stage = omni.usd.get_context().get_stage()
        light_prim = stage.DefinePrim("/World/lightDistant", "DistantLight")
        # Use the correct API to set attributes
        light_prim.GetAttribute("inputs:intensity").Set(3000.0)
        light_prim.GetAttribute("inputs:color").Set(Gf.Vec3f(0.75, 0.75, 0.75))

        import_config = _urdf.ImportConfig()
        import_config.convex_decomp = False
        import_config.fix_base = False
        import_config.make_default_prim = True
        import_config.self_collision = False
        import_config.distance_scale = 1
        import_config.density = 0.0

        urdf_path = "agrorob/agrorob_visualization.urdf"

        result, robot_model = omni.kit.commands.execute(
            "URDFParseFile",
            urdf_path=urdf_path,
            import_config=import_config
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

        # material_path = "/World/Looks/BlueMaterial"
        # material_body = UsdShade.Material.Define(stage, material_path)
        # shader = UsdShade.Shader.Define(stage, material_path + "/Shader")
        # shader.CreateIdAttr("UsdPreviewSurface")
        # shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.2, 1.0))  # Blue
        # shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
        # material_body.CreateSurfaceOutput().ConnectToSource(shader, "surface")
        # material_path = "/World/Looks/GreyMaterial"
        # material_wheels = UsdShade.Material.Define(stage, material_path)
        # shader = UsdShade.Shader.Define(stage, material_path + "/Shader")
        # shader.CreateIdAttr("UsdPreviewSurface")
        # shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.2, 1.0))  # Blue
        # shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
        # material_wheels.CreateSurfaceOutput().ConnectToSource(shader, "surface")

        # def bind_material_recursive(prim):
        #     if prim.IsA(UsdGeom.Gprim):
        #         UsdShade.MaterialBindingAPI(prim).Bind(material_body)
        #     for child in prim.GetChildren():
        #         bind_material_recursive(child)

        # bind_material_recursive(robot_prim)

        self.world.reset()

    def run(self):
        rclpy.init()
        node = JointStateListener()
        ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
        ros_thread.start()


        self.setup_scene()
        import omni.usd
        from pxr import UsdPhysics

        #WHEELS
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


        #ROTATION JOINTS
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
                body_shin_drive_apis[key] = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
        for key in ["FR", "RR", "FL", "RL"]:
            if key not in body_shin_drive_apis:
                print(f"Warning: Body_shin Drive API for {key} not found.")
                continue
            body_shin_drive_apis[key].GetDampingAttr().Set(100.0)
            body_shin_drive_apis[key].GetStiffnessAttr().Set(200.0)



        while self.simulation_app.is_running():

            velocity = node.speed * 250

            for key in ["FR", "RR", "FL", "RL"]:
                drive_apis[key].GetTargetVelocityAttr().Set(velocity if key in ["FR", "RR"] else -velocity)
                body_shin_drive_apis[key].GetTargetPositionAttr().Set(-node.front * 180 / 3.14 if key in ["FR", "FL"] else -node.back* 180 / 3.14)



            self.world.step(render=True)
            self.simulation_app.update()
        
        
        
        self.simulation_app.close()
        node.destroy_node()
        rclpy.shutdown()





class JointStateListener(Node):
    def __init__(self):
        super().__init__('joint_state_listener')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.speed = 0
        self.front = 0
        self.back = 0

    def listener_callback(self, msg):
        joint_positions = dict(zip(msg.name, msg.position))
        self.front  = joint_positions.get('front', 0)
        self.back = joint_positions.get('back', 0)

        #TODO make smooth movment 
        # self.front = float(np.unwrap([self.front, joint_positions.get('front', 0)])[-1])
        # self.back = float(np.unwrap([self.back, joint_positions.get('back', 0)])[-1])
        self.speed = joint_positions.get('speed', 0)








if __name__ == "__main__":
    URDFLoaderApp().run()