from isaacsim import SimulationApp

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
        from pxr import UsdPhysics, Gf, UsdGeom

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

        urdf_path = "/home/bobo/agrorob_ws/src/agrorob_visualization/urdf/agrorob_visualization.urdf"
        joint_prim_path = "/agrororob_visualization/joints/shin_wheel_FL"


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

        self.world.reset()

    def run(self):
        self.setup_scene()
        import omni.usd
        from pxr import UsdPhysics

        # Define joint paths for all four wheels
        wheel_joints = {
            "FL": "/agrorob_visualization/joints/shin_wheel_FL",
            "FR": "/agrorob_visualization/joints/shin_wheel_FR",
            "RL": "/agrorob_visualization/joints/shin_wheel_RL",
            "RR": "/agrorob_visualization/joints/shin_wheel_RR",
        }

        # Get drive APIs for all wheels
        stage = omni.usd.get_context().get_stage()
        drive_apis = {}
        for key, path in wheel_joints.items():
            joint_prim = stage.GetPrimAtPath(path)
            if joint_prim:
                drive_apis[key] = UsdPhysics.DriveAPI.Get(joint_prim, "angular")

        # Set velocities: right wheels positive, left wheels negative
        right_velocity = 50.0
        left_velocity = -right_velocity

        while self.simulation_app.is_running():
            if drive_apis.get("FR"):
                drive_apis["FR"].GetDampingAttr().Set(6000.0)
                drive_apis["FR"].GetStiffnessAttr().Set(0.0)
                drive_apis["FR"].GetTargetVelocityAttr().Set(right_velocity)
            if drive_apis.get("RR"):
                drive_apis["RR"].GetDampingAttr().Set(6000.0)
                drive_apis["RR"].GetStiffnessAttr().Set(0.0)
                drive_apis["RR"].GetTargetVelocityAttr().Set(right_velocity)
            if drive_apis.get("FL"):
                drive_apis["FL"].GetDampingAttr().Set(6000.0)
                drive_apis["FL"].GetStiffnessAttr().Set(0.0)
                drive_apis["FL"].GetTargetVelocityAttr().Set(left_velocity)
            if drive_apis.get("RL"):
                drive_apis["RL"].GetDampingAttr().Set(6000.0)
                drive_apis["RL"].GetStiffnessAttr().Set(0.0)
                drive_apis["RL"].GetTargetVelocityAttr().Set(left_velocity)

            self.world.step(render=True)
            self.simulation_app.update()
        self.simulation_app.close()

if __name__ == "__main__":
    URDFLoaderApp().run()