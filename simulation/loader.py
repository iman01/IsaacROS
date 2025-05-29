from isaacsim import SimulationApp

class URDFLoaderApp:
    def __init__(self):
        self.simulation_app = SimulationApp({"headless": False})
        self.world = None

    def setup_scene(self):
        import isaacsim.core.utils.stage as stage_utils
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import GroundPlane
        from isaacsim.asset.importer.urdf import _urdf
        import omni.kit.commands

        self.world = World(stage_units_in_meters=1.0)
        ground = GroundPlane("/World/GroundPlane", size=100.0)

        import_config = _urdf.ImportConfig()
        import_config.convex_decomp = False
        import_config.fix_base = False
        import_config.make_default_prim = True
        import_config.self_collision = False
        import_config.distance_scale = 1
        import_config.density = 0.0

        urdf_path = "/home/bobo/agrorob_ws/src/agrorob_visualization/urdf/agrorob_visualization.urdf"

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

        self.world.reset()

    def run(self):
        self.setup_scene()
        while self.simulation_app.is_running():
            self.world.step(render=True)
            self.simulation_app.update()
        self.simulation_app.close()

if __name__ == "__main__":
    URDFLoaderApp().run()