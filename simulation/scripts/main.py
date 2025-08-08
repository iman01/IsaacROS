from isaacsim import SimulationApp
from .config import load_config

def main(cfg_path="simulation/config/default.yaml"):
    cfg = load_config(cfg_path)

    sim = SimulationApp({"headless": cfg.headless})
    try:
        # enable extensions first
        import isaacsim.core.utils.extensions as extensions
        extensions.enable_extension("isaacsim.ros2.bridge")
        extensions.enable_extension("isaacsim.replicator.synthetic_recorder")
        extensions.enable_extension("omni.kit.window.script_editor")


        # late imports AFTER SimulationApp exists
        from isaacsim.core.api import World
        import omni.usd
        import omni.replicator.core as rep

        world = World(stage_units_in_meters=1.0)
        stage = omni.usd.get_context().get_stage()

        # build scene
        from .scene import build_world, load_crops, spawn_robot, spawn_ghost, setup_cameras, setup_collisions
        build_world(world, stage)
        load_crops(stage, cfg.crops_usdc, cfg.semantics_yaml)
        robot_path = spawn_robot(stage, cfg.robot_urdf)
        ghost_path = spawn_ghost(stage, cfg.ghost_urdf, cfg.ghost_opacity) if cfg.ghost_opacity > 0 else None
        if ghost_path:
            setup_collisions(stage, robot_path, ghost_path)
        cameras = setup_cameras(cfg.cameras)

        # joints + ROS
        from .control import configure_joints, apply_targets, rad_s_to_deg_s, rad_to_deg
        drive, steer = configure_joints(stage, prefix=robot_path, suffix="")
        ghost_drive = ghost_steer = None
        if ghost_path:
            ghost_drive, ghost_steer = configure_joints(stage, prefix=ghost_path, suffix="_ghost")

        import rclpy
        rclpy.init()
        from .ros_nodes import JointStateListener, GhostJointStateListener, RosRuntime
        nodes = [JointStateListener()]
        if ghost_path: nodes.append(GhostJointStateListener())
        ros = RosRuntime(nodes)
        ros.start()

        rep.orchestrator.set_capture_on_play(True)
        world.reset()
        while sim.is_running():
            n = nodes[0]
            wheel_deg_s = {"FL": -(rad_s_to_deg_s(n.fl_vel)),
                           "FR":   rad_s_to_deg_s(n.fr_vel),
                           "RL": -(rad_s_to_deg_s(n.rl_vel)),
                           "RR":   rad_s_to_deg_s(n.rr_vel)}
            steer_deg = {"FL": rad_to_deg(n.fl), "FR": rad_to_deg(n.fr),
                         "RL": rad_to_deg(n.rl), "RR": rad_to_deg(n.rr)}
            apply_targets(drive, steer, wheel_deg_s, steer_deg)

            if ghost_path:
                g = nodes[1]
                wheel_deg_s_g = {"FL": -(rad_s_to_deg_s(g.fl_vel)),
                                 "FR":   rad_s_to_deg_s(g.fr_vel),
                                 "RL": -(rad_s_to_deg_s(g.rl_vel)),
                                 "RR":   rad_s_to_deg_s(g.rr_vel)}
                steer_deg_g = {"FL": rad_to_deg(g.fl), "FR": rad_to_deg(g.fr),
                               "RL": rad_to_deg(g.rl), "RR": rad_to_deg(g.rr)}
                apply_targets(ghost_drive, ghost_steer, wheel_deg_s_g, steer_deg_g)

            world.step(render=True)
            sim.update()
    except Exception:
        import traceback; traceback.print_exc()
        raise
    finally:
        sim.close()