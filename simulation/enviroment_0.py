import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from omni.isaac.orbit.envs import BaseEnvCfg
from omni.isaac.orbit.scene import InteractiveSceneCfg
from omni.isaac.orbit.utils import configclass

import omni.isaac.orbit.envs.mdp as mdp

# from vx300s import VX300S_CFG, BODY_JOINTS, GRIPPER_JOINTS

@configclass
class SceneConfig(InteractiveSceneCfg):

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )
    distant_light = AssetBaseCfg(
        prim_path="/World/DistantLight",
        spawn=sim_utils.DistantLightCfg(color=(0.9, 0.9, 0.9), intensity=2500.0),
        init_state=AssetBaseCfg.InitialStateCfg(rot=(0.738, 0.477, 0.477, 0.0)),
    )

    # table
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.CuboidCfg(
            size=(0.75, 1.5, 0.4),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=1000.0),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.35, 0.2))
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0, 0, 0.2)),
    )

    # # box 1
    # box_1 = RigidObjectCfg(
    #     prim_path="{ENV_REGEX_NS}/Box_1",
    #     spawn=sim_utils.CuboidCfg(
    #         size=(0.07, 0.07, 0.07),
    #         rigid_props=sim_utils.RigidBodyPropertiesCfg(),
    #         mass_props=sim_utils.MassPropertiesCfg(mass=0.250),
    #         collision_props=sim_utils.CollisionPropertiesCfg(),
    #         visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.25, 0.25, 0.0), metallic=0.2)
    #     ),
    #     init_state=RigidObjectCfg.InitialStateCfg(pos=(-0.1, 0, 0.45)),
    # )
    
    # # box 2
    # box_2 = RigidObjectCfg(
    #     prim_path="{ENV_REGEX_NS}/Box_2",
    #     spawn=sim_utils.CuboidCfg(
    #         size=(0.07, 0.07, 0.07),
    #         rigid_props=sim_utils.RigidBodyPropertiesCfg(),
    #         mass_props=sim_utils.MassPropertiesCfg(mass=0.250),
    #         collision_props=sim_utils.CollisionPropertiesCfg(),
    #         visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.25, 0.25), metallic=0.2)
    #     ),
    #     init_state=RigidObjectCfg.InitialStateCfg(pos=(0.1, 0, 0.45)),
    # )

    # # vx300s
    # robot_left: ArticulationCfg = VX300S_CFG.replace(prim_path="{ENV_REGEX_NS}/RobotLeft")
    # robot_right: ArticulationCfg = VX300S_CFG.replace(prim_path="{ENV_REGEX_NS}/RobotRight")

    # robot_left.init_state.pos = (0.0, -0.45, 0.4)
    # robot_right.init_state.pos = (0.0, 0.45, 0.4)

    # robot_left.init_state.rot = (0.707, 0.0, 0.0, 0.707)
    # robot_right.init_state.rot = (-0.707, 0.0, 0.0, 0.707)




@configclass
class ActionsCfg:
    joint_positions_left = mdp.JointPositionActionCfg(asset_name="robot_left", joint_names=BODY_JOINTS, use_default_offset=False)
    gripper_left_l = mdp.JointEffortActionCfg(asset_name="robot_left", joint_names=["left_finger"], offset=2.5, scale=-1)
    gripper_left_r = mdp.JointEffortActionCfg(asset_name="robot_left", joint_names=["right_finger"], offset=-2.5)

    joint_positions_right = mdp.JointPositionActionCfg(asset_name="robot_right", joint_names=BODY_JOINTS, use_default_offset=False)
    gripper_right_l = mdp.JointEffortActionCfg(asset_name="robot_right", joint_names=["left_finger"], offset=2.5, scale=-1)
    gripper_right_r = mdp.JointEffortActionCfg(asset_name="robot_right", joint_names=["right_finger"], offset=-2.5)

@configclass
class AlohaEnvCfg(BaseEnvCfg):
    scene: InteractiveSceneCfg = SceneConfig(num_envs=1, env_spacing=3.0, replicate_physics=True)
    actions = ActionsCfg()

    def __post_init__(self) -> None:
        self.decimation = 1
        self.viewer.eye = (8.0, 0.0, 5.0)
        self.sim.dt = 1 / 60.0