from dataclasses import dataclass
from typing import Tuple, List
import yaml

@dataclass
class CameraCfg:
    name: str
    prim_path: str
    position: Tuple[float,float,float]
    orientation_euler_deg: Tuple[float,float,float]
    resolution: Tuple[int,int]=(256,256)
    frequency: int=60

@dataclass
class SimCfg:
    headless: bool
    robot_urdf: str
    ghost_urdf: str | None
    ghost_opacity: float
    crops_usdc: str
    semantics_yaml: str
    cameras: List[CameraCfg]

def load_config(path: str) -> SimCfg:
    with open(path) as f:
        raw = yaml.safe_load(f)
    cams = [CameraCfg(**c) for c in raw["cameras"]]
    return SimCfg(
        headless=raw.get("headless", False),
        robot_urdf=raw["robot_urdf"],
        ghost_urdf=raw.get("ghost_urdf"),
        ghost_opacity=float(raw.get("ghost_opacity", 0)),
        crops_usdc=raw["crops_usdc"],
        semantics_yaml=raw["semantics_yaml"],
        cameras=cams,
    )