import math

def rad_to_deg(x): return x * 180.0 / math.pi
def rad_s_to_deg_s(x): return x * 180.0 / math.pi

def configure_joints(stage, prefix="/agrorob_visualization", suffix=""):
    from pxr import UsdPhysics
    wheels = { "FL": f"{prefix}/joints/shin_wheel_FL{suffix}",
               "FR": f"{prefix}/joints/shin_wheel_FR{suffix}",
               "RL": f"{prefix}/joints/shin_wheel_RL{suffix}",
               "RR": f"{prefix}/joints/shin_wheel_RR{suffix}" }
    steer  = { "FL": f"{prefix}/joints/body_shin_FL{suffix}",
               "FR": f"{prefix}/joints/body_shin_FR{suffix}",
               "RL": f"{prefix}/joints/body_shin_RL{suffix}",
               "RR": f"{prefix}/joints/body_shin_RR{suffix}" }
    
    d = {k: UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(v), "angular")
         for k,v in wheels.items() if stage.GetPrimAtPath(v).IsValid()}
    s = {k: UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(v), "angular")
         for k,v in steer.items() if stage.GetPrimAtPath(v).IsValid()}
    
    for k in d:
         d[k].GetMaxForceAttr().Set(1e6)
         d[k].GetDampingAttr().Set(1000.0)
         d[k].GetStiffnessAttr().Set(0.0)
    for k in s:
         s[k].GetDampingAttr().Set(100.0)
         s[k].GetStiffnessAttr().Set(200.0)
    return d, s

def apply_targets(drive, steer, wheel_deg_s: dict, steer_deg: dict):
    for k, api in drive.items():
        api.GetTargetVelocityAttr().Set(wheel_deg_s[k])
    for k, api in steer.items():
        api.GetTargetPositionAttr().Set(steer_deg[k])
