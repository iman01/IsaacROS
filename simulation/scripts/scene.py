from typing import List
from .config import CameraCfg

def build_world(world, stage):
    from isaacsim.core.api.objects import GroundPlane
    from pxr import Gf

    gp = GroundPlane("/World/defaultGroundPlane", size=100.0)

    create_textured_ground(stage, "simulation/world/dirt.png")


    l1 = stage.DefinePrim("/World/lightD1", "DistantLight")
    l1.GetAttribute("inputs:intensity").Set(1000.0)
    l1.GetAttribute("inputs:color").Set(Gf.Vec3f(0.75,0.75,0.75))

def create_textured_ground(stage, image_path: str, size=100.0, tiling=(100.0, 100.0),
                           plane_path="/World/TexturedPlane", mtl_path="/World/Materials/Ground"):
    from pxr import UsdGeom, UsdShade, Sdf, Gf, Vt

    # 1) Simple quad slightly above z=0 to avoid z-fighting
    mesh = UsdGeom.Mesh.Define(stage, plane_path)
    z = 0.01
    pts = Vt.Vec3fArray([
        Gf.Vec3f(-size, -size, z),
        Gf.Vec3f( size, -size, z),
        Gf.Vec3f( size,  size, z),
        Gf.Vec3f(-size,  size, z),
    ])
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))

    # 2) Base UVs (0..1); tiling handled by Transform2d node
    uvs = Vt.Vec2fArray([
        Gf.Vec2f(0.0, 0.0),
        Gf.Vec2f(1.0, 0.0),
        Gf.Vec2f(1.0, 1.0),
        Gf.Vec2f(0.0, 1.0),
    ])
    pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.Float2Array, UsdGeom.Tokens.vertex
    )
    pv.Set(uvs)

    # 3) Material graph: Primvar -> Transform2d(scale=tiling) -> UVTexture -> PreviewSurface
    mtl = UsdShade.Material.Define(stage, mtl_path)

    # Preview surface
    surf = UsdShade.Shader.Define(stage, mtl_path + "/PreviewSurface")
    surf.CreateIdAttr("UsdPreviewSurface")

    # Primvar reader (create OUTPUT and wire that)
    st_reader = UsdShade.Shader.Define(stage, mtl_path + "/StReader")
    st_reader.CreateIdAttr("UsdPrimvarReader_float2")
    st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    st_out = st_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    # UV tiling transform
    xform2d = UsdShade.Shader.Define(stage, mtl_path + "/UVTransform")
    xform2d.CreateIdAttr("UsdTransform2d")
    xform2d.CreateInput("scale", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*tiling))
    xform2d.CreateInput("translation", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(0.0, 0.0))
    xform2d.CreateInput("rotation", Sdf.ValueTypeNames.Float).Set(0.0)
    xform2d.CreateInput("in", Sdf.ValueTypeNames.Float2).ConnectToSource(st_out)
    xform_out = xform2d.CreateOutput("out", Sdf.ValueTypeNames.Float2)

    # Texture sampler
    tex = UsdShade.Shader.Define(stage, mtl_path + "/BaseColorTex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(image_path)
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(xform_out)
    tex_rgb_out = tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    # Connect texture rgb -> diffuseColor (connect to the OUTPUT, not the shader)
    diffuse_in = surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f)
    diffuse_in.ConnectToSource(tex_rgb_out)

    # Bind material to the mesh
    mtl.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(mesh).Bind(mtl)

    return plane_path



def load_crops(stage, usdc_path: str, semantics_yaml: str):
    import omni.kit.commands, omni.usd, yaml
    from pxr import UsdGeom, Gf
    omni.kit.commands.execute("CreateReference", path_to="/World/Crops",
                              asset_path=usdc_path, usd_context=omni.usd.get_context())
    xform = UsdGeom.Xform(stage.GetPrimAtPath("/World/Crops"))

    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, -1.0, 0.02))

    with open(semantics_yaml) as f:
        cfg = yaml.safe_load(f)


    from isaacsim.core.utils.semantics import add_update_semantics
    labels = {**{k:v.get("plant_type","unknown") for k,v in cfg.get("field",{}).get("beds",{}).items()},
              **{k:"weed" for k in cfg.get("field",{}).get("weeds",{}).keys()}}
    

    root = stage.GetPrimAtPath("/World/Crops")
    for child in root.GetChildren():
        name = child.GetName()
        if name in {"_materials","stones","ground"} or name not in labels:
            continue
        for sub in child.GetChildren():
            if sub.IsValid() and child.GetTypeName()=="Xform":
                add_update_semantics(sub, labels[name])

def spawn_robot(stage, urdf_path: str) -> str:
    import omni.kit.commands
    from isaacsim.asset.importer.urdf import _urdf
    from pxr import UsdGeom

    cfg = _urdf.ImportConfig()
    cfg.fix_base=False
    cfg.make_default_prim=True

    _, model = omni.kit.commands.execute("URDFParseFile", urdf_path=urdf_path, import_config=cfg)
    _, prim_path = omni.kit.commands.execute("URDFImportRobot", urdf_robot=model, import_config=cfg)

    prim = stage.GetPrimAtPath(prim_path)
    xform_api = prim.GetAttribute("xformOp:translate")
    if not xform_api:
        xform = UsdGeom.Xformable(prim)
        xform.AddTranslateOp().Set((0.0, 0.0, 2.2))
    else:
        xform_api.Set((0.0, 0.0, 2.2))
    return prim_path

def spawn_ghost(stage, urdf_path: str, opacity: float) -> str:
    import omni.kit.commands, omni.usd
    from isaacsim.asset.importer.urdf import _urdf
    from pxr import UsdGeom, UsdShade, Sdf
    
    cfg = _urdf.ImportConfig()
    cfg.make_default_prim=False
    cfg.fix_base=False
    cfg.density=0.0

    _, model = omni.kit.commands.execute("URDFParseFile", urdf_path=urdf_path, import_config=cfg)
    _, prim_path = omni.kit.commands.execute("URDFImportRobot", urdf_robot=model, import_config=cfg)

    ghost_path = "/ghost_robot"
    omni.kit.commands.execute("MovePrim", path_from=prim_path, path_to=ghost_path)
    prim = stage.GetPrimAtPath(ghost_path)     
    xform_api = prim.GetAttribute("xformOp:translate")
    if not xform_api:
        xform = UsdGeom.Xformable(prim)
        xform.AddTranslateOp().Set((0.0, 0.0, 2.2))
    else:
        xform_api.Set((0.0, 0.0, 2.2))
    # transparency

    import carb.settings
    carb.settings.get_settings().set("/rtx/raytracing/fractionalCutoutOpacity", True)
    looks = stage.GetPrimAtPath(ghost_path + "/Looks")
    if looks.IsValid():
        for m in looks.GetChildren():
            for child in m.GetChildren():
                if child.GetTypeName()=="Shader":
                    sh = UsdShade.Shader(child)
                    sh.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
                    sh.CreateInput("opacity_constant", Sdf.ValueTypeNames.Float).Set(opacity)
    return ghost_path

def setup_collisions(stage, robot_path: str, ghost_path: str):
    from pxr import UsdPhysics, Usd, Sdf

    def _mk(name):
        cg = UsdPhysics.CollisionGroup.Define(stage, f"/World/{name}")
        coll_api = Usd.CollectionAPI.Apply(cg.GetPrim(), UsdPhysics.Tokens.colliders)
        return cg, coll_api.CreateIncludesRel(), cg.CreateFilteredGroupsRel()
    

    a_i, b_i = _mk("CollisionGroupA")[1], _mk("CollisionGroupB")[1]
    a_f = stage.GetPrimAtPath("/World/CollisionGroupA").GetAttribute("physics:filteredGroups")
    b_f = stage.GetPrimAtPath("/World/CollisionGroupB").GetAttribute("physics:filteredGroups")
    # Simpler:
    UsdPhysics.CollisionGroup.Get(stage, "/World/CollisionGroupA").CreateFilteredGroupsRel().AddTarget(Sdf.Path("/World/CollisionGroupB"))
    UsdPhysics.CollisionGroup.Get(stage, "/World/CollisionGroupB").CreateFilteredGroupsRel().AddTarget(Sdf.Path("/World/CollisionGroupA"))
    a_i.AddTarget(robot_path); b_i.AddTarget(ghost_path)


def setup_cameras(cameras: List[CameraCfg]):
    import isaacsim.core.utils.numpy.rotations as rot
    import omni.replicator.core as rep
    import omni.syntheticdata._syntheticdata as sd
    import omni.graph.core as og
    import omni.kit.commands


    from isaacsim.sensors.camera import Camera

    created = {}
    for c in cameras:
        q = rot.euler_angles_to_quats(c.orientation_euler_deg, degrees=True)
        cam = Camera(prim_path=c.prim_path, position=c.position,
                     frequency=c.frequency, resolution=c.resolution, orientation=q)
        cam.initialize()


        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                    sd.SensorType.Rgb.name
                )
        writer = rep.writers.get(rv + "ROS2PublishImage")
        writer.initialize(frameId=c.prim_path.split("/")[-1], nodeNamespace="", queueSize=1, topicName=c.name+"_rgb")
        writer.attach([cam._render_product_path])

        gate = omni.syntheticdata.SyntheticData._get_node_path(rv + "IsaacSimulationGate", cam._render_product_path)

        og.Controller.attribute(gate + ".inputs:step").set(int(60 / c.frequency))
        created[c.name] = cam
    return created