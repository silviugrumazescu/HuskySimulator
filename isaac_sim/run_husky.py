"""Milestone 1: copy the existing Husky, then render it without ROS or motion.

Host:      python3 run_husky.py --export-from cpr-sim-jazzy
Container: /isaac-sim/python.sh /workspace/run_husky.py
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


def read_container(container, *command):
    """Read existing container files; never copy to or mutate that container."""
    return subprocess.check_output(['docker', 'exec', container, *command])


def export_robot(container, assets):
    """Copy only meshes referenced by the fully expanded, configured robot URDF."""
    urdf_path = assets / 'husky.urdf'
    if urdf_path.exists():
        raise FileExistsError(f'{urdf_path} already exists; export will not overwrite it.')
    raw = read_container(container, 'bash', '-c',
                         'source /opt/ros/jazzy/setup.bash && xacro /etc/clearpath/robot.urdf.xacro')
    robot = ET.fromstring(raw)
    copied = {}
    for mesh in robot.findall('.//mesh'):
        filename = mesh.attrib['filename']
        if filename not in copied:
            if filename.startswith('package://'):
                package, relative = filename[len('package://'):].split('/', 1)
                prefix = read_container(container, 'bash', '-c',
                    'source /opt/ros/jazzy/setup.bash && ros2 pkg prefix --share "$1"',
                    'bash', package).decode().strip()
                source = f'{prefix}/{relative}'
                target = assets / 'meshes' / package / relative
            elif filename.startswith('file:///opt/ros/jazzy/share/'):
                relative = filename[len('file:///opt/ros/jazzy/share/'):]
                source = filename[len('file://'):]
                target = assets / 'meshes' / relative
            else:
                raise ValueError(f'Unexpected mesh location: {filename}')
            data = read_container(container, 'cat', source)
            # These models embed materials. Abort rather than silently omit external textures.
            if source.endswith('.dae'):
                dae = ET.fromstring(data)
                if dae.findall('.//{*}library_images/{*}image/{*}init_from'):
                    raise ValueError(f'{source} needs external textures; inspect before exporting.')
                unnamed = [m for m in dae.findall('.//{*}library_materials/{*}material')
                           if not m.get('name')]
                if unnamed:
                    # The 6.0 converter requires names; the D435 mesh only supplies an ID.
                    for material in unnamed:
                        material.set('name', material.get('id'))
                    ET.register_namespace('', dae.tag.split('}')[0].lstrip('{'))
                    data = ET.tostring(dae, encoding='utf-8', xml_declaration=True)
            target.parent.mkdir(parents=True, exist_ok=True)
            with target.open('xb') as stream:
                stream.write(data)
            copied[filename] = target.relative_to(assets).as_posix()
            print(f'Copied {source} ({len(data):,} bytes)', flush=True)
        # The import sees /workspace/assets in the Isaac container.
        mesh.set('filename', f'/workspace/assets/{copied[filename]}')
    # These plugins belong to Gazebo/the real hardware, not to this static USD import.
    for element in list(robot):
        if element.tag in ('gazebo', 'ros2_control'):
            robot.remove(element)
    assets.mkdir(parents=True, exist_ok=True)
    with urdf_path.open('xb') as stream:
        stream.write(ET.tostring(robot, encoding='utf-8', xml_declaration=True))
    print(f'Exported {urdf_path}; {len(copied)} unique meshes. Source files were unchanged.')


def camera_pose_from_urdf(urdf, link_name):
    """Compose fixed joints to retain the configured camera's actual mounting pose."""
    import numpy as np
    joints = {j.find('child').get('link'): j for j in urdf.findall('joint')}
    transform = np.eye(4)
    chain = []
    while link_name in joints:
        joint = joints[link_name]
        if joint.get('type') != 'fixed':
            raise ValueError('The milestone camera must be attached through fixed joints.')
        chain.append(joint)
        link_name = joint.find('parent').get('link')
    for joint in reversed(chain):
        origin = joint.find('origin')
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0]*3
        r,p,y = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0]*3
        cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
        rotation = np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                             [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
        local = np.eye(4)
        local[:3,:3],local[:3,3] = rotation,xyz
        transform = transform @ local
    return transform


def capture(args):
    # Start Kit before importing USD, Replicator, or other Isaac extensions.
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'renderer': 'RealTimePathTracing',
                         'width': 640, 'height': 480})
    exit_code = 0
    try:
        import numpy as np
        import omni.usd
        import omni.replicator.core as rep
        from isaacsim.core.utils.extensions import enable_extension
        from isaacsim.storage.native import get_assets_root_path
        from pxr import Gf, Usd, UsdGeom, UsdLux
        from PIL import Image

        output = Path(args.output)
        output.mkdir(parents=True, exist_ok=False)
        enable_extension('isaacsim.asset.importer.urdf')
        app.update()
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
        print('Importing copied Husky URDF', flush=True)
        config = URDFImporterConfig(urdf_path=args.urdf, usd_path=str(output),
                                    merge_fixed_joints=True, merge_mesh=False,
                                    fix_base=True, collision_from_visuals=False)
        robot_usd = Path(URDFImporter(config).import_urdf())
        if not robot_usd.exists():
            raise RuntimeError(f'URDF import did not produce {robot_usd}')
        # The importer opens an intermediate stage; start our capture scene afterward.
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        world = UsdGeom.Xform.Define(stage, '/World')
        stage.SetDefaultPrim(world.GetPrim())

        print('Loading NVIDIA environment', flush=True)
        root = get_assets_root_path()
        if not root:
            raise RuntimeError('NVIDIA online asset root is unavailable.')
        environment = root + f'/Isaac/Environments/{args.environment}'
        env = UsdGeom.Xform.Define(stage, '/World/Environment')
        env.GetPrim().GetReferences().AddReference(environment)
        for _ in range(100):
            app.update()

        robot = UsdGeom.Xform.Define(stage, '/World/Husky')
        robot.GetPrim().GetReferences().AddReference(str(robot_usd))
        base_position = np.array([args.x, args.y, 0.13228])  # Existing URDF base_footprint offset.
        robot.AddTranslateOp().Set(Gf.Vec3d(*base_position))
        light = UsdLux.DomeLight.Define(stage, '/World/FillLight')
        light.CreateIntensityAttr(500.0)

        def make_camera(path, position, target, up):
            camera = UsdGeom.Camera.Define(stage, path)
            aperture = 20.955
            camera.CreateHorizontalApertureAttr(aperture)
            camera.CreateVerticalApertureAttr(aperture * 480 / 640)
            camera.CreateFocalLengthAttr(aperture / (2 * math.tan(1.25/2)))
            camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
            view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*position), Gf.Vec3d(*target), Gf.Vec3d(*up))
            UsdGeom.Xformable(camera).AddTransformOp().Set(view.GetInverse())
            product = rep.create.render_product(path, (640,480))
            annotator = rep.AnnotatorRegistry.get_annotator('rgb')
            annotator.attach(product)
            return annotator

        pose = camera_pose_from_urdf(ET.parse(args.urdf).getroot(), 'camera_0_link')
        camera_position = base_position + pose[:3,3]
        front = make_camera('/World/FrontCamera', camera_position,
                            camera_position + pose[:3,0], pose[:3,2])
        overview = make_camera('/World/OverviewCamera', base_position + [2.5,-2.5,1.8],
                               base_position + [0,0,0.2], [0,0,1])
        # Render only: no ROS extension, timeline play, controller, or action subscriber.
        print('Rendering two stationary camera views', flush=True)
        for _ in range(40):
            app.update()
        rep.orchestrator.step(rt_subframes=16, pause_timeline=True)
        meshes = [p for p in Usd.PrimRange(robot.GetPrim(), Usd.TraverseInstanceProxies())
                  if p.IsA(UsdGeom.Mesh)]
        if not meshes:
            raise RuntimeError('The robot reference contains no mesh geometry.')
        for label, annotator in [('front_camera', front), ('husky_overview', overview)]:
            pixels = annotator.get_data()
            if pixels.shape[:2] != (480,640) or not np.isfinite(pixels).all():
                raise RuntimeError(f'{label}: invalid camera array {pixels.shape}')
            if pixels[:,:,:3].std() < 2:
                raise RuntimeError(f'{label}: image appears blank.')
            Image.fromarray(pixels[:,:,:3].astype(np.uint8)).save(output / f'{label}.png')
        stage.GetRootLayer().Export(str(output / 'scene.usda'))
        metadata = dict(environment=environment, robot_import=str(robot_usd),
                        camera_position=camera_position.tolist(), resolution=[640,480],
                        horizontal_fov_rad=1.25, robot_mesh_count=len(meshes),
                        renderer='RealTimePathTracing', stationary=True, ros_enabled=False)
        (output / 'capture.json').write_text(json.dumps(metadata, indent=2))
        print('CAPTURE_COMPLETE', json.dumps(metadata), flush=True)
    except Exception:
        # Kit's shutdown can suppress an in-flight Python exception; print it first.
        import traceback
        exit_code = 1
        traceback.print_exc()
        raise
    finally:
        app.close(exit_code=exit_code)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--export-from', help='Existing Clearpath container; host-only export step.')
    parser.add_argument('--urdf', default='/workspace/assets/husky.urdf')
    parser.add_argument('--output', default='/output/capture')
    parser.add_argument('--environment', default='Office/office.usd')
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    args = parser.parse_args()
    if args.export_from:
        export_robot(args.export_from, Path(__file__).resolve().parent / 'assets')
    else:
        capture(args)


if __name__ == '__main__':
    main()
