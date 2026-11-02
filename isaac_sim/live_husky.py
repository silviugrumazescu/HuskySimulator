"""Interactive Husky: PhysX wheel drives, local browser controls, and ROS 2.

Run with --test --no-stream for a finite physical driving check on a flat floor.
The simulation stays in ROS domain 84 by default, separate from Gazebo.
"""
import argparse
import base64
from concurrent.futures import Future
from dataclasses import asdict
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import socket
from pathlib import Path
import queue
import threading
import time
from urllib.parse import urlparse, parse_qs

from husky_controller import ControlConfig, HuskyController, KinematicController, WHEELS
from run_husky import camera_pose_from_urdf
from scenario_manager import ScenarioManager, spawn_props


def start_controls(controller, state, port, scenarios=None, camera_jobs=None):
    """HTTP threads only enqueue velocity requests; the simulation thread owns PhysX."""
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, code, data, content_type='application/json'):
            data = data if isinstance(data, bytes) else json.dumps(data).encode()
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = urlparse(self.path).path
            if path == '/':
                self.reply(200, Path('/workspace/drive.html').read_bytes(), 'text/html; charset=utf-8')
            elif path == '/api/state':
                self.reply(200, dict(state, controller=controller.state(),
                                    scenario=scenarios.status() if scenarios else None))
            elif scenarios and path == '/api/scenarios':
                try:
                    self.reply(200, scenarios.library.read())
                except Exception as exc:
                    self.reply(400, dict(error=str(exc)))
            elif scenarios and path == '/api/scenarios/video-frame':
                # Image, pose and timestamp are captured together on the sim thread.
                # The batch recorder is CPU-only and never changes a command.
                with scenarios.lock:
                    frame = getattr(scenarios, 'video_frame', None)
                    if frame is None:
                        return self.reply(503, dict(error='Waiting for a camera frame'))
                    data = dict(frame, scenario=scenarios.status(), camera=state.get('camera'),
                                controller=controller.state(), prediction={k:scenarios.model.get(k) for k in
                                ('trajectory_m','trajectory_origin','prediction_epoch','mode',
                                 'model_path','resume_step','inference_seconds','prediction_seq')})
                self.reply(200, data)
            elif scenarios and path in ('/api/scenarios/camera.jpg', '/api/scenarios/goal.jpg'):
                data = getattr(scenarios, 'current_jpeg', None) if path.endswith('camera.jpg') else scenarios.goal_jpeg
                self.reply(200, data, 'image/jpeg') if data else self.reply(404, dict(error='No image yet'))
            elif scenarios and path in ('/api/scenarios/results','/api/scenarios/results.csv','/api/scenarios/run','/api/scenarios/run-image'):
                try:
                    query = parse_qs(urlparse(self.path).query)
                    if path.endswith('results.csv'):
                        self.reply(200, scenarios.results_csv(), 'text/csv; charset=utf-8')
                    elif path.endswith('/results'):
                        self.reply(200, dict(results=scenarios.results()))
                    elif path.endswith('/run'):
                        self.reply(200, scenarios.result_detail(query.get('id',[''])[0]))
                    else:
                        name = query.get('name',[''])[0]
                        if name not in ('start.jpg','end.jpg','goal.jpg'):
                            raise ValueError('Unknown recording image')
                        folder = scenarios.result_folder(query.get('id',[''])[0])
                        self.reply(200, (folder/name).read_bytes(), 'image/jpeg')
                except Exception as exc:
                    self.reply(400, dict(error=str(exc)))
            else:
                self.reply(404, dict(error='Unknown path'))

        def do_POST(self):
            if self.path not in ('/api/command', '/api/trajectory', '/api/camera') and not self.path.startswith('/api/scenarios/'):
                return self.reply(404, dict(error='Unknown path'))
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= (5*1024*1024 if self.path.startswith('/api/scenarios/') else 32768):
                    raise ValueError('Invalid request size')
                command = json.loads(self.rfile.read(length))
                if not isinstance(command, dict):
                    raise ValueError('Command must be a JSON object')
                if self.path == '/api/camera':
                    if not state.get('ready') or camera_jobs is None:
                        raise ValueError('Wait for the simulator to be ready')
                    future = Future()
                    camera_jobs.put((command, future))
                    return self.reply(200, dict(ok=True, camera=future.result(timeout=5)))
                if scenarios and self.path.startswith('/api/scenarios/'):
                    action = self.path.rsplit('/',1)[-1]
                    if action == 'save':
                        result = scenarios.library.save(command['scenario'])
                        self.reply(200, dict(ok=True, scenario=result))
                    elif action == 'image':
                        file = scenarios.library.save_image(command['base64'])
                        self.reply(200, dict(ok=True, file=file))
                    elif action == 'review':
                        self.reply(200, dict(ok=True, review=scenarios.review(command)))
                    else:
                        scenarios.request(action, command)
                        self.reply(202, dict(ok=True))
                    return
                if not isinstance(command.get('enabled'), bool):
                    raise ValueError('enabled must be boolean')
                enabled = command['enabled'] and state.get('ready', False) and state.get('playing', False)
                if self.path == '/api/trajectory':
                    if not isinstance(controller, KinematicController):
                        return self.reply(409, dict(error='Trajectory execution requires kinematic mode'))
                    if scenarios and not scenarios.accepts_trajectory(command):
                        return self.reply(200, dict(ok=True, ignored=True))
                    controller.follow_trajectory(dict(command, enabled=enabled), time.monotonic())
                    return self.reply(200, dict(ok=True))
                if scenarios:
                    if enabled:
                        scenarios.manual_command()
                    elif scenarios.phase == 'running':
                        return self.reply(200, dict(ok=True))
                controller.command(float(command['linear']), float(command['angular']),
                                   time.monotonic(), enabled)
                self.reply(200, dict(ok=True))
            except Exception as exc:
                if self.path in ('/api/command', '/api/trajectory'):
                    controller.stop()
                self.reply(400, dict(error=str(exc)))
    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def check_driving(samples):
    """Check measured body motion, rather than accepting wheel targets as motion."""
    end = {phase: [s for s in samples if s['phase'] == phase][-1]
           for phase in ('settle', 'forward', 'stop', 'reverse', 'turn', 'timeout')}
    checks = {
        'forward_motion': end['forward']['x']-end['settle']['x'] > 0.3,
        'reverse_motion': end['reverse']['x']-end['stop']['x'] < -0.3,
        'forward_speed': abs(end['forward']['forward_speed']-0.3) < 0.08,
        'normal_stop': abs(end['stop']['forward_speed']) < 0.05,
        'left_turn': end['turn']['yaw']-end['reverse']['yaw'] > 0.1,
        'timeout_disables_drive': not end['timeout']['controller']['enabled'],
        'timeout_brakes_body': abs(end['timeout']['yaw_rate']) < 0.05,
        'ground_contact': all(0.08 < s['z'] < 0.25 for s in samples),
    }
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-stream', action='store_true')
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--output', help='Fresh output directory; defaults to a timestamped session.')
    parser.add_argument('--domain-id', type=int, default=84)
    parser.add_argument('--port', type=int, default=8211)
    parser.add_argument('--scenarios', default='/scenarios/warehouse.yaml')
    parser.add_argument('--drive-mode', choices=('physics', 'kinematic'),
                        default=os.environ.get('ISAAC_DRIVE_MODE', 'physics'),
                        help='physics: wheel forces; kinematic: ideal planar body motion, retaining collisions.')
    parser.add_argument('--stream-host', default=os.environ.get('ISAAC_STREAM_HOST', 'auto'),
                        help='IPv4 address advertised for WebRTC; auto uses the default-route interface.')
    parser.add_argument('--environment', default=os.environ.get('ISAAC_ENVIRONMENT', 'Office/office.usd'),
                        help='USD path relative to the Isaac/Environments asset directory.')
    parser.add_argument('--spawn', default=os.environ.get('ISAAC_SPAWN', 'auto'),
                        help='Robot x,y,yaw_deg; auto selects a start pose for the chosen map.')
    parser.add_argument('--fill-light-intensity', type=float,
                        default=os.environ.get('ISAAC_FILL_LIGHT', '500'),
                        help='Added dome-light intensity; 0 disables the extra fill light.')
    parser.add_argument('--camera-hfov-deg', type=float,
                        default=os.environ.get('ISAAC_CAMERA_HFOV_DEG', str(math.degrees(1.25))),
                        help='RGB camera horizontal field of view in degrees.')
    parser.add_argument('--camera-offset', default=os.environ.get('ISAAC_CAMERA_OFFSET', '0,0,0'),
                        help='Virtual camera XYZ offset from the URDF mount, in base-frame metres.')
    parser.add_argument('--wheel-torque', type=float, default=20.0, help='Assumed per-wheel torque limit (Nm).')
    parser.add_argument('--wheel-damping', type=float, default=20.0, help='Assumed velocity-loop gain (Nm s/rad).')
    parser.add_argument('--friction', type=float, default=0.7, help='Assumed tyre friction coefficient.')
    args = parser.parse_args()
    stream_host = args.stream_host
    if stream_host == 'auto':
        # UDP connect selects a route without sending a packet to this reserved address.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as route:
            route.connect(('192.0.2.1', 9))
            stream_host = route.getsockname()[0]
    if any(not math.isfinite(v) or v <= 0 for v in (args.wheel_torque, args.wheel_damping, args.friction)):
        parser.error('Torque, damping, and friction must be finite positive values.')
    if not math.isfinite(args.fill_light_intensity) or args.fill_light_intensity < 0:
        parser.error('Fill-light intensity must be finite and nonnegative.')
    if not math.isfinite(args.camera_hfov_deg) or not 0 < args.camera_hfov_deg < 180:
        parser.error('Camera horizontal FOV must be finite and between 0 and 180 degrees.')
    try:
        camera_offset = [float(v) for v in args.camera_offset.split(',')]
        if len(camera_offset) != 3 or not all(math.isfinite(v) and abs(v) <= 1 for v in camera_offset):
            raise ValueError()
    except ValueError:
        parser.error('Camera offset must be three finite XYZ values within ±1 metre.')
    spawn = args.spawn
    if spawn == 'auto':
        # Point into each warehouse instead of at the wall near its origin.
        starts = {'Digital_Twin_Warehouse/small_warehouse_digital_twin.usd': '-3,1,-90',
                  'Simple_Warehouse/full_warehouse.usd': '0,0,90'}
        spawn = '0,0,0' if args.test else starts.get(args.environment, '0,0,0')
    try:
        spawn_x, spawn_y, spawn_yaw_deg = map(float, spawn.split(','))
    except ValueError:
        parser.error('Spawn must be auto or three comma-separated numbers: x,y,yaw_deg.')
    if not all(math.isfinite(v) for v in (spawn_x, spawn_y, spawn_yaw_deg)):
        parser.error('Spawn values must be finite.')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    control_config = ControlConfig.from_clearpath(Path('/workspace/assets/control.yaml'))
    kinematic = args.drive_mode == 'kinematic'
    controller = (KinematicController(control_config, (spawn_x,spawn_y,math.radians(spawn_yaw_deg)))
                  if kinematic else HuskyController(control_config))
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'hide_ui': False, 'renderer': 'RealTimePathTracing',
                         'width': 1280, 'height': 720, 'window_width': 1280, 'window_height': 720})
    exit_code, server, node = 0, None, None
    state = dict(ready=False, playing=False, stream_host=stream_host, drive_mode=args.drive_mode,
                 x=spawn_x, y=spawn_y, yaw=math.radians(spawn_yaw_deg))
    camera_jobs = queue.Queue()
    scenarios = (None if args.test else ScenarioManager(args.scenarios, '/output/scenarios',
                  controller, state, args.environment))
    try:
        import numpy as np
        import omni.usd
        import omni.replicator.core as rep
        import xml.etree.ElementTree as ET
        from isaacsim.core.experimental.utils.app import enable_extension
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.core.api import World
        from isaacsim.storage.native import get_assets_root_path
        from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, PhysxSchema
        from PIL import Image

        if not args.no_stream:
            app.set_setting('/app/livestream/allowResize', True)
            app.set_setting('/app/window/drawMouse', True)
            app.set_setting('/exts/omni.kit.livestream.app/primaryStream/streamType', 'webrtc')
            app.set_setting('/exts/omni.kit.livestream.app/primaryStream/allowDynamicResize', True)
            app.set_setting('/exts/omni.kit.livestream.app/primaryStream/publicIp', stream_host)
            app.set_setting('/exts/omni.kit.livestream.app/primaryStream/signalPort', 49100)
            app.set_setting('/exts/omni.kit.livestream.app/primaryStream/streamPort', 47998)
            app.set_setting('/exts/omni.services.livestream.session/quitOnSessionEnded', False)
            enable_extension('omni.kit.livestream.app')
        server = start_controls(controller, state, args.port, scenarios, camera_jobs)

        output = Path(args.output or '/output/live-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
        output.mkdir(parents=True, exist_ok=False)
        enable_extension('isaacsim.asset.importer.urdf')
        app.update()
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
        config = URDFImporterConfig(urdf_path='/workspace/assets/husky.urdf', usd_path=str(output),
                    merge_fixed_joints=True, fix_base=False, collision_from_visuals=False,
                    joint_drive_type='force', joint_target_type='velocity',
                    override_joint_stiffness=0.0, override_joint_damping=args.wheel_damping)
        robot_usd = URDFImporter(config).import_urdf()
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        world_prim = UsdGeom.Xform.Define(stage, '/World')
        stage.SetDefaultPrim(world_prim.GetPrim())
        world = World(physics_dt=1/120, rendering_dt=1/30, stage_units_in_meters=1.0)
        if args.test:
            world.scene.add_default_ground_plane()
        else:
            env = UsdGeom.Xform.Define(stage, '/World/Environment')
            env.GetPrim().GetReferences().AddReference(
                get_assets_root_path()+'/Isaac/Environments/'+args.environment)
        robot = UsdGeom.Xform.Define(stage, '/World/Husky')
        robot.GetPrim().GetReferences().AddReference(robot_usd)
        robot_translation = robot.AddTranslateOp()
        robot_translation.Set(Gf.Vec3d(spawn_x,spawn_y,0.13228 if kinematic else 0.16))
        robot_rotation = robot.AddRotateZOp()
        robot_rotation.Set(spawn_yaw_deg)
        light = UsdLux.DomeLight.Define(stage, '/World/FillLight')
        light.CreateIntensityAttr(args.fill_light_intensity)

        # Keep the imported mass, inertia, four wheel joints, and primitive colliders.
        # Drive force is bounded: the solver must produce traction and tyre scrub.
        joints = {p.GetName(): p for p in Usd.PrimRange(robot.GetPrim()) if p.IsA(UsdPhysics.RevoluteJoint)}
        for name in WHEELS:
            drive = UsdPhysics.DriveAPI.Apply(joints[name], 'angular')
            drive.CreateTypeAttr('force')
            drive.CreateStiffnessAttr(0.0)
            # Angular USD drive gains use degrees; our CLI uses Nm s/rad.
            drive.CreateDampingAttr(0.0 if kinematic else math.radians(args.wheel_damping))
            drive.CreateMaxForceAttr(0.0 if kinematic else args.wheel_torque)
            drive.CreateTargetVelocityAttr(0.0)
        material = stage.DefinePrim('/World/TyreMaterial', 'Material')
        physics_material = UsdPhysics.MaterialAPI.Apply(material)
        physics_material.CreateStaticFrictionAttr(0.0 if kinematic else args.friction)
        physics_material.CreateDynamicFrictionAttr(0.0 if kinematic else args.friction)
        physics_material.CreateRestitutionAttr(0.0)
        if kinematic:
            PhysxSchema.PhysxMaterialAPI.Apply(material).CreateFrictionCombineModeAttr('min')
        from pxr import UsdShade
        for prim in Usd.PrimRange(robot.GetPrim()):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
            if kinematic and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                body.CreateDisableGravityAttr(True)
                body.CreateLinearDampingAttr(0.0)
                body.CreateAngularDampingAttr(0.0)
            if prim.IsA(UsdGeom.Cylinder) and prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material(material),
                                        UsdShade.Tokens.weakerThanDescendants, 'physics')
        roots = [p for p in Usd.PrimRange(robot.GetPrim()) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        for prim in roots:
            tuning = PhysxSchema.PhysxArticulationAPI.Apply(prim)
            tuning.CreateSolverPositionIterationCountAttr(16)
            tuning.CreateSolverVelocityIterationCountAttr(4)

        # PhysX authors local link poses as the robot moves. Keep the imported
        # rest values so a reset restores both the parent and its dynamic links.
        rest_link_ops = [(op.GetAttr(),op.Get()) for prim in Usd.PrimRange(robot.GetPrim())
                         if prim.HasAPI(UsdPhysics.RigidBodyAPI)
                         for op in UsdGeom.Xformable(prim).GetOrderedXformOps()]

        # Retain the URDF orientation; the optional translation is an explicit
        # virtual-camera experiment relative to the configured hardware mount.
        pose = camera_pose_from_urdf(ET.parse('/workspace/assets/husky.urdf').getroot(), 'camera_0_link')
        urdf_camera_position = pose[:3,3].copy()
        pose[:3,3] += camera_offset
        camera_path = '/World/Husky/Geometry/base_link/FrontCamera'
        camera = UsdGeom.Camera.Define(stage, camera_path)
        camera.CreateHorizontalApertureAttr(20.955)
        camera.CreateVerticalApertureAttr(20.955*480/640)
        camera.CreateFocalLengthAttr(20.955/(2*math.tan(math.radians(args.camera_hfov_deg)/2)))
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.05,100))
        matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*pose[:3,3]),
                       Gf.Vec3d(*(pose[:3,3]+pose[:3,0])), Gf.Vec3d(*pose[:3,2])).GetInverse()
        camera_transform = UsdGeom.Xformable(camera).AddTransformOp()
        camera_transform.Set(matrix)

        def camera_settings():
            local = UsdGeom.Xformable(camera).GetLocalTransformation()
            position = [float(v) for v in local.ExtractTranslation()]
            aperture, focal = camera.GetHorizontalApertureAttr().Get(), camera.GetFocalLengthAttr().Get()
            return dict(horizontal_fov_deg=math.degrees(2*math.atan(aperture/(2*focal))),
                        resolution=[640,480], position_in_base_m=position,
                        urdf_position_in_base_m=urdf_camera_position.tolist(),
                        offset_m=[p-float(q) for p,q in zip(position,urdf_camera_position)])

        def update_camera(command):
            # Only called on the simulation thread; native camera edits are
            # read back too, so the panel and trial metadata describe the view.
            if controller.state()['enabled'] or (scenarios and
                    (scenarios.run is not None or scenarios.phase in ('loading','starting'))):
                raise ValueError('Stop motion and finish the current trial/load before changing the camera')
            offset = command.get('offset_m', camera_settings()['offset_m'])
            fov = float(command.get('horizontal_fov_deg', camera_settings()['horizontal_fov_deg']))
            if (not isinstance(offset,list) or len(offset)!=3 or
                    not all(isinstance(v,(int,float)) and not isinstance(v,bool) and
                            math.isfinite(v) and abs(v)<=1 for v in offset)):
                raise ValueError('Camera XYZ offsets must be finite numbers within ±1 metre')
            if not math.isfinite(fov) or not 0 < fov < 180:
                raise ValueError('Horizontal FOV must be between 0 and 180 degrees')
            if scenarios:
                scenarios.abort('Camera changed; motion disabled. Reload image-goal scenarios before recording.')
            local = UsdGeom.Xformable(camera).GetLocalTransformation()
            local.SetTranslateOnly(Gf.Vec3d(*(urdf_camera_position+offset)))
            UsdGeom.Xformable(camera).MakeMatrixXform().Set(local)
            aperture = camera.GetHorizontalApertureAttr().Get()
            camera.GetFocalLengthAttr().Set(aperture/(2*math.tan(math.radians(fov)/2)))
            # Publish applied settings before a following Load/Record request.
            state['camera'] = metadata['camera'] = camera_settings()
            (output/'settings.json').write_text(json.dumps(metadata,indent=2))
            return state['camera']
        rgb = rep.AnnotatorRegistry.get_annotator('rgb')
        rgb.attach(rep.create.render_product(camera_path, (640,480)))
        from isaacsim.core.utils.viewports import set_camera_view
        c, s = math.cos(math.radians(spawn_yaw_deg)), math.sin(math.radians(spawn_yaw_deg))
        set_camera_view(eye=[spawn_x+1.5*(c+s),spawn_y+1.5*(s-c),1.8],
                        target=[spawn_x,spawn_y,0.35])

        enable_extension('isaacsim.ros2.bridge')
        app.update()
        import rclpy
        from geometry_msgs.msg import TwistStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Image as RosImage
        from rosgraph_msgs.msg import Clock
        from builtin_interfaces.msg import Time
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        context = Context()
        rclpy.init(context=context, domain_id=args.domain_id)
        node = rclpy.create_node('isaac_husky', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        def receive_command(msg):
            if scenarios and scenarios.epoch:
                return
            if state.get('ready') and state.get('playing'):
                try:
                    stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                    age = state.get('sim_time', 0.0)-stamp if stamp else 0.0
                    if age > controller.config.timeout or age < -0.1:
                        return
                    controller.command(msg.twist.linear.x, msg.twist.angular.z,
                                       time.monotonic()-max(0.0, age))
                except ValueError:
                    controller.stop()
        subscription = node.create_subscription(TwistStamped, '/a200_0000/cmd_vel', receive_command, 10)
        odom_pub = node.create_publisher(Odometry, '/a200_0000/platform/odom/filtered', 10)
        image_pub = node.create_publisher(RosImage, '/a200_0000/sensors/camera_0/color/image', qos_profile_sensor_data)
        clock_pub = node.create_publisher(Clock, '/clock', 10)
        world.reset()
        for _ in range(60):
            world.step(render=False)
            if _ % 4 == 0:
                world.render()
        articulation = Articulation('/World/Husky')
        dofs = articulation.get_dof_indices(list(WHEELS)).numpy().tolist()
        root_com = (articulation.get_link_coms(link_indices=[0])[0].numpy()[0,0]
                    if kinematic else np.zeros(3))
        _, damping = articulation.get_dof_gains(dof_indices=dofs)
        effective_damping = damping.numpy()[0]
        if not np.allclose(effective_damping, 0.0 if kinematic else args.wheel_damping, rtol=1e-4):
            raise RuntimeError('Physics drive damping does not match the requested Nm s/rad gain.')
        state.update(ready=True, playing=True, control_config=asdict(controller.config))
        print('LIVE_READY http://127.0.0.1:8211 ROS_DOMAIN_ID='+str(args.domain_id), flush=True)
        metadata = dict(environment=args.environment, drive_mode=args.drive_mode,
                        collisions_enabled=True, fill_light_intensity=args.fill_light_intensity,
                        spawn=dict(x=spawn_x, y=spawn_y, yaw_deg=spawn_yaw_deg),
                        camera=camera_settings(),
                        control=asdict(controller.config), assumed_wheel_torque_Nm=args.wheel_torque,
                        assumed_drive_damping_Nms_rad=args.wheel_damping, assumed_friction=args.friction,
                        physics_hz=120, ros_domain=args.domain_id, odometry='simulation ground truth')
        metadata['effective_tyre_friction'] = 0.0 if kinematic else args.friction
        metadata['effective_wheel_torque_limit_Nm'] = 0.0 if kinematic else args.wheel_torque
        metadata['root_com_local_m'] = root_com.tolist()
        metadata['effective_drive_damping_Nms_rad'] = effective_damping.tolist()
        metadata['stream_host'] = stream_host
        (output/'settings.json').write_text(json.dumps(metadata,indent=2))
        stage.GetRootLayer().Export(str(output/'scene.usda'))

        # USD/PhysX changes happen only here, on the simulation thread.
        # Teleporting is exclusive to explicit Load/Reset and image-goal capture.
        def reset_robot(where):
            nonlocal articulation, dofs
            controller.stop()
            yaw = math.radians(where['yaw_deg'])
            # Stop before authoring: PhysX otherwise restores the old local link
            # pose over a live teleport. Reinitialize physics from this USD pose;
            # the stage, camera products, viewer and loaded model stay alive.
            world.stop()
            if scenarios:
                with scenarios.lock:
                    scenarios.contact_pairs.clear()
                    scenarios.contacts_pending.clear()
            # Synchronize the USD parent as well as the physics articulation.
            # Camera rendering and native Stop/Play must see the same reset pose.
            robot_translation.Set(Gf.Vec3d(where['x'],where['y'],.13228 if kinematic else .16))
            robot_rotation.Set(where['yaw_deg'])
            for attr,value in rest_link_ops:
                attr.Set(value)
            world.reset()
            articulation = Articulation('/World/Husky')
            dofs = articulation.get_dof_indices(list(WHEELS)).numpy().tolist()
            articulation.set_dof_positions([[0]*4], dof_indices=dofs)
            articulation.set_dof_velocities([[0]*4], dof_indices=dofs)
            articulation.set_dof_velocity_targets([[0]*4], dof_indices=dofs)
            articulation.set_velocities([[0,0,0]], [[0,0,0]])
            if kinematic:
                controller.pose = (where['x'],where['y'],yaw)
            state.update(x=where['x'],y=where['y'],yaw=yaw,z=.13228 if kinematic else .16,
                         forward_speed=0.,yaw_rate=0.,sim_time=world.current_time)

        def settled_frame(where):
            reset_robot(where)
            for _ in range(12):
                articulation.set_velocities([[0,0,0]], [[0,0,0]])
                world.step(render=False, update_fabric=True)
                world.render()
            actual_positions, actual_orientations = articulation.get_world_poses()
            actual = actual_positions.numpy()[0]
            if math.hypot(float(actual[0])-where['x'],float(actual[1])-where['y']) > .05:
                raise RuntimeError(f'Capture pose mismatch: requested ({where["x"]:.2f}, {where["y"]:.2f}), '
                                   f'actual ({actual[0]:.2f}, {actual[1]:.2f}). Choose a free viewpoint.')
            qw,qx,qy,qz = actual_orientations.numpy()[0]
            yaw = math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
            error = math.atan2(math.sin(yaw-math.radians(where['yaw_deg'])),
                               math.cos(yaw-math.radians(where['yaw_deg'])))
            if abs(error) > .02:
                raise RuntimeError('Robot heading changed during capture; choose a free viewpoint')
            pixels = rgb.get_data()
            if pixels.shape[:2] != (480,640):
                raise RuntimeError('Camera has not produced a valid frame')
            import io
            buffer = io.BytesIO()
            Image.fromarray(np.ascontiguousarray(pixels[:,:,:3], dtype=np.uint8)).save(buffer,format='JPEG',quality=95)
            return buffer.getvalue()

        def prepare_scenario(scenario):
            if not world.is_playing():
                raise ValueError('Press Play in Isaac before loading a scenario')
            spawn_props(stage, scenario['props'], get_assets_root_path())
            goal_jpeg = None
            if scenario['mode'] in ('image','pose_image'):
                file = scenario['image_goal'].get('file')
                if file:
                    path = (scenarios.library.path.parent/file).resolve()
                    if not path.is_relative_to(scenarios.library.path.parent.resolve()):
                        raise ValueError('Image path leaves the scenario directory')
                    import io
                    buffer = io.BytesIO()
                    Image.open(path).convert('RGB').save(buffer,format='JPEG',quality=95)
                    goal_jpeg = buffer.getvalue()
                else:
                    goal_jpeg = settled_frame(scenario['image_goal']['viewpoint'])
            start_jpeg = settled_frame(scenario['robot'])
            controller.stop()
            articulation.set_velocities([[0,0,0]], [[0,0,0]])
            where=scenario['robot']
            state.update(x=where['x'],y=where['y'],yaw=math.radians(where['yaw_deg']),sim_time=world.current_time)
            state['observation_epoch'] = scenarios.epoch
            c,s = math.cos(state['yaw']),math.sin(state['yaw'])
            set_camera_view(eye=[state['x']+2*(c+s),state['y']+2*(s-c),2.2],
                            target=[state['x'],state['y'],.35])
            return dict(goal_jpeg=goal_jpeg,start_jpeg=start_jpeg,min_sim_time=world.current_time)

        contact_subscription = None
        if scenarios:
            scenarios.settings = metadata
            from pxr import PhysicsSchemaTools
            from omni.physx import get_physx_simulation_interface
            from omni.physx.bindings._physx import ContactEventType
            bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
            # The infinite Physics ground plane has no finite BBox.
            ground = {'/World/Environment/GroundPlane/CollisionPlane'}
            for prim in stage.Traverse():
                if prim.HasAPI(UsdPhysics.CollisionAPI) and not str(prim.GetPath()).startswith('/World/Husky'):
                    if prim.GetTypeName() == 'Plane' and prim.GetAttribute('axis').Get() in (None, 'Z'):
                        ground.add(str(prim.GetPath()))
                        continue
                    box = bounds.ComputeWorldBound(prim).ComputeAlignedRange()
                    if not box.IsEmpty() and box.GetMax()[2] < .08 and box.GetMin()[2] < .02:
                        ground.add(str(prim.GetPath()))
            def contacts(headers, data):
                for header in headers:
                    pair = [str(PhysicsSchemaTools.intToSdfPath(getattr(header, field)))
                            for field in ('collider0','collider1')]
                    robot_contact = [p.startswith('/World/Husky/') for p in pair]
                    if sum(robot_contact) != 1 or any(p in ground for p in pair):
                        continue
                    scenarios.contact(pair, header.type != ContactEventType.CONTACT_LOST)
            contact_subscription = get_physx_simulation_interface().subscribe_contact_report_events(contacts)
            scenarios.contact_reporting = True

        samples, next_control, next_image, next_odom = [], 0.0, 0.0, 0.0
        steps = 0
        start_sim = world.current_time
        last_sim = world.current_time
        previous_phase = None
        next_camera_poll = 0.
        while app.is_running():
            frame_start = time.monotonic()
            try:
                camera_command, camera_future = camera_jobs.get_nowait()
            except queue.Empty:
                pass
            else:
                try:
                    camera_future.set_result(update_camera(camera_command))
                except Exception as exc:
                    camera_future.set_exception(exc)
            # Keep settings accurate when the native Property panel is used.
            if frame_start >= next_camera_poll:
                next_camera_poll = frame_start+.2
                current_camera = camera_settings()
                state['camera'] = current_camera
                if current_camera != metadata['camera']:
                    metadata['camera'] = current_camera
                    (output/'settings.json').write_text(json.dumps(metadata,indent=2))
            if scenarios:
                if scenarios.pump(prepare_scenario) == 'prepare':
                    last_sim = world.current_time
                    next_control = next_image = next_odom = world.current_time
                if not scenarios.epoch:
                    state['observation_epoch'] = ''
            executor.spin_once(timeout_sec=0.0)
            playing = world.is_playing()
            state['playing'] = playing
            if not playing:
                controller.stop()
                if scenarios:
                    scenarios.tick()
                if not world.is_stopped():
                    articulation.set_dof_velocity_targets([[0]*4], dof_indices=dofs)
                    if kinematic:
                        articulation.set_velocities([[0,0,0]], [[0,0,0]])
                app.update()
                time.sleep(0.01)
                continue
            if world.current_time < last_sim:
                # The native Stop/Play controls reset simulation time and physics views.
                controller.stop()
                if scenarios and scenarios.epoch:
                    scenarios.abort('Simulation reset; Load / Reset the scenario again')
                next_control = next_image = next_odom = world.current_time
            last_sim = world.current_time
            if not articulation.is_physics_tensor_entity_valid():
                # Native Play creates a fresh tensor view after Stop destroyed it.
                world.render()
                if not world.is_playing():
                    continue
                articulation = Articulation('/World/Husky')
                dofs = articulation.get_dof_indices(list(WHEELS)).numpy().tolist()
                if not articulation.is_physics_tensor_entity_valid():
                    continue
            t = world.current_time-start_sim
            phase = None
            if args.test:
                phase = ('settle' if t<1 else 'forward' if t<3 else 'stop' if t<4 else
                         'reverse' if t<6 else 'turn' if t<8 else 'timeout')
                if phase != previous_phase:
                    print('TEST_PHASE', phase, flush=True)
                    previous_phase = phase
                if phase != 'timeout':
                    v,w = {'settle':(0,0),'forward':(.3,0),'stop':(0,0),'reverse':(-.3,0),'turn':(0,.5)}[phase]
                    controller.command(v,w,frame_start)
                if t>=9.5:
                    break
            if not kinematic and world.current_time >= next_control:
                targets = controller.advance(1/controller.config.rate, frame_start)
                articulation.set_dof_velocity_targets([targets], dof_indices=dofs)
                next_control = world.current_time+1/controller.config.rate-1e-6
            if kinematic:
                vx,vy,w = controller.body_velocity((state['x'],state['y'],state['yaw']),
                                                   1/120,frame_start)
                # Direct velocity control removes tyre scrub and motor lag. The
                # normal PhysX contact solver still blocks obstacle penetration.
                vz = max(-0.3, min(0.3, (0.13228-state.get('z',0.13228))*120))
                # PhysX sets COM velocity; waypoints/odometry use the base origin.
                c,s = math.cos(state['yaw']),math.sin(state['yaw'])
                rx,ry = c*root_com[0]-s*root_com[1], s*root_com[0]+c*root_com[1]
                articulation.set_velocities([[vx-w*ry,vy+w*rx,vz]], [[0,0,w]])
            world.step(render=False)
            steps += 1
            if steps % 4 == 0:
                world.render()
            if not world.is_playing():
                # A GUI Stop/Pause can happen inside the render call itself.
                controller.stop()
                state['playing'] = False
                continue
            positions, orientations = articulation.get_world_poses()
            position, q = positions.numpy()[0], orientations.numpy()[0]
            linear, angular = articulation.get_velocities()
            linear, angular = linear.numpy()[0], angular.numpy()[0]
            qw,qx,qy,qz = [float(x) for x in q]
            yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
            forward = float(math.cos(yaw)*linear[0]+math.sin(yaw)*linear[1])
            if kinematic:
                # Measure base-frame translation, including any contact blockage.
                forward = ((float(position[0])-state['x'])*math.cos(yaw)+
                           (float(position[1])-state['y'])*math.sin(yaw))*120
            state.update(x=float(position[0]), y=float(position[1]), z=float(position[2]),
                         yaw=yaw, forward_speed=forward, yaw_rate=float(angular[2]),
                         sim_time=world.current_time)
            stamp = Time(sec=int(world.current_time), nanosec=int(world.current_time%1*1e9))
            if world.current_time >= next_odom:
                next_odom += 0.02
                clock_pub.publish(Clock(clock=stamp))
                msg = Odometry()
                msg.header.stamp = stamp
                suffix = '__scenario_'+state['observation_epoch'] if state.get('observation_epoch') else ''
                msg.header.frame_id, msg.child_frame_id = 'odom'+suffix, 'base_link'
                msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float,position)
                msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z = qw,qx,qy,qz
                msg.twist.twist.linear.x, msg.twist.twist.angular.z = forward,float(angular[2])
                odom_pub.publish(msg)
            if world.current_time >= next_image:
                next_image = world.current_time+0.2
                pixels = rgb.get_data()
                if pixels.shape[:2] == (480,640):
                    frame = np.ascontiguousarray(pixels[:,:,:3],dtype=np.uint8)
                    image = RosImage()
                    image.header.stamp = stamp
                    suffix = '__scenario_'+state['observation_epoch'] if state.get('observation_epoch') else ''
                    image.header.frame_id = 'camera_0_color_optical_frame'+suffix
                    image.height,image.width,image.encoding,image.step = 480,640,'rgb8',640*3
                    image.data = frame.tobytes()
                    image_pub.publish(image)
                    if not (output/'front_camera.png').exists():
                        Image.fromarray(frame).save(output/'front_camera.png')
                    if scenarios:
                        import io
                        buf = io.BytesIO()
                        Image.fromarray(frame).save(buf,format='JPEG',quality=85)
                        scenarios.current_jpeg = buf.getvalue()
                        scenarios.video_frame = dict(sim_time=world.current_time,
                            frame_epoch=state.get('observation_epoch',''),
                            pose={k:state[k] for k in ('x','y','yaw')},
                            jpeg_base64=base64.b64encode(scenarios.current_jpeg).decode('ascii'))
                if args.test:
                    samples.append(dict(t=t, phase=phase, **state, controller=controller.state(),
                             wheels=articulation.get_dof_velocities(dof_indices=dofs).numpy()[0].tolist()))
            if scenarios:
                scenarios.tick(frame if 'frame' in locals() else None)
            # Keep simulation time near wall time for watchdog and teleoperation behavior.
            time.sleep(max(0.0, 1/120-(time.monotonic()-frame_start)))
        controller.stop()
        articulation.set_dof_velocity_targets([[0]*4], dof_indices=dofs)
        if kinematic:
            articulation.set_velocities([[0,0,0]], [[0,0,0]])
        if args.test:
            (output/'drive_test.json').write_text(json.dumps(samples,indent=2))
            checks = check_driving(samples)
            (output/'checks.json').write_text(json.dumps(checks,indent=2))
            print('DRIVE_TEST_CHECKS', checks, flush=True)
            if not all(checks.values()):
                raise RuntimeError('Physical driving checks failed; see '+str(output/'checks.json'))
    except Exception:
        import traceback
        exit_code=1
        traceback.print_exc()
        raise
    finally:
        controller.stop()
        if scenarios:
            scenarios.close()
        if server:
            server.shutdown()
        if node:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)
        app.close(exit_code=exit_code)


if __name__ == '__main__':
    main()
