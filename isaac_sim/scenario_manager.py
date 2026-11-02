"""Editable warehouse trials using the existing simulator and OmniVLA servers.

Library validation and trial bookkeeping are ordinary Python. USD imports stay
inside spawn_props(). HTTP work runs on workers; pump() and tick() run on the
simulation thread. A collision is an observation, never a termination condition.
"""
import base64
import csv
from concurrent.futures import Future, ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path
import queue
import re
import threading
import time
from urllib.request import Request, urlopen
import uuid

import yaml

MODES = ('language', 'pose', 'image', 'language_pose', 'pose_image')
KINDS = ('barrel', 'crate', 'pallet', 'cone', 'box', 'couch', 'usd')
IDENTIFIER = re.compile(r'^[a-zA-Z0-9][a-zA-Z0-9_-]{0,79}$')


class ScenarioDumper(yaml.SafeDumper):
    """Keep pose/prop records compact while retaining readable YAML structure."""


def _dump_mapping(dumper, data):
    compact = 'kind' in data or 'viewpoint' in data or all(not isinstance(v,(dict,list)) for v in data.values())
    return dumper.represent_mapping('tag:yaml.org,2002:map', data, flow_style=compact)


ScenarioDumper.add_representer(dict, _dump_mapping)


def number(value, name, low=-1000, high=1000):
    if isinstance(value, bool):
        raise ValueError(f'{name} must be a number')
    value = float(value)
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name} must be within [{low}, {high}]')
    return value


def pose(value, name):
    if not isinstance(value, dict):
        raise ValueError(f'{name} must contain x, y, yaw_deg')
    return dict(x=number(value['x'], name+'.x'), y=number(value['y'], name+'.y'),
                yaw_deg=number(value.get('yaw_deg', 0), name+'.yaw_deg', -360, 360))


def validate_scenario(data):
    """Return a normalized copy. Nothing here changes the stage or model."""
    if not isinstance(data, dict) or not IDENTIFIER.fullmatch(str(data.get('id', ''))):
        raise ValueError('Scenario ID must use letters, numbers, underscores or hyphens')
    s = dict(id=data['id'], name=str(data.get('name', data['id']))[:160],
             description=str(data.get('description', ''))[:2000],
             family=str(data.get('family', 'custom'))[:80],
             difficulty=str(data.get('difficulty', 'custom'))[:80],
             mode=str(data.get('mode', 'language')), prompt=str(data.get('prompt', '')).strip()[:2000])
    if s['mode'] not in MODES:
        raise ValueError(f'Mode must be one of {MODES}')
    s['robot'] = pose(data['robot'], 'robot')
    s['pose_goal'] = pose(data['pose_goal'], 'pose_goal')
    image = data.get('image_goal', {})
    if not isinstance(image, dict):
        raise ValueError('image_goal must be an object')
    s['image_goal'] = dict(viewpoint=pose(image.get('viewpoint', s['pose_goal']), 'image_goal.viewpoint'))
    if image.get('file'):
        file = str(image['file'])
        if not re.fullmatch(r'images/[a-zA-Z0-9_-]+\.(jpg|png)', file):
            raise ValueError('Saved image must be an images/<name>.jpg or .png in the scenario directory')
        s['image_goal']['file'] = file
    e = data.get('evaluation', {})
    tolerance = e.get('heading_tolerance_deg')
    s['evaluation'] = dict(x=number(e.get('x', s['pose_goal']['x']), 'evaluation.x'),
                           y=number(e.get('y', s['pose_goal']['y']), 'evaluation.y'),
                           radius_m=number(e.get('radius_m', .75), 'radius_m', .05, 5),
                           timeout_s=number(e.get('timeout_s', 120), 'timeout_s', 1, 3600),
                           heading_tolerance_deg=None if tolerance is None else
                           number(tolerance, 'heading_tolerance_deg', 1, 180))
    # Optional language-test annotations are scoring only, never model inputs.
    for key in ('checkpoints', 'forbidden', 'distractors'):
        if key not in e:
            continue
        rows = e[key]
        if not isinstance(rows, list) or len(rows) > 32:
            raise ValueError(f'evaluation.{key} must be a list of at most 32 regions')
        normalized = []
        for row in rows:
            if not isinstance(row, dict) or not str(row.get('label', '')).strip():
                raise ValueError(f'Every {key} region needs a label')
            region = dict(label=str(row['label'])[:160])
            fields = ('x', 'y', 'radius_m') if key == 'distractors' else ('x_min', 'x_max', 'y_min', 'y_max')
            region.update({k: number(row[k], key+'.'+k) for k in fields})
            if key == 'distractors':
                number(region['radius_m'], 'distractor radius', .05, 5)
            elif region['x_min'] >= region['x_max'] or region['y_min'] >= region['y_max']:
                raise ValueError(f'{key} region must have min < max on both axes')
            normalized.append(region)
        if len({r['label'] for r in normalized}) != len(normalized):
            raise ValueError(f'{key} labels must be unique')
        s['evaluation'][key] = normalized
    motion = data.get('motion', {})
    s['motion'] = dict(max_linear=number(motion.get('max_linear', .3), 'max_linear', .01, .5),
                       max_angular=number(motion.get('max_angular', .5), 'max_angular', .01, 1))
    props = data.get('props', [])
    if not isinstance(props, list) or len(props) > 64:
        raise ValueError('props must be a list of at most 64 objects')
    s['props'], ids = [], set()
    for item in props:
        ident = str(item.get('id', ''))
        if not re.fullmatch(r'[a-zA-Z_][a-zA-Z0-9_]{0,79}', ident) or ident in ids:
            raise ValueError('Prop IDs must be valid and unique within the scenario')
        ids.add(ident)
        kind = str(item.get('kind', 'crate'))
        if kind not in KINDS:
            raise ValueError(f'Prop kind must be one of {KINDS}')
        size = item.get('size', [.6, .6, .9] if kind == 'barrel' else [.8, .8, .7])
        if not isinstance(size, list) or len(size) != 3:
            raise ValueError('Prop size must be [length, width, height] in metres')
        color = str(item.get('color', '#8a623d'))
        if not re.fullmatch(r'#[0-9a-fA-F]{6}', color):
            raise ValueError('Prop color must be #RRGGBB')
        prop = dict(id=ident, kind=kind, **pose(item, 'prop'),
                    z=number(item.get('z', 0), 'prop.z', 0, 20),
                    size=[number(v, 'prop.size', .02, 20) for v in size], color=color)
        if kind == 'usd':
            asset = str(item.get('asset', ''))
            if not asset.startswith(('Isaac/Props/', '/workspace/assets/')) or '..' in asset.split('/'):
                raise ValueError('USD assets must be Isaac/Props/... or /workspace/assets/...')
            if not asset.endswith(('.usd', '.usda', '.usdc')):
                raise ValueError('USD prop requires a USD asset path')
            prop['asset'] = asset
        s['props'].append(prop)
    return s


def region_entry(start, end, region):
    """First intersection parameter on a segment with an axis-aligned box."""
    lo, hi = 0., 1.
    for axis, a, b in zip(('x', 'y'), start, end):
        low, high = region[axis+'_min'], region[axis+'_max']
        if abs(b-a) < 1e-12:
            if not low <= a <= high:
                return None
        else:
            t0, t1 = sorted(((low-a)/(b-a), (high-a)/(b-a)))
            lo, hi = max(lo, t0), min(hi, t1)
            if lo > hi:
                return None
    return lo


def score_language_segment(run, evaluation, start, end):
    """Observe the robot centre. Return events; never alter a motion command."""
    events = []
    checkpoints = evaluation.get('checkpoints', [])
    last_entry = 0.
    while run['checkpoints_passed'] < len(checkpoints):
        region = checkpoints[run['checkpoints_passed']]
        entry = region_entry(start, end, region)
        if entry is None or entry < last_entry:
            break
        last_entry = entry
        run['checkpoints_passed'] += 1
        events.append(dict(event='checkpoint', label=region['label']))
    for region in evaluation.get('forbidden', []):
        if region['label'] not in run['route_violations'] and region_entry(start, end, region) is not None:
            run['route_violations'].append(region['label'])
            events.append(dict(event='route_violation', label=region['label']))
    dx, dy = end[0]-start[0], end[1]-start[1]
    length2 = dx*dx+dy*dy
    for region in evaluation.get('distractors', []):
        t = max(0., min(1., ((region['x']-start[0])*dx+(region['y']-start[1])*dy)/length2)) if length2 else 0.
        distance = math.hypot(start[0]+t*dx-region['x'], start[1]+t*dy-region['y'])
        if distance <= region['radius_m'] and region['label'] not in run['distractor_visits']:
            run['distractor_visits'].append(region['label'])
            events.append(dict(event='distractor_visit', label=region['label']))
    return events


class ScenarioLibrary:
    def __init__(self, path):
        self.path = Path(path)
        self.lock = threading.RLock()

    def read(self):
        with self.lock:
            data = yaml.safe_load(self.path.read_text())
            if not isinstance(data, dict) or data.get('schema') != 1:
                raise ValueError('Scenario file must have schema: 1')
            rows = [validate_scenario(s) for s in data.get('scenarios', [])]
            if len({s['id'] for s in rows}) != len(rows):
                raise ValueError('Duplicate scenario IDs')
            return dict(schema=1, environment=str(data['environment']), scenarios=rows)

    def save(self, scenario):
        scenario = validate_scenario(scenario)
        with self.lock:
            data = self.read()
            rows = data['scenarios']
            existing = next((i for i, s in enumerate(rows) if s['id'] == scenario['id']), None)
            if existing is None:
                rows.append(scenario)
            else:
                rows[existing] = scenario
            # Directory is the user-owned RW mount; replace is atomic for readers.
            temp = self.path.with_name('.warehouse-'+uuid.uuid4().hex+'.tmp')
            try:
                temp.write_text(yaml.dump(data, Dumper=ScenarioDumper, sort_keys=False, allow_unicode=True, width=150))
                os.replace(temp, self.path)
            finally:
                temp.unlink(missing_ok=True)
        return scenario

    def save_image(self, encoded):
        from PIL import Image
        raw = base64.b64decode(encoded, validate=True)
        if len(raw) > 3*1024*1024:
            raise ValueError('Image must be smaller than 3 MiB')
        im = Image.open(io.BytesIO(raw)).convert('RGB')
        if im.width > 8192 or im.height > 8192:
            raise ValueError('Image dimensions exceed 8192 pixels')
        im.thumbnail((1280, 960))
        folder = self.path.parent/'images'
        folder.mkdir(exist_ok=True)
        name = 'images/'+uuid.uuid4().hex+'.jpg'
        im.save(self.path.parent/name, quality=95)
        return name


class ScenarioManager:
    def __init__(self, library, output, controller, state, environment,
                 model_url='http://127.0.0.1:8080'):
        self.library = ScenarioLibrary(library)
        self.output, self.controller, self.state = Path(output), controller, state
        self.environment, self.model_url = environment, model_url
        self.lock = threading.RLock()
        self.workers = ThreadPoolExecutor(max_workers=2, thread_name_prefix='scenario')
        self.sim_jobs = queue.Queue()
        self.phase, self.message, self.epoch = 'idle', 'Choose a scenario', ''
        self.selected, self.model, self.run, self.last_result = None, {}, None, None
        self.job, self.poll = None, None
        self.cancel = threading.Event()
        self.next_poll, self.next_sample, self.model_received = 0., 0., 0.
        self.contact_pairs, self.contacts_pending = set(), []
        self.finish_request = None
        self.contact_reporting = False
        self.goal_jpeg, self.start_jpeg = None, None
        self.settings = {}
        self.study = {}
        self.loaded_camera = None

    def result_folder(self, ident):
        if not isinstance(ident, str) or not re.fullmatch(r'[a-zA-Z0-9_-]{1,160}', ident):
            raise ValueError('Invalid run ID')
        folder = (self.output/ident).resolve()
        if not folder.is_relative_to(self.output.resolve()) or not (folder/'summary.json').is_file():
            raise ValueError('Recorded run not found')
        return folder

    def results(self):
        rows = []
        for file in sorted(self.output.glob('*/summary.json'), reverse=True):
            try:
                row = json.loads(file.read_text())
                review = file.parent/'review.json'
                row['review'] = json.loads(review.read_text()) if review.exists() else {}
                rows.append(row)
            except (OSError, ValueError):
                continue
        return rows

    def review(self, body):
        folder = self.result_folder(body.get('id'))
        score = body.get('score')
        review = dict(score=None if score is None else number(score, 'score', 0, 100),
                      notes=str(body.get('notes', ''))[:8000],
                      updated_at=datetime.now(timezone.utc).isoformat())
        for key in ('arrival', 'route_adherence', 'collision_avoidance'):
            value = body.get(key)
            if value not in ('pass', 'fail', 'unsure', 'na', None):
                raise ValueError('Review judgments must be pass, fail, unsure or na')
            review[key] = value
        with self.lock:
            temp = folder/('.review-'+uuid.uuid4().hex+'.tmp')
            temp.write_text(json.dumps(review, indent=2))
            os.replace(temp, folder/'review.json')
        return review

    def result_detail(self, ident):
        folder = self.result_folder(ident)
        samples = []
        with (folder/'trajectory.jsonl').open() as stream:
            for line in stream:
                row = json.loads(line)
                if row.get('event') == 'sample':
                    samples.append({k:row.get(k) for k in ('x','y','yaw','elapsed_s')})
        # Bound the browser plot size; the original full log remains on disk.
        stride = max(1, math.ceil(len(samples)/3000))
        return dict(snapshot=yaml.safe_load((folder/'scenario.yaml').read_text()),
                    samples=samples[::stride]+samples[-1:])

    def results_csv(self):
        stream = io.StringIO()
        fields = ('id','study','repeat','scenario_id','mode','prompt','model_path','resume_step',
                  'camera_name','horizontal_fov_deg','camera_offset_x','camera_offset_y','camera_offset_z',
                  'scenario_sha256','outcome','elapsed_s','travelled_m','reached_goal','collision_count',
                  'contact_seconds','checkpoints_passed','checkpoints_total','route_compliant','route_observed_compliant',
                  'collision_free_success','score','arrival','route_adherence','collision_avoidance','notes')
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for result in self.results():
            row = {k:result.get(k) for k in fields}
            row.update({k:result.get('review',{}).get(k) for k in
                        ('score','arrival','route_adherence','collision_avoidance','notes')})
            camera = result.get('camera', {})
            row['horizontal_fov_deg'] = camera.get('horizontal_fov_deg')
            for i, axis in enumerate('xyz'):
                row['camera_offset_'+axis] = camera.get('offset_m', [None]*3)[i]
            # User-entered text stays text when a spreadsheet opens the CSV.
            writer.writerow({k:("'"+v if isinstance(v,str) and v.lstrip().startswith(('=','+','-','@')) else v)
                             for k,v in row.items()})
        return stream.getvalue().encode('utf-8-sig')

    def model_request(self, route, data=None):
        req = Request(self.model_url+route, data=None if data is None else json.dumps(data).encode(),
                      headers={'Content-Type': 'application/json'})
        with urlopen(req, timeout=1.5) as response:
            result = json.load(response)
        if result.get('ok') is False:
            raise RuntimeError(result.get('error', 'Model request failed'))
        return result

    def status(self):
        with self.lock:
            run = None if self.run is None else {k: v for k, v in self.run.items() if not k.startswith('_')}
            return json.loads(json.dumps(dict(phase=self.phase, message=self.message, epoch=self.epoch,
                selected=self.selected, run=run, last_result=self.last_result,
                contact_reporting=self.contact_reporting,
                active_contacts=[list(pair) for pair in self.contact_pairs],
                model_online=time.monotonic()-self.model_received < 3,
                model={k: self.model.get(k) for k in ('mode', 'ready', 'inference_seconds',
                    'prediction_seq', 'max_linear', 'max_angular', 'execution_mode', 'message', 'motion_enabled',
                    'model_path', 'resume_step', 'model_loaded', 'checkpoint_loading', 'checkpoint_error')},
                has_goal_image=self.goal_jpeg is not None)))

    def accepts_trajectory(self, command):
        with self.lock:
            if not self.epoch:
                return not command.get('scenario_epoch')
            return (command.get('scenario_epoch') == self.epoch and
                    (not command.get('enabled') or self.phase in ('running', 'manual')))

    def manual_command(self):
        with self.lock:
            if self.epoch and self.phase not in ('idle', 'aborted'):
                self.abort('Manual driving took ownership')
                self.phase = 'idle'

    def abort(self, reason='Aborted by user'):
        with self.lock:
            self.cancel.set()
            self.controller.stop()
            self.finish_request = ('aborted', reason)
            if self.run is None:
                self.phase, self.message = 'aborted', reason
        # STOP is independent of a long load task. No scene/physics work here.
        self.workers.submit(self._stop_model, self.epoch)

    def _stop_model(self, epoch=None):
        try:
            epoch = self.epoch if epoch is None else epoch
            if epoch:
                self.model_request('/api/scenario/stop', {'epoch': epoch})
            else:
                self.model_request('/api/stop', {})
        except Exception:
            # Local gate and controller have already been disabled.
            pass

    def request(self, action, body):
        with self.lock:
            if body.get('epoch') is not None and body['epoch'] != self.epoch:
                raise ValueError('Scenario changed; refresh the controls and try again')
            if action == 'abort':
                self.abort()
                return
            if action == 'finish':
                if self.run is None:
                    raise ValueError('No recorded trial is running')
                self.controller.stop()
                self.finish_request = ('manual', 'Ended by reviewer')
                self.workers.submit(self._stop_model, self.epoch)
                return
            if self.job is not None and not self.job.done():
                raise ValueError('Wait for the current scenario operation to finish')
            if action == 'checkpoint':
                if self.run is not None or self.controller.state()['enabled']:
                    raise ValueError('Finish the trial and stop motion before switching checkpoints')
                self.phase, self.message = 'switching', 'Loading checkpoint; motion remains off'
                self.job = self.workers.submit(self._guarded, self._checkpoint, str(body.get('checkpoint','')))
            elif action == 'load':
                if self.run is not None:
                    raise ValueError('Abort the running trial before loading another scenario')
                data = self.library.read()
                if data['environment'] != self.environment:
                    raise ValueError('Scenario library does not match the loaded warehouse')
                scenario = body.get('scenario')
                if scenario is None:
                    scenario = next((s for s in data['scenarios'] if s['id'] == body.get('id')), None)
                scenario = validate_scenario(scenario)
                if scenario['mode'] in ('language', 'language_pose') and not scenario['prompt']:
                    raise ValueError('This modality requires a prompt')
                self.epoch = uuid.uuid4().hex
                self.cancel = threading.Event()
                self.phase, self.message = 'loading', 'Stopping motion and preparing the scene'
                self.selected = scenario
                self.goal_jpeg = self.start_jpeg = None
                self.contact_pairs.clear()
                self.contacts_pending.clear()
                self.controller.stop()
                self.job = self.workers.submit(self._guarded, self._load, scenario)
            elif action in ('start', 'drive'):
                record = action == 'start'
                if record and self.phase != 'ready':
                    raise ValueError('Load/reset a scenario before starting')
                if not record and (not self.epoch or self.selected is None or
                                   self.phase not in ('ready', 'manual', 'finished', 'aborted', 'idle')):
                    raise ValueError('Load a scenario before enabling model motion')
                if self.run is not None:
                    raise ValueError('Abort the recorded trial before starting unrecorded motion')
                if self.state.get('drive_mode') != 'kinematic':
                    raise ValueError('Scenario evaluation uses kinematic execution; set ISAAC_DRIVE_MODE=kinematic')
                if record and not self.contact_reporting:
                    raise ValueError('Contact reporting is unavailable; a trial cannot claim collision metrics')
                if record:
                    camera = body.get('camera', {})
                    if not isinstance(camera,dict):
                        raise ValueError('Camera must contain FOV and mount offsets')
                    if camera:
                        offset = camera.get('offset_m')
                        if not isinstance(offset,list) or len(offset)!=3:
                            raise ValueError('Camera needs three mount offsets')
                        camera = dict(offset_m=[number(v,'camera offset',-1,1) for v in offset],
                                      horizontal_fov_deg=number(camera.get('horizontal_fov_deg'),'FOV',.01,179.99))
                    repeat = number(body.get('repeat',1), 'repeat', 1, 10000)
                    if repeat != int(repeat):
                        raise ValueError('Repeat must be an integer')
                    self.study = dict(study=str(body.get('study',''))[:120],
                                      repeat=int(repeat),
                                      camera_name=str(body.get('camera_name','custom'))[:120],
                                      expected_checkpoint=str(body.get('checkpoint','')),
                                      expected_camera=camera)
                self.phase, self.message = 'starting', 'Waiting for a fresh model prediction'
                self.cancel = threading.Event()
                self.job = self.workers.submit(self._guarded, self._start, record)
            elif action == 'release':
                if self.run is not None:
                    raise ValueError('Abort the trial before leaving scenario mode')
                self.controller.stop()
                self.cancel = threading.Event()
                self.job = self.workers.submit(self._guarded, self._release)
            else:
                raise ValueError('Unknown scenario operation')

    def _checkpoint(self, checkpoint):
        self.model_request('/api/checkpoint', {'checkpoint':checkpoint})
        deadline = time.monotonic()+240
        while time.monotonic() < deadline:
            time.sleep(.3)
            model = self.model_request('/api/status')
            if not model.get('checkpoint_loading'):
                if not model.get('model_loaded') or model.get('checkpoint_error'):
                    raise RuntimeError(model.get('checkpoint_error') or 'Checkpoint failed to load')
                with self.lock:
                    self.phase, self.message = 'idle', 'Checkpoint loaded — Load / Reset before recording'
                return
        raise RuntimeError('Checkpoint load timed out; inspect model status')

    def _release(self):
        epoch = self.epoch or self.model_request('/api/status').get('scenario_epoch')
        if epoch:
            self.model_request('/api/scenario/release', {'epoch': epoch})
        with self.lock:
            self.epoch, self.selected = '', None
            self.phase, self.message = 'idle', 'Free driving — configure goals on the OmniVLA dashboard'

    def _guarded(self, function, *args):
        try:
            function(*args)
        except Exception as exc:
            with self.lock:
                self.controller.stop()
                self.finish_request = ('aborted' if self.cancel.is_set() else 'error', str(exc))
                self.phase = 'aborted' if self.cancel.is_set() else 'error'
                self.message = str(exc)
            self._stop_model()

    def _check_cancel(self):
        if self.cancel.is_set():
            raise RuntimeError('Operation cancelled')

    def _sim(self, op, data=None):
        self._check_cancel()
        future = Future()
        self.sim_jobs.put((self.epoch, op, data, future))
        result = future.result(timeout=60)
        self._check_cancel()
        return result

    def _load(self, scenario):
        self.model_request('/api/scenario/prepare', {'epoch': self.epoch})
        prepared = self._sim('prepare', scenario)
        self._check_cancel()
        config = dict(epoch=self.epoch, mode=scenario['mode'], instruction=scenario['prompt'],
                      goal_world=scenario['pose_goal'], min_sim_time=prepared['min_sim_time'],
                      **scenario['motion'])
        if prepared.get('goal_jpeg'):
            config['goal_jpeg_base64'] = base64.b64encode(prepared['goal_jpeg']).decode()
        self.model_request('/api/scenario/configure', config)
        with self.lock:
            self._check_cancel()
            self.goal_jpeg, self.start_jpeg = prepared.get('goal_jpeg'), prepared['start_jpeg']
            self.loaded_camera = json.loads(json.dumps(self.state.get('camera',{})))
            self.phase, self.message = 'ready', 'Ready — inspect the scene, then Start & Record'

    def _start(self, record=True):
        # A paused single step verifies the image/odom generation and goal together.
        status = self.model_request('/api/status')
        if not status.get('model_loaded') or status.get('checkpoint_loading'):
            raise ValueError('Wait for the checkpoint to load')
        if record:
            if self.loaded_camera and self.loaded_camera != self.state.get('camera'):
                raise ValueError('Camera changed after reset; Load / Reset again before recording')
            expected = self.study.get('expected_checkpoint')
            checkpoint = {'original':('omnivla-original',120000), 'cast':('omnivla-finetuned-cast',210000)}
            if expected and (Path(status['model_path']).name, status['resume_step']) != checkpoint.get(expected):
                raise ValueError('Selected checkpoint is not loaded; switch it before recording')
            camera = self.study.get('expected_camera')
            actual = self.state.get('camera',{})
            if camera and (abs(number(camera['horizontal_fov_deg'],'FOV')-actual['horizontal_fov_deg']) > .01 or
                           any(abs(a-b) > .0001 for a,b in zip(camera['offset_m'],actual['offset_m']))):
                raise ValueError('Camera fields have not been applied; Apply camera, then Load / Reset')
        before = status['prediction_seq']
        if not status['inference_paused']:
            self.model_request('/api/scenario/stop', {'epoch': self.epoch})
        self.model_request('/api/step', {})
        deadline = time.monotonic()+20
        while time.monotonic() < deadline:
            self._check_cancel()
            status = self.model_request('/api/status')
            if status.get('prediction_epoch') == self.epoch and status['prediction_seq'] > before and status['ready']:
                break
            time.sleep(.1)
        else:
            raise RuntimeError('No fresh prediction arrived. Check model and ROS bridge; '+status.get('message', ''))
        self._sim('begin' if record else 'drive', status)
        self._check_cancel()
        self.model_request('/api/scenario/start', {'epoch': self.epoch, 'recording': record})
        with self.lock:
            if self.phase not in ('running', 'manual'):
                self._stop_model()

    def pump(self, prepare):
        """Called on the simulation thread before stepping physics."""
        try:
            epoch, op, data, future = self.sim_jobs.get_nowait()
        except queue.Empty:
            return
        try:
            if epoch != self.epoch or self.cancel.is_set():
                raise RuntimeError('Scenario operation cancelled')
            if op == 'prepare':
                result = prepare(data)
            elif op == 'begin':
                with self.lock:
                    self._begin(data)
                result = True
            elif op == 'drive':
                with self.lock:
                    self.phase, self.message = 'manual', 'Model motion controls active — no recording'
                result = True
            else:
                raise ValueError('Unknown simulation operation')
            future.set_result(result)
        except Exception as exc:
            future.set_exception(exc)
        return op

    def _begin(self, model):
        self.output.mkdir(parents=True, exist_ok=True)
        ident = datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S-%f')+'-'+self.selected['id']
        folder = self.output/ident
        folder.mkdir()
        snapshot = dict(scenario=self.selected, simulation=json.loads(json.dumps(self.settings)), model={k: model.get(k) for k in
            ('mode', 'modality_id', 'model_path', 'resume_step', 'execution_mode', 'waypoint_spacing',
             'waypoint_dt', 'waypoint_index', 'steering_gain', 'max_linear', 'max_angular')}, epoch=self.epoch,
            collision_policy='record and continue', stopping_is_not_scored=True)
        snapshot['study'] = self.study
        snapshot['controller_at_start'] = self.controller.state()
        snapshot['actual_start'] = {k:self.state.get(k) for k in ('x','y','yaw','z','sim_time')}
        snapshot['scenario_sha256'] = hashlib.sha256(json.dumps(self.selected, sort_keys=True).encode()).hexdigest()
        (folder/'scenario.yaml').write_text(yaml.safe_dump(snapshot, sort_keys=False))
        if self.start_jpeg:
            (folder/'start.jpg').write_bytes(self.start_jpeg)
        if self.goal_jpeg:
            (folder/'goal.jpg').write_bytes(self.goal_jpeg)
        self.run = dict(id=ident, scenario_id=self.selected['id'], mode=self.selected['mode'],
                        study=self.study.get('study',''), repeat=self.study.get('repeat',1),
                        camera_name=self.study.get('camera_name','custom'), camera=snapshot['simulation']['camera'],
                        model_path=model.get('model_path'), resume_step=model.get('resume_step'),
                        prompt=self.selected['prompt'], scenario_sha256=snapshot['scenario_sha256'],
                        elapsed_s=0., travelled_m=0., goal_distance_m=None, min_goal_distance_m=None,
                        collision_count=0, contact_seconds=0., tracking_error_max_m=0.,
                        checkpoints_passed=0, checkpoints_total=len(self.selected['evaluation'].get('checkpoints', [])),
                        route_violations=[], distractor_visits=[],
                        _folder=folder, _log=(folder/'trajectory.jsonl').open('w'),
                        _start_sim=self.state['sim_time'], _start_wall=time.monotonic(),
                        _previous=(self.state['x'], self.state['y']), _tracking_sum=0., _samples=0,
                        _last_prediction=None)
        self.contacts_pending.clear()
        for pair in self.contact_pairs:
            self.contacts_pending.append(dict(event='contact', colliders=list(pair), at_start=True))
        self.next_sample = 0.
        self.phase, self.message = 'running', 'Recording — collisions are logged and the trial continues'

    def contact(self, pair, active):
        """PhysX callback: deduplicate persistent pairs into contact episodes."""
        with self.lock:
            pair = tuple(sorted(pair))
            if active:
                if pair not in self.contact_pairs and self.run is not None:
                    self.contacts_pending.append(dict(event='contact', colliders=list(pair)))
                self.contact_pairs.add(pair)
            else:
                self.contact_pairs.discard(pair)

    def tick(self, frame=None):
        """Record at 10 Hz; only arrival, timeout, abort or an error ends a run."""
        now = time.monotonic()
        if self.poll is not None and self.poll.done():
            try:
                model = self.poll.result()
                with self.lock:
                    self.model, self.model_received = model, now
            except Exception:
                pass
            self.poll = None
        if now >= self.next_poll and self.poll is None:
            self.next_poll = now+.25
            self.poll = self.workers.submit(self.model_request, '/api/status')
        with self.lock:
            if self.finish_request:
                outcome, message = self.finish_request
                self.finish_request = None
                if self.run is not None:
                    self._finish(outcome, message, frame)
                return
            run = self.run
            if run is None:
                return
            if not self.state.get('playing'):
                self._finish('aborted', 'Simulation paused/stopped from the native interface', frame)
                return
            sim = self.state['sim_time']
            if sim < run['_start_sim']:
                self._finish('aborted', 'Simulation time reset', frame)
                return
            x,y = self.state['x'],self.state['y']
            run['travelled_m'] += math.hypot(x-run['_previous'][0], y-run['_previous'][1])
            run['elapsed_s'] = sim-run['_start_sim']
            for event in score_language_segment(run, self.selected['evaluation'], run['_previous'], (x,y)):
                run['_log'].write(json.dumps(dict(event, elapsed_s=run['elapsed_s']))+'\n')
            run['_previous'] = (x,y)
            run['elapsed_s'] = sim-run['_start_sim']
            target = self.selected['evaluation']
            distance = math.hypot(x-target['x'], y-target['y'])
            run['goal_distance_m'] = distance
            run['min_goal_distance_m'] = distance if run['min_goal_distance_m'] is None else min(distance, run['min_goal_distance_m'])
            if sim >= self.next_sample:
                delta = min(.2, max(0, sim-self.next_sample+.1)) if self.next_sample else 0
                self.next_sample = sim+.1
                if self.contact_pairs:
                    run['contact_seconds'] += delta
                for contact in self.contacts_pending:
                    contact['elapsed_s'] = run['elapsed_s']
                    run['_log'].write(json.dumps(contact)+'\n')
                    run['collision_count'] += 1
                self.contacts_pending.clear()
                control = self.controller.state()
                error = control.get('tracking_error_m')
                if error is not None:
                    run['tracking_error_max_m'] = max(run['tracking_error_max_m'], error)
                    run['_tracking_sum'] += error
                    run['_samples'] += 1
                row = dict(event='sample', elapsed_s=run['elapsed_s'], x=x,y=y,yaw=self.state['yaw'],
                    goal_distance_m=distance, controller=control, active_contacts=[list(p) for p in self.contact_pairs])
                if self.model.get('prediction_epoch') == self.epoch and self.model.get('prediction_seq') != run['_last_prediction']:
                    run['_last_prediction'] = self.model.get('prediction_seq')
                    row['prediction'] = {k: self.model.get(k) for k in ('prediction_seq','trajectory_m',
                        'trajectory_raw','encoded_goal_pose','trajectory_origin','relative_goal','inference_seconds',
                        'observation_sim_time','configuration_revision')}
                run['_log'].write(json.dumps(row)+'\n')
                run['_log'].flush()
            heading = target['heading_tolerance_deg']
            yaw_error = abs(math.degrees(math.atan2(math.sin(self.state['yaw']-math.radians(self.selected['pose_goal']['yaw_deg'])),
                                                      math.cos(self.state['yaw']-math.radians(self.selected['pose_goal']['yaw_deg'])))))
            if distance <= target['radius_m'] and (heading is None or yaw_error <= heading):
                self._finish('reached', 'Goal region reached', frame)
            elif run['elapsed_s'] >= target['timeout_s']:
                self._finish('timeout', 'Scenario time limit reached', frame)
            elif now-run['_start_wall'] > 3:
                if now-self.model_received > 3:
                    self._finish('error', 'Model server connection lost', frame)
                elif (self.model.get('camera_age_seconds') or 0) > 3:
                    self._finish('error', 'Camera/ROS bridge stopped delivering frames', frame)
                elif self.model.get('scenario_epoch') != self.epoch or not self.model.get('scenario_running'):
                    self._finish('aborted', 'Trial was stopped or changed from the model dashboard', frame)

    def _finish(self, outcome, message, frame):
        self.controller.stop()
        run = self.run
        for contact in self.contacts_pending:
            contact['elapsed_s'] = run['elapsed_s']
            run['_log'].write(json.dumps(contact)+'\n')
            run['collision_count'] += 1
        self.contacts_pending.clear()
        result = {k: v for k,v in run.items() if not k.startswith('_')}
        evaluation = self.selected['evaluation']
        has_route_rule = bool(evaluation.get('checkpoints') or evaluation.get('forbidden'))
        language_active = self.selected['mode'] in ('language','language_pose')
        route_ok = run['checkpoints_passed'] == run['checkpoints_total'] and not run['route_violations']
        result.update(outcome=outcome, reached_goal=outcome=='reached',
                      route_observed_compliant=bool(route_ok) if has_route_rule else None,
                      route_compliant=bool(route_ok) if has_route_rule and language_active else None,
                      instruction_success=(outcome=='reached' and bool(route_ok)) if language_active else None,
                      collision_free_success=outcome=='reached' and run['collision_count']==0,
                      tracking_error_mean_m=run['_tracking_sum']/run['_samples'] if run['_samples'] else None,
                      stopping_scored=False, message=message)
        run['_log'].close()
        if frame is not None:
            from PIL import Image
            Image.fromarray(frame).save(run['_folder']/'end.jpg', quality=85)
        (run['_folder']/'summary.json').write_text(json.dumps(result, indent=2))
        self.last_result, self.run = result, None
        self.phase, self.message = 'finished', message
        self.workers.submit(self._stop_model, self.epoch)

    def close(self):
        with self.lock:
            if self.run is not None:
                self._finish('aborted', 'Simulator shut down', None)
        self.workers.shutdown(wait=False, cancel_futures=True)


def spawn_props(stage, props, assets_root):
    """Replace only our /World/Scenario namespace; warehouse assets stay referenced.

    The six procedural prop types use metres and need no downloaded mesh files.
    Each decorative part is visual only; one simple collision shape bounds each
    prop. Custom USD references retain their supplied collision geometry.
    """
    from pxr import Gf, UsdGeom, UsdPhysics
    if stage.GetPrimAtPath('/World/Scenario'):
        stage.RemovePrim('/World/Scenario')
    UsdGeom.Xform.Define(stage, '/World/Scenario')
    for item in props:
        path = '/World/Scenario/'+item['id']
        root = UsdGeom.Xform.Define(stage, path)
        root.AddTranslateOp().Set(Gf.Vec3d(item['x'], item['y'], item['z']))
        root.AddRotateZOp().Set(item['yaw_deg'])
        display = [int(item['color'][i:i+2],16)/255 for i in (1,3,5)]
        # HTML colours are sRGB; USD displayColor is linear.
        color = tuple(v/12.92 if v <= .04045 else ((v+.055)/1.055)**2.4 for v in display)
        sx,sy,sz = item['size']

        def part(name, shape, size, xyz, rgb, collision=False):
            obj = getattr(UsdGeom, shape).Define(stage, path+'/'+name)
            if shape == 'Cube':
                obj.CreateSizeAttr(1.)
            elif shape in ('Cylinder','Cone'):
                obj.CreateRadiusAttr(.5)
                obj.CreateHeightAttr(1.)
                obj.CreateAxisAttr('Z')
            obj.AddTranslateOp().Set(Gf.Vec3d(*xyz))
            obj.AddScaleOp().Set(Gf.Vec3f(*size))
            obj.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
            if collision:
                UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
            return obj

        if item['kind'] == 'usd':
            asset = item['asset']
            root.GetPrim().GetReferences().AddReference(assets_root+'/'+asset if asset.startswith('Isaac/') else asset)
            root.AddScaleOp().Set(Gf.Vec3f(sx,sy,sz))
        elif item['kind'] == 'couch':
            # One bounding collider, with visible seat, back, arms and cushions.
            part('Collider','Cube',(sx,sy,sz),(0,0,sz/2),color,True).GetPrim().GetAttribute('visibility').Set('invisible')
            part('Base','Cube',(sx*.96,sy*.94,sz*.28),(0,0,sz*.18),(.08,.07,.06))
            part('Seat','Cube',(sx*.82,sy*.85,sz*.24),(0,-sy*.04,sz*.43),color)
            part('Back','Cube',(sx,sy*.2,sz*.75),(0,sy*.4,sz*.625),color)
            for i,sign in enumerate((-1,1)):
                part('Arm'+str(i),'Cube',(sx*.09,sy,sz*.62),(sign*sx*.455,0,sz*.4),color)
                part('Cushion'+str(i),'Cube',(sx*.39,sy*.62,sz*.07),(sign*sx*.205,-sy*.08,sz*.585),color)
        elif item['kind'] == 'barrel':
            part('Body','Cylinder',(sx,sy,sz),(0,0,sz/2),color,True)
            for i,z in enumerate((.04,sz*.32,sz*.68,sz-.04)):
                part('Rim'+str(i),'Cylinder',(sx*1.025,sy*1.025,.025),(0,0,z),(.15,.16,.17))
            part('Lid','Cylinder',(sx*.95,sy*.95,.012),(0,0,sz+.005),color)
        elif item['kind'] == 'cone':
            part('Body','Cone',(sx,sy,sz),(0,0,sz/2),color,True)
            part('Base','Cube',(sx,sy,.04),(0,0,.02),(.12,.12,.12))
        elif item['kind'] in ('crate','pallet'):
            body = part('Collider','Cube',(sx,sy,sz),(0,0,sz/2),color,True)
            if item['kind'] == 'crate':
                for i,z in enumerate((.09,sz-.09)):
                    part('Band'+str(i),'Cube',(sx*1.01,sy*1.01,.06),(0,0,z),(.32,.22,.12))
                for i,z in enumerate((sz*.27,sz*.5,sz*.73)):
                    part('Seam'+str(i),'Cube',(sx*1.012,sy*1.012,.008),(0,0,z),(.42,.29,.17))
            else:
                for i in range(5):
                    part('Plank'+str(i),'Cube',(sx,sy*.16,.05),(0,(i-2)*sy*.2,sz+.025),color)
        else:
            part('Body','Cube',(sx,sy,sz),(0,0,sz/2),color,True)
