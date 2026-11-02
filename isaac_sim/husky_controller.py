"""Clearpath wheel control and optional ideal planar trajectory execution.

This module contains no simulator or ROS code. Physics mode requests wheel
speeds; kinematic mode requests ideal body velocities. Neither teleports poses.
"""
from dataclasses import dataclass
import math
import threading

WHEELS = ('front_left_wheel_joint', 'rear_left_wheel_joint',
          'front_right_wheel_joint', 'rear_right_wheel_joint')


@dataclass(frozen=True)
class ControlConfig:
    radius: float = 0.1651
    track: float = 0.555
    turning_multiplier: float = 1.875
    rate: float = 20.0
    timeout: float = 0.5
    max_linear: float = 1.0
    max_angular: float = 1.0
    linear_acceleration: float = 1.0
    angular_acceleration: float = 1.0

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError('Control settings must be finite positive values.')

    @classmethod
    def from_clearpath(cls, path):
        import yaml
        data = yaml.safe_load(path.read_text())['a200_0000']
        p = data['platform_velocity_controller']['ros__parameters']
        return cls(radius=p['wheel_radius'], track=p['wheel_separation'],
                   turning_multiplier=p['wheel_separation_multiplier'],
                   rate=data['controller_manager']['ros__parameters']['update_rate'],
                   timeout=p['cmd_vel_timeout'], max_linear=p['linear.x.max_velocity'],
                   max_angular=p['angular.z.max_velocity'],
                   linear_acceleration=p['linear.x.max_acceleration'],
                   angular_acceleration=p['angular.z.max_acceleration'])


class HuskyController:
    def __init__(self, config):
        self.config = config
        self.lock = threading.Lock()
        self.target = (0.0, 0.0)
        self.velocity = (0.0, 0.0)
        self.last_command = -math.inf
        self.enabled = False

    def command(self, linear, angular, now, enabled=True):
        if not all(math.isfinite(x) for x in (linear, angular, now)):
            raise ValueError('Commands must contain finite numbers.')
        c = self.config
        # Like preserve_turning_radius: scale both axes together at saturation.
        scale = max(1.0, abs(linear)/c.max_linear, abs(angular)/c.max_angular)
        with self.lock:
            self.target = (linear/scale, angular/scale) if enabled else (0.0, 0.0)
            self.last_command = now
            self.enabled = enabled
            if not enabled:
                self.velocity = (0.0, 0.0)

    def stop(self):
        with self.lock:
            self.enabled = False
            self.target = self.velocity = (0.0, 0.0)

    def advance(self, dt, now):
        c = self.config
        with self.lock:
            if not self.enabled or now-self.last_command > c.timeout:
                # Fail-stop sets zero motor targets. Mechanical braking is still finite.
                self.enabled = False
                self.velocity = (0.0, 0.0)
            else:
                def ramp(old, target, acceleration):
                    delta = max(-acceleration*dt, min(acceleration*dt, target-old))
                    return old+delta
                self.velocity = (ramp(self.velocity[0], self.target[0], c.linear_acceleration),
                                 ramp(self.velocity[1], self.target[1], c.angular_acceleration))
            v,w = self.velocity
            half_width = c.track*c.turning_multiplier/2
            left, right = (v-w*half_width)/c.radius, (v+w*half_width)/c.radius
            return [left, left, right, right]

    def state(self):
        with self.lock:
            return dict(enabled=self.enabled, target=list(self.target), limited=list(self.velocity))


class KinematicController(HuskyController):
    """Execute model poses or a selected waypoint without tyre/motor dynamics.

    Predictions are local to their observation's odometry. Each new plan starts
    at the current robot pose, then visits all predicted poses in order, unless
    waypoint_index requests the authors' velocity conversion. No path planning
    or obstacle avoidance is added by either option.
    """
    def __init__(self, config, pose):
        super().__init__(config)
        self.lock = threading.RLock()
        self.pose = tuple(pose)
        self.plan_id = None
        self.segments = []
        self.segment = 0
        self.elapsed = 0.0
        self.reference = None
        self.path_limits = (config.max_linear, config.max_angular)
        self.waypoint_index = None

    @staticmethod
    def waypoint_velocity(row, interval, max_linear, max_angular):
        """Authors' run_omnivla.py conversion, for an already metric waypoint."""
        dx, dy, hx, hy = map(float, row)
        if abs(dx) < 1e-8 and abs(dy) < 1e-8:
            v, w = 0.0, math.atan2(hy, hx)/interval
        elif abs(dx) < 1e-8:
            v, w = 0.0, math.copysign(math.pi/(2*interval), dy)
        else:
            v, w = dx/interval, math.atan(dy/dx)/interval
        v, w = min(.5, max(0.0, v)), min(1.0, max(-1.0, w))
        if v > max_linear and abs(w) <= .001:
            return max_linear, 0.0
        scale = max(1.0, v/max_linear, abs(w)/max_angular)
        return v/scale, w/scale

    @staticmethod
    def angle(value):
        return math.atan2(math.sin(value), math.cos(value))

    def stop(self):
        with self.lock:
            super().stop()
            self.plan_id, self.reference = None, None
            self.segments = []
            self.waypoint_index = None

    def command(self, linear, angular, now, enabled=True):
        # A manual command explicitly takes ownership from trajectory playback.
        with self.lock:
            self.plan_id, self.reference = None, None
            self.segments = []
            self.waypoint_index = None
            super().command(linear, angular, now, enabled)

    def follow_trajectory(self, data, now):
        if not data['enabled']:
            self.stop()
            return
        expiry = float(data['expires_at'])
        if not math.isfinite(expiry) or not 0 < expiry-now <= 1.0:
            raise ValueError('Expired trajectory command')
        plan_id = data['plan_id']
        if not isinstance(plan_id, str) or not 0 < len(plan_id) <= 128:
            raise ValueError('Invalid plan ID')
        limits = float(data['max_linear']), float(data['max_angular'])
        interval = float(data['waypoint_dt'])
        if not all(math.isfinite(v) and 0 < v <= 1 for v in (*limits, interval)):
            raise ValueError('Trajectory speed limits and interval must be within (0, 1]')
        origin = tuple(float(data['trajectory_origin'][k]) for k in ('x', 'y', 'yaw'))
        rows = data['trajectory_m']
        if not all(math.isfinite(v) for v in origin) or not isinstance(rows, list) or not 1 <= len(rows) <= 128:
            raise ValueError('Invalid trajectory origin or horizon')
        points = []
        c, s = math.cos(origin[2]), math.sin(origin[2])
        for row in rows:
            if len(row) != 4:
                raise ValueError('Expected [forward, left, heading_x, heading_y]')
            x,y,hx,hy = map(float, row)
            if not all(math.isfinite(v) for v in (x,y,hx,hy)) or math.hypot(hx,hy) < 1e-6:
                raise ValueError('Nonfinite trajectory or missing heading')
            points.append((origin[0]+c*x-s*y, origin[1]+s*x+c*y,
                           origin[2]+math.atan2(hy,hx)))
        if 'waypoint_index' in data:
            index = data['waypoint_index']
            if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < len(rows):
                raise ValueError('Waypoint index is outside the predicted horizon')
            v, w = self.waypoint_velocity(rows[index], interval, *limits)
            with self.lock:
                self.command(v, w, now)
                self.waypoint_index = index
            return
        with self.lock:
            self.waypoint_index = None
            if plan_id != self.plan_id:
                start = self.pose
                segments = []
                for end in points:
                    duration = max(interval, math.hypot(end[0]-start[0], end[1]-start[1])/limits[0],
                                   abs(self.angle(end[2]-start[2]))/limits[1])
                    segments.append((start, end, duration))
                    start = end
                self.segments, self.plan_id = segments, plan_id
                self.segment, self.elapsed = 0, 0.0
                self.path_limits = limits
            self.last_command, self.enabled = now, True

    def body_velocity(self, pose, dt, now):
        """World-frame vx, vy, yaw rate; contacts may prevent their execution."""
        with self.lock:
            self.pose = tuple(pose)
            if not self.enabled or now-self.last_command > self.config.timeout:
                self.stop()
                return 0.0, 0.0, 0.0
            if self.plan_id is None:
                super().advance(dt, now)
                v,w = self.velocity
                # Average velocity of the exact differential-drive arc this step.
                yaw = pose[2]
                if abs(w) < 1e-8:
                    return v*math.cos(yaw), v*math.sin(yaw), w
                return (v*(math.sin(yaw+w*dt)-math.sin(yaw))/(w*dt),
                        v*(math.cos(yaw)-math.cos(yaw+w*dt))/(w*dt), w)
            remaining = dt
            while remaining > 0 and self.segment < len(self.segments):
                start,end,duration = self.segments[self.segment]
                step = min(remaining, duration-self.elapsed)
                self.elapsed += step
                remaining -= step
                alpha = min(1.0, self.elapsed/duration)
                self.reference = (start[0]+alpha*(end[0]-start[0]),
                                  start[1]+alpha*(end[1]-start[1]),
                                  start[2]+alpha*self.angle(end[2]-start[2]))
                if alpha >= 1.0:
                    self.segment += 1
                    self.elapsed = 0.0
            vx,vy = ((self.reference[i]-pose[i])/dt for i in (0,1))
            w = self.angle(self.reference[2]-pose[2])/dt
            scale = max(1.0, math.hypot(vx,vy)/self.path_limits[0], abs(w)/self.path_limits[1])
            vx,vy,w = vx/scale,vy/scale,w/scale
            self.target = self.velocity = (vx*math.cos(pose[2])+vy*math.sin(pose[2]), w)
            return vx,vy,w

    def state(self):
        with self.lock:
            return dict(super().state(), reference_pose=self.reference,
                        waypoint_index=self.waypoint_index,
                        trajectory_segment=self.segment if self.plan_id else None,
                        tracking_error_m=None if self.reference is None else
                        math.hypot(self.reference[0]-self.pose[0], self.reference[1]-self.pose[1]))
