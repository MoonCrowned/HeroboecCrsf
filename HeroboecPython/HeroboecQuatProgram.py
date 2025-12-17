#!/usr/bin/env python3

import time
import json
import os
import threading
import sys
import select
import math
from dataclasses import dataclass
from typing import Optional, Tuple

from CRSF import CrsfController, CrsfTelemetryData
from TransformMath import Vector3

try:
    import gpiod
except ImportError:
    gpiod = None


_log_file = None


def init_logger():
    global _log_file
    if _log_file is not None:
        return
    try:
        base_dir = os.path.dirname(os.path.abspath(__file__))
        ts = time.strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(base_dir, f"{ts}_quat_program_log.txt")
        _log_file = open(log_path, "w", encoding="utf-8")
    except Exception:
        _log_file = None


def log_print(*args, **kwargs):
    sep = kwargs.get("sep", " ")
    end = kwargs.get("end", "\n")
    text = sep.join(str(a) for a in args)

    sys.stdout.write(text + end)
    sys.stdout.flush()

    if _log_file is not None:
        try:
            _log_file.write(text + end)
            _log_file.flush()
        except Exception:
            pass


def wait_for_trigger(gpio_ctx, hold_time: float):
    interval = 0.05
    key_grace = 0.3

    line = None
    if gpio_ctx is not None:
        line = gpio_ctx[1]

    hold_since = None
    last_r_event = None

    while True:
        now = time.time()

        pin_active = False
        if line is not None:
            try:
                value = line.get_value()
            except Exception:
                value = 0
            pin_active = (value == 1)

        key_event = False
        key_active = False

        stdin_ok = sys.stdin is not None and sys.stdin.isatty()
        if stdin_ok:
            if os.name == "nt":
                try:
                    import msvcrt

                    while msvcrt.kbhit():
                        ch = msvcrt.getch()
                        try:
                            c = ch.decode("utf-8", errors="ignore")
                        except Exception:
                            c = ""
                        if c.lower() == "r":
                            key_event = True
                except Exception:
                    pass
            else:
                try:
                    import termios
                    import tty

                    fd = sys.stdin.fileno()
                    old_settings = termios.tcgetattr(fd)
                    try:
                        tty.setcbreak(fd)
                        rlist, _, _ = select.select([sys.stdin], [], [], 0)
                        if rlist:
                            ch = sys.stdin.read(1)
                            if ch.lower() == "r":
                                key_event = True
                    finally:
                        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                except Exception:
                    pass

        if key_event:
            last_r_event = now

        if last_r_event is not None and (now - last_r_event) <= key_grace:
            key_active = True

        condition = pin_active or key_active

        if condition:
            if hold_since is None:
                hold_since = now
            else:
                elapsed = now - hold_since
                if elapsed >= hold_time:
                    if pin_active:
                        log_print("Триггер по GPIO PIN_26")
                        return "pin"
                    log_print("Триггер по клавише R")
                    return "key"
        else:
            hold_since = None

        time.sleep(interval)


def _clamp(v: float, vmin: float, vmax: float) -> float:
    if v < vmin:
        return vmin
    if v > vmax:
        return vmax
    return v


def _wrap_angle_deg_180(deg: float) -> float:
    return (deg + 900.0) % 360.0 - 180.0


def _angle_error_deg(current_deg: float, target_deg: float) -> float:
    return (target_deg - current_deg + 900.0) % 360.0 - 180.0


def _vec2_yaw_deg_xy(direction: Vector3) -> float:
    # Unity: up=(0,1,0). yaw around up. Project direction to XZ plane.
    # Here we define yaw angle 0 as pointing to +Z.
    x = float(direction.x)
    z = float(direction.z)
    if abs(x) < 1e-9 and abs(z) < 1e-9:
        return 0.0
    # atan2(x, z): 0 at +Z, +90 at +X.
    return _wrap_angle_deg_180((180.0 / math.pi) * math.atan2(x, z))


def _tilt_deg_from_forward(forward: Vector3) -> float:
    # tilt from vertical (0,1,0)
    f = forward.normalized
    dot = _clamp(Vector3.Dot(f, Vector3.up), -1.0, 1.0)
    return (180.0 / math.pi) * math.acos(dot)


@dataclass
class PidState:
    integral: float = 0.0


class HeroboecQuatProgramScenario:
    def __init__(self, controller: CrsfController, config_path: Optional[str] = None):
        self.controller = controller
        self.running = False

        if config_path is None:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(base_dir, "HeroboecQuatProgram_config.json")

        self._load_config(config_path)

        self._attitude_lock = threading.Lock()
        self._ctrl_thread: Optional[threading.Thread] = None
        self._ctrl_running = False

        self._desired_roll = 0.0
        self._desired_pitch = 0.0
        self._desired_yaw = 0.0
        self._desired_throttle = -1.0

        self.zero_altitude_baro = 0.0
        self.zero_azimuth_yaw = 0.0

        self.takeoff_start_down = Vector3.down
        self.takeoff_target_down = Vector3.down

        self.target_azimuth_deg = 0.0
        self.target_height_m = 0.0

        self._alt_pid = PidState()
        self._last_loop_ts = None

        self._landing_touchdown_armed = False
        self._landing_prev_vspeed = 0.0

    def _load_config(self, path: str):
        with open(path, "r", encoding="utf-8") as f:
            cfg = json.load(f)

        self.port = str(cfg.get("port", "COM5"))
        self.trigger_hold_time = float(cfg.get("trigger_hold_time", 2.0))

        self.channel_roll = int(cfg.get("channel_roll", 0))
        self.channel_pitch = int(cfg.get("channel_pitch", 1))
        self.channel_yaw = int(cfg.get("channel_yaw", 3))
        self.channel_throttle = int(cfg.get("channel_throttle", 2))
        self.channel_arm = int(cfg.get("channel_arm", 4))
        self.channel_mode = int(cfg.get("channel_mode", 5))

        self.arm_value = float(cfg.get("arm_value", 1.0))
        self.disarm_value = float(cfg.get("disarm_value", 0.0))
        self.mode_acro_value = float(cfg.get("mode_acro_value", 1.0))

        self.spool_to_half_time = float(cfg.get("spool_to_half_time", 3.0))
        self.half_throttle = float(cfg.get("half_throttle", -0.4))
        self.spool_hold_time = float(cfg.get("spool_hold_time", 2.0))
        self.to_takeoff_throttle_time = float(cfg.get("to_takeoff_throttle_time", 1.0))
        self.takeoff_throttle = float(cfg.get("takeoff_throttle", 1.0))
        self.vertical_takeoff_time = float(cfg.get("vertical_takeoff_time", 6.0))

        self.attitude_kp_pitch = float(cfg.get("attitude_kp_pitch", 3.0))
        self.attitude_kp_yaw = float(cfg.get("attitude_kp_yaw", 3.0))
        self.attitude_kp_roll = float(cfg.get("attitude_kp_roll", 3.0))

        self.axis_invert_pitch = bool(cfg.get("axis_invert_pitch", 0))
        self.axis_invert_yaw = bool(cfg.get("axis_invert_yaw", 0))
        self.axis_invert_roll = bool(cfg.get("axis_invert_roll", 0))

        self.max_cmd_pitch = float(cfg.get("max_cmd_pitch", 0.8))
        self.max_cmd_yaw = float(cfg.get("max_cmd_yaw", 0.8))
        self.max_cmd_roll = float(cfg.get("max_cmd_roll", 0.8))

        self.program_azimuth1 = float(cfg.get("program_azimuth1", 0.0))
        self.program_height1 = float(cfg.get("program_height1", 20.0))
        self.program_time1 = float(cfg.get("program_time1", 20.0))

        self.program_azimuth2 = float(cfg.get("program_azimuth2", 120.0))
        self.program_height2 = float(cfg.get("program_height2", 20.0))
        self.program_time2 = float(cfg.get("program_time2", 20.0))

        self.program_azimuth3 = float(cfg.get("program_azimuth3", -120.0))
        self.program_height3 = float(cfg.get("program_height3", 20.0))
        self.program_time3 = float(cfg.get("program_time3", 20.0))

        self.change_azimuth_speed_deg_s = float(cfg.get("change_azimuth_speed_deg_s", 20.0))
        self.change_height_speed_m_s = float(cfg.get("change_height_speed_m_s", 6.0))

        self.cruise_throttle = float(cfg.get("cruise_throttle", 0.4))

        self.alt_kp = float(cfg.get("alt_kp", 0.8))
        self.alt_ki = float(cfg.get("alt_ki", 0.05))
        self.alt_kd_vspeed = float(cfg.get("alt_kd_vspeed", 0.4))
        self.alt_integral_limit = float(cfg.get("alt_integral_limit", 80.0))

        self.pitch_bias_deg = float(cfg.get("pitch_bias_deg", 8.0))
        self.pitch_limit_deg = float(cfg.get("pitch_limit_deg", 25.0))

        self.landing_start_after_program = bool(cfg.get("landing_start_after_program", 1))

        self.landing_vspeed_high_mps = float(cfg.get("landing_vspeed_high_mps", -3.0))
        self.landing_vspeed_mid_mps = float(cfg.get("landing_vspeed_mid_mps", -2.0))
        self.landing_vspeed_low_mps = float(cfg.get("landing_vspeed_low_mps", -1.0))

        self.landing_alt_mid_m = float(cfg.get("landing_alt_mid_m", 40.0))
        self.landing_alt_low_m = float(cfg.get("landing_alt_low_m", 20.0))

        self.landing_throttle_hover = float(cfg.get("landing_throttle_hover", 0.1))
        self.landing_throttle_kp = float(cfg.get("landing_throttle_kp", 0.35))
        self.landing_throttle_min = float(cfg.get("landing_throttle_min", -1.0))
        self.landing_throttle_max = float(cfg.get("landing_throttle_max", 0.6))

        self.touchdown_enable_below_m = float(cfg.get("touchdown_enable_below_m", 40.0))
        self.touchdown_prev_vspeed_below = float(cfg.get("touchdown_prev_vspeed_below", -0.6))
        self.touchdown_vspeed_above = float(cfg.get("touchdown_vspeed_above", -0.1))
        self.touchdown_max_tilt_deg = float(cfg.get("touchdown_max_tilt_deg", 35.0))
        self.touchdown_alt_below_m = float(cfg.get("touchdown_alt_below_m", 15.0))

        self.control_rate_hz = float(cfg.get("control_rate_hz", 50))

    def _apply_outputs(self):
        with self._attitude_lock:
            r = self._desired_roll
            p = self._desired_pitch
            y = self._desired_yaw
            t = self._desired_throttle

        self.controller.set_channel(self.channel_roll, r)
        self.controller.set_channel(self.channel_pitch, p)
        self.controller.set_channel(self.channel_yaw, y)
        self.controller.set_channel(self.channel_throttle, t)

    def _set_outputs(self, roll: float, pitch: float, yaw: float, throttle: float):
        with self._attitude_lock:
            self._desired_roll = float(roll)
            self._desired_pitch = float(pitch)
            self._desired_yaw = float(yaw)
            self._desired_throttle = float(throttle)

    def stop_all(self):
        log_print("Остановка сценария...")
        self.running = False
        self._ctrl_running = False

        if self._ctrl_thread and self._ctrl_thread.is_alive():
            self._ctrl_thread.join(timeout=1.0)

        time.sleep(0.2)

        self.controller.set_channel(self.channel_roll, 0.0)
        self.controller.set_channel(self.channel_pitch, 0.0)
        self.controller.set_channel(self.channel_yaw, 0.0)
        self.controller.set_channel(self.channel_throttle, -1.0)
        time.sleep(0.2)
        self.controller.set_channel(self.channel_arm, self.disarm_value)
        time.sleep(0.2)
        log_print("Остановлено")

    def tween_alpha(self, duration: float, callback, check_running: bool = True) -> bool:
        start_time = time.time()
        while True:
            elapsed = time.time() - start_time
            if elapsed >= duration:
                callback(1.0)
                break

            alpha = elapsed / duration
            callback(alpha)

            if check_running and not self.running:
                return False

            time.sleep(0.02)

        return True

    def _start_control_loop(self):
        if self._ctrl_running:
            return
        self._ctrl_running = True
        self._last_loop_ts = None
        self._ctrl_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._ctrl_thread.start()

    def _control_loop(self):
        while self.running and self._ctrl_running:
            ts = time.time()
            if self._last_loop_ts is None:
                dt = 1.0 / max(1.0, self.control_rate_hz)
            else:
                dt = max(1e-3, ts - self._last_loop_ts)
            self._last_loop_ts = ts

            tlm = self.controller.get_telemetry()
            self._update_control(tlm, dt)

            self._apply_outputs()

            time.sleep(max(0.0, (1.0 / max(1.0, self.control_rate_hz)) - (time.time() - ts)))

    def _update_control(self, tlm: CrsfTelemetryData, dt: float):
        # This gets overwritten depending on phase by updating targets.
        # In cruise: target azimuth/height.
        # In takeoff/landing: enforce vertical.
        _ = tlm
        _ = dt

    def _compute_vertical_attitude_cmd(self, tlm: CrsfTelemetryData, desired_down: Optional[Vector3] = None) -> Tuple[float, float, float]:
        # Keep Forward pointing up (0,1,0). Use pitch/yaw from Forward error.
        # Use roll control by aligning current Down to desired_down projected on plane perpendicular to Forward.
        tr = tlm.quaternion.unity_transform
        f = tr.Forward.normalized
        d = tr.Down.normalized

        desired_f = Vector3.up
        err_world = Vector3.Cross(f, desired_f)
        # Express error axis in body frame:
        # body.x = right  -> pitch torque axis
        # body.y = up     -> yaw torque axis
        # body.z = forward-> roll torque axis (for vertical takeoff roll ~= yaw around world up)
        err_body = tr.InverseTransformVector(err_world)
        pitch_cmd = _clamp(err_body.x * self.attitude_kp_pitch, -self.max_cmd_pitch, self.max_cmd_pitch)
        yaw_cmd = _clamp(err_body.y * self.attitude_kp_yaw, -self.max_cmd_yaw, self.max_cmd_yaw)

        if self.axis_invert_pitch:
            pitch_cmd = -pitch_cmd
        if self.axis_invert_yaw:
            yaw_cmd = -yaw_cmd

        roll_cmd = 0.0
        if desired_down is not None:
            # Align current down direction to desired down around forward axis
            # Compute signed angle between projected vectors in plane orthogonal to desired_f.
            fd = desired_f
            cur = Vector3.ProjectOnPlane(d, fd).normalized
            tgt = Vector3.ProjectOnPlane(desired_down.normalized, fd).normalized
            if cur.sqrMagnitude > 1e-10 and tgt.sqrMagnitude > 1e-10:
                angle = Vector3.SignedAngle(cur, tgt, fd)
                roll_cmd = _clamp((angle / 45.0) * self.attitude_kp_roll, -self.max_cmd_roll, self.max_cmd_roll)

        if self.axis_invert_roll:
            roll_cmd = -roll_cmd

        return roll_cmd, pitch_cmd, yaw_cmd

    def _compute_cruise_pitch_cmd(self, height_error_m: float, vspeed_mps: float, dt: float) -> float:
        # PID-like output in degrees then mapped to -1..1. Negative pitch channel typically nose-up.
        # We'll output desired pitch angle in degrees then convert.
        self._alt_pid.integral += height_error_m * dt
        self._alt_pid.integral = _clamp(self._alt_pid.integral, -self.alt_integral_limit, self.alt_integral_limit)

        pitch_deg = (
            self.pitch_bias_deg
            + self.alt_kp * height_error_m
            + self.alt_ki * self._alt_pid.integral
            - self.alt_kd_vspeed * vspeed_mps
        )
        pitch_deg = _clamp(pitch_deg, -self.pitch_limit_deg, self.pitch_limit_deg)

        # Map degrees to channel [-1..1] with symmetric mapping.
        # Negative channel in older configs corresponded to nose-up. Keep same convention: pitch_cmd = -pitch_deg / pitch_limit_deg.
        if abs(self.pitch_limit_deg) <= 1e-6:
            return 0.0
        cmd = -pitch_deg / self.pitch_limit_deg
        return _clamp(cmd, -self.max_cmd_pitch, self.max_cmd_pitch)

    def _compute_cruise_yaw_cmd(self, tlm: CrsfTelemetryData) -> float:
        current = _wrap_angle_deg_180(tlm.angles.yaw - self.zero_azimuth_yaw)
        err = _angle_error_deg(current, self.target_azimuth_deg)
        # Map error to command via proportional + clamp.
        cmd = _clamp((err / 45.0) * self.attitude_kp_yaw, -self.max_cmd_yaw, self.max_cmd_yaw)
        if self.axis_invert_yaw:
            cmd = -cmd
        return cmd

    def _set_target_azimuth_slew(self, target_deg: float, duration_s: float) -> bool:
        start = self.target_azimuth_deg
        target = _wrap_angle_deg_180(target_deg)
        diff = _angle_error_deg(start, target)

        def _update(alpha: float):
            self.target_azimuth_deg = _wrap_angle_deg_180(start + diff * alpha)

        return self.tween_alpha(duration_s, _update)

    def _set_target_height_slew(self, target_m: float, duration_s: float) -> bool:
        start = self.target_height_m
        target = float(target_m)

        def _update(alpha: float):
            self.target_height_m = start + (target - start) * alpha

        return self.tween_alpha(duration_s, _update)

    def _set_targets_step(self, azimuth_deg: float, height_m: float) -> bool:
        az = _wrap_angle_deg_180(float(azimuth_deg))
        h = float(height_m)

        az_diff = abs(_angle_error_deg(self.target_azimuth_deg, az))
        az_dur = 0.0
        if self.change_azimuth_speed_deg_s > 1e-6:
            az_dur = az_diff / self.change_azimuth_speed_deg_s

        h_diff = abs(h - self.target_height_m)
        h_dur = 0.0
        if self.change_height_speed_m_s > 1e-6:
            h_dur = h_diff / self.change_height_speed_m_s

        duration = max(az_dur, h_dur)
        if duration <= 0.05:
            self.target_azimuth_deg = az
            self.target_height_m = h
            return True

        ok1 = True
        ok2 = True

        start_az = self.target_azimuth_deg
        start_h = self.target_height_m
        diff_az = _angle_error_deg(start_az, az)

        def _update(alpha: float):
            self.target_azimuth_deg = _wrap_angle_deg_180(start_az + diff_az * alpha)
            self.target_height_m = start_h + (h - start_h) * alpha

        ok1 = self.tween_alpha(duration, _update)
        _ = ok2
        return ok1

    def _start_vertical_takeoff(self) -> bool:
        log_print("Вертикальный взлёт: ACRO, удержание Forward=(0,1,0)")

        # Determine desired down at end of takeoff: point to first azimuth.
        # We define azimuth 0 as +Z, +90 as +X.
        import math

        yaw_rad = math.radians(_wrap_angle_deg_180(self.program_azimuth1))
        # direction in XZ
        az_dir = Vector3(math.sin(yaw_rad), 0.0, math.cos(yaw_rad)).normalized
        self.takeoff_start_down = self.controller.get_telemetry().quaternion.unity_transform.Down
        self.takeoff_target_down = az_dir

        start_time = time.time()
        self._landing_touchdown_armed = False

        while self.running and (time.time() - start_time) < self.vertical_takeoff_time:
            alpha = (time.time() - start_time) / max(1e-6, self.vertical_takeoff_time)
            desired_down = Vector3.Lerp(self.takeoff_start_down, self.takeoff_target_down, alpha)

            tlm = self.controller.get_telemetry()
            r, p, y = self._compute_vertical_attitude_cmd(tlm, desired_down=desired_down)
            #r, p, y = self._compute_vertical_attitude_cmd(tlm, self.takeoff_start_down)

            print(f"r={r:.2f} p={p:.2f} y={y:.2f}")

            self._set_outputs(r, p, y, self.takeoff_throttle)
            self._apply_outputs()

            time.sleep(1.0 / max(5.0, self.control_rate_hz))

        return self.running

    def _start_cruise_program(self) -> bool:
        log_print("Переход к программе высота+азимут (yaw по attitude, roll удерживаем ~0)")

        self._alt_pid.integral = 0.0

        def _cruise_update(tlm: CrsfTelemetryData, dt: float):
            # Height from baro altitude.
            alt = float(tlm.baro_altitude.altitude_meters - self.zero_altitude_baro)
            vs = float(tlm.baro_altitude.vertical_speed_mps)

            height_error = self.target_height_m - alt
            pitch_cmd = self._compute_cruise_pitch_cmd(height_error, vs, dt)
            yaw_cmd = self._compute_cruise_yaw_cmd(tlm)

            # keep roll around 0
            roll_cmd = 0.0

            self._set_outputs(roll_cmd, pitch_cmd, yaw_cmd, self.cruise_throttle)

            log_print(
                f"CR alt={alt:.1f} tH={self.target_height_m:.1f} vs={vs:.2f} eH={height_error:.1f} "
                f"yaw={_wrap_angle_deg_180(tlm.angles.yaw - self.zero_azimuth_yaw):.1f} tYaw={self.target_azimuth_deg:.1f} "
                f"cmd r={roll_cmd:.2f} p={pitch_cmd:.2f} y={yaw_cmd:.2f} thr={self.cruise_throttle:.2f}"
            )

        self._update_control = _cruise_update
        self._start_control_loop()

        # Initialize targets at the end of takeoff to first point
        self.target_azimuth_deg = _wrap_angle_deg_180(self.program_azimuth1)
        self.target_height_m = float(self.program_height1)

        # Run 1
        log_print(f"Цели #1: az={self.target_azimuth_deg:.1f} h={self.target_height_m:.1f} t={self.program_time1:.1f}s")
        time.sleep(max(0.0, self.program_time1))
        if not self.running:
            return False

        # Transition to 2
        log_print("Переход к целям #2")
        if not self._set_targets_step(self.program_azimuth2, self.program_height2):
            return False
        time.sleep(max(0.0, self.program_time2))
        if not self.running:
            return False

        # Transition to 3
        log_print("Переход к целям #3")
        if not self._set_targets_step(self.program_azimuth3, self.program_height3):
            return False
        time.sleep(max(0.0, self.program_time3))
        if not self.running:
            return False

        return True

    def _start_landing(self) -> bool:
        log_print("Посадка: вертикально + управление throttle по vario")

        # Landing control loop overrides update handler
        self._landing_touchdown_armed = False
        self._landing_prev_vspeed = 0.0

        def _landing_update(tlm: CrsfTelemetryData, dt: float):
            alt = float(tlm.baro_altitude.altitude_meters - self.zero_altitude_baro)
            vs = float(tlm.baro_altitude.vertical_speed_mps)

            # desired descent rate depending on altitude
            if alt <= self.landing_alt_low_m:
                target_vs = self.landing_vspeed_low_mps
            elif alt <= self.landing_alt_mid_m:
                target_vs = self.landing_vspeed_mid_mps
            else:
                target_vs = self.landing_vspeed_high_mps

            # Vertical attitude
            r, p, y = self._compute_vertical_attitude_cmd(tlm, desired_down=self.takeoff_target_down)

            # throttle control
            vs_err = target_vs - vs
            thr = self.landing_throttle_hover + self.landing_throttle_kp * vs_err
            thr = _clamp(thr, self.landing_throttle_min, self.landing_throttle_max)

            # touchdown detection arm
            if alt <= self.touchdown_enable_below_m:
                self._landing_touchdown_armed = True

            tilt = _tilt_deg_from_forward(tlm.quaternion.unity_transform.Forward)

            touchdown = False
            if self._landing_touchdown_armed:
                # detect vario step to near zero or tilt spike close to ground
                if (self._landing_prev_vspeed <= self.touchdown_prev_vspeed_below) and (vs >= self.touchdown_vspeed_above):
                    touchdown = True
                if alt <= self.touchdown_alt_below_m and tilt >= self.touchdown_max_tilt_deg:
                    touchdown = True

            self._landing_prev_vspeed = vs

            if touchdown:
                # kill throttle and disarm
                self._set_outputs(0.0, 0.0, 0.0, -1.0)
                self._apply_outputs()
                self.controller.set_channel(self.channel_arm, self.disarm_value)
                self.running = False
                self._ctrl_running = False
                log_print(f"TOUCHDOWN detected: alt={alt:.1f} vs={vs:.2f} tilt={tilt:.1f}")
                return

            self._set_outputs(r, p, y, thr)

            log_print(
                f"LD alt={alt:.1f} vs={vs:.2f} tVs={target_vs:.2f} thr={thr:.2f} tilt={tilt:.1f} "
                f"cmd r={r:.2f} p={p:.2f} y={y:.2f} armed={int(self._landing_touchdown_armed)}"
            )

            _ = dt

        self._update_control = _landing_update
        self._start_control_loop()

        # Wait until landing ends (touchdown sets running=False)
        while self.running:
            time.sleep(0.1)

        return True

    def start_scenario_sync(self):
        self.running = True
        log_print("=== НАЧАЛО СЦЕНАРИЯ (HeroboecQuatProgram) ===")

        log_print("Инициализация...")
        log_print(
            f"Channels: roll={self.channel_roll} pitch={self.channel_pitch} yaw={self.channel_yaw} throttle={self.channel_throttle} "
            f"arm={self.channel_arm} mode={self.channel_mode}"
        )

        self.controller.set_channel(self.channel_roll, 0.0)
        self.controller.set_channel(self.channel_pitch, 0.0)
        self.controller.set_channel(self.channel_yaw, 0.0)
        self.controller.set_channel(self.channel_throttle, -1.0)
        self.controller.set_channel(self.channel_arm, self.arm_value)
        self.controller.set_channel(self.channel_mode, self.mode_acro_value)

        time.sleep(2.0)
        if not self.running:
            return

        tlm = self.controller.get_telemetry()
        self.zero_altitude_baro = float(tlm.baro_altitude.altitude_meters)
        self.zero_azimuth_yaw = float(tlm.angles.yaw)
        log_print(f"Zero: baro={self.zero_altitude_baro:.2f} yaw={self.zero_azimuth_yaw:.2f}")

        # spool-up
        log_print("Spool-up: до half_throttle")
        ok = self.tween_alpha(
            self.spool_to_half_time,
            lambda a: self.controller.set_channel(self.channel_throttle, (-1.0 + (self.half_throttle + 1.0) * a)),
        )
        if not ok:
            return

        log_print("Hold на half")
        time.sleep(self.spool_hold_time)
        if not self.running:
            return

        log_print("Переход к takeoff_throttle")
        ok = self.tween_alpha(
            self.to_takeoff_throttle_time,
            lambda a: self.controller.set_channel(self.channel_throttle, (self.half_throttle + (self.takeoff_throttle - self.half_throttle) * a)),
        )
        if not ok:
            return

        # vertical takeoff with quaternion stabilization
        if not self._start_vertical_takeoff():
            return

        # # program flight
        # if not self._start_cruise_program():
        #     return

        # # landing
        # if self.landing_start_after_program:
        #     if not self._start_landing():
        #         return

        log_print("=== СЦЕНАРИЙ ЗАВЕРШЕН ===")
        self.stop_all()


def on_telemetry(telemetry: CrsfTelemetryData):
    _ = telemetry


def main():
    init_logger()

    controller = CrsfController()
    controller.subscribe('all', on_telemetry)

    scenario = HeroboecQuatProgramScenario(controller)

    gpio_ctx = None
    if gpiod is not None and os.name != "nt":
        try:
            chip = gpiod.Chip("gpiochip3")
            line = chip.get_line(15)
            line.request(consumer="heroboec_quat_program", type=gpiod.LINE_REQ_DIR_IN)
            gpio_ctx = (chip, line)
            log_print("GPIO PIN_26 доступен для запуска сценария")
        except Exception as e:
            log_print(f"Не удалось инициализировать GPIO: {e}")
            gpio_ctx = None

    try:
        controller.connect(
            port=scenario.port,
            baud_rate=420000,
            send_rate=int(max(5, scenario.control_rate_hz))
        )

        log_print(
            f"\nОжидание сигнала запуска: логическая 1 на PIN_26 или клавиша R ({scenario.trigger_hold_time} секунд). "
            f"Нажмите Ctrl+C для выхода.\n"
        )

        while True:
            _ = wait_for_trigger(gpio_ctx, scenario.trigger_hold_time)
            log_print("Запуск сценария HeroboecQuatProgram...")
            scenario.start_scenario_sync()
            log_print("Ожидание 60 секунд перед следующим запуском...")
            time.sleep(60.0)

    except KeyboardInterrupt:
        log_print("\n\nПрерывание пользователем")
    except Exception as e:
        log_print(f"Ошибка: {e}")
        import traceback

        traceback.print_exc()
        if _log_file is not None:
            try:
                traceback.print_exc(file=_log_file)
                _log_file.flush()
            except Exception:
                pass
    finally:
        scenario.stop_all()
        controller.disconnect()
        if gpio_ctx is not None:
            chip, line = gpio_ctx
            try:
                line.release()
            except Exception:
                pass
            try:
                chip.close()
            except Exception:
                pass
        log_print("Завершено")
        if _log_file is not None:
            try:
                _log_file.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
