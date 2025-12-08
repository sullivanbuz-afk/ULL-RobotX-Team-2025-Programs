#!/usr/bin/env python3
"""
Policy node (bearing + distance) with gate detection:
- Gate = red+green pair whose lateral separation is within [min_gate_width_m, max_gate_width_m].
- If a valid gate exists -> steer toward gate center.
- Else: red/green = avoidance (steer away), black = circle mode only.

Inputs (Float32MultiArray on /cam_left/minimal and /cam_right/minimal):
  Preferred 5-tuple per detection: [cls_idx, conf, bearing_deg, dist_m, spare]
  Fallback 4-tuple per detection:  [cls_idx, conf, zone_num,   dist_m]
    where zone_num ∈ {-1,0,+1}. Bearing is approximated from HFOV.

Output (Float32MultiArray on /thrust_cmd): [uL, uR]

Now also includes optional CSV logging for ML:
  - controlled by `log_enabled` (bool) and `log_dir` (string).
"""

from typing import List, Tuple, Optional
import os
import csv
import time
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray

# Class mapping used by your detector
# 0=black, 1=green, 2=red
CLASS_ORDER = ['black', 'green', 'red']
IDX_BLACK, IDX_GREEN, IDX_RED = 0, 1, 2


class PolicyNode(Node):
    def __init__(self):
        super().__init__('policy_node')

        # ---------- Topics ----------
        self.declare_parameter('left_topic',   'cam_left/minimal')
        self.declare_parameter('right_topic',  'cam_right/minimal')
        self.declare_parameter('thrust_topic', 'thrust_cmd')

        # ---------- Timing / staleness ----------
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('stale_timeout_s', 1.0)

        # How long to keep last maneuver after detections disappear
        self.declare_parameter('no_det_hold_s', 5.0)  # seconds

        # ---------- Camera geometry (for zone->bearing fallback) ----------
        self.declare_parameter('hfov_deg', 87.0)        # Horizontal FOV
        self.declare_parameter('bearing_max_deg', 43.5) # ≈ HFOV/2

        # ---------- Steering & speed ----------
        self.declare_parameter('turn_deadband_deg', 4.0)
        self.declare_parameter('steer_gain', 0.8)

        # ---------- Distance thresholds ----------
        self.declare_parameter('panic_distance_m', 2.0)
        self.declare_parameter('slow_distance_m',  6.0)
        self.declare_parameter('far_distance_m',  12.0)

        # ---------- Forward throttle levels ----------
        self.declare_parameter('forward_base', 0.8)
        self.declare_parameter('forward_slow', 0.5)
        self.declare_parameter('forward_min',  0.4)

        # ---------- Output smoothing ----------
        self.declare_parameter('output_alpha', 0.30)

        # ---------- Gate logic ----------
        # Accept only gates between 3 and 10 meters wide
        self.declare_parameter('min_gate_width_m', 3.0)
        self.declare_parameter('max_gate_width_m', 10.0)
        self.declare_parameter('gate_max_range_diff_m', 2.0)  # red/green ranges must be roughly similar

        # ---------- Black buoy circle mode ----------
        self.declare_parameter('circle_enable', True)
        self.declare_parameter('circle_target_radius_m', 3.0)
        self.declare_parameter('circle_bearing_setpoint_deg', +30.0)  # keep black buoy ~30° to left (CW circle)
        self.declare_parameter('circle_kp_radius', 0.08)   # forward bias from radius error
        self.declare_parameter('circle_kp_bearing', 0.6)   # steer toward bearing SP

        # ---------- Logging (for ML) ----------
        self.declare_parameter('log_enabled', False)
        self.declare_parameter('log_dir', '~/logs')

        # ---------- Resolve params ----------
        left_topic        = self.get_parameter('left_topic').value
        right_topic       = self.get_parameter('right_topic').value
        thrust_topic      = self.get_parameter('thrust_topic').value
        rate_hz           = float(self.get_parameter('control_rate_hz').value)
        self.stale_to     = float(self.get_parameter('stale_timeout_s').value)
        self.no_det_hold_s = float(self.get_parameter('no_det_hold_s').value)

        self.hfov_deg        = float(self.get_parameter('hfov_deg').value)
        self.bearing_max_deg = float(self.get_parameter('bearing_max_deg').value)
        self.deadband_deg    = float(self.get_parameter('turn_deadband_deg').value)
        self.steer_gain      = float(self.get_parameter('steer_gain').value)

        self.panic_dist = float(self.get_parameter('panic_distance_m').value)
        self.slow_dist  = float(self.get_parameter('slow_distance_m').value)
        self.far_dist   = float(self.get_parameter('far_distance_m').value)

        self.fwd_base = float(self.get_parameter('forward_base').value)
        self.fwd_slow = float(self.get_parameter('forward_slow').value)
        self.fwd_min  = float(self.get_parameter('forward_min').value)

        self.out_alpha = float(self.get_parameter('output_alpha').value)

        self.min_gate_w   = float(self.get_parameter('min_gate_width_m').value)
        self.max_gate_w   = float(self.get_parameter('max_gate_width_m').value)
        self.gate_dz_max  = float(self.get_parameter('gate_max_range_diff_m').value)

        self.circle_enable   = bool(self.get_parameter('circle_enable').value)
        self.circle_radius_m = float(self.get_parameter('circle_target_radius_m').value)
        self.circle_b_sp_deg = float(self.get_parameter('circle_bearing_setpoint_deg').value)
        self.circle_kp_r     = float(self.get_parameter('circle_kp_radius').value)
        self.circle_kp_b     = float(self.get_parameter('circle_kp_bearing').value)

        # Logging resolve
        self.log_enabled = bool(self.get_parameter('log_enabled').value)
        self.log_dir = os.path.expanduser(self.get_parameter('log_dir').value)
        self.log_file = None
        self.log_writer = None
        if self.log_enabled:
            self._init_logger()

        # ---------- ROS I/O ----------
        self.pub_thrust = self.create_publisher(Float32MultiArray, thrust_topic, 10)

        self.last_L = None
        self.last_R = None
        self.last_L_t: Optional[float] = None
        self.last_R_t: Optional[float] = None

        self.create_subscription(Float32MultiArray, left_topic,  self.cb_left,  qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, right_topic, self.cb_right, qos_profile_sensor_data)

        tick_period = 1.0 / max(rate_hz, 1.0)
        self.timer = self.create_timer(tick_period, self.tick)

        self.uL_filt: Optional[float] = None
        self.uR_filt: Optional[float] = None

        # Remember last time we had any meaningful detection (for hold logic)
        self.last_detection_t: Optional[float] = None

        self.get_logger().info(
            "policy_node (gate 3–10m + avoidance + black circle + logging) online | "
            f"gate [{self.min_gate_w}-{self.max_gate_w}] m, dz<{self.gate_dz_max} m | "
            f"no_det_hold_s={self.no_det_hold_s} | log_enabled={self.log_enabled}"
        )

    # ---------- Logging helpers ----------
    def _init_logger(self):
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            fname = time.strftime("policy_log_%Y%m%d_%H%M%S.csv")
            path = os.path.join(self.log_dir, fname)
            self.log_file = open(path, "w", newline="")
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow([
                "t_epoch",
                "mode",
                "cmd_time",              # previously detect_ts; now just "command timestamp"
                "mindist",
                "n_red",
                "n_green",
                "n_black",
                "selected_cls",
                "selected_bearing_deg",
                "selected_dist_m",
                "forward_cmd",
                "steer_cmd",
                "uL_cmd",
                "uR_cmd",
            ])
            self.get_logger().info(f"Data logging enabled → {path}")
        except Exception as e:
            self.get_logger().error(f"Failed to init logger: {e}")
            self.log_enabled = False
            self.log_file = None
            self.log_writer = None

    def _maybe_log(
        self,
        mode: str,
        cmd_time: Optional[float],
        mindist: Optional[float],
        selected_cls: Optional[int],
        selected_bearing: Optional[float],
        selected_dist: Optional[float],
        forward: Optional[float],
        steer: Optional[float],
        uL: float,
        uR: float,
        n_red: int,
        n_green: int,
        n_black: int,
    ):
        if not self.log_enabled or self.log_writer is None:
            return
        try:
            self.log_writer.writerow([
                time.time(),  # log wall time of row
                mode,
                float(cmd_time) if cmd_time is not None else "",
                mindist if mindist is not None else "",
                n_red,
                n_green,
                n_black,
                selected_cls if selected_cls is not None else "",
                selected_bearing if selected_bearing is not None else "",
                selected_dist if selected_dist is not None else "",
                forward if forward is not None else "",
                steer if steer is not None else "",
                uL,
                uR,
            ])
            self.log_file.flush()
        except Exception as e:
            self.get_logger().error(f"Logging error: {e}")
            self.log_enabled = False

    def destroy_node(self):
        if self.log_file is not None:
            try:
                self.log_file.close()
            except Exception:
                pass
        super().destroy_node()

    # ---------- Helpers ----------
    @staticmethod
    def _chunk(v: List[float], k: int) -> List[List[float]]:
        n = len(v) - (len(v) % k)
        return [v[i:i+k] for i in range(0, n, k)]

    def _parse_minimal(self, data: List[float]) -> List[Tuple[int, float, float, float]]:
        """
        Returns list of (cls_idx, conf, bearing_deg, dist_m).
        Accepts:
          * 5-tuples: [cls, conf, bearing_deg, dist_m, spare]
          * 4-tuples: [cls, conf, zone,        dist_m]  -> zone -> approx bearing
        """
        v = list(map(float, data or []))

        if len(v) >= 5 and (len(v) % 5 == 0):
            out = []
            for cls_f, conf, bearing_deg, dist_m, _ in self._chunk(v, 5):
                try:
                    cid = int(cls_f)
                    if 0 <= cid < len(CLASS_ORDER):
                        out.append((cid, float(conf), float(bearing_deg), float(dist_m)))
                except Exception:
                    continue
            return out

        out = []
        if len(v) >= 4 and (len(v) % 4 == 0):
            zone_to_bearing = {-1.0: -self.hfov_deg/3.0, 0.0: 0.0, 1.0: self.hfov_deg/3.0}
            for cls_f, conf, zone, dist_m in self._chunk(v, 4):
                try:
                    cid = int(cls_f)
                    if 0 <= cid < len(CLASS_ORDER):
                        b = zone_to_bearing.get(float(zone), 0.0)
                        out.append((cid, float(conf), b, float(dist_m)))
                except Exception:
                    continue
        return out

    @staticmethod
    def _nearest(dets: List[Tuple[int, float, float, float]],
                 mask_cls: Optional[List[int]] = None):
        """
        dets: list of (cls, conf, bearing_deg, dist_m)
        mask_cls: optional list of class indices to include
        """
        cands = [d for d in dets if (mask_cls is None or d[0] in mask_cls)]
        if not cands:
            return None
        return min(cands, key=lambda t: t[3])

    # ---------- Gate utilities ----------
    @staticmethod
    def _deg2rad(d: float) -> float:
        return d * math.pi / 180.0

    def _gate_candidates(self,
                         reds:  List[Tuple[int, float, float, float]],
                         greens:List[Tuple[int, float, float, float]]):
        """
        Build all (red, green) pairs as candidates.
        Each item is ((r_cls, r_conf, r_b, r_Z), (g_cls, g_conf, g_b, g_Z)).
        """
        for r in reds:
            for g in greens:
                yield r, g

    def _gate_width_and_center_bearing(self, red, green):
        """
        red/green: (cls, conf, bearing_deg, Z)
        Returns (width_m, center_bearing_deg, mean_Z, dz)
        """
        _, _, br_deg, Zr = red
        _, _, bg_deg, Zg = green

        br = self._deg2rad(br_deg)
        bg = self._deg2rad(bg_deg)

        # Lateral positions at camera plane
        xr = Zr * math.tan(br)
        xg = Zg * math.tan(bg)

        width = abs(xr - xg)
        Zm = 0.5 * (Zr + Zg)
        dz = abs(Zr - Zg)

        xc = 0.5 * (xr + xg)
        # Center bearing from (xc, Zm)
        bc = math.degrees(math.atan2(xc, Zm + 1e-6))
        return width, bc, Zm, dz

    # ---------- Subs ----------
    def cb_left(self, msg: Float32MultiArray):
        self.last_L = msg
        self.last_L_t = time.monotonic()

    def cb_right(self, msg: Float32MultiArray):
        self.last_R = msg
        self.last_R_t = time.monotonic()

    # ---------- Speed profile ----------
    def _forward_from_dist(self, dist_m: float) -> float:
        if dist_m >= self.far_dist:
            return self.fwd_base
        if dist_m <= self.slow_dist:
            t = np.clip(dist_m / max(1e-6, self.slow_dist), 0.0, 1.0)
            return (1.0 - t) * self.fwd_min + t * self.fwd_slow
        t = np.clip((dist_m - self.slow_dist) / max(1e-6, (self.far_dist - self.slow_dist)), 0.0, 1.0)
        return (1.0 - t) * self.fwd_slow + t * self.fwd_base

    # ---------- Output smoothing ----------
    def _publish_thrust_smooth(self, uL: float, uR: float):
        """
        Smooths uL/uR and publishes [uL_filt, uR_filt] on /thrust_cmd.
        """
        a = self.out_alpha
        if self.uL_filt is None:
            self.uL_filt, self.uR_filt = float(uL), float(uR)
        else:
            self.uL_filt = (1.0 - a) * self.uL_filt + a * float(uL)
            self.uR_filt = (1.0 - a) * self.uR_filt + a * float(uR)

        msg = Float32MultiArray()
        msg.data = [self.uL_filt, self.uR_filt]
        self.pub_thrust.publish(msg)

    # ---------- Main control ----------
    def tick(self):
        now = time.monotonic()
        if self.last_L is None and self.last_R is None:
            return

        # Drop stale sides
        L_msg = self.last_L if (self.last_L and (now - (self.last_L_t or 0.0)) <= self.stale_to) \
                else Float32MultiArray(data=[])
        R_msg = self.last_R if (self.last_R and (now - (self.last_R_t or 0.0)) <= self.stale_to) \
                else Float32MultiArray(data=[])

        L = self._parse_minimal(getattr(L_msg, 'data', []))
        R = self._parse_minimal(getattr(R_msg, 'data', []))
        dets = L + R  # union of both cameras

        # Split by class for gate and behaviors
        reds   = [d for d in dets if d[0] == IDX_RED]
        greens = [d for d in dets if d[0] == IDX_GREEN]
        blacks = [d for d in dets if d[0] == IDX_BLACK]

        # Panic stop if anything is too close (any class)
        mindist = None
        for _, _, _, d in dets:
            mindist = d if mindist is None else min(mindist, d)
        if mindist is not None and mindist < self.panic_dist:
            self.last_detection_t = now
            cmd_ts = time.time()
            uL = 0.0
            uR = 0.0
            self._maybe_log(
                mode="panic",
                cmd_time=cmd_ts,
                mindist=mindist,
                selected_cls=None,
                selected_bearing=None,
                selected_dist=mindist,
                forward=0.0,
                steer=0.0,
                uL=uL,
                uR=uR,
                n_red=len(reds),
                n_green=len(greens),
                n_black=len(blacks),
            )
            self._publish_thrust_smooth(uL, uR)
            return

        # ---------- Try to form a valid gate (3–10 m) ----------
        best_gate = None  # (width, bc, Zm, dz, red, green)
        for red, green in self._gate_candidates(reds, greens):
            width, bc, Zm, dz = self._gate_width_and_center_bearing(red, green)
            if (self.min_gate_w <= width <= self.max_gate_w) and (dz <= self.gate_dz_max):
                # prefer the nearest valid gate (smallest mean Z)
                if (best_gate is None) or (Zm < best_gate[2]):
                    best_gate = (width, bc, Zm, dz, red, green)

        if best_gate is not None:
            width, bc, Zm, dz, red, green = best_gate
            b = float(bc)
            if abs(b) < self.deadband_deg:
                steer = 0.0
            else:
                norm = np.clip(b / max(1e-6, self.bearing_max_deg), -1.0, +1.0)
                steer = self.steer_gain * norm

            forward = self._forward_from_dist(Zm)
            uL = float(np.clip(forward - steer, -1.0, 1.0))
            uR = float(np.clip(forward + steer, -1.0, 1.0))
            self.last_detection_t = now
            cmd_ts = time.time()
            self._maybe_log(
                mode="gate",
                cmd_time=cmd_ts,
                mindist=mindist,
                selected_cls=None,
                selected_bearing=b,
                selected_dist=Zm,
                forward=forward,
                steer=steer,
                uL=uL,
                uR=uR,
                n_red=len(reds),
                n_green=len(greens),
                n_black=len(blacks),
            )
            self._publish_thrust_smooth(uL, uR)
            return

        # ---------- No valid gate: behavior split ----------
        # Black buoy circle (if enabled and detected)
        if self.circle_enable and blacks:
            blk = self._nearest(blacks)
            if blk is not None:
                _, _, b_deg, Z = blk
                # Maintain tangential course: keep black at a fixed bearing setpoint
                e_b = float(self.circle_b_sp_deg - b_deg)  # positive -> turn left
                e_r = float(self.circle_radius_m - Z)      # positive -> need to come closer
                steer = np.clip(self.circle_kp_b * (e_b / max(self.bearing_max_deg, 1e-6)), -1.0, 1.0)
                forward = self.fwd_min + self.circle_kp_r * e_r   # small bias from radius error
                forward = float(np.clip(forward, 0.05, self.fwd_base))
                uL = float(np.clip(forward - steer, -1.0, 1.0))
                uR = float(np.clip(forward + steer, -1.0, 1.0))
                self.last_detection_t = now
                cmd_ts = time.time()
                self._maybe_log(
                    mode="circle",
                    cmd_time=cmd_ts,
                    mindist=mindist,
                    selected_cls=IDX_BLACK,
                    selected_bearing=b_deg,
                    selected_dist=Z,
                    forward=forward,
                    steer=steer,
                    uL=uL,
                    uR=uR,
                    n_red=len(reds),
                    n_green=len(greens),
                    n_black=len(blacks),
                )
                self._publish_thrust_smooth(uL, uR)
                return

        # Red/Green avoidance (no gate)
        rg = self._nearest(dets, mask_cls=[IDX_RED, IDX_GREEN])
        if rg is not None:
            cls_idx, _, b_deg, Z = rg
            # steer AWAY from the buoy: invert sign of steer component
            if abs(b_deg) < self.deadband_deg:
                steer = 0.0
            else:
                norm = np.clip(b_deg / max(1e-6, self.bearing_max_deg), -1.0, +1.0)
                steer = -self.steer_gain * norm  # negative -> turn right if buoy is left, etc.

            forward = self._forward_from_dist(Z)
            uL = float(np.clip(forward - steer, -1.0, 1.0))
            uR = float(np.clip(forward + steer, -1.0, 1.0))
            self.last_detection_t = now
            cmd_ts = time.time()
            self._maybe_log(
                mode="rg_avoid",
                cmd_time=cmd_ts,
                mindist=mindist,
                selected_cls=cls_idx,
                selected_bearing=b_deg,
                selected_dist=Z,
                forward=forward,
                steer=steer,
                uL=uL,
                uR=uR,
                n_red=len(reds),
                n_green=len(greens),
                n_black=len(blacks),
            )
            self._publish_thrust_smooth(uL, uR)
            return

        # ---------- Nothing detected ----------
        # If we recently had a detection, keep last thrust instead of snapping to cruise.
        if (
            self.last_detection_t is not None
            and (now - self.last_detection_t) < self.no_det_hold_s
            and self.uL_filt is not None
            and self.uR_filt is not None
        ):
            # Hold the last maneuver (keeps turning / avoiding)
            uL = float(self.uL_filt)
            uR = float(self.uR_filt)
            self._maybe_log(
                mode="hold",
                cmd_time=None,
                mindist=mindist,
                selected_cls=None,
                selected_bearing=None,
                selected_dist=None,
                forward=None,
                steer=None,
                uL=uL,
                uR=uR,
                n_red=len(reds),
                n_green=len(greens),
                n_black=len(blacks),
            )
            self._publish_thrust_smooth(uL, uR)
        else:
            # Truly clear: go back to straight cruise
            uL = self.fwd_base
            uR = self.fwd_base
            self._maybe_log(
                mode="cruise",
                cmd_time=None,
                mindist=mindist,
                selected_cls=None,
                selected_bearing=None,
                selected_dist=None,
                forward=self.fwd_base,
                steer=0.0,
                uL=uL,
                uR=uR,
                n_red=len(reds),
                n_green=len(greens),
                n_black=len(blacks),
            )
            self._publish_thrust_smooth(uL, uR)


def main():
    rclpy.init()
    rclpy.spin(PolicyNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()