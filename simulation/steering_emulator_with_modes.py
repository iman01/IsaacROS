#!/usr/bin/env python3
"""
/cmd_vel ➜ /joint_states  (bidirectional)
- Steering: modes = car | 4ws | crab | pivot
- Speed model (forward + reverse kept as-is), plus BRAKE when throttle ~ 0:
    velocity_params/poly_speed.json
    velocity_params/speed_accel_poly.json
    velocity_params/speed_delay.json
    velocity_params/reverse_poly_speed.json
    velocity_params/reverse_speed_accel_poly.json
    velocity_params/reverse_speed_delay.json
    velocity_params/brake_decel_poly.json         (NEW)
    velocity_params/brake_delay.json              (NEW)

Publishes JointState with ['front_left','front_right','rear_left','rear_right','speed'].
"""

import json, math, pathlib, collections
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

HERE = pathlib.Path(__file__).resolve().parent
STEER_CFG_DIR = HERE / "steering_params"
VEL_CFG_DIR   = HERE / "velocity_params"

# ---------- helpers ----------
def _read_json(path: pathlib.Path, default=None):
    try:
        data = json.loads(path.read_text())
        # guard against NaN/inf in user files
        if isinstance(data, dict) and any(
            (isinstance(v, float) and (math.isnan(v) or math.isinf(v))) for v in data.values()
        ):
            return default
        return data
    except Exception:
        return default

deg = lambda r: r * 180.0 / math.pi
rad = lambda d: d * math.pi / 180.0
def poly(p, x):  # cubic a3..a0
    return ((p.get("a3", 0.0) * x + p.get("a2", 0.0)) * x + p.get("a1", 0.0)) * x + p.get("a0", 0.0)

def apoly(P, u):  # quadratic b2,b1,b0; always non-negative
    if P is None: return 0.0
    return max(0.0, P.get("b2", 0.0) * u * u + P.get("b1", 0.0) * u + P.get("b0", 0.0))

def abrake_mag(C, v_abs):  # quadratic c2,c1,c0; non-negative
    if C is None: return 0.0
    return max(0.0, C.get("c2",0.0)*v_abs*v_abs + C.get("c1",0.0)*v_abs + C.get("c0",0.0))

# ---------- steering calibration (unchanged) ----------
POLY_STEER = {
    "L": _read_json(STEER_CFG_DIR / "poly_front_left.json",  {"a3":0,"a2":0,"a1":1,"a0":0}),
    "R": _read_json(STEER_CFG_DIR / "poly_front_right.json", {"a3":0,"a2":0,"a1":1,"a0":0}),
}
RATE_RAW = _read_json(STEER_CFG_DIR / "steering_rate_profile.json", {"pos":90.0,"neg":90.0})  # deg/s
RATE = {"L":{"pos":abs(RATE_RAW.get("pos",90)),"neg":abs(RATE_RAW.get("neg",90))},
        "R":{"pos":abs(RATE_RAW.get("pos",90)),"neg":abs(RATE_RAW.get("neg",90))}}

# ---------- velocity calibration ----------
# Forward files (existing)
FWD_POLY  = _read_json(VEL_CFG_DIR / "poly_speed.json",          {"a3":0,"a2":0,"a1":1,"a0":0})
FWD_ACC   = _read_json(VEL_CFG_DIR / "speed_accel_poly.json",    {"b2":0.2,"b1":0.0,"b0":0.02})
FWD_DELAY = _read_json(VEL_CFG_DIR / "speed_delay.json",         {"Td_ms":0})

# Reverse files (existing)
REV_POLY  = _read_json(VEL_CFG_DIR / "reverse_poly_speed.json",       None)
REV_ACC   = _read_json(VEL_CFG_DIR / "reverse_speed_accel_poly.json", None)
REV_DELAY = _read_json(VEL_CFG_DIR / "reverse_speed_delay.json",      None)

# Brake files (NEW)
BRK_ACC   = _read_json(VEL_CFG_DIR / "brake_decel_poly.json",   None)   # c2,c1,c0 in m/s²
BRK_DELAY = _read_json(VEL_CFG_DIR / "brake_delay.json",        {"Td_ms":0})
BRK_TD_S  = max(0.0, float(BRK_DELAY.get("Td_ms", 0)) / 1000.0)         # clamp negatives to 0

class SteeringEmulator(Node):
    def __init__(self, hz: float = 30.0):
        super().__init__("steering_emulator",
            allow_undeclared_parameters=False,
            automatically_declare_parameters_from_overrides=False)

        self.declare_parameter("steering_mode", "car",
            ParameterDescriptor(description="car | 4ws | crab | pivot"))
        self.mode = self.get_parameter("steering_mode").value.lower()
        self.add_on_set_parameters_callback(self._on_set)

        self.hz = float(hz); self.dt = 1.0 / self.hz
        self.timeout = 2.0
        self._cmd_ang = 0.0      # rad
        self._cmd_thr = 0.0      # [-1..1]

        # Transport delays per direction
        self.delay_f = float(FWD_DELAY.get("Td_ms", 0)) / 1000.0
        self.delay_r = float((REV_DELAY or FWD_DELAY).get("Td_ms", 0)) / 1000.0

        self._q = collections.deque()  # (timestamp, throttle)
        self.v = 0.0                   # m/s
        self.fl = self.fr = self.rl = self.rr = 0.0

        # --- BRAKE state ---
        self.brake_deadband = 0.02     # |u| <= deadband → brake intent
        self.brake_arm_time = None     # wall-clock seconds when braking begins (after delay)
        self.brake_v_eps = 0.01        # snap-to-zero threshold

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(self.dt, self._tick)
        self.last_cmd_time = self.get_clock().now()

        self.get_logger().info(
            f"[steer] Hz={hz:.0f} mode={self.mode} | "
            f"[speed] delay_f={self.delay_f*1000:.0f}ms  delay_r={self.delay_r*1000:.0f}ms | "
            f"[brake] Td={BRK_TD_S*1000:.0f}ms"
        )

    def _on_set(self, params):
        for p in params:
            if p.name == "steering_mode":
                self.mode = str(p.value).lower()
        return SetParametersResult(successful=True)

    def _on_cmd(self, msg: Twist):
        # Accept full bidirectional range for throttle
        self._cmd_ang = -msg.angular.z
        self._cmd_thr = float(max(-1.0, min(1.0, msg.linear.x)))
        t = self.get_clock().now().nanoseconds * 1e-9
        self._q.append((t, self._cmd_thr))
        self.last_cmd_time = self.get_clock().now()

    def _delayed_u(self, now_s: float) -> float:
        """Return throttle after per-direction transport delay (fwd/rev)."""
        if not self._q:
            return self._cmd_thr
        target_delay = self.delay_f if self._cmd_thr >= 0.0 else self.delay_r
        if target_delay <= 0.0:
            return self._cmd_thr
        cutoff = now_s - target_delay
        while len(self._q) >= 2 and self._q[1][0] <= cutoff:
            self._q.popleft()
        if len(self._q) >= 2:
            (t0,u0),(t1,u1) = self._q[0], self._q[1]
            if t1 > t0:
                a = max(0.0, min(1.0, (cutoff - t0)/(t1 - t0)))
                return (1.0 - a)*u0 + a*u1
        return self._q[0][1]

    def _slew(self, cur, tgt, lim_pos, lim_neg):
        step = rad(lim_pos if tgt > cur else lim_neg) * self.dt
        return cur + max(-step, min(step, tgt - cur))

    def _tick(self):
        now = self.get_clock().now(); now_s = now.nanoseconds * 1e-9
        timed_out = (now - self.last_cmd_time).nanoseconds/1e9 > self.timeout

        u   = 0.0 if timed_out else self._delayed_u(now_s)   # throttle [-1..1]
        ang = 0.0 if timed_out else self._cmd_ang            # rad

        # --- steering (unchanged) ---
        tgtL = rad(poly(POLY_STEER["L"], deg(ang)))
        tgtR = rad(poly(POLY_STEER["R"], deg(ang)))
        if self.mode == "car":
            tfl,tfr,trl,trr = tgtL,tgtR,0.0,0.0
        elif self.mode == "4ws":
            tfl,tfr,trl,trr = tgtL,tgtR,-tgtL,-tgtR
        elif self.mode == "crab":
            tfl,tfr,trl,trr = tgtL,tgtR,tgtL,tgtR
        elif self.mode == "pivot":
            tfl,tfr,trl,trr = tgtL,-tgtR,-tgtL,tgtR
        else:
            tfl=tfr=trl=trr=0.0
        self.fl = self._slew(self.fl, tfl, RATE["L"]["pos"], RATE["L"]["neg"])
        self.fr = self._slew(self.fr, tfr, RATE["R"]["pos"], RATE["R"]["neg"])
        self.rl = self._slew(self.rl, trl, RATE["L"]["pos"], RATE["L"]["neg"])
        self.rr = self._slew(self.rr, trr, RATE["R"]["pos"], RATE["R"]["neg"])

        # -------------------- SPEED DYNAMICS --------------------
        # Decide which mode to use
        brake_intent = (abs(u) <= self.brake_deadband)

        if brake_intent and BRK_ACC is not None:
            # Arm once; apply transport delay BEFORE braking
            if self.brake_arm_time is None:
                self.brake_arm_time = now_s + BRK_TD_S

            if now_s < self.brake_arm_time:
                # still waiting out the delay — hold current v as target
                v_inf = self.v
                a_cap = 0.0
            else:
                # target is full stop; **adaptive** decel based on current |v|
                v_inf = 0.0
                a_cap = abrake_mag(BRK_ACC, abs(self.v))

            # snap to zero near rest to avoid a long tail
            if abs(self.v) < self.brake_v_eps and now_s >= (self.brake_arm_time or 0.0):
                self.v = 0.0
                a_cap = 0.0

        else:
            # Leaving brake intent → clear the arm
            self.brake_arm_time = None

            # --- Forward/Reverse (unchanged) ---
            if u >= 0.0:
                u_f = min(1.0, max(0.0, u))
                v_inf = poly(FWD_POLY, u_f)
                a_cap = apoly(FWD_ACC,  u_f)
            else:
                P = REV_POLY or FWD_POLY
                A = REV_ACC  or FWD_ACC
                u_r = min(1.0, max(0.0, -u))  # [-1..0) → (0..1]
                v_inf = -poly(P, u_r)
                a_cap =  apoly(A, u_r)

        # first-order acceleration limit toward v_inf
        dv = max(-a_cap*self.dt, min(a_cap*self.dt, v_inf - self.v))
        self.v += dv
        if timed_out and not brake_intent:
            self.v *= 0.98  # decay if no commands and not braking

        # publish joint states
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ["front_left","front_right","rear_left","rear_right","speed"]
        js.position = [self.fl, self.fr, self.rl, self.rr, self.v]
        self.pub.publish(js)

def main():
    rclpy.init()
    rclpy.spin(SteeringEmulator())
    rclpy.shutdown()

if __name__ == "__main__":
    main()

