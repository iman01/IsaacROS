#!/usr/bin/env python3
"""
/cmd_vel ➜ /joint_states   with *independent* L/R dynamics.
Modes: car | 4ws | crab | pivot
"""

import json, math, pathlib, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult



# ------------------------------------------------------------------ #
# 1 · Load configuration                                             #
# ------------------------------------------------------------------ #
CFG = pathlib.Path(__file__).resolve().parent / "steering_params"

def _load(name, default):
    try:
        return json.loads((CFG / name).read_text())
    except Exception:
        return default

POLY = {
    "L": _load("poly_front_left.json",
               {"a3": 0.0, "a2": 0.0, "a1": 1.0, "a0": 0.0}),
    "R": _load("poly_front_right.json",
               {"a3": 0.0, "a2": 0.0, "a1": 1.0, "a0": 0.0}),
    "speed": _load("poly_speed.json",
                {"a3": 0.0, "a2": 0.0, "a1": 1.0, "a0": 0.0})
}

RATE_RAW = _load("steering_rate_profile.json",
                 {"pos": 90.0, "neg": 90.0})          # deg / s

def _lim(side, sign):                     # resolves flat vs nested layout
    if "pos" in RATE_RAW:
        return abs(RATE_RAW[sign])
    return abs(RATE_RAW[side][sign])

RATE = {
    "L": {"pos": _lim("L", "pos"), "neg": _lim("L", "neg")},
    "R": {"pos": _lim("R", "pos"), "neg": _lim("R", "neg")},
    
}

SPEED_RATE_RAW = _load("speed_rate_profile.json",
                       {"pos": 1.0, "neg": 1.0})  # m/s²

SPEED_RATE = {
    "pos": abs(SPEED_RATE_RAW["pos"]),
    "neg": abs(SPEED_RATE_RAW["neg"])
}

deg = lambda r: r * 180./math.pi
rad = lambda d: d * math.pi/180.
poly = lambda p, x: ((p["a3"]*x + p["a2"])*x + p["a1"])*x + p["a0"]

# ------------------------------------------------------------------ #
class SteeringEmulator(Node):
    def __init__(self, hz: float = 20):
        super().__init__('steering_emulator', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)


        self.declare_parameter(
            "steering_mode", "car",
            ParameterDescriptor(description="car | 4ws | crab | pivot"))
        self.mode = self.get_parameter("steering_mode"
                    ).get_parameter_value().string_value.lower()
        
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.dt   = 1.0 / hz
        self.cmd  = 0.0           # latest /cmd_vel angular.z  (rad)
        self.speed = 0.0
        self.filtered_speed = 0.0

        # per‑wheel states (rad)
        self.fl = self.fr = self.rl = self.rr = 0.0

        # ROS I/O -------------------------------------------------- #
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(self.dt, self._tick)

        self.last_cmd = self.get_clock().now()
        self.timeout  = 2.0   # s → re‑centre if no cmd_vel

        self.get_logger().info(
            f"Hz={hz:.0f}  mode={self.mode}  "
            f"RATE L +/‑ = {RATE['L']['pos']}/{RATE['L']['neg']}°/s, "
            f"R +/‑ = {RATE['R']['pos']}/{RATE['R']['neg']}°/s")

    # ---------- callbacks --------------------------------------- #
    def _on_cmd(self, msg: Twist):
        self.cmd = msg.angular.z
        self.speed = msg.linear.x
        self.last_cmd = self.get_clock().now()

    def parameters_callback(self, params):
        for param in params:
            if param.name == "steering_mode":
                self.mode = param.value
        return SetParametersResult(successful=True)

    # ---------- helpers ----------------------------------------- #
    def _slew(self, cur, tgt, lim_pos, lim_neg):
        lim  = lim_pos if tgt > cur else lim_neg
        step = rad(lim) * self.dt
        d    = max(-step, min(step, tgt - cur))
        return cur + d
    
    def _slew_speed(self, cur, tgt):
        lim = SPEED_RATE["pos"] if tgt > cur else SPEED_RATE["neg"]
        step = lim * self.dt
        d = max(-step, min(step, tgt - cur))
        return cur + d
    
    def _speed_poly(self, raw_speed):
        # Example: separate forward/reverse behavior
        if raw_speed >= 0:
            return poly(POLY["speed"], raw_speed)
        else:
            return -poly(POLY["speed"], -raw_speed)


    # ---------- main loop --------------------------------------- #
    def _tick(self):
        now = self.get_clock().now()
        timed_out = (now - self.last_cmd).nanoseconds / 1e9 > self.timeout

        cmd_rad = 0.0 if timed_out else self.cmd
        tgt_speed = 0.0 if timed_out else self.speed

        # per‑wheel calibrated targets
        tgt_rad = {
            "L": rad(poly(POLY["L"], deg(cmd_rad))),
            "R": rad(poly(POLY["R"], deg(cmd_rad))),
            
        }

        # update front wheels
            # decide wheel targets based on mode
        if self.mode == "car":
            tgt_fl, tgt_fr = tgt_rad["L"], tgt_rad["R"]
            tgt_rl, tgt_rr = 0.0, 0.0

        elif self.mode == "4ws":
            tgt_fl, tgt_fr = tgt_rad["L"], tgt_rad["R"]
            tgt_rl, tgt_rr = -tgt_fl, -tgt_fr

        elif self.mode == "crab":
            tgt_fl, tgt_fr = tgt_rad["L"], tgt_rad["R"]
            tgt_rl, tgt_rr = tgt_fl, tgt_fr

        elif self.mode == "pivot":
            tgt_fl, tgt_fr = tgt_rad["L"], -tgt_rad["R"]
            tgt_rl, tgt_rr = -tgt_rad["L"], tgt_rad["R"]

        else:
            tgt_fl = tgt_fr = tgt_rl = tgt_rr = 0.0
        
        self.fl = self._slew(self.fl, tgt_fl, RATE["L"]["pos"], RATE["L"]["neg"])
        self.fr = self._slew(self.fr, tgt_fr, RATE["R"]["pos"], RATE["R"]["neg"])

        # update rear wheels
        self.rl = self._slew(self.rl, tgt_rl, RATE["L"]["pos"], RATE["L"]["neg"])
        self.rr = self._slew(self.rr, tgt_rr, RATE["R"]["pos"], RATE["R"]["neg"])
            
        self.filtered_speed = self._slew_speed(self.filtered_speed, tgt_speed)

        # Apply polynomial transform to smoothed speed
        filtered_output = self._speed_poly(self.filtered_speed)

        # publish ------------------------------------------------- #
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name     = ["front_left", "front_right",
                       "rear_left",  "rear_right", "speed"]
        js.position = [self.fl, self.fr, self.rl, self.rr, filtered_output]
        self.pub.publish(js)

# ------------------------------------------------------------------ #
def main():
    rclpy.init()
    rclpy.spin(SteeringEmulator())
    rclpy.shutdown()

if __name__ == "__main__":
    main()