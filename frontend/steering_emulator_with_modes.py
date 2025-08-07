#!/usr/bin/env python3
"""
/cmd_vel ➜ /joint_states   with *independent* L/R dynamics.
Modes: car | 4ws | crab | pivot
"""


from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
import json, math, pathlib, rclpy, collections, numpy as np



# ------------------------------------------------------------------ #
# 1 · Load configuration                                             #
# ------------------------------------------------------------------ #
BASE = pathlib.Path(__file__).resolve().parent      # 🔸 NEW
CFG_STEER = BASE / "steering_params"
CFG_VEL   = BASE / "velocity_params"
WHEEL_RADIUS = 0.78 #meters


def _load_from(folder, name, default):
    try: return json.loads((folder / name).read_text())
    except Exception: return default


POLY = {
    "L":     _load_from(CFG_STEER, "poly_front_left.json",
                        {"a3":0,"a2":0,"a1":1,"a0":0}),
    "R":     _load_from(CFG_STEER, "poly_front_right.json",
                        {"a3":0,"a2":0,"a1":1,"a0":0}),
    "speed": _load_from(CFG_VEL,   "poly_speed.json",
                        {"a3":0,"a2":1,"a1":2,"a0":0}),
}

RATE_RAW = _load_from(CFG_STEER, "steering_rate_profile.json",
                      {"pos": 90, "neg": 90})

def _lim(side, sign):                     # resolves flat vs nested layout
    if "pos" in RATE_RAW:
        return abs(RATE_RAW[sign])
    return abs(RATE_RAW[side][sign])

RATE = {
    "L": {"pos": _lim("L", "pos"), "neg": _lim("L", "neg")},
    "R": {"pos": _lim("R", "pos"), "neg": _lim("R", "neg")},
    
}

ACCEL_POLY = _load_from(CFG_VEL, "speed_accel_poly.json",
                        {"b2":-0.01,"b1":0.03,"b0":0.01})
DELAY_S    = _load_from(CFG_VEL, "speed_delay.json",
                        {"Td_ms":250})["Td_ms"]/1000.0



RPM_LIMITS = (100, 180)

deg = lambda r: r * 180./math.pi
rad = lambda d: d * math.pi/180.
poly = lambda p, x: ((p["a3"]*x + p["a2"])*x + p["a1"])*x + p["a0"]
ms_to_rad = lambda ms : ms/WHEEL_RADIUS


def a_max(u):                      
    return max(0.0,
               ACCEL_POLY["b2"]*u*u +
               ACCEL_POLY["b1"]*u   +
               ACCEL_POLY["b0"])

# ------------------------------------------------------------------ #
class SteeringEmulator(Node):
    def __init__(self, hz: float = 20):
        super().__init__('steering_emulator', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)


        self.declare_parameter(
            "steering_mode", "car",
            ParameterDescriptor(description="car | 4ws | crab | pivot"))
        self.mode = self.get_parameter("steering_mode"
                    ).get_parameter_value().string_value.lower()
        
        self.declare_parameter(
            "RPM", 160,
            ParameterDescriptor(description="RPM of robot's engine"))
        self.rpm = self.get_parameter("RPM"
                    ).get_parameter_value().integer_value
        
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.dt   = 1.0 / hz
        self.cmd_ang  = 0.0           # latest /cmd_vel angular.z  (rad)
        self.speed = 0.0
        self.filtered_speed = 0.0

        self.v         = 0.0                      # current speed
        self.out_vel   = 0.0
        self.delay_q   = collections.deque()      # 🔸 NEW (t,u)
        

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
        self.cmd_ang = -msg.angular.z
        self.cmd_thr = msg.linear.x
        self.last_cmd = self.get_clock().now()

        now = self.last_cmd.nanoseconds*1e-9
        self.delay_q.append((now, self.cmd_thr))

    def parameters_callback(self, params):
        for param in params:
            if param.name == "steering_mode":
                self.mode = param.value
            elif param.name == "RPM":
                if int(param.value) >= RPM_LIMITS[0] and int(param.value) <= RPM_LIMITS[1]:
                    self.rpm = int(param.value) 
                else: 
                    return SetParametersResult(successful=False)

        return SetParametersResult(successful=True)

    # ---------- helpers ----------------------------------------- #
    def _slew(self, cur, tgt, lim_pos, lim_neg):
        lim  = lim_pos if tgt > cur else lim_neg
        step = rad(lim) * self.dt
        d    = max(-step, min(step, tgt - cur))
        return cur + d
    



    # ---------- main loop --------------------------------------- #
    def _tick(self):
        now = self.get_clock().now()
        if (now - self.last_cmd).nanoseconds/1e9 > self.timeout:
            self.cmd_ang = 0.0; self.cmd_thr = 0.0; self.delay_q.clear()



        # per‑wheel calibrated targets
        tgt_rad = {
            "L": rad(poly(POLY["L"], deg(self.cmd_ang))),
            "R": rad(poly(POLY["R"], deg(self.cmd_ang))),
            
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
            
        now_s = now.nanoseconds*1e-9
        while self.delay_q and now_s - self.delay_q[0][0] >= DELAY_S:
            _, self._u_eff = self.delay_q.popleft()
        u_eff = getattr(self,"_u_eff",0.0)

        v_inf = poly(POLY["speed"], u_eff)
        dv_lim = a_max(u_eff)*self.dt
        dv = max(-dv_lim, min(dv_lim, v_inf - self.v))
        self.v += dv    
        self.out_vel = ms_to_rad(self.v)
        wheel_angular_velocities = {"fl": self.out_vel, "fr": self.out_vel, "rl": self.out_vel, "rr": self.out_vel} #TODO: Add logic for speed calculation while steering

        # publish ------------------------------------------------- #
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name     = ["front_left", "front_right",
                       "rear_left",  "rear_right"]
        js.position = [self.fl, self.fr, self.rl, self.rr]

        js.velocity = [
            wheel_angular_velocities["fl"],
            wheel_angular_velocities["fr"],
            wheel_angular_velocities["rl"],
            wheel_angular_velocities["rr"],
        ]
        self.pub.publish(js)






# ------------------------------------------------------------------ #
def main():
    rclpy.init()
    rclpy.spin(SteeringEmulator())
    rclpy.shutdown()

if __name__ == "__main__":
    main()